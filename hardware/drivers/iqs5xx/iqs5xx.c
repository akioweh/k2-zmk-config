/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * Register protocol: IQS5xx-B000 datasheet, revision 2.1, sections 5 and 8.
 */
#define DT_DRV_COMPAT azoteq_iqs5xx_ptp

#include "iqs5xx.h"
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/atomic.h>
#include <zmk/ptp/queue.h>

LOG_MODULE_REGISTER(iqs5xx_ptp, CONFIG_ZMK_LOG_LEVEL);

/* USB/keyboard work must not delay sensor acquisition. Transport queues remain
 * independently bounded; this thread never changes the ASIC sampling rate. */
K_THREAD_STACK_DEFINE(sample_stack, CONFIG_ZMK_IQS5XX_WORKQUEUE_STACK_SIZE);
static struct k_work_q sample_queue;

struct config {
    struct i2c_dt_spec bus;
    struct gpio_dt_spec ready, reset;
    uint16_t x, y;
    uint8_t filter;
    bool swap, invert_x, invert_y;
};
struct observation {
    struct zmk_ptp_queue_header header;
    struct zmk_ptp_frame frame;
};
/* Acquisition never waits for output admission. Overflow invalidates history,
 * rather than silently reusing a slot whose lift was dropped. */
K_MSGQ_DEFINE(observations, sizeof(struct observation), 4, 4);
struct data {
    const struct device *dev;
    struct gpio_callback ready;
    struct k_work_delayable work, output;
    struct k_mutex lock, send_lock;
    struct k_spinlock window_lock, output_lock;
    atomic_t paused, acquiring;
    int64_t window_ticks;
    bool window_valid;
    struct zmk_ptp_frame frame, attempted, accepted;
    uint32_t ticket, output_epoch;
    uint16_t cancel_time;
    bool cancel_pending, lifting, configured;
};

static int read_reg(const struct config *cfg, uint16_t reg, void *bytes, size_t size) {
    uint8_t address[2];
    sys_put_be16(reg, address);
    int err = i2c_write_read_dt(&cfg->bus, address, 2, bytes, size);
    if (err) {
        /* Low-power wake can NACK the first addressing (datasheet 8.8.2). */
        k_sleep(K_USEC(200));
        err = i2c_write_read_dt(&cfg->bus, address, 2, bytes, size);
    }
    return err;
}

static int write_reg(const struct config *cfg, uint16_t reg, uint16_t value, bool wide) {
    uint8_t bytes[4];
    sys_put_be16(reg, bytes);
    if (wide) {
        sys_put_be16(value, bytes + 2);
    } else {
        bytes[2] = value;
    }
    return i2c_write_dt(&cfg->bus, bytes, wide ? 4 : 3);
}

static int close_window(const struct config *cfg) {
    /* Datasheet 8.7: address 0xEEEE, one arbitrary data byte, then STOP. */
    return write_reg(cfg, 0xeeee, 0, false);
}

static int set_suspend(const struct config *cfg, bool suspend) {
    uint8_t control;
    int err = read_reg(cfg, 0x0432, &control, 1);
    if (!err) {
        /* Never repeat the adjacent RESET command. */
        control &= ~BIT(1);
        control = suspend ? control | BIT(0) : control & ~BIT(0);
        err = write_reg(cfg, 0x0432, control, false);
    }
    int end = close_window(cfg);
    return err ? err : end;
}

static int configure(const struct config *cfg) {
    uint8_t product[2], xy, status;
    /* Also recover from an interrupted/failed PM transition. */
    int err = set_suspend(cfg, false);
    if (err) {
        return err;
    }
    err = read_reg(cfg, 0, product, sizeof(product));
    if (err) {
        return err;
    }
    uint16_t id = sys_get_be16(product);
    if (id != 58 && id != 40 && id != 52) {
        return -ENODEV;
    }
    err = read_reg(cfg, 0x0669, &xy, 1);
    if (err) {
        return err;
    }
    err = read_reg(cfg, 0x000f, &status, 1);
    if (err) {
        return err;
    }
    if (status & BIT(7)) {
        /* Retune after reset, not after transient communication failures. */
        err = write_reg(cfg, 0x0431, BIT(7) | BIT(5), false);
        if (err) {
            return err;
        }
    }
    /* Keep factory sensing/palm settings; perform the board's axis swap in
     * software. */
    const struct {
        uint16_t reg, value;
        bool wide;
    } settings[] = {
        {0x058f, BIT(0) | BIT(2) | BIT(3) | BIT(6), false}, /* Event mode, XY, re-ATI, touch */
        {0x0669, (xy & ~7) | (cfg->invert_x ? BIT(0) : 0) | (cfg->invert_y ? BIT(1) : 0), false},
        {0x066a, CONFIG_ZMK_TRACKPAD_FINGERS, false},
        {0x066e, cfg->x, true},
        {0x0670, cfg->y, true},
        {0x0632, cfg->filter, false},
        {0x06b7, 0, false},
        {0x06b8, 0, false}, /* No firmware gestures. */
        {0x057a, 10, true},
        {0x057c, 10, true},
        {0x0582, 640, true},                       /* Preserve the board's low-power idle rate. */
        {0x058e, BIT(6) | BIT(5) | BIT(2), false}, /* Setup, watchdog, re-ATI */
    };
    for (size_t i = 0; i < ARRAY_SIZE(settings); i++) {
        err = write_reg(cfg, settings[i].reg, settings[i].value, settings[i].wide);
        if (err) {
            return err;
        }
    }
    return 0;
}

/* Union by hardware slot: cancel both the admitted lifetime and a failed
 * attempt (peripheral submission can cache it despite returning an error). */
static struct zmk_ptp_frame cancellation(struct data *data, uint16_t time) {
    struct zmk_ptp_frame frame = {.scan_time = time};
    const struct zmk_ptp_frame *sources[] = {&data->accepted, &data->attempted};
    for (int id = 0; id < ZMK_PTP_MAX_CONTACTS; id++) {
        const struct zmk_ptp_contact *found = NULL;
        for (int s = 0; s < ARRAY_SIZE(sources); s++) {
            for (int i = 0; i < sources[s]->contact_count; i++) {
                if (sources[s]->contacts[i].id == id) {
                    found = &sources[s]->contacts[i];
                }
            }
        }
        if (found) {
            frame.contacts[frame.contact_count] = *found;
            frame.contacts[frame.contact_count++].confidence = false;
        }
    }
    return frame;
}

static void enqueue(struct data *data, const struct zmk_ptp_frame *frame) {
    struct observation item = {.frame = *frame}, scratch;
    k_spinlock_key_t key = k_spin_lock(&data->output_lock);
    item.header = (struct zmk_ptp_queue_header){
        .ticket = ++data->ticket,
        .state = zmk_ptp_frame_state(frame),
        .motion = zmk_ptp_frame_state(frame) == zmk_ptp_frame_state(&data->frame),
    };
    if (zmk_ptp_queue_put(&observations, &item, &scratch) < 0) {
        k_msgq_purge(&observations);
        data->output_epoch++;
        data->cancel_pending = true;
        data->lifting = false;
        data->cancel_time = frame->scan_time;
        item.header.motion = false;
        k_msgq_put(&observations, &item, K_NO_WAIT);
    }
    k_spin_unlock(&data->output_lock, key);
    data->frame = *frame;
    k_work_reschedule(&data->output, K_NO_WAIT);
}

static void output(struct k_work *work) {
    struct data *data = CONTAINER_OF(k_work_delayable_from_work(work), struct data, output);
    k_mutex_lock(&data->send_lock, K_FOREVER);
    while (!atomic_get(&data->paused)) {
        struct observation item, scratch;
        k_spinlock_key_t key = k_spin_lock(&data->output_lock);
        uint32_t epoch = data->output_epoch;
        bool cancel = data->cancel_pending, lift = data->lifting;
        uint16_t time = data->cancel_time;
        bool available = k_msgq_peek(&observations, &item) == 0;
        k_spin_unlock(&data->output_lock, key);
        if (!cancel && !available) {
            break;
        }
        struct zmk_ptp_frame frame =
            cancel ? (lift ? (struct zmk_ptp_frame){.scan_time = time} : cancellation(data, time))
                   : item.frame;
        data->attempted = frame;
        int err = zmk_ptp_submit_frame(&frame);
        if (!err) {
            data->accepted = frame;
            key = k_spin_lock(&data->output_lock);
            if (epoch == data->output_epoch) {
                if (cancel) {
                    data->lifting = !lift;
                    data->cancel_pending = !lift;
                } else {
                    zmk_ptp_queue_get(&observations, item.header.ticket, &scratch);
                }
            }
            k_spin_unlock(&data->output_lock, key);
        } else {
            /* Retry the same observation/time. Neither the sensor nor its
             * communication window is held while the transport is unavailable. */
            k_work_reschedule(&data->output, K_MSEC(err == -ENODEV || err == -EMSGSIZE ? 50 : 5));
            break;
        }
    }
    k_mutex_unlock(&data->send_lock);
}

static void sample(struct data *data) {
    const struct config *cfg = data->dev->config;
    uint8_t sample[IQS5XX_SAMPLE_SIZE];
    int err;
    if (!data->configured) {
        atomic_clear(&data->acquiring);
        k_spinlock_key_t key = k_spin_lock(&data->window_lock);
        data->window_valid = false;
        k_spin_unlock(&data->window_lock, key);
        err = configure(cfg);
        /* Configuration's forced RDY edges are not scans. Once setup is done,
         * capture a real next window even if it arrives during termination. */
        if (!err) {
            atomic_set(&data->acquiring, 1);
        }
        int end = close_window(cfg);
        err = err ? err : end;
        if (err) {
            atomic_clear(&data->acquiring);
            LOG_WRN("Configuration failed: %d", err);
            k_work_schedule_for_queue(&sample_queue, &data->work, K_MSEC(250));
            return;
        }
        data->configured = true;
    }
    /* Normal sampling never forces a window. A level check also catches a
     * missed edge; it cannot recover the original scan-completion time. */
    k_spinlock_key_t key = k_spin_lock(&data->window_lock);
    if (gpio_pin_get_dt(&cfg->ready) <= 0) {
        data->window_valid = false; /* An expired window cannot timestamp a later one. */
        k_spin_unlock(&data->window_lock, key);
        if (data->frame.contact_count) {
            k_work_schedule_for_queue(&sample_queue, &data->work, K_MSEC(50));
        }
        return;
    }
    int64_t ticks = data->window_valid ? data->window_ticks : k_uptime_ticks();
    data->window_valid = false;
    k_spin_unlock(&data->window_lock, key);
    uint16_t time = (uint16_t)(k_ticks_to_us_floor64(ticks) / 100);
    /* Unlike setup/PM reads, do not retry a sample by forcing another window:
     * that new scan would no longer belong to the captured RDY timestamp. */
    uint8_t address[] = {0x00, 0x0f};
    err = i2c_write_read_dt(&cfg->bus, address, sizeof(address), sample, sizeof(sample));
    int end = close_window(cfg);
    err = err ? err : end;
    struct zmk_ptp_frame frame;
    if (!err) {
        err = iqs5xx_decode(sample, cfg->x, cfg->y, cfg->swap, &frame);
    }
    if (err) {
        if (err != -ECANCELED) {
            data->configured = false;
            atomic_clear(&data->acquiring);
        }
        if (data->frame.contact_count) {
            /* Invalidate the old lifetime before lifting, with the time of the
             * observation that detected its loss, not the transport retry time. */
            frame = data->frame;
            frame.scan_time = time;
            for (int i = 0; i < frame.contact_count; i++) {
                frame.contacts[i].confidence = false;
            }
            enqueue(data, &frame);
            frame = (struct zmk_ptp_frame){.scan_time = time};
            enqueue(data, &frame);
        }
        k_work_schedule_for_queue(&sample_queue, &data->work, K_MSEC(data->configured ? 50 : 250));
        return;
    }
    frame.scan_time = time;
    enqueue(data, &frame);
    if (frame.contact_count) {
        /* Check RDY level, not forced I2C polling. This does not invent scans
         * for a stationary contact in event mode. */
        k_work_schedule_for_queue(&sample_queue, &data->work, K_MSEC(50));
    }
}

static void work(struct k_work *item) {
    struct data *data = CONTAINER_OF(k_work_delayable_from_work(item), struct data, work);
    const struct config *cfg = data->dev->config;
    k_mutex_lock(&data->lock, K_FOREVER);
    if (!atomic_get(&data->paused)) {
        sample(data);
        /* Service a window arriving during the read/close without bypassing
         * recovery backoff. Its IRQ has already captured its own timestamp. */
        if (data->configured && gpio_pin_get_dt(&cfg->ready) > 0) {
            k_work_reschedule_for_queue(&sample_queue, &data->work, K_NO_WAIT);
        }
    }
    k_mutex_unlock(&data->lock);
}

static void ready(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct data *data = CONTAINER_OF(cb, struct data, ready);
    if (!atomic_get(&data->paused) && atomic_get(&data->acquiring)) {
        k_spinlock_key_t key = k_spin_lock(&data->window_lock);
        data->window_ticks = k_uptime_ticks();
        data->window_valid = true;
        k_spin_unlock(&data->window_lock, key);
        k_work_reschedule_for_queue(&sample_queue, &data->work, K_NO_WAIT);
    }
}

#if IS_ENABLED(CONFIG_PM_DEVICE)
static bool cleanup_failed(int err) {
    /* Peripherals cache physical observations despite transport errors. A local
     * host producer must not emit a confident lift after failed cancellation. */
    if (IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)) {
        return err == -EAGAIN || err == -ENOMSG || err == -ENOSPC || err == -ENOMEM;
    }
    return err != 0;
}

static int cancel_contacts(struct data *data) {
    uint16_t time = zmk_ptp_scan_time();
    /* Even a partially failed suspend must not replay pre-cancellation data.
     * On veto, output completes this barrier before accepting a fresh touch. */
    k_spinlock_key_t key = k_spin_lock(&data->output_lock);
    k_msgq_purge(&observations);
    data->output_epoch++;
    data->cancel_pending = true;
    data->lifting = false;
    data->cancel_time = time;
    k_spin_unlock(&data->output_lock, key);
    struct zmk_ptp_frame frame = cancellation(data, time);
    data->attempted = frame;
    int err = zmk_ptp_submit_frame(&frame);
    if (cleanup_failed(err)) {
        return -EBUSY;
    }
    data->accepted = frame;
    data->lifting = true;
    frame = (struct zmk_ptp_frame){.scan_time = time};
    data->attempted = frame;
    err = zmk_ptp_submit_frame(&frame);
    if (cleanup_failed(err)) {
        return -EBUSY;
    }
    data->frame = data->accepted = data->attempted = frame;
    data->cancel_pending = data->lifting = false;
    return 0;
}

static int pm_action(const struct device *dev, enum pm_device_action action) {
    if (action != PM_DEVICE_ACTION_SUSPEND && action != PM_DEVICE_ACTION_RESUME) {
        return -ENOTSUP;
    }
    struct data *data = dev->data;
    const struct config *cfg = dev->config;
    k_mutex_lock(&data->lock, K_FOREVER);
    k_mutex_lock(&data->send_lock, K_FOREVER);
    atomic_set(&data->paused, 1);
    atomic_clear(&data->acquiring);
    k_work_cancel_delayable(&data->output);
    k_spinlock_key_t key = k_spin_lock(&data->window_lock);
    data->window_valid = false;
    k_spin_unlock(&data->window_lock, key);
    int err = gpio_pin_interrupt_configure_dt(&cfg->ready, GPIO_INT_DISABLE);
    /* The mutex drains running I/O; a late queued callback sees paused.
     * No synchronous cancellation is needed while holding the I/O mutex. */
    k_work_cancel_delayable(&data->work);
    if (!err && action == PM_DEVICE_ACTION_SUSPEND) {
        err = cancel_contacts(data);
    }
    if (!err) {
        err = set_suspend(cfg, action == PM_DEVICE_ACTION_SUSPEND);
    }
    if (action == PM_DEVICE_ACTION_RESUME || err) {
        if (err && action == PM_DEVICE_ACTION_SUSPEND) {
            /* PM keeps the device active on failure. Resume/reconfigure it. */
            set_suspend(cfg, false);
        }
        data->configured = false;
        if (!err || action == PM_DEVICE_ACTION_SUSPEND) {
            int irq = gpio_pin_interrupt_configure_dt(&cfg->ready, GPIO_INT_EDGE_TO_ACTIVE);
            if (!irq) {
                atomic_clear(&data->paused);
                k_work_reschedule_for_queue(&sample_queue, &data->work, K_NO_WAIT);
                k_work_reschedule(&data->output, K_NO_WAIT);
            }
            if (!err) {
                err = irq;
            }
        }
    }
    k_mutex_unlock(&data->send_lock);
    k_mutex_unlock(&data->lock);
    return err;
}
#endif

static int init(const struct device *dev) {
    const struct config *cfg = dev->config;
    struct data *data = dev->data;
    data->dev = dev;
    k_work_queue_start(&sample_queue, sample_stack, K_THREAD_STACK_SIZEOF(sample_stack),
                       K_PRIO_PREEMPT(5), NULL);
    k_mutex_init(&data->lock);
    k_mutex_init(&data->send_lock);
    k_work_init_delayable(&data->work, work);
    k_work_init_delayable(&data->output, output);
    if (!i2c_is_ready_dt(&cfg->bus) || !gpio_is_ready_dt(&cfg->ready) ||
        !gpio_is_ready_dt(&cfg->reset)) {
        return -ENODEV;
    }
    int err = gpio_pin_configure_dt(&cfg->ready, GPIO_INPUT);
    if (err) {
        return err;
    }
    err = gpio_pin_configure_dt(&cfg->reset, GPIO_OUTPUT_INACTIVE);
    if (err) {
        return err;
    }
    k_sleep(K_MSEC(10));
    err = gpio_pin_set_dt(&cfg->reset, 1);
    if (err) {
        return err;
    }
    k_sleep(K_MSEC(610));
    gpio_init_callback(&data->ready, ready, BIT(cfg->ready.pin));
    err = gpio_add_callback(cfg->ready.port, &data->ready);
    if (err) {
        return err;
    }
    err = gpio_pin_interrupt_configure_dt(&cfg->ready, GPIO_INT_EDGE_TO_ACTIVE);
    if (!err) {
        k_work_schedule_for_queue(&sample_queue, &data->work, K_NO_WAIT);
    }
    return err;
}

#define INSTANCE(n)                                                                                \
    BUILD_ASSERT(DT_INST_PROP(n, switch_xy)                                                        \
                     ? (DT_INST_PROP(n, y_resolution) <= CONFIG_ZMK_TRACKPAD_LOGICAL_X &&          \
                        DT_INST_PROP(n, x_resolution) <= CONFIG_ZMK_TRACKPAD_LOGICAL_Y)            \
                     : (DT_INST_PROP(n, x_resolution) <= CONFIG_ZMK_TRACKPAD_LOGICAL_X &&          \
                        DT_INST_PROP(n, y_resolution) <= CONFIG_ZMK_TRACKPAD_LOGICAL_Y),           \
                 "Sensor coordinates exceed the host's logical range");                            \
    static struct data data_##n;                                                                   \
    static const struct config config_##n = {                                                      \
        .bus = I2C_DT_SPEC_INST_GET(n),                                                            \
        .ready = GPIO_DT_SPEC_INST_GET(n, rdy_gpios),                                              \
        .reset = GPIO_DT_SPEC_INST_GET(n, rst_gpios),                                              \
        .x = DT_INST_PROP(n, x_resolution),                                                        \
        .y = DT_INST_PROP(n, y_resolution),                                                        \
        .filter = DT_INST_PROP(n, filter_settings),                                                \
        .swap = DT_INST_PROP(n, switch_xy),                                                        \
        .invert_x = DT_INST_PROP(n, invert_x),                                                     \
        .invert_y = DT_INST_PROP(n, invert_y),                                                     \
    };                                                                                             \
    PM_DEVICE_DT_INST_DEFINE(n, pm_action);                                                        \
    DEVICE_DT_INST_DEFINE(n, init, PM_DEVICE_DT_INST_GET(n), &data_##n, &config_##n, POST_KERNEL,  \
                          90, NULL);

BUILD_ASSERT(CONFIG_ZMK_TRACKPAD_FINGERS == 5, "IQS5xx uses five stable hardware slots");
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1, "One logical touchpad is supported");
DT_INST_FOREACH_STATUS_OKAY(INSTANCE)
