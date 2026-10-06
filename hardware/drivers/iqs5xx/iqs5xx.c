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
struct data {
    const struct device *dev;
    struct gpio_callback ready;
    struct k_work_delayable work;
    struct k_mutex lock;
    atomic_t paused, servicing;
    struct zmk_ptp_frame frame;
    bool pending, barrier, cancelling, configured, await_ready;
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

/* Link/MTU errors on a peripheral still cache the physical observation in the
 * generic API. Keep reading the sensor so an offline lift replaces a held
 * finger. */
static bool retry_admission(int err) {
    return err == -EAGAIN || err == -ENOMSG || err == -ENOSPC || err == -ENOMEM;
}

static void sample(struct data *data) {
    const struct config *cfg = data->dev->config;
    if (data->pending) {
        int err = zmk_ptp_submit_frame(&data->frame);
        if (retry_admission(err)) {
            /* A peripheral must refresh a blocked held snapshot with an observed
             * offline lift. Preserve lift/cancellation barriers before reusing IDs.
             */
            bool refresh = IS_ENABLED(CONFIG_ZMK_SPLIT) &&
                           !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL) &&
                           data->frame.contact_count && !data->cancelling && !data->barrier;
            if (!refresh) {
                k_work_schedule_for_queue(&sample_queue, &data->work, K_MSEC(5));
                return;
            }
        }
        data->pending = data->barrier = false;
        if (data->cancelling) {
            data->cancelling = false;
            data->frame = (struct zmk_ptp_frame){.scan_time = zmk_ptp_scan_time()};
            data->pending = true;
            k_work_schedule_for_queue(&sample_queue, &data->work, K_NO_WAIT);
            return;
        }
    }

    uint8_t sample[IQS5XX_SAMPLE_SIZE];
    int err;
    if (!data->configured) {
        err = configure(cfg);
        int end = close_window(cfg);
        err = err ? err : end;
        if (err) {
            LOG_WRN("Configuration failed: %d", err);
            k_work_schedule_for_queue(&sample_queue, &data->work, K_MSEC(250));
            return;
        }
        data->configured = true;
    }
    /* Suspend retains sensor data. Wait for a fresh post-resume RDY window,
     * rather than replaying old contacts through a forced read. */
    if (data->await_ready && gpio_pin_get_dt(&cfg->ready) <= 0) {
        return;
    }
    data->await_ready = false;
    err = read_reg(cfg, 0x000f, sample, sizeof(sample));
    int end = close_window(cfg);
    err = err ? err : end;
    struct zmk_ptp_frame frame;
    if (!err) {
        err = iqs5xx_decode(sample, cfg->x, cfg->y, cfg->swap, &frame);
    }
    if (err) {
        if (err != -ECANCELED) {
            data->configured = false;
        }
        if (!data->frame.contact_count) {
            k_work_schedule_for_queue(&sample_queue, &data->work,
                                      K_MSEC(data->configured ? 50 : 250));
            return;
        }
        /* Mark a lost/palm contact unintentional before lifting it. */
        for (int i = 0; i < data->frame.contact_count; i++) {
            data->frame.contacts[i].confidence = false;
        }
        data->frame.scan_time = zmk_ptp_scan_time();
        data->cancelling = true;
        data->pending = true;
        k_work_schedule_for_queue(&sample_queue, &data->work, K_MSEC(5));
        return;
    }
    frame.scan_time = zmk_ptp_scan_time();
    /* A heartbeat can now contend with this independent reader. Never refresh
     * an unadmitted state change, even when its contacts are still held. */
    data->barrier = zmk_ptp_frame_state(&frame) != zmk_ptp_frame_state(&data->frame);
    data->frame = frame;
    data->pending = true;
    int admission = zmk_ptp_submit_frame(&frame);
    data->pending = retry_admission(admission);
    if (data->pending || frame.contact_count || gpio_pin_get_dt(&cfg->ready) > 0) {
        /* Active polling also checks sensor health; split heartbeats alone cannot.
         */
        k_work_schedule_for_queue(&sample_queue, &data->work, K_MSEC(data->pending ? 5 : 50));
    }
}

static void work(struct k_work *item) {
    struct data *data = CONTAINER_OF(k_work_delayable_from_work(item), struct data, work);
    const struct config *cfg = data->dev->config;
    k_mutex_lock(&data->lock, K_FOREVER);
    if (!atomic_get(&data->paused)) {
        /* Forced I2C communication also raises RDY. */
        atomic_set(&data->servicing, 1);
        sample(data);
        atomic_clear(&data->servicing);
        /* Service a pending window without bypassing backoff. */
        if (data->configured && !data->pending && gpio_pin_get_dt(&cfg->ready) > 0) {
            k_work_reschedule_for_queue(&sample_queue, &data->work, K_NO_WAIT);
        }
    }
    k_mutex_unlock(&data->lock);
}

static void ready(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct data *data = CONTAINER_OF(cb, struct data, ready);
    if (!atomic_get(&data->paused) && !atomic_get(&data->servicing)) {
        k_work_reschedule_for_queue(&sample_queue, &data->work, K_NO_WAIT);
    }
}

#if IS_ENABLED(CONFIG_PM_DEVICE)
static bool cleanup_failed(int err) {
    /* Peripherals cache physical observations despite transport errors. A local
     * host producer must not emit a confident lift after failed cancellation. */
    if (IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)) {
        return retry_admission(err);
    }
    return err != 0;
}

static int cancel_contacts(struct data *data) {
    if (data->frame.contact_count) {
        for (int i = 0; i < data->frame.contact_count; i++) {
            data->frame.contacts[i].confidence = false;
        }
        data->frame.scan_time = zmk_ptp_scan_time();
        data->pending = data->cancelling = true;
        if (cleanup_failed(zmk_ptp_submit_frame(&data->frame))) {
            return -EBUSY;
        }
    }
    if (cleanup_failed(zmk_ptp_release())) {
        return -EBUSY;
    }
    data->frame = (struct zmk_ptp_frame){0};
    data->pending = data->cancelling = false;
    return 0;
}

static int pm_action(const struct device *dev, enum pm_device_action action) {
    if (action != PM_DEVICE_ACTION_SUSPEND && action != PM_DEVICE_ACTION_RESUME) {
        return -ENOTSUP;
    }
    struct data *data = dev->data;
    const struct config *cfg = dev->config;
    k_mutex_lock(&data->lock, K_FOREVER);
    atomic_set(&data->paused, 1);
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
        data->await_ready = true;
        if (!err || action == PM_DEVICE_ACTION_SUSPEND) {
            int irq = gpio_pin_interrupt_configure_dt(&cfg->ready, GPIO_INT_EDGE_TO_ACTIVE);
            if (!irq) {
                atomic_clear(&data->paused);
                k_work_reschedule_for_queue(&sample_queue, &data->work, K_NO_WAIT);
            }
            if (!err) {
                err = irq;
            }
        }
    }
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
    k_work_init_delayable(&data->work, work);
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
