/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <errno.h>
#include <zephyr/sys/byteorder.h>
#include <zmk/ptp.h>

/* IQS5xx-B000: 0x000F..0x0038, five stable seven-byte finger slots (5.2.6). */
#define IQS5XX_SAMPLE_SIZE 42

static inline int iqs5xx_decode(const uint8_t sample[IQS5XX_SAMPLE_SIZE], uint16_t max_x,
                                uint16_t max_y, bool swap, struct zmk_ptp_frame *frame) {
    *frame = (struct zmk_ptp_frame){0};
    if (sample[0] & BIT(7)) {
        return -EPIPE; /* Controller reset: reconfigure before accepting contacts. */
    }
    if (sample[1] & (BIT(1) | BIT(2))) {
        return -ECANCELED; /* Palm / too many fingers; XY outputs are suppressed. */
    }
    if (sample[2] > CONFIG_ZMK_TRACKPAD_FINGERS) {
        return -EINVAL;
    }
    if (!sample[2]) {
        return 0; /* Do not interpret residual coordinates as active contacts. */
    }
    for (int id = 0; id < ZMK_PTP_MAX_CONTACTS; id++) {
        const uint8_t *p = sample + 7 + id * 7;
        uint16_t x = sys_get_be16(p), y = sys_get_be16(p + 2);
        if (!sys_get_be16(p + 4) || !p[6] || x == UINT16_MAX || y == UINT16_MAX) {
            continue;
        }
        if (x > max_x || y > max_y) {
            return -EINVAL;
        }
        frame->contacts[frame->contact_count++] = (struct zmk_ptp_contact){
            .id = id,
            .x = swap ? y : x,
            .y = swap ? x : y,
            .confidence = true,
        };
    }
    return frame->contact_count == sample[2] ? 0 : -EINVAL;
}
