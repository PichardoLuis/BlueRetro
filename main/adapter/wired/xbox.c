/*
 * Copyright (c) 2025, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ps.h"
#include "xbox.h"

void xbox_meta_init(struct wired_ctrl *ctrl_data) {
    ps_meta_init(ctrl_data);
}

void xbox_init_buffer(int32_t dev_mode, struct wired_data *wired_data) {
    ps_init_buffer(dev_mode, wired_data);
}

void xbox_from_generic(int32_t dev_mode, struct wired_ctrl *ctrl_data, struct wired_data *wired_data) {
    ps_from_generic(dev_mode, ctrl_data, wired_data);
}

void xbox_fb_to_generic(int32_t dev_mode, struct raw_fb *raw_fb_data, struct generic_fb *fb_data) {
    (void)dev_mode;
    fb_data->wired_id = raw_fb_data->header.wired_id;
    fb_data->type = raw_fb_data->header.type;

    if (raw_fb_data->header.data_len == 0) {
        fb_data->state = 0;
        fb_data->lf_pwr = fb_data->hf_pwr = 0;
    }
    else {
        switch (fb_data->type) {
            case FB_TYPE_RUMBLE:
                fb_data->state = (raw_fb_data->data[0] || raw_fb_data->data[1]) ? 1 : 0;
                fb_data->lf_pwr = raw_fb_data->data[0];
                fb_data->hf_pwr = raw_fb_data->data[1];
                break;
            case FB_TYPE_STATUS_LED:
                fb_data->led = raw_fb_data->data[0];
                break;
        }
    }
}
