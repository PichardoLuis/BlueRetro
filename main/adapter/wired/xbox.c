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
    ps_fb_to_generic(dev_mode, raw_fb_data, fb_data);
}
