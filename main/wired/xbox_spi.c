/*
 * Copyright (c) 2025, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <string.h>
#include "soc/io_mux_reg.h"
#include "esp_private/periph_ctrl.h"
#include <soc/spi_periph.h>
#include <esp32/rom/ets_sys.h>
#include <esp32/rom/gpio.h>
#include "hal/clk_gate_ll.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system/gpio.h"
#include "adapter/adapter.h"
#include "adapter/config.h"
#include "wired_bare.h"
#include "xbox_spi.h"

#define XBOX_SPI_PORT_MAX 2
#define XBOX_SPI_PAYLOAD_SIZE 32
#define XBOX_SPI_PACKET_SIZE (XBOX_SPI_PAYLOAD_SIZE + 4)

#define XBOX_SPI_MAGIC_STATE 0x58
#define XBOX_SPI_MAGIC_FB 0x52

#define XBOX_SPI_FB_FLAG_RUMBLE 0x01
#define XBOX_SPI_FB_FLAG_LED 0x02

/* Shared SPI bus + per-port CS */
#define XBOX_SPI_SCK_PIN 18
#define XBOX_SPI_MOSI_PIN 23
#define XBOX_SPI_MISO_PIN 19
#define XBOX_SPI_CS_P1_PIN 5
#define XBOX_SPI_CS_P2_PIN 4

#define XBOX_SPI_CS_P1_MASK (1U << XBOX_SPI_CS_P1_PIN)
#define XBOX_SPI_CS_P2_MASK (1U << XBOX_SPI_CS_P2_PIN)

static uint8_t active_port = 0;
static uint8_t packet[XBOX_SPI_PACKET_SIZE];
static uint8_t rx_packet[XBOX_SPI_PACKET_SIZE];
static bool monitor_started = false;
static uint8_t rumble_lf_cache[XBOX_SPI_PORT_MAX] = {0xFF, 0xFF};
static uint8_t rumble_hf_cache[XBOX_SPI_PORT_MAX] = {0xFF, 0xFF};
static uint8_t led_cache[XBOX_SPI_PORT_MAX] = {0xFF, 0xFF};
_Static_assert((XBOX_SPI_PACKET_SIZE % 4) == 0, "XBOX SPI packet size must be 4-byte aligned");
static struct spi_cfg cfg = {
    .hw = &SPI2,
    .write_bit_order = 0,
    .read_bit_order = 0,
    .clk_idle_edge = 0,
    .clk_i_edge = 1,
    .miso_delay_mode = 1,
    .miso_delay_num = 0,
    .mosi_delay_mode = 0,
    .mosi_delay_num = 0,
    .write_bit_len = 0,
    .read_bit_len = (XBOX_SPI_PACKET_SIZE * 8) - 1,
    .inten = 0,
};

static inline void xbox_spi_write_buffer(const uint8_t *data, uint32_t len) {
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t word = 0;

        memcpy(&word, &data[i], 4);
        SPI2.data_buf[(i / 4)] = word;
    }
}

static inline void xbox_spi_build_packet(uint8_t port) {
    uint8_t *out = wired_adapter.data[port].output;
    uint8_t *mask = wired_adapter.data[port].output_mask;

    packet[0] = XBOX_SPI_MAGIC_STATE;
    packet[1] = port;
    packet[2] = wired_adapter.data[port].frame_cnt;
    packet[3] = config.out_cfg[port].dev_mode;

    for (uint32_t i = 0; i < XBOX_SPI_PAYLOAD_SIZE; i++) {
        packet[4 + i] = mask[i] ? mask[i] : out[i];
    }

    xbox_spi_write_buffer(packet, XBOX_SPI_PACKET_SIZE);
}

static inline void xbox_spi_read_buffer(uint8_t *data, uint32_t len) {
    for (uint32_t i = 0; i < len; i += 4) {
        uint32_t word = SPI2.data_buf[(i / 4)];

        memcpy(&data[i], &word, 4);
    }
}

static void xbox_spi_handle_fb(void) {
    /* rx_packet layout:
     * [0]=magic, [1]=port, [2]=seq/reserved, [3]=flags, [4]=lf, [5]=hf, [6]=led
     */
    if (rx_packet[0] != XBOX_SPI_MAGIC_FB) {
        return;
    }

    uint8_t port = rx_packet[1];
    uint8_t flags = rx_packet[3];

    if (port >= XBOX_SPI_PORT_MAX) {
        return;
    }

    if (port != active_port) {
        return;
    }

    if (flags & XBOX_SPI_FB_FLAG_RUMBLE) {
        if (config.out_cfg[port].acc_mode & ACC_RUMBLE) {
            uint8_t lf = rx_packet[4];
            uint8_t hf = rx_packet[5];

            if (rumble_lf_cache[port] != lf || rumble_hf_cache[port] != hf) {
                rumble_lf_cache[port] = lf;
                rumble_hf_cache[port] = hf;
                struct raw_fb fb_data = {0};
                fb_data.header.wired_id = port;
                fb_data.header.type = FB_TYPE_RUMBLE;
                fb_data.header.data_len = 2;
                fb_data.data[0] = lf;
                fb_data.data[1] = hf;
                adapter_q_fb(&fb_data);
            }
        }
    }

    if (flags & XBOX_SPI_FB_FLAG_LED) {
        uint8_t led = rx_packet[6];

        if (led_cache[port] != led) {
            led_cache[port] = led;

            struct raw_fb fb_data = {0};
            fb_data.header.wired_id = port;
            fb_data.header.type = FB_TYPE_STATUS_LED;
            fb_data.header.data_len = 1;
            fb_data.data[0] = led;
            adapter_q_fb(&fb_data);
        }
    }
}

static inline void xbox_spi_select_port(uint8_t port) {
    if (port >= XBOX_SPI_PORT_MAX) {
        return;
    }

    active_port = port;
    gpio_matrix_in((port == 0) ? XBOX_SPI_CS_P1_PIN : XBOX_SPI_CS_P2_PIN, HSPICS0_IN_IDX, false);
    xbox_spi_build_packet(port);

    SPI2.slave.sync_reset = 1;
    SPI2.slave.sync_reset = 0;
    SPI2.slave.trans_done = 0;
    SPI2.cmd.usr = 1;
}

static void xbox_spi_monitor_cs(void) {
    const uint32_t cs_mask = XBOX_SPI_CS_P1_MASK | XBOX_SPI_CS_P2_MASK;
    uint32_t prev_state = GPIO.in & cs_mask;
    uint32_t spin = 0;

    while (1) {
        uint32_t cur_state = GPIO.in & cs_mask;
        uint32_t change = cur_state ^ prev_state;

        if (SPI2.slave.trans_done) {
            SPI2.slave.trans_done = 0;
            xbox_spi_read_buffer(rx_packet, XBOX_SPI_PACKET_SIZE);
            xbox_spi_handle_fb();
            /* frame_cnt increments once per completed SPI transaction (trans_done). */
            ++wired_adapter.data[active_port].frame_cnt;
            xbox_spi_build_packet(active_port);
            SPI2.cmd.usr = 1;
        }

        if (change && SPI2.cmd.usr == 0) {
            uint32_t stable_state = GPIO.in & cs_mask;
            if (stable_state != cur_state) {
                prev_state = stable_state;
                continue;
            }
            bool cs1 = !(cur_state & XBOX_SPI_CS_P1_MASK);
            bool cs2 = !(cur_state & XBOX_SPI_CS_P2_MASK);

            if (cs1 && cs2) {
                prev_state = cur_state;
                continue;
            }

            if (cs1) {
                xbox_spi_select_port(0);
            }
            else if (cs2) {
                xbox_spi_select_port(1);
            }
            prev_state = cur_state;
        }
        if (((cur_state & cs_mask) == cs_mask) && !SPI2.slave.trans_done) {
            ets_delay_us(50);
        }
        else {
            ets_delay_us(2);
        }
        if ((++spin & 0x3FF) == 0) {
            taskYIELD();
        }
    }
}

static void xbox_spi_monitor_task(void *arg) {
    (void)arg;
    xbox_spi_monitor_cs();
}

void xbox_spi_init(uint32_t package) {
    gpio_config_t io_conf = {0};
    const uint32_t cs_mask = XBOX_SPI_CS_P1_MASK | XBOX_SPI_CS_P2_MASK;
    uint32_t cs_state = 0;

    (void)package;

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pin_bit_mask = 1ULL << XBOX_SPI_SCK_PIN;
    gpio_config_iram(&io_conf);
    gpio_matrix_in(XBOX_SPI_SCK_PIN, HSPICLK_IN_IDX, false);

    io_conf.pin_bit_mask = (1ULL << XBOX_SPI_CS_P1_PIN) | (1ULL << XBOX_SPI_CS_P2_PIN);
    gpio_config_iram(&io_conf);
    gpio_matrix_in(XBOX_SPI_CS_P1_PIN, HSPICS0_IN_IDX, false);

    gpio_set_pull_mode_iram(XBOX_SPI_MOSI_PIN, GPIO_PULLUP_ONLY);
    gpio_set_direction_iram(XBOX_SPI_MOSI_PIN, GPIO_MODE_INPUT);
    gpio_matrix_in(XBOX_SPI_MOSI_PIN, HSPID_IN_IDX, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG_IRAM[XBOX_SPI_MOSI_PIN], PIN_FUNC_GPIO);

    gpio_set_level_iram(XBOX_SPI_MISO_PIN, 1);
    gpio_set_direction_iram(XBOX_SPI_MISO_PIN, GPIO_MODE_OUTPUT);
    gpio_matrix_out(XBOX_SPI_MISO_PIN, HSPIQ_OUT_IDX, false, false);
    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG_IRAM[XBOX_SPI_MISO_PIN], PIN_FUNC_GPIO);

    periph_ll_enable_clk_clear_rst(PERIPH_HSPI_MODULE);
    spi_init(&cfg);
    cs_state = GPIO.in & cs_mask;
    if (!(cs_state & XBOX_SPI_CS_P2_MASK)) {
        active_port = 1;
    }
    else {
        active_port = 0;
    }
    xbox_spi_select_port(active_port);
    if (!monitor_started) {
        monitor_started = true;
        xTaskCreatePinnedToCore(xbox_spi_monitor_task, "xbox_spi_cs", 2048, NULL, 12, NULL, 1);
    }
}
