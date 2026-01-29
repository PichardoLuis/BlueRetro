/*
 * Copyright (c) 2025, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include "soc/io_mux_reg.h"
#include "esp_private/periph_ctrl.h"
#include <soc/spi_periph.h>
#include <esp32/rom/ets_sys.h>
#include <esp32/rom/gpio.h>
#include "hal/clk_gate_ll.h"
#include "driver/gpio.h"
#include "system/gpio.h"
#include "adapter/adapter.h"
#include "adapter/config.h"
#include "wired_bare.h"
#include "xbox_spi.h"

#define XBOX_SPI_PORT_MAX 2
#define XBOX_SPI_PAYLOAD_SIZE 32
#define XBOX_SPI_PACKET_SIZE (XBOX_SPI_PAYLOAD_SIZE + 4)

#define XBOX_SPI_MAGIC 0x58

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

    packet[0] = XBOX_SPI_MAGIC;
    packet[1] = port;
    packet[2] = wired_adapter.data[port].frame_cnt;
    packet[3] = config.out_cfg[port].dev_mode;

    for (uint32_t i = 0; i < XBOX_SPI_PAYLOAD_SIZE; i++) {
        packet[4 + i] = mask[i] ? mask[i] : out[i];
    }

    ++wired_adapter.data[port].frame_cnt;
    xbox_spi_write_buffer(packet, XBOX_SPI_PACKET_SIZE);
}

static inline void xbox_spi_select_port(uint8_t port) {
    if (port >= XBOX_SPI_PORT_MAX) {
        return;
    }

    active_port = port;
    gpio_matrix_in((port == 0) ? XBOX_SPI_CS_P1_PIN : XBOX_SPI_CS_P2_PIN, HSPICS0_IN_IDX, false);
    xbox_spi_build_packet(port);

    SPI2.slave.sync_reset = 1;
    SPI2.slave.trans_done = 0;
    SPI2.cmd.usr = 1;
}

static void xbox_spi_monitor_cs(void) {
    const uint32_t cs_mask = XBOX_SPI_CS_P1_MASK | XBOX_SPI_CS_P2_MASK;
    uint32_t prev_state = GPIO.in & cs_mask;

    while (1) {
        uint32_t cur_state = GPIO.in & cs_mask;
        uint32_t change = cur_state ^ prev_state;

        if (change) {
            if (!(cur_state & XBOX_SPI_CS_P1_MASK)) {
                xbox_spi_select_port(0);
            }
            else if (!(cur_state & XBOX_SPI_CS_P2_MASK)) {
                xbox_spi_select_port(1);
            }
            prev_state = cur_state;
        }
    }
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
    xbox_spi_monitor_cs();
}
