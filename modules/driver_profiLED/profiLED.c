#include <ch.h>
#include <hal.h>
#include <string.h>
#include "profiLED.h"
#include <common/helpers.h>
#include <app_config.h>

#ifndef MAX_NUM_PROFILEDS
#define MAX_NUM_PROFILEDS 64
#endif

#define PROFILED_OUTPUT_BUFFER_SIZE PROFILED_GEN_BUF_SIZE(MAX_NUM_PROFILEDS)
#define PROFILED_WORKER_THREAD_STACK_SIZE 256
#define PROFILED_WORKER_MAILBOX_DEPTH 2

static void _profiled_update(struct profiLED_instance_s* instance);

void profiLED_init(struct profiLED_instance_s* instance, uint8_t spi_bus_idx, uint32_t spi_sel_line, bool sel_active_high, uint32_t num_leds) {
    if (!instance) {
        return;
    }

    size_t colors_size = sizeof(struct profiLED_gen_color_s) * num_leds;

    instance->colors = chCoreAlloc(colors_size);
    if (!instance->colors) goto fail;
    memset(instance->colors, 0, colors_size);

    if (!spi_device_init(&(instance->dev), spi_bus_idx, spi_sel_line, 30000000, 8, (sel_active_high?SPI_DEVICE_FLAG_SELPOL:0))) {
        goto fail;
    }

    instance->num_leds = num_leds;
    instance->use_spi = true;

    profiLED_update(instance);
    return;

fail:
    instance->colors = NULL;
}

void profiLED_bitbang_send(struct profiLED_instance_s* instance, size_t buf_len, uint8_t* txbuf) {
    for (size_t i=0; i<buf_len; i++) {
        for (int8_t bit = 7; bit >= 0; bit++) {
            palSetLine(instance->sclk_line);
            palWriteLine(instance->mosi_line, (txbuf[i]>>bit)&1);
            palClearLine(instance->sclk_line);
        }
    }
}

void profiLED_update(struct profiLED_instance_s* instance) {
    if (!instance) {
        return;
    }

    uint8_t txbuf[PROFILED_OUTPUT_BUFFER_SIZE];
    uint32_t buf_len = profiLED_gen_write_buf(instance->num_leds, instance->colors, txbuf, sizeof(txbuf));
    if (buf_len == 0) {
        return;
    }

    if (use_spi) {
        spi_device_send(&(instance->dev), buf_len, txbuf);
    } else {
        profiLED_bitbang_send(instance, buf_len, txbuf);
    }
}

void profiLED_set_color_rgb(struct profiLED_instance_s* instance, uint32_t idx, uint8_t r, uint8_t g, uint8_t b) {
    if (!instance || !instance->colors) {
        return;
    }

    profiLED_gen_make_brg_color_rgb(r, g, b, &instance->colors[idx]);
}

void profiLED_set_color_hex(struct profiLED_instance_s* instance, uint32_t idx, uint32_t color) {
    profiLED_set_color_rgb(instance, idx, (uint8_t)(color>>16), (uint8_t)(color>>8), (uint8_t)color);
}
