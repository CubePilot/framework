#pragma once

#include <modules/spi_device/spi_device.h>
#include "profiLED_gen.h"
#include <stdbool.h>

struct profiLED_instance_s {
    bool use_spi;
    struct spi_device_s dev;
    uint32_t sclk_line;
    uint32_t mosi_line;
    uint32_t num_leds;
    struct profiLED_gen_color_s* colors;
};

void profiLED_init(struct profiLED_instance_s* instance, uint8_t spi_bus_idx, uint32_t spi_sel_line, bool sel_active_high, uint32_t num_leds);
void profiLED_init_nospi(struct profiLED_instance_s* instance, uint32_t sclk_line, uint32_t mosi_line, uint32_t num_leds);
void profiLED_update(struct profiLED_instance_s* instance);
void profiLED_set_color_rgb(struct profiLED_instance_s* instance, uint32_t idx, uint8_t r, uint8_t g, uint8_t b);
void profiLED_set_color_hex(struct profiLED_instance_s* instance, uint32_t idx, uint32_t color);
