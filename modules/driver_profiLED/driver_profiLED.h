#pragma once

#ifdef MODULE_SPI_DEVICE_ENABLED
#include <modules/spi_device/spi_device.h>
#endif

#include <stdint.h>

struct profiLED_color_s {
    uint8_t b;
    uint8_t r;
    uint8_t g;
};

typedef struct profiLED_color_s (*profiLED_color_func)(uint32_t led_idx, void* ctx);
void profiLED_output(uint32_t sclk_line, uint32_t mosi_line, uint32_t num_leds, profiLED_color_func color_func, void* ctx);

#ifdef MODULE_SPI_DEVICE_ENABLED
bool profiLED_spi_dev_init(struct spi_device_s* dev, uint8_t spi_bus_idx, uint32_t spi_sel_line, bool sel_active_high, uint32_t speed_hz);
void profiLED_output_spi(struct spi_device_s* dev, uint32_t num_leds, profiLED_color_func color_func, void* ctx);
#endif

struct profiLED_color_s profiLED_make_color_from_hex(uint32_t hex_val);
struct profiLED_color_s profiLED_make_color_from_rgb_float(float r, float g, float b);
