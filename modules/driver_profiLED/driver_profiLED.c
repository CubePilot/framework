#include <ch.h>
#include <string.h>
#include "driver_profiLED.h"
#include <common/helpers.h>
#include <app_config.h>
#include <hal.h>

#define PROFILED_SEND_BUF_SIZE 32

void profiLED_output_gpio(uint32_t sclk_line, uint32_t mosi_line, uint32_t num_leds, profiLED_color_func color_func, void* ctx) {
    const uint32_t min_bits = num_leds*25+50;
    const uint8_t num_leading_zeros = 8-min_bits%8 + 50;
    const uint32_t output_stream_byte_length = (min_bits+7)/8;

    uint32_t curr_led_idx = 0;

    union {
        struct profiLED_color_s struct_val;
        uint8_t bytes_val[3];
    } curr_led_color;

    curr_led_color.struct_val = color_func(curr_led_idx, ctx);

    for (uint32_t i=0; i<output_stream_byte_length; i++) {
        uint8_t byte = 0;
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint32_t out_bit_idx = i*8+bit;
            uint8_t bit_val;
            if (out_bit_idx < num_leading_zeros) {
                bit_val = 0;
            } else if ((out_bit_idx-num_leading_zeros) % 25 == 0) {
                bit_val = 1;
            } else {
                uint32_t in_bit_idx = out_bit_idx - num_leading_zeros - (out_bit_idx - num_leading_zeros)/25;
                uint32_t in_led_idx = in_bit_idx/24;

                if (curr_led_idx != in_led_idx) {
                    curr_led_idx = in_led_idx;
                    curr_led_color.struct_val = color_func(curr_led_idx, ctx);
                }

                bit_val = (curr_led_color.bytes_val[(in_bit_idx%24)/8] >> (8-in_bit_idx%8)) & 1;
            }

            palClearLine(sclk_line);
            // TODO insert delay
            palWriteLine(mosi_line, bit_val);
            // TODO insert delay
            palSetLine(sclk_line);
            // TODO insert delay
        }
    }
}

#ifdef MODULE_SPI_DEVICE_ENABLED
bool profiLED_spi_dev_init(struct spi_device_s* dev, uint8_t spi_bus_idx, uint32_t spi_sel_line, bool sel_active_high, uint32_t speed_hz) {
    return spi_device_init(dev, spi_bus_idx, spi_sel_line, speed_hz, 8, (sel_active_high?SPI_DEVICE_FLAG_SELPOL:0));
}

void profiLED_output_spi(struct spi_device_s* dev, uint32_t num_leds, profiLED_color_func color_func, void* ctx) {
    const uint32_t min_bits = num_leds*25+50;
    const uint8_t num_leading_zeros = 8-min_bits%8 + 50;
    const uint32_t output_stream_byte_length = (min_bits+7)/8;

    uint32_t curr_led_idx = 0;

    union {
        struct profiLED_color_s struct_val;
        uint8_t bytes_val[3];
    } curr_led_color;

    curr_led_color.struct_val = color_func(curr_led_idx, ctx);

    uint8_t send_buf[PROFILED_SEND_BUF_SIZE];
    uint8_t send_buf_len = 0;

    for (uint32_t i=0; i<output_stream_byte_length; i++) {
        uint8_t byte = 0;
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint32_t out_bit_idx = i*8+bit;
            uint8_t bit_val;
            if (out_bit_idx < num_leading_zeros) {
                bit_val = 0;
            } else if ((out_bit_idx-num_leading_zeros) % 25 == 0) {
                bit_val = 1;
            } else {
                uint32_t in_bit_idx = out_bit_idx - num_leading_zeros - (out_bit_idx - num_leading_zeros)/25;
                uint32_t in_led_idx = in_bit_idx/24;

                if (curr_led_idx != in_led_idx) {
                    curr_led_idx = in_led_idx;
                    curr_led_color.struct_val = color_func(curr_led_idx, ctx);
                }

                bit_val = (curr_led_color.bytes_val[(in_bit_idx%24)/8] >> (8-in_bit_idx%8)) & 1;
            }

            byte |= bit_val << (7-bit);
        }
        send_buf[send_buf_len++] = byte;
        if (send_buf_len >= PROFILED_SEND_BUF_SIZE) {
            spi_device_send(dev, send_buf_len, send_buf);
            send_buf_len = 0;
        }
    }
    if (send_buf_len > 0) {
        spi_device_send(dev, send_buf_len, send_buf);
        send_buf_len = 0;
    }
}
#endif

struct profiLED_color_s profiLED_make_color_from_hex(uint32_t color) {
    struct profiLED_color_s ret;
    ret.r = (uint8_t)(color>>16);
    ret.g = (uint8_t)(color>>8);
    ret.b = (uint8_t)color;
    return ret;
}

struct profiLED_color_s profiLED_make_color_from_rgb_float(float r, float g, float b) {
    struct profiLED_color_s ret;
    ret.r = r < 0 ? 0 : (r > 1 ? 255 : r*255);
    ret.g = g < 0 ? 0 : (g > 1 ? 255 : g*255);
    ret.b = b < 0 ? 0 : (b > 1 ? 255 : b*255);
    return ret;
}
