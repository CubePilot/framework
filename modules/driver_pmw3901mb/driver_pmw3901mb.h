#pragma once

#include <modules/spi_device/spi_device.h>
#include "pmw3901mb_internal.h"

enum pmw3901mb_type_t {
    PMW3901MB_TYPE_PMW3901MB,
    PMW3901MB_TYPE_PAW3903,
};

struct pmw3901mb_instance_s {
    struct spi_device_s spi_dev;
    enum pmw3901mb_type_t pmw3901mb_type;
    uint16_t frame_read_idx;
    bool in_frame_capture_mode;
};

struct pmw3901mb_motion_report_s {
    uint8_t motion;
    uint8_t observation;
    int16_t delta_x;
    int16_t delta_y;
    uint8_t squal;
    uint8_t rawdata_sum;
    uint8_t max_rawdata;
    uint8_t min_rawdata;
    uint8_t shutter_upper;
    uint8_t shutter_lower;
};

bool pmw3901mb_init(struct pmw3901mb_instance_s* instance, uint8_t spi_idx, uint32_t select_line);
bool pmw3901mb_burst_read(struct pmw3901mb_instance_s* instance, struct pmw3901mb_motion_report_s* ret);
bool pmw3901mb_motion_detected(struct pmw3901mb_instance_s* instance);
int16_t pmw3901mb_read_dx(struct pmw3901mb_instance_s* instance);
int16_t pmw3901mb_read_dy(struct pmw3901mb_instance_s* instance);
void pmw3901mb_write(struct pmw3901mb_instance_s* instance, uint8_t reg, uint8_t value);
uint8_t pmw3901mb_read(struct pmw3901mb_instance_s* instance, uint8_t reg);

void pmw3901mb_frame_capture_start(struct pmw3901mb_instance_s* instance);
size_t pmw3901mb_frame_capture_get_frame_chunk(struct pmw3901mb_instance_s* instance, size_t len, uint8_t* chunk);
bool pmw3901mb_in_frame_capture_mode(struct pmw3901mb_instance_s* instance);
