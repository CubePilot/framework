#pragma once

#include <modules/pubsub/pubsub.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/spi_device/spi_device.h>

/**
 * Calibration PROM as reported by the device.
 */
struct __attribute__((packed)) prom_s {
    uint16_t factory_setup;
    uint16_t c1_pressure_sens;
    uint16_t c2_pressure_offset;
    uint16_t c3_temp_coeff_pres_sens;
    uint16_t c4_temp_coeff_pres_offset;
    uint16_t c5_reference_temp;
    uint16_t c6_temp_coeff_temp;
    uint16_t serial_and_crc;
};

union __attribute__((packed)) prom_u {
    uint16_t c[8];
    struct prom_s s;
};

struct ms5611_instance_s {
    struct spi_device_s spi_dev;
    struct pubsub_topic_s* topic;
    struct worker_thread_s* worker_thread;
    struct worker_thread_timer_task_s task;
    union prom_u prom;
    systime_t conversion_start_time;
    uint32_t D1;
    uint32_t D2;
    bool prom_read_ok;
    uint8_t process_step;
};

struct ms5611_sample_s {
    systime_t timestamp;
    float pressure_pa;
    float temperature_K;
};

void ms5611_init(struct ms5611_instance_s* instance, SPIDriver* spi_driver, uint32_t select_line, struct worker_thread_s* worker_thread, struct pubsub_topic_s* topic);
