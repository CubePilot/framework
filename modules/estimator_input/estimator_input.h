#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <modules/sensor_registry/sensor_registry.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/param/param.h>
#include <ch.h>

struct estimator_input_s;

struct delay_buffer_element_s {
    struct delay_buffer_element_s* next;
    struct delay_buffer_element_s* next_in_port;
    struct estimator_input_port_s* port;
    struct sensor_measurement_s meas;
};

struct estimator_input_port_s {
    enum sensor_type_t sensor_type;
    enum sensor_measurement_type_t measurement_type;
    uint8_t priority;
    float max_rate_hz;
    bool listener_task_initialized;
    uint8_t listening_sensor_idx;
    uint8_t sensor_idx;
    uint32_t drop_too_old;
    uint32_t drop_alloc_fail;
    uint32_t drop_higher_prio;
    char sensor_idx_param_name[92];
    uint32_t last_timestamp_us;
    void* ctx;
    struct param_descriptor_uint8_s sensor_idx_param_desc;
    struct worker_thread_listener_task_s listener_task;
    struct delay_buffer_element_s* oldest_element;
    struct estimator_input_s* instance;
    struct estimator_input_port_s* next;
};

typedef void (*estimator_measurement_cb_t)(struct estimator_input_port_s* port, struct sensor_measurement_s* measurement, size_t measurement_size);

struct estimator_input_s {
    struct estimator_input_port_s* input_ports_head;
    uint32_t estimator_delay_us;
    memory_heap_t* heap;
    const char* estimator_name;
    systime_t last_processed_msg_time;
    estimator_measurement_cb_t cb;
    struct delay_buffer_element_s* oldest_element;
    struct worker_thread_s* worker_thread;
    struct worker_thread_timer_task_s process_measurements_task;
    struct worker_thread_timer_task_s periodic_task;
};

void estimator_input_init(struct estimator_input_s* instance, const char* estimator_name, memory_heap_t* heap, uint32_t estimator_delay, struct worker_thread_s* worker_thread, estimator_measurement_cb_t cb);

struct estimator_input_port_s* estimator_input_add_port(struct estimator_input_s* instance, enum sensor_type_t sensor_type, enum sensor_measurement_type_t measurement_type, uint8_t priority, float max_rate_hz, void* ctx);
