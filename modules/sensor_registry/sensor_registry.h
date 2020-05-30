#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <modules/pubsub/pubsub.h>
#include "sensor_types.h"

struct sensor_measurement_topic_s {
    enum sensor_measurement_type_t meas_type;
    struct pubsub_topic_s topic;
    struct sensor_measurement_topic_s* next;
};

struct sensor_s {
    uint8_t idx;
    struct sensor_measurement_topic_s* topic_list_head;
    struct sensor_s* next;
};

// Get measurement topic for a given sensor and measurement type - creates if necessary
struct pubsub_topic_s* sensor_registry_get_measurement_topic(enum sensor_type_t sensor_type, uint8_t idx, enum sensor_measurement_type_t meas_type);

void sensor_registry_publish_sensor_message(enum sensor_type_t sensor_type, uint8_t idx, enum sensor_measurement_type_t meas_type, systime_t meas_time, pubsub_message_writer_func_ptr writer_func, void* ctx);

size_t sensor_registry_get_meas_type_size(enum sensor_measurement_type_t meas_type);
const char* sensor_registry_get_sensor_type_name_str(enum sensor_type_t sensor_type);
