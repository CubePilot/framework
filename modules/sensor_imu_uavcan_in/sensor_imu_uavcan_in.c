#include <modules/uavcan/uavcan.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/sensor_registry/sensor_registry.h>
#include <modules/uavcan_node_registry/uavcan_node_registry.h>
#include <modules/uavcan_timesync/uavcan_timesync.h>
#include <modules/param/param.h>
#include <stdlib.h>
#include <string.h>
#include <uavcan.equipment.ahrs.RawIMU.h>

#ifndef SENSOR_UAVCAN_IN_WORKER_THREAD
#error Please define SENSOR_UAVCAN_IN_WORKER_THREAD in framework_conf.h.
#endif

#define WT SENSOR_UAVCAN_IN_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)


struct sensor_uavcan_input_filter_port_s {
    uint8_t sensor_idx;
    char src_hwid_param_name[92];
    char src_hwid_str[48];
    struct param_descriptor_string_s src_hwid_str_param_desc;
};

struct sensor_uavcan_input_filter_s {
    enum sensor_type_t sensor_type;
    struct sensor_uavcan_input_filter_port_s ports[3];
};


static void sensor_uavcan_input_filter_init(struct sensor_uavcan_input_filter_s* instance, enum sensor_type_t sensor_type) {
    memset(instance, 0, sizeof(*instance));
    instance->sensor_type = sensor_type;

    // initialize ports
    for (uint8_t i=0; i<3; i++) {
        struct sensor_uavcan_input_filter_port_s* port = &instance->ports[i];
        port->sensor_idx = 100+i;
        char port_number_str[4];
        itoa(port->sensor_idx, port_number_str, 10);
        char* param_name = port->src_hwid_param_name;
        #define PARAM_NAME_APPEND(dest, str) strncat((dest), (str), 91-strnlen(dest, 92))
        PARAM_NAME_APPEND(param_name, "OFFBOARD_");
        PARAM_NAME_APPEND(param_name, sensor_registry_get_sensor_type_name_str(instance->sensor_type));
        PARAM_NAME_APPEND(param_name, "_");
        PARAM_NAME_APPEND(param_name, port_number_str);
        PARAM_NAME_APPEND(param_name, "_HWID");
        port->src_hwid_str_param_desc = (struct param_descriptor_string_s){{PARAM_TYPE_STRING, 0, param_name, port->src_hwid_str}, 47, ""};
        param_register(&port->src_hwid_str_param_desc.header);
    }
}

uint8_t sensor_uavcan_input_filter_match_sensor_idx(struct sensor_uavcan_input_filter_s* instance, const struct uavcan_deserialized_message_s* msg_wrapper) {
    for (uint8_t i=0; i<3; i++) {
        struct sensor_uavcan_input_filter_port_s* port = &instance->ports[i];
        if (uavcan_node_registry_match_uuid_string(msg_wrapper->uavcan_idx, msg_wrapper->source_node_id, port->src_hwid_str)) {
            return port->sensor_idx;
        }
    }
    return 0;
}

struct sensor_uavcan_input_filter_s raw_imu_input_filter;
struct worker_thread_listener_task_s raw_imu_listener_task;
static void raw_imu_listener_task_func(size_t msg_size, const void* buf, void* ctx);

RUN_AFTER(UAVCAN_INIT) {
    sensor_uavcan_input_filter_init(&raw_imu_input_filter, SENSOR_TYPE_IMU);
    struct pubsub_topic_s* topic = uavcan_get_message_topic(0, &uavcan_equipment_ahrs_RawIMU_descriptor);
    worker_thread_add_listener_task(&WT, &raw_imu_listener_task, topic, raw_imu_listener_task_func, &raw_imu_input_filter);
}

static void imu_delta_writer_func(size_t msg_size, void* buf, void* ctx) {
    (void)msg_size;
    const struct uavcan_equipment_ahrs_RawIMU_s* msg = ctx;
    struct sensor_measurement_imu_delta_s* imu_delta = buf;

    imu_delta->delta_time = msg->integration_interval;
    for (uint8_t i=0; i<3; i++) {
        imu_delta->delta_angle[i] = msg->rate_gyro_integral[i];
        imu_delta->delta_velocity[i] = msg->accelerometer_integral[i];
    }
}

static void raw_imu_listener_task_func(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    const struct uavcan_deserialized_message_s* msg_wrapper = buf;
    struct sensor_uavcan_input_filter_s* input_filter = ctx;

    uint8_t sensor_idx = sensor_uavcan_input_filter_match_sensor_idx(input_filter, msg_wrapper);

    if (sensor_idx == 0) {
        return;
    }

    const struct uavcan_equipment_ahrs_RawIMU_s* msg = (const struct uavcan_equipment_ahrs_RawIMU_s*)msg_wrapper->msg;

    systime_t meas_time;
    if (!uavcan_timesync_get_systime_at_bus_time(msg->timestamp.usec, &meas_time)) {
        meas_time = msg_wrapper->rx_timestamp;
    }

    sensor_registry_publish_sensor_message(SENSOR_TYPE_IMU, sensor_idx, SENSOR_MEASUREMENT_TYPE_IMU_DELTA, meas_time, imu_delta_writer_func, msg);
}
