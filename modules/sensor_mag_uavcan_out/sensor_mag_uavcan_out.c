#include <modules/uavcan/uavcan.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/sensor_registry/sensor_registry.h>

#include <uavcan.equipment.ahrs.MagneticFieldStrength.h>

#ifndef SENSOR_UAVCAN_OUT_WORKER_THREAD
#error Please define SENSOR_UAVCAN_OUT_WORKER_THREAD in framework_conf.h.
#endif

#define WT SENSOR_UAVCAN_OUT_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct worker_thread_listener_task_s mag_listener_task;
static void mag_listener_task_func(size_t msg_size, const void* buf, void* ctx);

RUN_AFTER(UAVCAN_INIT) {
    struct pubsub_topic_s* topic = sensor_registry_get_measurement_topic(SENSOR_TYPE_MAG, 0, SENSOR_MEASUREMENT_TYPE_MAG_FIELD);
    worker_thread_add_listener_task(&WT, &mag_listener_task, topic, mag_listener_task_func, NULL);
}

static void mag_listener_task_func(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    const struct sensor_measurement_s* meas = buf;
    const struct sensor_measurement_mag_field_s* msg_body = (const struct sensor_measurement_mag_field_s*)meas->msg_body;
    uavcan_broadcast(0, &uavcan_equipment_ahrs_MagneticFieldStrength_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &msg_body->mag);
}
