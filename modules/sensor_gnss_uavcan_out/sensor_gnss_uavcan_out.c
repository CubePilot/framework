#include <modules/uavcan/uavcan.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/sensor_registry/sensor_registry.h>

#include <uavcan.equipment.gnss.Fix2.h>
#include <uavcan.equipment.gnss.Auxiliary.h>

#ifndef SENSOR_UAVCAN_OUT_WORKER_THREAD
#error Please define SENSOR_UAVCAN_OUT_WORKER_THREAD in framework_conf.h.
#endif

#define WT SENSOR_UAVCAN_OUT_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct worker_thread_listener_task_s fix2_listener_task;
static void fix2_listener_task_func(size_t msg_size, const void* buf, void* ctx);

static struct worker_thread_listener_task_s aux_listener_task;
static void aux_listener_task_func(size_t msg_size, const void* buf, void* ctx);

RUN_AFTER(UAVCAN_INIT) {
    struct pubsub_topic_s* topic = sensor_registry_get_measurement_topic(SENSOR_TYPE_GNSS, 0, SENSOR_MEASUREMENT_TYPE_GNSS_FIX2);
    worker_thread_add_listener_task(&WT, &fix2_listener_task, topic, fix2_listener_task_func, NULL);

    topic = sensor_registry_get_measurement_topic(SENSOR_TYPE_GNSS, 0, SENSOR_MEASUREMENT_TYPE_GNSS_AUX);
    worker_thread_add_listener_task(&WT, &aux_listener_task, topic, aux_listener_task_func, NULL);
}

static void fix2_listener_task_func(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    const struct sensor_measurement_s* meas = buf;
    const struct sensor_measurement_gnss_fix2_s* msg_body = (const struct sensor_measurement_gnss_fix2_s*)meas->msg_body;
    uavcan_broadcast(0, &uavcan_equipment_gnss_Fix2_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &msg_body->fix2);
}

static void aux_listener_task_func(size_t msg_size, const void* buf, void* ctx) {
    (void)msg_size;
    (void)ctx;
    const struct sensor_measurement_s* meas = buf;
    const struct sensor_measurement_gnss_aux_s* msg_body = (const struct sensor_measurement_gnss_aux_s*)meas->msg_body;
    uavcan_broadcast(0, &uavcan_equipment_gnss_Auxiliary_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &msg_body->aux);
}
