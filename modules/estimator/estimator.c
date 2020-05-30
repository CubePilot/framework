#include <modules/estimator_input/estimator_input.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/datalogger/datalogger.h>
#include <common/ctor.h>
#include <ch.h>
#include <modules/uavcan_debug/uavcan_debug.h>

#ifndef ESTIMATOR_WORKER_THREAD
#error Please define ESTIMATOR_WORKER_THREAD in framework_conf.h.
#endif

#define WT ESTIMATOR_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

static struct estimator_s {
    struct estimator_input_s estimator_input;
    struct logfile_s input_log;
    memory_heap_t heap;
} estimator;

static PUBSUB_TOPIC_GROUP_CREATE(estimator_log_topic_group, 16384)
static struct pubsub_topic_s log_topic;

static void meas_cb(struct estimator_input_port_s* port, struct sensor_measurement_s* measurement, size_t measurement_size) {
    (void)port;
    pubsub_publish_message(&log_topic, measurement_size, pubsub_copy_writer_func, measurement);
}

RUN_AFTER(INIT_END) {
    void* heap_mem = chCoreAlloc(65536);
    chHeapObjectInit(&estimator.heap, heap_mem, 65536);
    pubsub_init_topic(&log_topic, &estimator_log_topic_group);
    logger_start_log(&estimator.input_log, "/ESTIMATOR_IN");
    logger_subscribe(&estimator.input_log, &log_topic);

    estimator_input_init(&estimator.estimator_input, "ESTIMATOR", &estimator.heap, 200000, &WT, meas_cb);
    estimator_input_add_port(&estimator.estimator_input, SENSOR_TYPE_IMU, SENSOR_MEASUREMENT_TYPE_IMU_DELTA, 255, 0, NULL);
    estimator_input_add_port(&estimator.estimator_input, SENSOR_TYPE_GNSS, SENSOR_MEASUREMENT_TYPE_GNSS_ECEF, 254, 50, NULL);
}
