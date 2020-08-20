extern "C" {
#include <modules/estimator_input/estimator_input.h>
#include <modules/worker_thread/worker_thread.h>
#include <modules/datalogger/datalogger.h>
#include <common/ctor.h>
#include <ch.h>
#include <modules/uavcan_debug/uavcan_debug.h>
}

#include "InertialNavigationEstimator.h"
using namespace Eigen;

#ifndef ESTIMATOR_WORKER_THREAD
#error Please define ESTIMATOR_WORKER_THREAD in framework_conf.h.
#endif

#define WT ESTIMATOR_WORKER_THREAD
WORKER_THREAD_DECLARE_EXTERN(WT)

const float accel_sigma = 1.0;
const float gyro_sigma = 2e-1;
const float gbias_pnoise = 1e-3;
const float abias_pnoise_xy = 3e-3;
const float abias_pnoise_z = 3e-3;
const float gscale_pnoise = 1e-5;

const InertialNavigationEstimator::Params params = {accel_sigma,gyro_sigma,gbias_pnoise,abias_pnoise_xy,abias_pnoise_z,gscale_pnoise};

static struct estimator_s {
    struct estimator_input_s estimator_input;
    struct logfile_s input_log;
    memory_heap_t heap;
    InertialNavigationEstimator estimator { params };
    bool initialized;
    Vector3f accel;
    bool have_imu_data;
} instance;

static PUBSUB_TOPIC_GROUP_CREATE(estimator_log_topic_group, 16384)
static struct pubsub_topic_s log_topic;

static void meas_cb(struct estimator_input_port_s* port, struct sensor_measurement_s* meas, size_t measurement_size) {
    (void)port;
    // NOTE: this is a vulnerability, since publishers always succeed, the logger thread could block us
    // TODO a non-blocking publish
    pubsub_publish_message(&log_topic, measurement_size, pubsub_copy_writer_func, meas);

    if (meas->type == SENSOR_MEASUREMENT_TYPE_IMU_DELTA) {
        struct sensor_measurement_imu_delta_s* imu_delta_meas = reinterpret_cast<struct sensor_measurement_imu_delta_s*>(meas->msg_body);
        instance.accel(0) = imu_delta_meas->delta_velocity[0]/imu_delta_meas->delta_time;
        instance.accel(1) = imu_delta_meas->delta_velocity[1]/imu_delta_meas->delta_time;
        instance.accel(2) = imu_delta_meas->delta_velocity[2]/imu_delta_meas->delta_time;
        instance.have_imu_data = true;

        if (instance.initialized) {
             instance.estimator.predict(imu_delta_meas->delta_time, Vector3f(imu_delta_meas->delta_angle[0], imu_delta_meas->delta_angle[1], imu_delta_meas->delta_angle[2]), Vector3f(imu_delta_meas->delta_velocity[0], imu_delta_meas->delta_velocity[1], imu_delta_meas->delta_velocity[2]));
        }
    } else if (meas->type == SENSOR_MEASUREMENT_TYPE_GNSS_ECEF) {
        struct sensor_measurement_gnss_ecef_s* gnss_ecef_meas = reinterpret_cast<struct sensor_measurement_gnss_ecef_s*>(meas->msg_body);

        if (!instance.initialized && instance.have_imu_data) {
            instance.estimator.initialize(instance.accel, Vector3d(gnss_ecef_meas->pos_ecef[0], gnss_ecef_meas->pos_ecef[1], gnss_ecef_meas->pos_ecef[2]), Vector3f(gnss_ecef_meas->vel_ecef[0], gnss_ecef_meas->vel_ecef[1], gnss_ecef_meas->vel_ecef[2]));
            instance.initialized = true;
        } else if (instance.initialized) {
            for (uint8_t i=0; i<2; i++) {
                instance.estimator.fuse_pos_element_ecef(gnss_ecef_meas->pos_ecef[i], pow(gnss_ecef_meas->hAcc, 2), i);

            }
            instance.estimator.fuse_pos_element_ecef(gnss_ecef_meas->pos_ecef[2], pow(gnss_ecef_meas->vAcc, 2), 2);

            for (uint8_t i=0; i<3; i++) {
                instance.estimator.fuse_vel_element_ecef(gnss_ecef_meas->vel_ecef[i], pow(gnss_ecef_meas->sAcc*2, 2), i);
            }
        }
    }
}

RUN_AFTER(INIT_END) {
    void* heap_mem = chCoreAlloc(65536);
    chHeapObjectInit(&instance.heap, heap_mem, 65536);
    pubsub_init_topic(&log_topic, &estimator_log_topic_group);
    logger_start_log(&instance.input_log, "/ESTIMATOR_IN");
    logger_subscribe(&instance.input_log, &log_topic);

    estimator_input_init(&instance.estimator_input, "ESTIMATOR", &instance.heap, 50000, &WT, meas_cb);
    estimator_input_add_port(&instance.estimator_input, SENSOR_TYPE_IMU, SENSOR_MEASUREMENT_TYPE_IMU_DELTA, 255, 0, NULL);
    estimator_input_add_port(&instance.estimator_input, SENSOR_TYPE_GNSS, SENSOR_MEASUREMENT_TYPE_GNSS_ECEF, 254, 50, NULL);
}
