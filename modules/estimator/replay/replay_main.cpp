#include <stdio.h>
#include <stdint.h>
#include <iostream>

#include "InertialNavigationEstimator.h"
using namespace Eigen;

#define MAX_LOG_MSG_SIZE 2048

enum sensor_measurement_type_t {
    SENSOR_MEASUREMENT_TYPE_IMU_DELTA,
    SENSOR_MEASUREMENT_TYPE_GNSS_FIX2,
    SENSOR_MEASUREMENT_TYPE_GNSS_AUX,
    SENSOR_MEASUREMENT_TYPE_GNSS_ECEF,
    SENSOR_MEASUREMENT_TYPE_MAG_FIELD
};

struct sensor_measurement_s {
    uint32_t type;
    uint32_t meas_time;
    uint8_t msg_body[];
};

struct sensor_measurement_imu_delta_s {
    float delta_time;
    float delta_angle[3];
    float delta_velocity[3];
};

struct sensor_measurement_gnss_ecef_s {
    double pos_ecef[3];
    float vel_ecef[3];
    float hAcc;
    float vAcc;
    float sAcc;
};

#define IMU_DELAY 10
struct sensor_measurement_imu_delta_s imu_delay_buffer[IMU_DELAY];

const float accel_sigma = 0.004;
const float gyro_sigma = 7e-4;
const float gbias_pnoise = 3e-3;
const float abias_pnoise_xy = 1e-1;
const float abias_pnoise_z = 1e-1;
const float gscale_pnoise = 1e-4;


const InertialNavigationEstimator::Params params = {accel_sigma,gyro_sigma,gbias_pnoise,abias_pnoise_xy,abias_pnoise_z,gscale_pnoise};

static InertialNavigationEstimator ekf(params);

float accel[3];

static bool have_imu_data;
static bool initialized;

static uint16_t crc16_ccitt(const void *buf, size_t len, uint16_t crc) {
    for (size_t i = 0; i < len; i++) {
        crc = crc ^ (((uint8_t*)buf)[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

void process_valid_msg(uint32_t msg_size, void* msg) {
    if (msg_size < sizeof(struct sensor_measurement_s)) {
        printf("invalid data: msg size < minimum message size\n");
        return;
    }

    struct sensor_measurement_s* sensor_meas = reinterpret_cast<struct sensor_measurement_s*>(msg);
    sensor_meas->type &= 0xff;
//     printf("message %u %u %u\n", (unsigned)sensor_meas->type, (unsigned)sensor_meas->meas_time, (unsigned)msg_size);

    static float time_sum_imu;
    static float dt_sum_imu;
    static uint32_t last_time_imu;
    static uint32_t last_time_gnss;

    if (sensor_meas->type == SENSOR_MEASUREMENT_TYPE_IMU_DELTA) {
        struct sensor_measurement_imu_delta_s* imu_delta_meas = reinterpret_cast<struct sensor_measurement_imu_delta_s*>(sensor_meas->msg_body);

        time_sum_imu += (sensor_meas->meas_time-last_time_imu)*1e-6;
        dt_sum_imu += imu_delta_meas->delta_time;
//         printf("imu message %u %f\n", sensor_meas->meas_time-last_time_imu, imu_delta_meas->delta_time);
        last_time_imu = sensor_meas->meas_time;

        accel[0] = imu_delta_meas->delta_velocity[0]/imu_delta_meas->delta_time;
        accel[1] = imu_delta_meas->delta_velocity[1]/imu_delta_meas->delta_time;
        accel[2] = imu_delta_meas->delta_velocity[2]/imu_delta_meas->delta_time;
        have_imu_data = true;

        memmove(imu_delay_buffer, &imu_delay_buffer[1], sizeof(imu_delay_buffer)-sizeof(imu_delay_buffer[0]));
        imu_delay_buffer[IMU_DELAY-1] = *imu_delta_meas;

        imu_delta_meas = &imu_delay_buffer[0];

        if (initialized) {
            ekf.predict(imu_delta_meas->delta_time, Vector3f(imu_delta_meas->delta_angle[0], imu_delta_meas->delta_angle[1], imu_delta_meas->delta_angle[2]), Vector3f(imu_delta_meas->delta_velocity[0], imu_delta_meas->delta_velocity[1], imu_delta_meas->delta_velocity[2]));
        }
    } else if (sensor_meas->type == SENSOR_MEASUREMENT_TYPE_GNSS_ECEF) {
//         printf("gnss message %u %f %f\n", sensor_meas->meas_time-last_time_gnss, time_sum_imu, dt_sum_imu);
        time_sum_imu = 0;
        dt_sum_imu = 0;
        last_time_gnss = sensor_meas->meas_time;
        struct sensor_measurement_gnss_ecef_s* gnss_ecef_meas = reinterpret_cast<struct sensor_measurement_gnss_ecef_s*>(sensor_meas->msg_body);

        if (!initialized && have_imu_data) {
            printf("initializing\n");
            ekf.initialize(Vector3f(accel[0], accel[1], accel[2]), Vector3d(gnss_ecef_meas->pos_ecef[0], gnss_ecef_meas->pos_ecef[1], gnss_ecef_meas->pos_ecef[2]), Vector3f(gnss_ecef_meas->vel_ecef[0], gnss_ecef_meas->vel_ecef[1], gnss_ecef_meas->vel_ecef[2]));

            initialized = true;
        } else if (initialized) {
            InertialNavigationEstimator::Matrix6f R = InertialNavigationEstimator::Matrix6f::Zero();

            R(0,0) = R(1,1) = R(2,2) = SQ(gnss_ecef_meas->hAcc)+SQ(gnss_ecef_meas->vAcc);

            R(3,3) = R(4,4) = R(5,5) = pow(0.3+gnss_ecef_meas->sAcc, 2);
//             cout << R << endl;
//
            for (size_t i=0; i<ekf.get_mixand_count(); i++) {
                float heading, heading_sigma;
                ekf.get_mixand(i)->get_heading_and_heading_sigma(heading, heading_sigma);
                cout << ekf.get_mixand(i)->initial_heading*180/M_PI << " " << heading*180/M_PI << " " << heading_sigma*180/M_PI << endl << ekf.get_mixand(i)->x.transpose() << endl;
            }
            cout << endl << endl;
//
//             cout << "FUSION" << endl;
            ekf.fuse_pos_vel_ecef(Map<const Vector3d>(gnss_ecef_meas->pos_ecef), Map<const Vector3f>(gnss_ecef_meas->vel_ecef), R, 0.01);
            ekf.reduce();
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        printf("provide log file name as first argument\n");
        return 1;
    }

    FILE *fp;
    fp = fopen(argv[1], "r");

    fseek(fp, 0L, SEEK_END);
    const long file_size = ftell(fp);
    fseek(fp, 0L, SEEK_SET);

    uint32_t msg_size;
    uint16_t crc;
    uint8_t msg[MAX_LOG_MSG_SIZE];

    while (file_size-ftell(fp) > 6) {
//         printf("%u\n", (unsigned)sizeof(msg_size));
        fread(&msg_size, sizeof(msg_size), 1, fp);
//         printf("msg_size=%u\n", (unsigned)msg_size);
        if (msg_size > MAX_LOG_MSG_SIZE) {
            printf("invalid data: msg size > maximum message size\n");
            return 1;
        }

        if (msg_size+2 > file_size-ftell(fp)) {
            printf("invalid data: msg size > remaining bytes\n");
            return 1;
        }

        fread(&crc, sizeof(crc), 1, fp);
        fread(&msg, msg_size, 1, fp);

        uint16_t crc_computed = crc16_ccitt(msg, msg_size, 0);
        if (crc_computed != crc) {
            printf("invalid data: crc mismatch computed=%04X provided=%04X\n", crc_computed, crc);
            return 1;
        }

        process_valid_msg(msg_size, msg);
    }
}
