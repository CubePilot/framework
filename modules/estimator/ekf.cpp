#include "ekf.h"

#include <chrono>
#include <iostream>
#include <limits>
#include <assert.h>
#include <iomanip>

using namespace std;
using namespace Eigen;

UWBEKF::clock_state_idx_t UWBEKF::add_clock_scale_state() {
    auto x_clocks = x.bottomRightCorner<EKF_NUM_CLOCK_STATES,1>();
    auto P_clocks = P.bottomRightCorner<EKF_NUM_CLOCK_STATES,EKF_NUM_CLOCK_STATES>();

    for (uint8_t i=0; i<EKF_NUM_CLOCK_STATES; i++) {
        if (_clock_state_map[i].type == STATE_TYPE_NONE) {
            _clock_state_map[i] = {STATE_TYPE_TSCA_ERR, -1};
            x_clocks(i) = 0;
            P_clocks.row(i).setZero();
            P_clocks.col(i).setZero();
            P_clocks(i,i) = SQ(5e-5);
            return (clock_state_idx_t)i;
        }
    }
    return STATE_IDX_NONE;
}

UWBEKF::clock_state_idx_t UWBEKF::add_clock_offset_state(clock_state_idx_t clock_scale_state, float init_sigma) {
    if (clock_scale_state >= 0 && _clock_state_map[clock_scale_state].type != STATE_TYPE_TSCA_ERR) {
        return STATE_IDX_NONE;
    }

    auto x_clocks = x.bottomRightCorner<EKF_NUM_CLOCK_STATES,1>();
    auto P_clocks = P.bottomRightCorner<EKF_NUM_CLOCK_STATES,EKF_NUM_CLOCK_STATES>();

    for (uint8_t i=0; i<EKF_NUM_CLOCK_STATES; i++) {
        if (_clock_state_map[i].type == STATE_TYPE_NONE) {
            _clock_state_map[i] = {STATE_TYPE_TOFS, clock_scale_state};
            x_clocks(i) = 0;
            P_clocks.row(i).setZero();
            P_clocks.col(i).setZero();
            P_clocks(i,i) = SQ(init_sigma);

            return (clock_state_idx_t)i;
        }
    }

    return STATE_IDX_NONE;
}

bool UWBEKF::is_valid_time_offset_state(clock_state_idx_t idx) {
    if (idx < 0 || idx >= EKF_NUM_CLOCK_STATES) {
         return false;
    }
    return _clock_state_map[idx].type == STATE_TYPE_TOFS;
}

bool UWBEKF::is_valid_time_scale_state(clock_state_idx_t idx) {
    if (idx < 0 || idx >= EKF_NUM_CLOCK_STATES) {
         return false;
    }
    return _clock_state_map[idx].type == STATE_TYPE_TSCA_ERR;
}

void UWBEKF::print_info() {
    for (uint8_t i=0; i<EKF_NUM_CLOCK_STATES; i++) {
        cout << "_clock_state_map[" << (int)i << "].type = " << (int)_clock_state_map[i].type << endl;
        cout << "_clock_state_map[" << (int)i << "].tsca_idx = " << (int)_clock_state_map[i].tsca_idx << endl << endl;
    }
}

float UWBEKF::get_heading_sigma() {
    float subx0 = 1/(sqrt(((x(STATE_IDX_ROT_ERR0))*(x(STATE_IDX_ROT_ERR0))) + ((x(STATE_IDX_ROT_ERR1))*(x(STATE_IDX_ROT_ERR1))) + ((x(STATE_IDX_ROT_ERR2))*(x(STATE_IDX_ROT_ERR2))) + 1));
    float subx1 = quat.coeffs()(2)*subx0;
    float subx2 = 1/(pow(((x(STATE_IDX_ROT_ERR0))*(x(STATE_IDX_ROT_ERR0))) + ((x(STATE_IDX_ROT_ERR1))*(x(STATE_IDX_ROT_ERR1))) + ((x(STATE_IDX_ROT_ERR2))*(x(STATE_IDX_ROT_ERR2))) + 1, 3.0/2.0));
    float subx3 = subx2*x(STATE_IDX_ROT_ERR0);
    float subx4 = quat.coeffs()(1)*subx3;
    float subx5 = subx2*((x(STATE_IDX_ROT_ERR0))*(x(STATE_IDX_ROT_ERR0)));
    float subx6 = 2*subx3;
    float subx7 = quat.coeffs()(3)*subx3;
    float subx8 = quat.coeffs()(1)*subx0;
    float subx9 = quat.coeffs()(0)*subx0;
    float subx10 = quat.coeffs()(3)*subx0;
    float subx11 = -subx1*x(STATE_IDX_ROT_ERR0) - subx10*x(STATE_IDX_ROT_ERR1) - subx8 + subx9*x(STATE_IDX_ROT_ERR2);
    float subx12 = quat.coeffs()(0)*subx6*x(STATE_IDX_ROT_ERR1) - 2*quat.coeffs()(1)*subx5 + quat.coeffs()(2)*subx6 + 2*subx7*x(STATE_IDX_ROT_ERR2) + 2*subx8;
    float subx13 = -2*subx1 - 2*subx10*x(STATE_IDX_ROT_ERR2) + 2*subx8*x(STATE_IDX_ROT_ERR0) - 2*subx9*x(STATE_IDX_ROT_ERR1);
    float subx14 = 2*subx11*(subx1*x(STATE_IDX_ROT_ERR1) - subx10*x(STATE_IDX_ROT_ERR0) - subx8*x(STATE_IDX_ROT_ERR2) - subx9);
    float subx15 = subx1*x(STATE_IDX_ROT_ERR2) - subx10 + subx8*x(STATE_IDX_ROT_ERR1) + subx9*x(STATE_IDX_ROT_ERR0);
    float subx16 = -2*((subx11)*(subx11)) - 2*((-subx1 - subx10*x(STATE_IDX_ROT_ERR2) + subx8*x(STATE_IDX_ROT_ERR0) - subx9*x(STATE_IDX_ROT_ERR1))*(-subx1 - subx10*x(STATE_IDX_ROT_ERR2) + subx8*x(STATE_IDX_ROT_ERR0) - subx9*x(STATE_IDX_ROT_ERR1))) + 1;
    float subx17 = 1/(((subx16)*(subx16)) + ((subx13*subx15 + subx14)*(subx13*subx15 + subx14)));
    float subx18 = subx17*(-subx13*subx15 - subx14);
    float subx19 = -2*subx10;
    float subx20 = 2*subx1*x(STATE_IDX_ROT_ERR1) - 2*subx10*x(STATE_IDX_ROT_ERR0) - 2*subx8*x(STATE_IDX_ROT_ERR2) - 2*subx9;
    float subx21 = -2*subx1 - 2*subx10*x(STATE_IDX_ROT_ERR2) + 2*subx8*x(STATE_IDX_ROT_ERR0) - 2*subx9*x(STATE_IDX_ROT_ERR1);
    float subx22 = subx16*subx17*(subx11*(quat.coeffs()(0)*subx6 - quat.coeffs()(2)*subx6*x(STATE_IDX_ROT_ERR1) + 2*quat.coeffs()(3)*subx5 + subx19 + 2*subx4*x(STATE_IDX_ROT_ERR2)) + subx12*subx15 + subx20*(-quat.coeffs()(0)*subx3*x(STATE_IDX_ROT_ERR2) + quat.coeffs()(2)*subx5 - subx1 + subx4 + subx7*x(STATE_IDX_ROT_ERR1)) + subx21*(-quat.coeffs()(0)*subx5 - quat.coeffs()(2)*subx3*x(STATE_IDX_ROT_ERR2) - subx4*x(STATE_IDX_ROT_ERR1) + subx7 + subx9)) + subx18*(-2*subx11*(-quat.coeffs()(0)*subx6*x(STATE_IDX_ROT_ERR2) + 2*quat.coeffs()(2)*subx5 - 2*subx1 + 2*subx4 + 2*subx7*x(STATE_IDX_ROT_ERR1)) - subx12*subx13);
    float subx23 = subx2*x(STATE_IDX_ROT_ERR1);
    float subx24 = subx2*((x(STATE_IDX_ROT_ERR1))*(x(STATE_IDX_ROT_ERR1)));
    float subx25 = 2*subx23;
    float subx26 = 2*quat.coeffs()(0)*subx24 + quat.coeffs()(2)*subx25 + 2*quat.coeffs()(3)*subx23*x(STATE_IDX_ROT_ERR2) - 2*subx4*x(STATE_IDX_ROT_ERR1) - 2*subx9;
    float subx27 = subx16*subx17*(subx11*(quat.coeffs()(0)*subx25 + 2*quat.coeffs()(1)*subx23*x(STATE_IDX_ROT_ERR2) - 2*quat.coeffs()(2)*subx24 + 2*subx1 + 2*subx7*x(STATE_IDX_ROT_ERR1)) + subx15*subx26 + subx20*(-quat.coeffs()(0)*subx23*x(STATE_IDX_ROT_ERR2) + quat.coeffs()(1)*subx23 + quat.coeffs()(2)*subx3*x(STATE_IDX_ROT_ERR1) + quat.coeffs()(3)*subx24 - subx10) + subx21*(-quat.coeffs()(0)*subx3*x(STATE_IDX_ROT_ERR1) - quat.coeffs()(1)*subx24 - quat.coeffs()(2)*subx23*x(STATE_IDX_ROT_ERR2) + quat.coeffs()(3)*subx23 + subx8)) + subx18*(-2*subx11*(-quat.coeffs()(0)*subx25*x(STATE_IDX_ROT_ERR2) + 2*quat.coeffs()(1)*subx23 + quat.coeffs()(2)*subx6*x(STATE_IDX_ROT_ERR1) + 2*quat.coeffs()(3)*subx24 + subx19) - subx13*subx26);
    float subx28 = subx2*((x(STATE_IDX_ROT_ERR2))*(x(STATE_IDX_ROT_ERR2)));
    float subx29 = subx2*x(STATE_IDX_ROT_ERR2);
    float subx30 = quat.coeffs()(0)*subx25*x(STATE_IDX_ROT_ERR2) + 2*quat.coeffs()(2)*subx29 + 2*quat.coeffs()(3)*subx28 + subx19 - 2*subx4*x(STATE_IDX_ROT_ERR2);
    float subx31 = subx16*subx17*(subx11*(2*quat.coeffs()(0)*subx29 + 2*quat.coeffs()(1)*subx28 - quat.coeffs()(2)*subx25*x(STATE_IDX_ROT_ERR2) + 2*subx7*x(STATE_IDX_ROT_ERR2) - 2*subx8) + subx15*subx30 + subx20*(-quat.coeffs()(0)*subx28 + quat.coeffs()(1)*subx29 + quat.coeffs()(2)*subx3*x(STATE_IDX_ROT_ERR2) + quat.coeffs()(3)*subx23*x(STATE_IDX_ROT_ERR2) + subx9) + subx21*(-quat.coeffs()(0)*subx3*x(STATE_IDX_ROT_ERR2) - quat.coeffs()(1)*subx23*x(STATE_IDX_ROT_ERR2) - quat.coeffs()(2)*subx28 + quat.coeffs()(3)*subx29 + subx1)) + subx18*(-2*subx11*(-2*quat.coeffs()(0)*subx28 + 2*quat.coeffs()(1)*subx29 + quat.coeffs()(2)*subx6*x(STATE_IDX_ROT_ERR2) + 2*quat.coeffs()(3)*subx23*x(STATE_IDX_ROT_ERR2) + 2*subx9) - subx13*subx30);
    return sqrt(subx22*(P(0,0)*subx22 + P(0,1)*subx27 + P(0,2)*subx31) + subx27*(P(0,1)*subx22 + P(1,1)*subx27 + P(1,2)*subx31) + subx31*(P(0,2)*subx22 + P(1,2)*subx27 + P(2,2)*subx31));
}

void UWBEKF::initialize(const Vector3f& init_accel_vec, const Vector3d& ecef_pos, const Vector3f& ecef_vel, float yaw_angle, float yaw_sigma) {
    quat.setIdentity();
    x = x.setZero();
    P = P.setZero();
    auto rot_err_cov = P.topLeftCorner<3,3>();


//     {
//         auto accel_unit_vector = init_accel_vec.normalized();
//         cout << "roll " << atan2f(-accel_unit_vector[1],-accel_unit_vector[2]) << endl;
//         cout << "pitch " << asinf(accel_unit_vector[0]) << endl;
//     }

    float accel_sigma = 1;

    { //////// Begin generated code: Attitude covariance initialization ////////
        float subx0 = (1.0/2.0)*yaw_angle;
        float subx1 = ((init_accel_vec(0))*(init_accel_vec(0))) + ((init_accel_vec(1))*(init_accel_vec(1))) + ((init_accel_vec(2))*(init_accel_vec(2)));
        float subx2 = 1/(sqrt(subx1));
        float subx3 = (1.0/2.0)*asinf(init_accel_vec(0)*subx2);
        float subx4 = (1.0/2.0)*atan2f(-init_accel_vec(1)*subx2, -init_accel_vec(2)*subx2);
        float subx5 = sinf(subx4)*cos(subx0)*cos(subx3);
        float subx6 = sinf(subx0)*sinf(subx3)*cos(subx4);
        float subx7 = sinf(subx3)*cos(subx0)*cos(subx4);
        float subx8 = sinf(subx0)*sinf(subx4)*cos(subx3);
        float subx9 = sinf(subx3)*sinf(subx4)*cos(subx0);
        float subx10 = sinf(subx0)*cos(subx3)*cos(subx4);
        float subx11 = cos(subx0)*cos(subx3)*cos(subx4);
        float subx12 = sinf(subx0)*sinf(subx3)*sinf(subx4);
        float subx13 = ((accel_sigma)*(accel_sigma));
        float subx14 = (1.0/2.0)*subx10;
        float subx15 = 1/(sqrt(-((init_accel_vec(0))*(init_accel_vec(0)))/subx1 + 1));
        float subx16 = 1/(pow(subx1, 3.0/2.0));
        float subx17 = subx15*(-((init_accel_vec(0))*(init_accel_vec(0)))*subx16 + subx2);
        float subx18 = (1.0/2.0)*subx9;
        float subx19 = subx14*subx17 - subx17*subx18;
        float subx20 = (1.0/2.0)/(((init_accel_vec(1))*(init_accel_vec(1)))/subx1 + ((init_accel_vec(2))*(init_accel_vec(2)))/subx1);
        float subx21 = -init_accel_vec(1)*((init_accel_vec(2))*(init_accel_vec(2)))*subx20/((subx1)*(subx1)) + init_accel_vec(1)*subx2*subx20*(((init_accel_vec(2))*(init_accel_vec(2)))*subx16 - subx2);
        float subx22 = init_accel_vec(0)*init_accel_vec(2)*subx15*subx16;
        float subx23 = subx11*subx21 - subx12*subx21 - subx14*subx22 + subx18*subx22;
        float subx24 = ((init_accel_vec(1))*(init_accel_vec(1)))*init_accel_vec(2)*subx20/((subx1)*(subx1)) - init_accel_vec(2)*subx2*subx20*(((init_accel_vec(1))*(init_accel_vec(1)))*subx16 - subx2);
        float subx25 = init_accel_vec(0)*init_accel_vec(1)*subx15*subx16;
        float subx26 = subx11*subx24 - subx12*subx24 - subx14*subx25 + subx18*subx25;
        float subx27 = ((yaw_sigma)*(yaw_sigma));
        float subx28 = (1.0/2.0)*subx7;
        float subx29 = (1.0/2.0)*subx8;
        float subx30 = subx13*((subx19)*(subx19)) + subx13*((subx23)*(subx23)) + subx13*((subx26)*(subx26)) + subx27*((subx28 - subx29)*(subx28 - subx29));
        float subx31 = (1.0/2.0)/((1.0/4.0)*((subx10 + subx9)*(subx10 + subx9)) + (1.0/4.0)*((subx11 + subx12)*(subx11 + subx12)) + (1.0/4.0)*((subx5 + subx6)*(subx5 + subx6)) + (1.0/4.0)*((subx7 + subx8)*(subx7 + subx8)));
        float subx32 = subx31*(subx11 + subx12);
        float subx33 = (1.0/2.0)*subx11;
        float subx34 = (1.0/2.0)*subx12;
        float subx35 = subx17*subx33 - subx17*subx34;
        float subx36 = subx10*subx21 - subx21*subx9 - subx22*subx33 + subx22*subx34;
        float subx37 = subx10*subx24 - subx24*subx9 - subx25*subx33 + subx25*subx34;
        float subx38 = (1.0/2.0)*subx5;
        float subx39 = (1.0/2.0)*subx6;
        float subx40 = subx13*subx19*subx35 + subx13*subx23*subx36 + subx13*subx26*subx37 + subx27*(subx28 - subx29)*(subx38 - subx39);
        float subx41 = subx31*(subx10 + subx9);
        float subx42 = -subx17*subx28 + subx17*subx29;
        float subx43 = -subx21*subx5 + subx21*subx6 + subx22*subx28 - subx22*subx29;
        float subx44 = -subx24*subx5 + subx24*subx6 + subx25*subx28 - subx25*subx29;
        float subx45 = subx13*subx19*subx42 + subx13*subx23*subx43 + subx13*subx26*subx44 + subx27*(-subx14 + subx18)*(subx28 - subx29);
        float subx46 = subx31*(subx5 + subx6);
        float subx47 = subx17*subx38 - subx17*subx39;
        float subx48 = subx21*subx7 - subx21*subx8 - subx22*subx38 + subx22*subx39;
        float subx49 = subx24*subx7 - subx24*subx8 - subx25*subx38 + subx25*subx39;
        float subx50 = subx13*subx19*subx47 + subx13*subx23*subx48 + subx13*subx26*subx49 + subx27*(subx28 - subx29)*(subx33 - subx34);
        float subx51 = subx31*(subx7 + subx8);
        float subx52 = subx30*subx32 + subx40*subx41 - subx45*subx46 - subx50*subx51;
        float subx53 = subx13*((subx35)*(subx35)) + subx13*((subx36)*(subx36)) + subx13*((subx37)*(subx37)) + subx27*((subx38 - subx39)*(subx38 - subx39));
        float subx54 = subx13*subx35*subx42 + subx13*subx36*subx43 + subx13*subx37*subx44 + subx27*(-subx14 + subx18)*(subx38 - subx39);
        float subx55 = subx13*subx35*subx47 + subx13*subx36*subx48 + subx13*subx37*subx49 + subx27*(subx33 - subx34)*(subx38 - subx39);
        float subx56 = subx32*subx40 + subx41*subx53 - subx46*subx54 - subx51*subx55;
        float subx57 = subx13*((subx42)*(subx42)) + subx13*((subx43)*(subx43)) + subx13*((subx44)*(subx44)) + subx27*((-subx14 + subx18)*(-subx14 + subx18));
        float subx58 = subx13*subx42*subx47 + subx13*subx43*subx48 + subx13*subx44*subx49 + subx27*(-subx14 + subx18)*(subx33 - subx34);
        float subx59 = subx32*subx45 + subx41*subx54 - subx46*subx57 - subx51*subx58;
        float subx60 = subx13*((subx47)*(subx47)) + subx13*((subx48)*(subx48)) + subx13*((subx49)*(subx49)) + subx27*((subx33 - subx34)*(subx33 - subx34));
        float subx61 = subx32*subx50 + subx41*subx55 - subx46*subx58 - subx51*subx60;
        float subx62 = -subx30*subx41 + subx32*subx40 - subx45*subx51 + subx46*subx50;
        float subx63 = subx32*subx53 - subx40*subx41 + subx46*subx55 - subx51*subx54;
        float subx64 = subx32*subx54 - subx41*subx45 + subx46*subx58 - subx51*subx57;
        float subx65 = subx32*subx55 - subx41*subx50 + subx46*subx60 - subx51*subx58;
        float subx66 = subx30*subx51 + subx32*subx50 - subx40*subx46 - subx41*subx45;
        float subx67 = subx32*subx55 + subx40*subx51 - subx41*subx54 - subx46*subx53;
        float subx68 = subx32*subx58 - subx41*subx57 + subx45*subx51 - subx46*subx54;
        float subx69 = subx32*subx60 - subx41*subx58 - subx46*subx55 + subx50*subx51;

        quat.coeffs()(0) = subx5 + subx6;
        quat.coeffs()(1) = subx7 + subx8;
        quat.coeffs()(2) = subx10 + subx9;
        quat.coeffs()(3) = subx11 + subx12;

        rot_err_cov(0,0) = subx32*subx52 + subx41*subx56 - subx46*subx59 - subx51*subx61;
        rot_err_cov(0,1) = subx32*subx56 - subx41*subx52 + subx46*subx61 - subx51*subx59;
        rot_err_cov(0,2) = subx32*subx61 - subx41*subx59 - subx46*subx56 + subx51*subx52;
        rot_err_cov(1,0) = subx32*subx62 + subx41*subx63 - subx46*subx64 - subx51*subx65;
        rot_err_cov(1,1) = subx32*subx63 - subx41*subx62 + subx46*subx65 - subx51*subx64;
        rot_err_cov(1,2) = subx32*subx65 - subx41*subx64 - subx46*subx63 + subx51*subx62;
        rot_err_cov(2,0) = subx32*subx66 + subx41*subx67 - subx46*subx68 - subx51*subx69;
        rot_err_cov(2,1) = subx32*subx67 - subx41*subx66 + subx46*subx69 - subx51*subx68;
        rot_err_cov(2,2) = subx32*subx69 - subx41*subx68 - subx46*subx67 + subx51*subx66;
    } //////// End generated code: Attitude covariance initialization   ////////


//     quat.setZero();
//     quat(0) = 1;
//     quat_cov.setZero();

    for (uint8_t i=0; i<3; i++) {
        x(STATE_IDX_POS0+i) = 1;
        P(STATE_IDX_POS0+i,STATE_IDX_POS0+i) = SQ(2);
        P(STATE_IDX_VEL0+i,STATE_IDX_VEL0+i) = SQ(1);
        P(STATE_IDX_GBIAS0+i,STATE_IDX_GBIAS0+i) = SQ(0.2);
//         x(STATE_IDX_GSCALE0+i) = 1;
        P(STATE_IDX_GSCALE0+i,STATE_IDX_GSCALE0+i) = SQ(0.01);
        P(STATE_IDX_ABIAS0+i,STATE_IDX_ABIAS0+i) = SQ(0.2);
    }

//     cout << "x" << endl << x << endl << endl;
//     cout << "P" << endl << P << endl << endl;

//     zeroRotErr();
}

// static Vector3d quat_to_euler(Quaterniond quat) {
//     const float& qi = quat.coeffs()(0);
//     const float& qj = quat.coeffs()(1);
//     const float& qk = quat.coeffs()(2);
//     const float& qr = quat.coeffs()(3);
//     return Vector3d(atan2f(2*(qr*qi+qj*qk),1-2*(qi*qi+qj*qj)),
//                     asinf(2*(qr*qj-qk*qi)),
//                     atan2f(2*(qr*qk+qi*qj), 1-2*(qj*qj+qk*qk)));
// }

void UWBEKF::zeroRotErr() {
    MatrixSxS F = MatrixSxS::Identity();
    Quaternion del_quat;

    float subx0 = ((x(STATE_IDX_ROT_ERR0))*(x(STATE_IDX_ROT_ERR0))) + ((x(STATE_IDX_ROT_ERR1))*(x(STATE_IDX_ROT_ERR1))) + ((x(STATE_IDX_ROT_ERR2))*(x(STATE_IDX_ROT_ERR2))) + 1;
    float subx1 = 1/(sqrt(subx0));
    float subx2 = 1/(subx0);
    F(0,0) = 1;
    F(0,1) = subx2*x(STATE_IDX_ROT_ERR2);
    F(0,2) = -subx2*x(STATE_IDX_ROT_ERR1);
    F(1,0) = -subx2*x(STATE_IDX_ROT_ERR2);
    F(1,1) = 1;
    F(1,2) = subx2*x(STATE_IDX_ROT_ERR0);
    F(2,0) = subx2*x(STATE_IDX_ROT_ERR1);
    F(2,1) = -subx2*x(STATE_IDX_ROT_ERR0);
    F(2,2) = 1;
    del_quat.coeffs()(0) = -subx1*x(STATE_IDX_ROT_ERR0);
    del_quat.coeffs()(1) = -subx1*x(STATE_IDX_ROT_ERR1);
    del_quat.coeffs()(2) = -subx1*x(STATE_IDX_ROT_ERR2);
    del_quat.coeffs()(3) = -subx1;

    x.topRows<3>().setZero();
    P = F*P*F.transpose();
    quat = quat * del_quat;
}

void UWBEKF::get_state_and_covariance_different_quat(const UWBEKF::Quaternion& new_quat, UWBEKF::MatrixSx1& x_ret, UWBEKF::MatrixSxS& P_ret) {
    x_ret.block<N_STATES-3,1>(3,0) = x.block<N_STATES-3,1>(3,0);

    MatrixSxS F = MatrixSxS::Identity();
    float subx0 = 1/(new_quat.coeffs()(0)*quat.coeffs()(0) + new_quat.coeffs()(1)*quat.coeffs()(1) + new_quat.coeffs()(2)*quat.coeffs()(2) + new_quat.coeffs()(3)*quat.coeffs()(3));
    float subx1 = subx0*(new_quat.coeffs()(0)*quat.coeffs()(3) + new_quat.coeffs()(1)*quat.coeffs()(2) - new_quat.coeffs()(2)*quat.coeffs()(1) - new_quat.coeffs()(3)*quat.coeffs()(0));
    float subx2 = subx0*(-new_quat.coeffs()(0)*quat.coeffs()(2) + new_quat.coeffs()(1)*quat.coeffs()(3) + new_quat.coeffs()(2)*quat.coeffs()(0) - new_quat.coeffs()(3)*quat.coeffs()(1));
    float subx3 = subx0*(new_quat.coeffs()(0)*quat.coeffs()(1) - new_quat.coeffs()(1)*quat.coeffs()(0) + new_quat.coeffs()(2)*quat.coeffs()(3) - new_quat.coeffs()(3)*quat.coeffs()(2));
    float subx4 = subx1*x(STATE_IDX_ROT_ERR0) + subx2*x(STATE_IDX_ROT_ERR1) + subx3*x(STATE_IDX_ROT_ERR2) + 1;
    float subx5 = 1/(subx4);
    float subx6 = 1/(((subx4)*(subx4)));
    x_ret(0) = -subx1 - subx5*(subx2*x(STATE_IDX_ROT_ERR2) - subx3*x(STATE_IDX_ROT_ERR1)) + x(STATE_IDX_ROT_ERR0);
    x_ret(1) = -subx2 - subx5*(-subx1*x(STATE_IDX_ROT_ERR2) + subx3*x(STATE_IDX_ROT_ERR0)) + x(STATE_IDX_ROT_ERR1);
    x_ret(2) = -subx3 - subx5*(subx1*x(STATE_IDX_ROT_ERR1) - subx2*x(STATE_IDX_ROT_ERR0)) + x(STATE_IDX_ROT_ERR2);
    F(0,0) = -subx1*subx6*(-subx2*x(STATE_IDX_ROT_ERR2) + subx3*x(STATE_IDX_ROT_ERR1)) + 1;
    F(0,1) = subx2*subx6*(subx2*x(STATE_IDX_ROT_ERR2) - subx3*x(STATE_IDX_ROT_ERR1)) + subx3*subx5;
    F(0,2) = -subx2*subx5 + subx3*subx6*(subx2*x(STATE_IDX_ROT_ERR2) - subx3*x(STATE_IDX_ROT_ERR1));
    F(1,0) = subx1*subx6*(-subx1*x(STATE_IDX_ROT_ERR2) + subx3*x(STATE_IDX_ROT_ERR0)) - subx3*subx5;
    F(1,1) = -subx2*subx6*(subx1*x(STATE_IDX_ROT_ERR2) - subx3*x(STATE_IDX_ROT_ERR0)) + 1;
    F(1,2) = subx1*subx5 + subx3*subx6*(-subx1*x(STATE_IDX_ROT_ERR2) + subx3*x(STATE_IDX_ROT_ERR0));
    F(2,0) = subx1*subx6*(subx1*x(STATE_IDX_ROT_ERR1) - subx2*x(STATE_IDX_ROT_ERR0)) + subx2*subx5;
    F(2,1) = -subx1*subx5 + subx2*subx6*(subx1*x(STATE_IDX_ROT_ERR1) - subx2*x(STATE_IDX_ROT_ERR0));
    F(2,2) = -subx3*subx6*(-subx1*x(STATE_IDX_ROT_ERR1) + subx2*x(STATE_IDX_ROT_ERR0)) + 1;

    P_ret = F*P*F.transpose();
}

void UWBEKF::addProcessNoise(float uwb_dt, float imu_dt, Vector3f del_ang, Vector3f del_vel) {
    float accel_sigma = _params.accel_sigma;
    float tsca_err_pnoise = _params.tsca_err_pnoise;
    float tofs_pnoise = _params.tofs_pnoise;
    float gyro_sigma = _params.gyro_sigma;
    float gscale_pnoise = _params.gscale_pnoise;
    float gyro_cross_sigma = 0.0;
    float gbias_pnoise = _params.gbias_pnoise;
    float accel_scale_sigma = 0.0;
    float accel_cross_sigma = 0.0;
    float abias_pnoise_xy = _params.abias_pnoise_xy;
    float abias_pnoise_z = _params.abias_pnoise_z;

    { //////// Begin generated code: Process noise         ////////
        float subx0 = (1.0/2.0)*x(STATE_IDX_GSCALE0) + 1.0/2.0;
        float subx1 = 1/(sqrt(((x(STATE_IDX_ROT_ERR0))*(x(STATE_IDX_ROT_ERR0))) + ((x(STATE_IDX_ROT_ERR1))*(x(STATE_IDX_ROT_ERR1))) + ((x(STATE_IDX_ROT_ERR2))*(x(STATE_IDX_ROT_ERR2))) + 1));
        float subx2 = del_ang(0)*(x(STATE_IDX_GSCALE0) + 1);
        float subx3 = -1.0/2.0*imu_dt*x(STATE_IDX_GBIAS0) + (1.0/2.0)*subx2;
        float subx4 = subx1*x(STATE_IDX_ROT_ERR0);
        float subx5 = del_ang(1)*(x(STATE_IDX_GSCALE1) + 1);
        float subx6 = -1.0/2.0*imu_dt*x(STATE_IDX_GBIAS1) + (1.0/2.0)*subx5;
        float subx7 = subx1*x(STATE_IDX_ROT_ERR1);
        float subx8 = del_ang(2)*(x(STATE_IDX_GSCALE2) + 1);
        float subx9 = -1.0/2.0*imu_dt*x(STATE_IDX_GBIAS2) + (1.0/2.0)*subx8;
        float subx10 = subx1*x(STATE_IDX_ROT_ERR2);
        float subx11 = -subx1 + subx10*subx9 + subx3*subx4 + subx6*subx7;
        float subx12 = 1/(subx11);
        float subx13 = 1/(((subx11)*(subx11)));
        float subx14 = subx13*(-subx1*subx3 + subx1*subx6*x(STATE_IDX_ROT_ERR2) - subx4 - subx7*subx9);
        float subx15 = -subx0*subx1*subx12 - subx0*subx14*subx4;
        float subx16 = ((gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS1) + subx5) + gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS2) + subx8) + gyro_sigma*imu_dt)*(gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS1) + subx5) + gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS2) + subx8) + gyro_sigma*imu_dt));
        float subx17 = (1.0/2.0)*x(STATE_IDX_GSCALE1) + 1.0/2.0;
        float subx18 = subx10*subx12*subx17 - subx14*subx17*subx7;
        float subx19 = ((gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS0) + subx2) + gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS2) + subx8) + gyro_sigma*imu_dt)*(gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS0) + subx2) + gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS2) + subx8) + gyro_sigma*imu_dt));
        float subx20 = (1.0/2.0)*x(STATE_IDX_GSCALE2) + 1.0/2.0;
        float subx21 = -subx10*subx14*subx20 - subx12*subx20*subx7;
        float subx22 = ((gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS0) + subx2) + gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS1) + subx5) + gyro_sigma*imu_dt)*(gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS0) + subx2) + gyro_cross_sigma*(-imu_dt*x(STATE_IDX_GBIAS1) + subx5) + gyro_sigma*imu_dt));
        float subx23 = subx13*(-subx1*subx3*x(STATE_IDX_ROT_ERR2) - subx1*subx6 + subx4*subx9 - subx7);
        float subx24 = -subx10*subx20*subx23 + subx12*subx20*subx4;
        float subx25 = -subx1*subx12*subx17 - subx17*subx23*subx7;
        float subx26 = -subx0*subx10*subx12 - subx0*subx23*subx4;
        float subx27 = subx15*subx16*subx26 + subx18*subx19*subx25 + subx21*subx22*subx24;
        float subx28 = subx13*(subx1*subx3*x(STATE_IDX_ROT_ERR1) - subx1*subx9 - subx10 - subx4*subx6);
        float subx29 = -subx1*subx12*subx20 - subx10*subx20*subx28;
        float subx30 = -subx12*subx17*subx4 - subx17*subx28*subx7;
        float subx31 = subx0*subx12*subx7 - subx0*subx28*subx4;
        float subx32 = subx15*subx16*subx31 + subx18*subx19*subx30 + subx21*subx22*subx29;
        float subx33 = subx16*subx26*subx31 + subx19*subx25*subx30 + subx22*subx24*subx29;
        float subx34 = ((imu_dt)*(imu_dt));
        float subx35 = ((gbias_pnoise)*(gbias_pnoise))*subx34;
        float subx36 = ((gscale_pnoise)*(gscale_pnoise))*subx34;
        float subx37 = 2.0*quat.coeffs()(0)*subx4 + 2.0*quat.coeffs()(1)*subx7 + 2.0*quat.coeffs()(2)*subx10 - 2.0*quat.coeffs()(3)*subx1;
        float subx38 = quat.coeffs()(0)*subx10 - quat.coeffs()(1)*subx1 - quat.coeffs()(2)*subx4 - quat.coeffs()(3)*subx7;
        float subx39 = -quat.coeffs()(0)*subx1 - quat.coeffs()(1)*subx10 + quat.coeffs()(2)*subx7 - quat.coeffs()(3)*subx4;
        float subx40 = subx37*(quat.coeffs()(0)*subx7 - quat.coeffs()(1)*subx4 + quat.coeffs()(2)*subx1 + quat.coeffs()(3)*subx10) + 2.0*subx38*subx39;
        float subx41 = del_vel(2) - imu_dt*x(STATE_IDX_ABIAS2);
        float subx42 = del_vel(1) - imu_dt*x(STATE_IDX_ABIAS1);
        float subx43 = del_vel(0) - imu_dt*x(STATE_IDX_ABIAS0);
        float subx44 = ((accel_cross_sigma*subx41 + accel_cross_sigma*subx43 + accel_scale_sigma*subx42 + accel_sigma*imu_dt)*(accel_cross_sigma*subx41 + accel_cross_sigma*subx43 + accel_scale_sigma*subx42 + accel_sigma*imu_dt));
        float subx45 = -quat.coeffs()(0)*subx7 + quat.coeffs()(1)*subx4 - quat.coeffs()(2)*subx1 - quat.coeffs()(3)*subx10;
        float subx46 = subx37*subx38 + 2.0*subx39*subx45;
        float subx47 = ((accel_cross_sigma*subx42 + accel_cross_sigma*subx43 + accel_scale_sigma*subx41 + accel_sigma*imu_dt)*(accel_cross_sigma*subx42 + accel_cross_sigma*subx43 + accel_scale_sigma*subx41 + accel_sigma*imu_dt));
        float subx48 = ((quat.coeffs()(0)*subx4 + quat.coeffs()(1)*subx7 + quat.coeffs()(2)*subx10 - quat.coeffs()(3)*subx1)*(quat.coeffs()(0)*subx4 + quat.coeffs()(1)*subx7 + quat.coeffs()(2)*subx10 - quat.coeffs()(3)*subx1));
        float subx49 = -((subx38)*(subx38)) + 1.0*((subx39)*(subx39)) - ((subx45)*(subx45)) + subx48;
        float subx50 = ((accel_cross_sigma*subx41 + accel_cross_sigma*subx42 + accel_scale_sigma*subx43 + accel_sigma*imu_dt)*(accel_cross_sigma*subx41 + accel_cross_sigma*subx42 + accel_scale_sigma*subx43 + accel_sigma*imu_dt));
        float subx51 = subx37*(quat.coeffs()(0)*subx1 + quat.coeffs()(1)*subx10 - quat.coeffs()(2)*subx7 + quat.coeffs()(3)*subx4) + 2.0*subx38*subx45;
        float subx52 = 1.0*((subx38)*(subx38)) - ((subx39)*(subx39)) - ((subx45)*(subx45)) + subx48;
        float subx53 = subx37*subx45 + 2.0*subx38*subx39;
        float subx54 = subx40*subx44*subx52 + subx46*subx47*subx51 + subx49*subx50*subx53;
        float subx55 = -((subx38)*(subx38)) - ((subx39)*(subx39)) + 1.0*((subx45)*(subx45)) + subx48;
        float subx56 = subx37*subx39 + 2.0*subx38*subx45;
        float subx57 = subx37*(-quat.coeffs()(0)*subx10 + quat.coeffs()(1)*subx1 + quat.coeffs()(2)*subx4 + quat.coeffs()(3)*subx7) + 2.0*subx39*subx45;
        float subx58 = subx40*subx44*subx56 + subx46*subx47*subx55 + subx49*subx50*subx57;
        float subx59 = subx44*subx52*subx56 + subx47*subx51*subx55 + subx50*subx53*subx57;

        P(0,0) += ((subx15)*(subx15))*subx16 + ((subx18)*(subx18))*subx19 + ((subx21)*(subx21))*subx22;
        P(0,1) += subx27;
        P(0,2) += subx32;
        P(1,0) += subx27;
        P(1,1) += subx16*((subx26)*(subx26)) + subx19*((subx25)*(subx25)) + subx22*((subx24)*(subx24));
        P(1,2) += subx33;
        P(2,0) += subx32;
        P(2,1) += subx33;
        P(2,2) += subx16*((subx31)*(subx31)) + subx19*((subx30)*(subx30)) + subx22*((subx29)*(subx29));
        P(3,3) += subx35;
        P(4,4) += subx35;
        P(5,5) += subx35;
        P(6,6) += subx36;
        P(7,7) += subx36;
        P(8,8) += subx36;
        P(9,9) += ((abias_pnoise_xy)*(abias_pnoise_xy))*subx34;
        P(10,10) += ((abias_pnoise_xy)*(abias_pnoise_xy))*subx34;
        P(11,11) += ((abias_pnoise_z)*(abias_pnoise_z))*subx34;
        P(15,15) += ((subx40)*(subx40))*subx44 + ((subx46)*(subx46))*subx47 + ((subx49)*(subx49))*subx50;
        P(15,16) += subx54;
        P(15,17) += subx58;
        P(16,15) += subx54;
        P(16,16) += subx44*((subx52)*(subx52)) + subx47*((subx51)*(subx51)) + subx50*((subx53)*(subx53));
        P(16,17) += subx59;
        P(17,15) += subx58;
        P(17,16) += subx59;
        P(17,17) += subx44*((subx56)*(subx56)) + subx47*((subx55)*(subx55)) + subx50*((subx57)*(subx57));
    } //////// End generated code: Process noise           ////////

    auto P_clocks = P.bottomRightCorner<EKF_NUM_CLOCK_STATES,EKF_NUM_CLOCK_STATES>();

    for (uint8_t i=0; i<EKF_NUM_CLOCK_STATES; i++) {
        if (_clock_state_map[i].type == STATE_TYPE_TSCA_ERR) {
            P_clocks(i,i) += SQ(tsca_err_pnoise)*uwb_dt;
            for (uint8_t j=0; j<EKF_NUM_CLOCK_STATES; j++) {
                if (_clock_state_map[j].type == STATE_TYPE_TSCA_ERR) {
                    P_clocks(i,j) += SQ(tsca_err_pnoise)*uwb_dt;
                }
            }
        } else if (_clock_state_map[i].type == STATE_TYPE_TOFS) {
            P_clocks(i,i) += SQ(tofs_pnoise*uwb_dt);
        }
    }
}

void UWBEKF::predict(float uwb_dt, float imu_dt, Vector3f del_ang, Vector3f del_vel) {
    cout << endl << "######## PREDICT BEGIN #########" << endl;
//     cout << "dt " << uwb_dt << " " << imu_dt << endl;
//     cout << "del_ang" << endl << del_ang/dt << endl << endl;
//     cout << "del_vel" << endl << del_vel/dt << endl << endl;

//     cout << "x" << endl << x << endl << endl;
//     cout << "P" << endl << P << endl << endl;

    MatrixSx1 f = MatrixSx1::Zero();
    MatrixSxS F = MatrixSxS::Zero();

    { //////// Begin generated code: Prediction model      ////////
        float subx0 = del_ang(0)*(x(STATE_IDX_GSCALE0) + 1) - imu_dt*x(STATE_IDX_GBIAS0);
        float subx1 = del_ang(1)*(x(STATE_IDX_GSCALE1) + 1) - imu_dt*x(STATE_IDX_GBIAS1);
        float subx2 = del_ang(2)*(x(STATE_IDX_GSCALE2) + 1) - imu_dt*x(STATE_IDX_GBIAS2);
        float subx3 = sqrt(((subx0)*(subx0)) + ((subx1)*(subx1)) + ((subx2)*(subx2)));
        float subx4 = ((subx3 > 0) ? ( 1/(subx3) ) : ( 1 ))*sinf(0.5*subx3)/cos(0.5*subx3);
        float subx5 = del_vel(0) - imu_dt*x(STATE_IDX_ABIAS0);
        float subx6 = del_vel(1) - imu_dt*x(STATE_IDX_ABIAS1);
        float subx7 = 2.0*quat.coeffs()(0);
        float subx8 = del_vel(2) - imu_dt*x(STATE_IDX_ABIAS2);

        f(0,0) = subx0*subx4;
        f(1,0) = subx1*subx4;
        f(2,0) = subx2*subx4;
        f(3,0) = x(STATE_IDX_GBIAS0);
        f(4,0) = x(STATE_IDX_GBIAS1);
        f(5,0) = x(STATE_IDX_GBIAS2);
        f(6,0) = x(STATE_IDX_GSCALE0);
        f(7,0) = x(STATE_IDX_GSCALE1);
        f(8,0) = x(STATE_IDX_GSCALE2);
        f(9,0) = x(STATE_IDX_ABIAS0);
        f(10,0) = x(STATE_IDX_ABIAS1);
        f(11,0) = x(STATE_IDX_ABIAS2);
        f(12,0) = imu_dt*x(STATE_IDX_VEL0) + x(STATE_IDX_POS0);
        f(13,0) = imu_dt*x(STATE_IDX_VEL1) + x(STATE_IDX_POS1);
        f(14,0) = imu_dt*x(STATE_IDX_VEL2) + x(STATE_IDX_POS2);
        f(15,0) = subx5*(1.0*((quat.coeffs()(0))*(quat.coeffs()(0))) - ((quat.coeffs()(1))*(quat.coeffs()(1))) - ((quat.coeffs()(2))*(quat.coeffs()(2))) + ((quat.coeffs()(3))*(quat.coeffs()(3)))) + subx6*(quat.coeffs()(1)*subx7 - 2.0*quat.coeffs()(2)*quat.coeffs()(3)) + subx8*(2.0*quat.coeffs()(1)*quat.coeffs()(3) + quat.coeffs()(2)*subx7) + x(STATE_IDX_VEL0);
        f(16,0) = subx5*(quat.coeffs()(1)*subx7 + 2.0*quat.coeffs()(2)*quat.coeffs()(3)) + subx6*(-((quat.coeffs()(0))*(quat.coeffs()(0))) + 1.0*((quat.coeffs()(1))*(quat.coeffs()(1))) - ((quat.coeffs()(2))*(quat.coeffs()(2))) + ((quat.coeffs()(3))*(quat.coeffs()(3)))) + subx8*(2.0*quat.coeffs()(1)*quat.coeffs()(2) - quat.coeffs()(3)*subx7) + x(STATE_IDX_VEL1);
        f(17,0) = 9.8065499999999997*imu_dt + subx5*(-2.0*quat.coeffs()(1)*quat.coeffs()(3) + quat.coeffs()(2)*subx7) + subx6*(2.0*quat.coeffs()(1)*quat.coeffs()(2) + quat.coeffs()(3)*subx7) + subx8*(-((quat.coeffs()(0))*(quat.coeffs()(0))) - ((quat.coeffs()(1))*(quat.coeffs()(1))) + 1.0*((quat.coeffs()(2))*(quat.coeffs()(2))) + ((quat.coeffs()(3))*(quat.coeffs()(3)))) + x(STATE_IDX_VEL2);

        F(0,0) = ((-1.0/2.0*del_ang(0)*(x(STATE_IDX_GSCALE0) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS0))*(-1.0/2.0*del_ang(0)*(x(STATE_IDX_GSCALE0) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS0))) + 1;
        F(0,1) = (1.0/2.0)*del_ang(2)*(x(STATE_IDX_GSCALE2) + 1) - 1.0/2.0*imu_dt*x(STATE_IDX_GBIAS2) + (-1.0/2.0*del_ang(0)*(x(STATE_IDX_GSCALE0) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS0))*(-1.0/2.0*del_ang(1)*(x(STATE_IDX_GSCALE1) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS1));
        F(0,2) = -1.0/2.0*del_ang(1)*(x(STATE_IDX_GSCALE1) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS1) + (-1.0/2.0*del_ang(0)*(x(STATE_IDX_GSCALE0) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS0))*(-1.0/2.0*del_ang(2)*(x(STATE_IDX_GSCALE2) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS2));
        F(0,3) = -1.0/2.0*imu_dt;
        F(0,6) = (1.0/2.0)*del_ang(0);
        F(1,0) = -1.0/2.0*del_ang(2)*(x(STATE_IDX_GSCALE2) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS2) + (-1.0/2.0*del_ang(0)*(x(STATE_IDX_GSCALE0) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS0))*(-1.0/2.0*del_ang(1)*(x(STATE_IDX_GSCALE1) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS1));
        F(1,1) = ((-1.0/2.0*del_ang(1)*(x(STATE_IDX_GSCALE1) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS1))*(-1.0/2.0*del_ang(1)*(x(STATE_IDX_GSCALE1) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS1))) + 1;
        F(1,2) = (1.0/2.0)*del_ang(0)*(x(STATE_IDX_GSCALE0) + 1) - 1.0/2.0*imu_dt*x(STATE_IDX_GBIAS0) + (-1.0/2.0*del_ang(1)*(x(STATE_IDX_GSCALE1) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS1))*(-1.0/2.0*del_ang(2)*(x(STATE_IDX_GSCALE2) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS2));
        F(1,4) = -1.0/2.0*imu_dt;
        F(1,7) = (1.0/2.0)*del_ang(1);
        F(2,0) = (1.0/2.0)*del_ang(1)*(x(STATE_IDX_GSCALE1) + 1) - 1.0/2.0*imu_dt*x(STATE_IDX_GBIAS1) + (-1.0/2.0*del_ang(0)*(x(STATE_IDX_GSCALE0) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS0))*(-1.0/2.0*del_ang(2)*(x(STATE_IDX_GSCALE2) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS2));
        F(2,1) = -1.0/2.0*del_ang(0)*(x(STATE_IDX_GSCALE0) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS0) + (-1.0/2.0*del_ang(1)*(x(STATE_IDX_GSCALE1) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS1))*(-1.0/2.0*del_ang(2)*(x(STATE_IDX_GSCALE2) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS2));
        F(2,2) = ((-1.0/2.0*del_ang(2)*(x(STATE_IDX_GSCALE2) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS2))*(-1.0/2.0*del_ang(2)*(x(STATE_IDX_GSCALE2) + 1) + (1.0/2.0)*imu_dt*x(STATE_IDX_GBIAS2))) + 1;
        F(2,5) = -1.0/2.0*imu_dt;
        F(2,8) = (1.0/2.0)*del_ang(2);
        F(3,3) = 1;
        F(4,4) = 1;
        F(5,5) = 1;
        F(6,6) = 1;
        F(7,7) = 1;
        F(8,8) = 1;
        F(9,9) = 1;
        F(10,10) = 1;
        F(11,11) = 1;
        F(12,12) = 1;
        F(12,15) = imu_dt;
        F(13,13) = 1;
        F(13,16) = imu_dt;
        F(14,14) = 1;
        F(14,17) = imu_dt;
        F(15,0) = (del_vel(1) - imu_dt*x(STATE_IDX_ABIAS1))*(4.0*quat.coeffs()(0)*quat.coeffs()(2) + 4.0*quat.coeffs()(1)*quat.coeffs()(3)) + (del_vel(2) - imu_dt*x(STATE_IDX_ABIAS2))*(-4.0*quat.coeffs()(0)*quat.coeffs()(1) + 4.0*quat.coeffs()(2)*quat.coeffs()(3));
        F(15,1) = (del_vel(0) - imu_dt*x(STATE_IDX_ABIAS0))*(-4.0*quat.coeffs()(0)*quat.coeffs()(2) - 4*quat.coeffs()(1)*quat.coeffs()(3)) + (del_vel(2) - imu_dt*x(STATE_IDX_ABIAS2))*(2.0*((quat.coeffs()(0))*(quat.coeffs()(0))) - 2.0*((quat.coeffs()(1))*(quat.coeffs()(1))) - 2.0*((quat.coeffs()(2))*(quat.coeffs()(2))) + 2.0*((quat.coeffs()(3))*(quat.coeffs()(3))));
        F(15,2) = (del_vel(0) - imu_dt*x(STATE_IDX_ABIAS0))*(4.0*quat.coeffs()(0)*quat.coeffs()(1) - 4*quat.coeffs()(2)*quat.coeffs()(3)) + (del_vel(1) - imu_dt*x(STATE_IDX_ABIAS1))*(-2.0*((quat.coeffs()(0))*(quat.coeffs()(0))) + 2.0*((quat.coeffs()(1))*(quat.coeffs()(1))) + 2.0*((quat.coeffs()(2))*(quat.coeffs()(2))) - 2.0*((quat.coeffs()(3))*(quat.coeffs()(3))));
        F(15,9) = -imu_dt*(1.0*((quat.coeffs()(0))*(quat.coeffs()(0))) - ((quat.coeffs()(1))*(quat.coeffs()(1))) - ((quat.coeffs()(2))*(quat.coeffs()(2))) + ((quat.coeffs()(3))*(quat.coeffs()(3))));
        F(15,10) = -imu_dt*(2.0*quat.coeffs()(0)*quat.coeffs()(1) - 2.0*quat.coeffs()(2)*quat.coeffs()(3));
        F(15,11) = -imu_dt*(2.0*quat.coeffs()(0)*quat.coeffs()(2) + 2.0*quat.coeffs()(1)*quat.coeffs()(3));
        F(15,15) = 1;
        F(16,0) = (del_vel(1) - imu_dt*x(STATE_IDX_ABIAS1))*(-4*quat.coeffs()(0)*quat.coeffs()(3) + 4.0*quat.coeffs()(1)*quat.coeffs()(2)) + (del_vel(2) - imu_dt*x(STATE_IDX_ABIAS2))*(2.0*((quat.coeffs()(0))*(quat.coeffs()(0))) - 2.0*((quat.coeffs()(1))*(quat.coeffs()(1))) + 2.0*((quat.coeffs()(2))*(quat.coeffs()(2))) - 2.0*((quat.coeffs()(3))*(quat.coeffs()(3))));
        F(16,1) = (del_vel(0) - imu_dt*x(STATE_IDX_ABIAS0))*(4.0*quat.coeffs()(0)*quat.coeffs()(3) - 4.0*quat.coeffs()(1)*quat.coeffs()(2)) + (del_vel(2) - imu_dt*x(STATE_IDX_ABIAS2))*(4.0*quat.coeffs()(0)*quat.coeffs()(1) + 4.0*quat.coeffs()(2)*quat.coeffs()(3));
        F(16,2) = (del_vel(0) - imu_dt*x(STATE_IDX_ABIAS0))*(-2.0*((quat.coeffs()(0))*(quat.coeffs()(0))) + 2.0*((quat.coeffs()(1))*(quat.coeffs()(1))) - 2.0*((quat.coeffs()(2))*(quat.coeffs()(2))) + 2.0*((quat.coeffs()(3))*(quat.coeffs()(3)))) + (del_vel(1) - imu_dt*x(STATE_IDX_ABIAS1))*(-4.0*quat.coeffs()(0)*quat.coeffs()(1) - 4*quat.coeffs()(2)*quat.coeffs()(3));
        F(16,9) = -imu_dt*(2.0*quat.coeffs()(0)*quat.coeffs()(1) + 2.0*quat.coeffs()(2)*quat.coeffs()(3));
        F(16,10) = -imu_dt*(-((quat.coeffs()(0))*(quat.coeffs()(0))) + 1.0*((quat.coeffs()(1))*(quat.coeffs()(1))) - ((quat.coeffs()(2))*(quat.coeffs()(2))) + ((quat.coeffs()(3))*(quat.coeffs()(3))));
        F(16,11) = -imu_dt*(-2.0*quat.coeffs()(0)*quat.coeffs()(3) + 2.0*quat.coeffs()(1)*quat.coeffs()(2));
        F(16,16) = 1;
        F(17,0) = (del_vel(1) - imu_dt*x(STATE_IDX_ABIAS1))*(-2.0*((quat.coeffs()(0))*(quat.coeffs()(0))) - 2.0*((quat.coeffs()(1))*(quat.coeffs()(1))) + 2.0*((quat.coeffs()(2))*(quat.coeffs()(2))) + 2.0*((quat.coeffs()(3))*(quat.coeffs()(3)))) + (del_vel(2) - imu_dt*x(STATE_IDX_ABIAS2))*(-4*quat.coeffs()(0)*quat.coeffs()(3) - 4.0*quat.coeffs()(1)*quat.coeffs()(2));
        F(17,1) = (del_vel(0) - imu_dt*x(STATE_IDX_ABIAS0))*(2.0*((quat.coeffs()(0))*(quat.coeffs()(0))) + 2.0*((quat.coeffs()(1))*(quat.coeffs()(1))) - 2.0*((quat.coeffs()(2))*(quat.coeffs()(2))) - 2.0*((quat.coeffs()(3))*(quat.coeffs()(3)))) + (del_vel(2) - imu_dt*x(STATE_IDX_ABIAS2))*(4.0*quat.coeffs()(0)*quat.coeffs()(2) - 4*quat.coeffs()(1)*quat.coeffs()(3));
        F(17,2) = (del_vel(0) - imu_dt*x(STATE_IDX_ABIAS0))*(4.0*quat.coeffs()(0)*quat.coeffs()(3) + 4.0*quat.coeffs()(1)*quat.coeffs()(2)) + (del_vel(1) - imu_dt*x(STATE_IDX_ABIAS1))*(-4.0*quat.coeffs()(0)*quat.coeffs()(2) + 4.0*quat.coeffs()(1)*quat.coeffs()(3));
        F(17,9) = -imu_dt*(2.0*quat.coeffs()(0)*quat.coeffs()(2) - 2.0*quat.coeffs()(1)*quat.coeffs()(3));
        F(17,10) = -imu_dt*(2.0*quat.coeffs()(0)*quat.coeffs()(3) + 2.0*quat.coeffs()(1)*quat.coeffs()(2));
        F(17,11) = -imu_dt*(-((quat.coeffs()(0))*(quat.coeffs()(0))) - ((quat.coeffs()(1))*(quat.coeffs()(1))) + 1.0*((quat.coeffs()(2))*(quat.coeffs()(2))) + ((quat.coeffs()(3))*(quat.coeffs()(3))));
        F(17,17) = 1;
    } //////// End generated code: Prediction model        ////////


    auto x_clocks = x.bottomRightCorner<EKF_NUM_CLOCK_STATES,1>();
    auto f_clocks = f.bottomRightCorner<EKF_NUM_CLOCK_STATES,1>();
    auto F_clocks = F.bottomRightCorner<EKF_NUM_CLOCK_STATES,EKF_NUM_CLOCK_STATES>();


    for (uint8_t i=0; i<EKF_NUM_CLOCK_STATES; i++) {
        if (_clock_state_map[i].type == STATE_TYPE_TSCA_ERR) {
            f_clocks(i,0) = x_clocks(i,0);
            F_clocks(i,i) = 1;
        } else if (_clock_state_map[i].type == STATE_TYPE_TOFS) {
            f_clocks(i) = x_clocks(i);
            F_clocks(i,i) = 1;
            clock_state_idx_t tsca_idx = _clock_state_map[i].tsca_idx;
            if (tsca_idx != STATE_IDX_NONE) {
                assert(is_valid_time_scale_state(tsca_idx));
                f_clocks(i) += uwb_dt * x_clocks(tsca_idx);
                F_clocks(i,tsca_idx) = uwb_dt;
            }
        }
    }

//     cout << "del_vel " << del_vel.transpose() << endl << endl;
//     cout << "(f-x)/imu_dt" << endl << (f-x)/imu_dt << endl << endl;

    P = F*P*F.transpose();

    addProcessNoise(uwb_dt, imu_dt, del_ang, del_vel);

    x = f;

    // Symmetrificate
//     P = (P+P.transpose())*0.5;
    zeroRotErr();
}

bool UWBEKF::fuse_uwb(const UWBObservation& obs, float& pdf_ret) {
    cout << endl << "######## UPDATE BEGIN #########" << endl;

    auto tx_tofs_idx = obs.tx_tofs_idx;
    if (tx_tofs_idx != STATE_IDX_NONE && !is_valid_time_offset_state(tx_tofs_idx)) {
//         cout << "tx idx invalid" << endl;
        return false;
    }

    float tx_timestamp_var = SQ(_params.tx_timestamp_sigma_m/SPEED_OF_LIGHT);
    float rx_timestamp_var = SQ(_params.rx_timestamp_sigma_m/SPEED_OF_LIGHT);

    // Construct z, h, R, and H
    MatrixOx1 _z;
    MatrixOx1 _h;
    MatrixOxO _R = MatrixOxO::Ones() * tx_timestamp_var + MatrixOxO::Identity() * rx_timestamp_var * 2;
    MatrixOxS _H = MatrixOxS::Zero();
    auto _H_clocks = _H.rightCols<EKF_NUM_CLOCK_STATES>();
    auto x_clocks = x.bottomRows<EKF_NUM_CLOCK_STATES>();
    const auto& tx_pos = obs.tx_pos;
    const auto& tx_ts = obs.tx_timestamp;

    uint8_t obs_count = 0;
    for (size_t i=0; i<obs.num_rx_timestamps; i++) {
        const uint8_t obs_idx = obs_count;

        auto rx_tofs_idx = obs.rx_timestamps[i].rx_tofs_idx;
        if (rx_tofs_idx != STATE_IDX_NONE && !is_valid_time_offset_state(rx_tofs_idx)) {
//             cout << "invalid rx_tofs_idx: " << (int)rx_tofs_idx << endl;
            continue;
        }

        // NOTE: if z is large, float precision will be an issue
        // We keep a separate int64 time offset and apply it prior to converting to float - z should be near-zero in the nominal case.

        const auto& rx_ts = obs.rx_timestamps[i].rx_timestamp;
//         cout << "rx stamp " << obs.rx_timestamps[i].rx_timestamp << endl;
//         cout << "tx stamp " << obs.tx_timestamp << endl;

        int64_t diff = (rx_ts-tx_ts)&0xffffffffff;
        if (diff & 0x8000000000) { // sign-extend
            diff |= 0xFFFFFF0000000000;
        }

        _z(obs_idx) = diff * DW1000_TICK_SECONDS;
        assert(fabs(_z(obs_idx)) < 100000);

        const auto& rx_pos = obs.rx_timestamps[i].rx_pos;

        // Compute h and H without time offset states
        if (obs.tx_is_tag) {
            const auto& other_pos = rx_pos;
            auto pos = x.block<3,1>(STATE_IDX_POS0,0);

            _h(obs_idx) = (pos-other_pos).norm()/SPEED_OF_LIGHT;
            _H.block<1,3>(obs_idx,STATE_IDX_POS0) = (pos-other_pos).transpose()/(SPEED_OF_LIGHT*(pos-other_pos).norm());
        } else if (obs.rx_timestamps[i].rx_is_tag) {
            const auto& other_pos = tx_pos;
            auto pos = x.block<3,1>(STATE_IDX_POS0,0);

            _h(obs_idx) = (pos-other_pos).norm()/SPEED_OF_LIGHT;
            _H.block<1,3>(obs_idx,STATE_IDX_POS0) = (pos - other_pos).transpose()/(SPEED_OF_LIGHT*(pos-other_pos).norm());
        } else {
            _h(obs_idx) = (rx_pos - tx_pos).norm()/SPEED_OF_LIGHT;
        }

        // NOTE: the generated code does not have time offset, scale factor or antenna delay states. These are
        // done by hand to allow flexibility in extending the EKF to support a larger numbers of nodes.
        if (rx_tofs_idx != STATE_IDX_NONE) {
            _h(obs_idx) += x_clocks(rx_tofs_idx);
            _H_clocks(obs_idx, rx_tofs_idx) = 1;
        }

        if (tx_tofs_idx != STATE_IDX_NONE) {
            _h(obs_idx) -= x_clocks(tx_tofs_idx);
            _H_clocks(obs_idx, tx_tofs_idx) = -1;
        }

        obs_count++;
    }

    if (obs_count == 0) {
        cout << "update fail: no observations" << endl;
        return false;
    }


    auto H = _H.topLeftCorner(obs_count,N_STATES);
    auto R = _R.topLeftCorner(obs_count,obs_count);
    auto z = _z.topLeftCorner(obs_count,1);
    auto h = _h.topLeftCorner(obs_count,1);

    // TODO figure out how to optimize this to consider H's sparseness. Or will the optimizer do it for us?
    MatrixOxO _S;
    auto S = _S.topLeftCorner(obs_count,obs_count);
    S.noalias() = H*P*H.transpose() + R;

//     cout << "S_AL" << endl << S << endl << endl;

    MatrixOxO _S_I;
    auto S_I = _S_I.topLeftCorner(obs_count,obs_count);

    auto S_solver = S.ldlt();
    S_I = S_solver.solve(MatrixOxO::Identity().topLeftCorner(obs_count,obs_count));

    if (S_solver.info() != Eigen::Success) {
//         cout << "llt failed" << endl;
//         cout << "H" << endl << H << endl << endl;
//         cout << "S" << endl << S << endl << endl;
        return false;
    }


    MatrixOx1 _y;
    auto y = _y.topRows(obs_count);
    y.noalias() = z-h;

//     cout << "H" << endl << H << endl << endl;
//     cout << "R" << endl << R << endl << endl;
//     cout << "P" << endl << P << endl << endl;
//     cout << "P-P.T" << endl << P-P.transpose() << endl << endl;
//     cout << "z" << endl << z << endl << endl;
//     cout << "h" << endl << h << endl << endl;
//     cout << "H" << endl << H << endl << endl;
//     cout << "y" << endl << y << endl << endl;
//     cout << "S" << endl << S << endl << endl;

    auto NIS = y.transpose()*S_I*y;

    float S_det = S_solver.vectorD().prod();
    pdf_ret = exp(-0.5*(NIS[0]))/sqrt(pow(2*M_PI, obs_count)*S_det);

    MatrixSxO _K;
    auto K = _K.leftCols(obs_count);
    K.noalias() = P*H.transpose()*S_I;
//     cout << "K_AL" << endl << K << endl << endl;
    auto temp = (MatrixSxS::Identity()-K*H).eval();

    auto P_n = (temp*P*temp.transpose() + K*R*K.transpose()).eval();


    for (uint8_t i=0; i<N_STATES; i++) {
        if (P_n(i,i) < 0) {
//             cout << "badly conditioned covariance" << endl;
//             cout << "P_n" << endl << P_n << endl << endl;
            return false;
        }
    }

    x.noalias() += K*y;
    P = P_n;

    // Symmetrificate
//     MatrixSxS L = P.ldlt().matrixL();
//     MatrixSx1 D = P.ldlt().vectorD();
//     P = L*D.asDiagonal()*L.transpose();
    P = (P+P.transpose())*0.5;
    for (size_t i=0; i<N_STATES; i++) {
        for (size_t j=i+1; j<N_STATES; j++) {
            P(j,i) = P(i,j);
        }
    }

    zeroRotErr();

    return true;
}

// bool UWBEKF::fuse_pos_element_ecef(const double pos_obs, const float obs_var, uint8_t element_idx, float& pdf_ret) {
//     cout << endl << "######## UPDATE BEGIN #########" << endl;
//
//     // Construct z, h, R, and H
//     float z = pos_obs-_ecef_origin(element_idx);
//
//     float h = x(STATE_IDX_POS0+element_idx,0);
//
//     float R = obs_var;
//
//     Matrix1xS _H = Matrix1xS::Zero();
//     _H(0,STATE_IDX_POS0+element_idx) = 1;
//     auto H = _H.topLeftCorner(1,N_STATES);
//
//     // TODO figure out how to optimize this to consider H's sparseness. Or will the optimizer do it for us?
//     float S = H*P*H.transpose() + R;
//
//     float S_I = 1.0f/S;
//
//     float y = z-h;
//
//     float NIS = S_I*y*y;
//
//     float S_det = S;
//     pdf_ret = exp(-0.5*(NIS))/sqrt(2*M_PI*S_det);
//
//     MatrixSx1 K;
//     K.noalias() = P*H.transpose()*S_I;
//
//     auto subx1 = (MatrixSxS::Identity()-K*H).eval();
//     auto P_n = (subx1*P*subx1.transpose() + K*R*K.transpose()).eval();
//
//
//     for (uint8_t i=0; i<N_STATES; i++) {
//         if (P_n(i,i) < 0) {
// //             cout << "badly conditioned covariance" << endl;
// //             cout << "P_n" << endl << P_n << endl << endl;
//             return false;
//         }
//     }
//
//     x.noalias() += K*y;
//     P = P_n;
//
//     // Symmetrificate
// //     MatrixSxS L = P.ldlt().matrixL();
// //     MatrixSx1 D = P.ldlt().vectorD();
// //     P = L*D.asDiagonal()*L.transpose();
//     for (size_t i=0; i<N_STATES; i++) {
//         for (size_t j=i+1; j<N_STATES; j++) {
//             float temp = (P(j,i)+P(i,j))*0.5f;
//             P(j,i) = P(i,j) = temp;
//         }
//     }
//
//     zeroRotErr();
//
//     return true;
// }
//
// bool UWBEKF::fuse_vel_element_ecef(const float vel_obs, const float obs_var, uint8_t element_idx, float& pdf_ret) {
//     cout << endl << "######## UPDATE BEGIN #########" << endl;
//
//     // Construct z, h, R, and H
//     float z = vel_obs;
//
//     float h = x(STATE_IDX_VEL0+element_idx,0);
//
//     float R = obs_var;
//
//     Matrix1xS _H = Matrix1xS::Zero();
//     _H(0,STATE_IDX_VEL0+element_idx) = 1;
//     auto H = _H.topLeftCorner(1,N_STATES);
//
//     // TODO figure out how to optimize this to consider H's sparseness. Or will the optimizer do it for us?
//     float S = H*P*H.transpose() + R;
//
//     float S_I = 1.0f/S;
//
//     float y = z-h;
//
//     float NIS = S_I*y*y;
//
//     float S_det = S;
//     pdf_ret = exp(-0.5*(NIS))/sqrt(2*M_PI*S_det);
//
//     MatrixSx1 K;
//     K.noalias() = P*H.transpose()*S_I;
//
//     auto subx1 = (MatrixSxS::Identity()-K*H).eval();
//     auto P_n = (subx1*P*subx1.transpose() + K*R*K.transpose()).eval();
//
//
//     for (uint8_t i=0; i<N_STATES; i++) {
//         if (P_n(i,i) < 0) {
// //             cout << "badly conditioned covariance" << endl;
// //             cout << "P_n" << endl << P_n << endl << endl;
//             return false;
//         }
//     }
//
//     x.noalias() += K*y;
//     P = P_n;
//
//     // Symmetrificate
// //     MatrixSxS L = P.ldlt().matrixL();
// //     MatrixSx1 D = P.ldlt().vectorD();
// //     P = L*D.asDiagonal()*L.transpose();
//     for (size_t i=0; i<N_STATES; i++) {
//         for (size_t j=i+1; j<N_STATES; j++) {
//             float temp = (P(j,i)+P(i,j))*0.5f;
//             P(j,i) = P(i,j) = temp;
//         }
//     }
//
//     zeroRotErr();
//
//     return true;
// }
