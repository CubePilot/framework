#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC optimize("no-single-precision-constant")
#pragma GCC optimize("O3")

#include "InertialNavigationEstimator.h"
#include <math.h>
#include "geodesy.h"

#include <iostream>
using namespace std;

void InertialNavigationEstimator::set_ecef_origin(Vector3d new_ecef_origin) {
    Vector3f origin_delta = (new_ecef_origin-ecef_origin).cast<float>();

    // Update the ECEF origin
    ecef_origin = new_ecef_origin;

    // Update the LLA origin
    lla_origin = ecef2lla(ecef_origin);

    // Update the gravity vector
    gravity_vec = gravity_ecef(ecef_origin.cast<float>(), Vector3f::Zero());

    // Notify all mixands of ECEF origin update
    Mixand* mixand = mixand_list_head;
    while (mixand) {
        mixand->pos_origin_changed(origin_delta);
        mixand = mixand->next;
    }
}

InertialNavigationEstimator::Mixand* InertialNavigationEstimator::create_mixand() {
    Mixand* ret = mixand_pool.alloc();
    if (!ret) {
        return NULL;
    }

    new (ret)Mixand(ecef_origin, lla_origin, gravity_vec, params);

    ret->next = mixand_list_head;
    mixand_list_head = ret;

    return ret;
}

InertialNavigationEstimator::Mixand* InertialNavigationEstimator::get_mixand(uint32_t i) {
    uint32_t count = 0;
    Mixand* mixand_ptr = mixand_list_head;
    while (mixand_ptr) {
        if (count == i) {
            return mixand_ptr;
        }
        count++;
        mixand_ptr = mixand_ptr->next;
    }
    return nullptr;
}

void InertialNavigationEstimator::delete_mixand(const uint32_t i) {
    uint32_t count = 0;
    Mixand** delete_ptr = &mixand_list_head;
    while (*delete_ptr) {
        if (count == i) {
            (*delete_ptr)->~Mixand();
            mixand_pool.free(*delete_ptr);
            *delete_ptr = (*delete_ptr)->next;
            return;
        }
        count++;
        delete_ptr = &(*delete_ptr)->next;
    }
}

uint32_t InertialNavigationEstimator::get_mixand_count() {
    uint32_t count = 0;
    Mixand* mixand_ptr = mixand_list_head;
    while (mixand_ptr) {
        count++;
        mixand_ptr = mixand_ptr->next;
    }
    return count;
}

// bool InertialNavigationEstimator::select_split(int& i_ret) {
//     uint32_t mixand_count = get_mixand_count();
//     float maxVal = 0;
//     i_ret = -1;
//
//     if (mixand_count >= MAX_NUM_MIXANDS-1) {
//         return false;
//     }
//
//     for (size_t i=0; i<mixand_count; i++) {
//         float heading, heading_sigma;
//         float val = get_mixand(i)->get_heading_sigma() > 2*M_PI/GSF_MAX_MIXANDS ? get_mixand(i)->weight : 0;
//         if (val > maxVal) {
//             maxVal = val;
//             i_ret = i;
//         }
//     }
//
//     return i_ret != -1;
// }
//
// void InertialNavigationEstimator::split() {
//     int i;
//     if (!select_split(i)) {
//         return;
//     }
//
//     const auto& x_i = get_mixand(i)->x;
//     const auto& P_i = get_mixand(i)->P;
//     const auto& w_i = get_mixand(i)->x->weight;
//
//     SelfAdjointEigenSolver<MatrixSxS> eigenSolver(P_i);
//
//     if (eigenSolver.info() != Eigen::Success) {
//         return;
//     }
//
//     auto& nu = eigenSolver.eigenvectors();
//     auto& lambda = eigenSolver.eigenvalues();
//
//     double max_val = 0;
//     int selected_idx = -1;
//
//     for (int i=0; i<lambda.rows(); i++) {
//         double val = lambda(i) * nu.col(i).topRows<6>().norm();
//
//         if (val > max_val) {
//             max_val = val;
//             selected_idx = i;
//         }
//     }
//
//     if (selected_idx == -1) {
//         return;
//     }
//
//     double alpha = 0.5;
//     auto x_n_1 = (x_i + sqrt(lambda(selected_idx))*alpha*nu.col(selected_idx)).eval();
//     auto x_n_2 = (x_i - sqrt(lambda(selected_idx))*alpha*nu.col(selected_idx)).eval();
//     auto P_n = (P_i - lambda(selected_idx)*alpha*nu.col(selected_idx)*nu.col(selected_idx).transpose()).eval();
//     auto w_n = w_i/2;
//
//     for (int k=0; k<P_n.rows();k++) {
//         if (P_n(k,k) < 0) {
//             P_n(k,k) = 0;
//         }
//     }
//
// //     assert(P_n.trace() < P_i.trace());
//
//     for (size_t i=0; i<_mixand_list.size(); i++) {
//         if (_mixand_list[i]->weight == 0) {
//             _mixand_list[i]->x = x_n_2;
//             _mixand_list[i]->P = P_n;
//             _mixand_list[i]->weight = w_n;
//             _mixand_list[i]->quat = _mixand_list[split_idx]->quat;
//             _mixand_list[i]->collapsed_to = -1;
//             _mixand_list[split_idx]->x = x_n_1;
//             _mixand_list[split_idx]->P = P_n;
//             _mixand_list[split_idx]->weight = w_n;
//
//             _mixand_list[split_idx]->zeroRotErr();
//             _mixand_list[i]->zeroRotErr();
//             break;
//         }
//     }
// }


bool InertialNavigationEstimator::select_reduce(int32_t& i_ret, int32_t& j_ret, float& kld_ret) {
    uint32_t mixand_count = get_mixand_count();
    i_ret = -1;
    j_ret = -1;
    kld_ret = numeric_limits<float>::infinity();

    for (size_t i=0; i<mixand_count; i++) {
        for (size_t j=i+1; j<mixand_count; j++) {
            Mixand* mixand_i = get_mixand(i);
            Mixand* mixand_j = get_mixand(j);

            float w_i = mixand_i->weight;
            float w_j = mixand_j->weight;
            float w_ij = w_i+w_j;
            float w_i_ij = w_i/w_ij;
            float w_j_ij = w_j/w_ij;
            float x_i, P_i;
            mixand_i->get_heading_and_heading_sigma(x_i, P_i);
            P_i = SQ(P_i);
            float x_j, P_j;
            mixand_j->get_heading_and_heading_sigma(x_j, P_j);
            P_j = SQ(P_j);

            auto P_ij = w_i_ij*P_i + w_j_ij*P_j + w_i_ij*w_j_ij*(x_i-x_j)*(x_i-x_j);
            float kld = 0.5 * (w_ij*log(P_ij) - w_i*log(P_i) - w_j*log(P_j));

            if (kld < kld_ret) {
                kld_ret = kld;
                i_ret = i;
                j_ret = j;
            }
        }
    }

    return i_ret != -1;
}

void InertialNavigationEstimator::reduce(int32_t i, int32_t j) {
    Mixand* mixand_i = get_mixand(i);
    Mixand* mixand_j = get_mixand(j);

    MatrixSx1 x_i;
    MatrixSxS P_i;
    MatrixSx1 x_j;
    MatrixSxS P_j;

    float w_i = mixand_i->weight;
    float w_j = mixand_j->weight;

    Quaternion common_quat = mixand_i->quat.slerp(0.5, mixand_j->quat);

    mixand_i->get_state_and_covariance_different_quat(common_quat, x_i, P_i);
    mixand_j->get_state_and_covariance_different_quat(common_quat, x_j, P_j);

    float w_ij = w_i+w_j;
    float w_i_ij = w_i/w_ij;
    float w_j_ij = w_j/w_ij;

    MatrixSx1 x_ij;
    MatrixSxS P_ij;

    x_ij.noalias() = w_i_ij*x_i+w_j_ij*x_j;
    P_ij.noalias() = w_i_ij*P_i + w_j_ij*P_j + w_i_ij*w_j_ij*(x_i-x_j)*(x_i-x_j).transpose();

    for (int k=0; k<P_ij.rows();k++) {
        if (P_ij(k,k) < 0) {
            P_ij(k,k) = 0;
        }
    }

    mixand_i->weight = w_ij;
    mixand_i->x = x_ij;
    mixand_i->P = P_ij;
    mixand_i->quat = common_quat;

    delete_mixand(j);
}

void InertialNavigationEstimator::reduce() {
    int i,j;
    float kld;

    if (select_reduce(i,j,kld) && kld < 1e-3) {
        reduce(i,j);
    }
}

void InertialNavigationEstimator::compute_outputs() {
    x.setZero();
    Mixand* mixand = mixand_list_head;
    while (mixand) {
        x += mixand->x * mixand->weight;
        mixand = mixand->next;
    }

    set_ecef_origin(ecef_origin+x.block<3,1>(STATE_IDX_POS0,0).cast<double>());
}

void InertialNavigationEstimator::initialize(const Vector3f& init_accel_vec, const Vector3d& pos_ecef, const Vector3f& vel_ecef) {
    while(mixand_list_head) {
        Mixand* free_ptr = mixand_list_head;
        mixand_list_head = mixand_list_head->next;
        free_ptr->~Mixand();
        mixand_pool.free(free_ptr);
    }

    set_ecef_origin(pos_ecef);

    Mixand* mixand;
    size_t i=0;
    while (i < MAX_NUM_MIXANDS && (mixand = create_mixand())) {
        mixand->weight = 1.0/MAX_NUM_MIXANDS;
        mixand->initialize(init_accel_vec, vel_ecef, i*2*M_PI/MAX_NUM_MIXANDS + 60*M_PI/180, 4*M_PI/MAX_NUM_MIXANDS);
        i++;
    }
}

void InertialNavigationEstimator::predict(float imu_dt, Vector3f del_ang, Vector3f del_vel) {
    x.setZero();

    Mixand* mixand = mixand_list_head;
    while (mixand) {
        mixand->stage_predict(imu_dt, del_ang, del_vel);
        mixand->commit_staged_operation();
        x += mixand->x * mixand->weight;
        mixand = mixand->next;
    }
}

bool InertialNavigationEstimator::process_fusion_results(float probability_threshold) {
    bool success = false;
    float innovation_inlier_probability = 0;
    double pdf_sum = 0;
    double mixand_weight_sum = 0;

    Mixand* mixand = mixand_list_head;
    while (mixand) {
        if (mixand->fusion_result.success) {
            innovation_inlier_probability += mixand->weight * (1+erf(-sqrt(mixand->fusion_result.NIS)));
            pdf_sum += mixand->fusion_result.pdf;
            success = true;
        }
        mixand = mixand->next;
    }

    static int count;
    static int rej_count;
    static double probability_sum;
    count++;
    probability_sum += innovation_inlier_probability;
    cout << probability_sum/count << " " << rej_count << endl;

    if (!success || innovation_inlier_probability < probability_threshold) {
        rej_count++;
        // Discard this measurement
        return false;
    }

    // Apply fusion
    mixand = mixand_list_head;
    while (mixand) {
        if (mixand->fusion_result.success) {
            mixand->commit_staged_operation();
            mixand->weight *= mixand->fusion_result.pdf / pdf_sum;
        }
        mixand_weight_sum += mixand->weight;
        mixand = mixand->next;
    }

    // Re-normalize mixand weights
    mixand = mixand_list_head;
    while (mixand) {
        mixand->weight /= mixand_weight_sum;
        mixand = mixand->next;
    }

    return true;
}

bool InertialNavigationEstimator::fuse_pos_vel_ecef(const Vector3d& pos_ecef, const Vector3f& vel_ecef, const Matrix6f& R, float probability_threshold) {
    Vector6f z;
    z.block<3,1>(0,0) = (pos_ecef-ecef_origin).cast<float>();
    z.block<3,1>(3,0) = vel_ecef;

    Mixand* mixand = mixand_list_head;
    while (mixand) {
        mixand->stage_pos_vel_fusion(z, R);
        mixand = mixand->next;
    }

    // NOTE: hard-coded tuning constant
    return process_fusion_results(probability_threshold);
}
