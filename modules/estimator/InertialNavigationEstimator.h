#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdint.h>
using namespace std;

#ifndef M_PI
#define M_PI (3.14159265358979323846264338327950288)
#endif

#define SQ(x) ((x)*(x))

#define STATE_IDX_ROT_ERR0 0
#define STATE_IDX_ROT_ERR1 1
#define STATE_IDX_ROT_ERR2 2
#define STATE_IDX_GBIAS0 3
#define STATE_IDX_GBIAS1 4
#define STATE_IDX_GBIAS2 5
#define STATE_IDX_GSCALE0 6
#define STATE_IDX_GSCALE1 7
#define STATE_IDX_GSCALE2 8
#define STATE_IDX_ABIAS0 9
#define STATE_IDX_ABIAS1 10
#define STATE_IDX_ABIAS2 11
#define STATE_IDX_POS0 12
#define STATE_IDX_POS1 13
#define STATE_IDX_POS2 14
#define STATE_IDX_VEL0 15
#define STATE_IDX_VEL1 16
#define STATE_IDX_VEL2 17

#ifndef MAX_NUM_MIXANDS
#define MAX_NUM_MIXANDS 3
#endif

#define N_STATES 18

#define MEM_ALIGN_SIZE 256
#define MIXAND_MEM_ELEMENT_SIZE (MEM_ALIGN_SIZE*((sizeof(Mixand)+MEM_ALIGN_SIZE-1)/MEM_ALIGN_SIZE))

class InertialNavigationEstimator {
public:
    // Typedefs
    class Mixand;
    typedef Eigen::Vector3d Vector3d;
    typedef Eigen::Vector3f Vector3f;
    typedef Eigen::Matrix<float,6,6> Matrix6f;
    typedef Eigen::Matrix<float,6,1> Vector6f;

    typedef Eigen::Matrix<float,N_STATES,1> MatrixSx1;
    typedef Eigen::Matrix<double,N_STATES,1> MatrixSx1d;

    typedef Eigen::Matrix<float,1,N_STATES> Matrix1xS;
    typedef Eigen::Matrix<float,N_STATES,N_STATES> MatrixSxS;

    typedef Eigen::Quaternionf Quaternion;

    typedef struct {
        const float& accel_sigma;
        const float& gyro_sigma;
        const float& gbias_pnoise;
        const float& abias_pnoise_xy;
        const float& abias_pnoise_z;
        const float& gscale_pnoise;
    } Params;

    typedef struct {
        bool success;
        float inlier_probability;
    } FusionResult;

    InertialNavigationEstimator(const Params& _params) : params(_params) {
        for (size_t i=0; i<MAX_NUM_MIXANDS; i++) {
            mixand_pool.free(&mixand_pool_memory[MIXAND_MEM_ELEMENT_SIZE*i]);
        }
    }

    Mixand* get_mixand(const uint32_t i);
    uint32_t get_mixand_count();
    void initialize(const Vector3f& init_accel_vec, const Vector3d& pos_ecef, const Vector3f& vel_ecef);
    void predict(float imu_dt, Vector3f del_ang, Vector3f del_vel);
    bool fuse_pos_vel_ecef(const Vector3d& pos_ecef, const Vector3f& vel_ecef, const Matrix6f& R, float probability_threshold);

    void reduce();

    void compute_outputs();

    Quaternion get_quat_ecef();
    Quaternion get_quat_ned();
    Vector3f get_euler_321_ned();
    Vector3d get_pos_ecef();
    Vector3d get_pos_llh();
    Vector3f get_vel_ecef();
    Vector3f get_vel_ned();

    class Mixand {
    public:
        Mixand(const Vector3d& _ecef_origin, const Vector3d& _lla_origin, const Vector3f& _gravity_vec, const Params& _params) : ecef_origin(_ecef_origin), lla_origin(_lla_origin), gravity_vec(_gravity_vec), params(_params) { }
        void initialize(const Vector3f& init_accel_vec, const Vector3f& vel_ecef, float heading, float heading_sigma);

        void pos_origin_changed(const Vector3f& position_origin_delta);
        void get_state_and_covariance_different_quat(const Quaternion& new_quat, MatrixSx1& x_ret, MatrixSxS& P_ret);
        void get_heading_and_heading_sigma(float& heading, float& heading_sigma);

        void stage_predict(float imu_dt, const Vector3f& del_ang, const Vector3f& del_vel);
        void stage_pos_vel_fusion(const Vector6f& posvel_obs, const Matrix6f& R);

        void commit_staged_operation() {
            state_idx = (state_idx+1)&1;
            update_maps();
        }

        double weight;
        float initial_heading;

        // Intermediates that we may not want to re-compute

        Mixand* next;

        typedef struct {
            bool success;
            float pdf;
            float NIS;
        } MixandFusionResult;

        MixandFusionResult fusion_result {};

        Eigen::Map<MatrixSx1> x{_x[0].data()};
        Eigen::Map<MatrixSxS> P{_P[0].data()};
        Eigen::Map<Quaternion> quat{_quat[0].coeffs().data()};

    private:
        MatrixSx1 _x[2];
        MatrixSxS _P[2];
        Quaternion _quat[2];

        uint8_t state_idx {};

        Eigen::Map<MatrixSx1> x_n{_x[1].data()};
        Eigen::Map<MatrixSxS> P_n{_P[1].data()};
        Eigen::Map<Quaternion> quat_n{_quat[1].coeffs().data()};

        void addProcessNoise(float imu_dt, Vector3f del_ang, Vector3f del_vel);
        void zero_rot_err();
        void periodic_task();
        const Vector3d& ecef_origin;
        const Vector3d& lla_origin;
        const Vector3f& gravity_vec;
        const Params& params;

        void update_maps() {
            new (&x) Eigen::Map<MatrixSx1>(_x[state_idx].data());
            new (&P) Eigen::Map<MatrixSx1>(_P[state_idx].data());
            new (&quat) Eigen::Map<Quaternion>(_quat[state_idx].coeffs().data());

            new (&x_n) Eigen::Map<MatrixSx1>(_x[(state_idx+1)&1].data());
            new (&P_n) Eigen::Map<MatrixSxS>(_P[(state_idx+1)&1].data());
            new (&quat_n) Eigen::Map<Quaternion>(_quat[(state_idx+1)&1].coeffs().data());
        }

        uint32_t dbg_count;
    };

    Vector3d ecef_origin;
    Vector3d lla_origin;
    MatrixSx1 x;

private:
    void set_ecef_origin(Vector3d new_ecef_origin);

    Mixand* create_mixand();
    void delete_mixand(const uint32_t i);

    bool process_fusion_results(float probability_threshold);
    bool select_reduce(int32_t& i_ret, int32_t& j_ret, float& kld_ret);
    void reduce(int32_t i, int32_t j);

    template <typename T>
    class MemoryPool {
    public:
        T* alloc() {
            if (!mixand_pool_head) {
                return nullptr;
            }

            T* ret = mixand_pool_head;
            mixand_pool_head = *reinterpret_cast<T**>(mixand_pool_head);
            return ret;
        }
        void free(void* element) {
            *reinterpret_cast<T**>(element) = mixand_pool_head;
            mixand_pool_head = reinterpret_cast<T*>(element);
        }
    private:
        T* mixand_pool_head {};
    };


    MemoryPool<Mixand> mixand_pool;
    uint8_t mixand_pool_memory[MIXAND_MEM_ELEMENT_SIZE*MAX_NUM_MIXANDS] __attribute__((aligned(MEM_ALIGN_SIZE)));
    Mixand* mixand_list_head {};

    Params params;
    Vector3f gravity_vec;

};
