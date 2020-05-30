// #define EIGEN_NO_MALLOC
// #define EIGEN_RUNTIME_NO_MALLOC
// #define EIGEN_UNROLLING_LIMIT 0
// #define EIGEN_NO_DEBUG
// #define EIGEN_MALLOC_ALREADY_ALIGNED true

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <stdint.h>

#define FTYPE float

#define EKF_NUM_CLOCK_STATES 13

#define MAX_OBS_RX_TIMESTAMPS 5

#define SPEED_OF_LIGHT ((FTYPE)299792458.0)

#define DW1000_TICK_SECONDS ((FTYPE)(1.0/63897600000.0))

#define DW1000_TIME_SUBTRACT(a,b) (((int64_t)((a)-(b)))&0xffffffffff)
#define SECONDS_TO_DW1000_TIME(x) (x/DW1000_TICK_SECONDS)

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

#define CLOCK_STATE_START_IDX 18

#define N_STATES EKF_NUM_CLOCK_STATES+CLOCK_STATE_START_IDX
class UWBEKF {
public:
    // Typedefs
    typedef Eigen::Vector3d Vector3d;
    typedef Eigen::Vector3f Vector3f;
    typedef Eigen::Matrix<float,MAX_OBS_RX_TIMESTAMPS,1> MatrixOx1;
    typedef Eigen::Matrix<float,MAX_OBS_RX_TIMESTAMPS,MAX_OBS_RX_TIMESTAMPS> MatrixOxO;
    typedef Eigen::Matrix<float,MAX_OBS_RX_TIMESTAMPS,N_STATES> MatrixOxS;
    typedef Eigen::Matrix<float,N_STATES,MAX_OBS_RX_TIMESTAMPS> MatrixSxO;
    typedef Eigen::Matrix<float,N_STATES,1> MatrixSx1;
    typedef Eigen::Matrix<float,1,N_STATES> Matrix1xS;
    typedef Eigen::Matrix<float,N_STATES,N_STATES> MatrixSxS;
    typedef Eigen::Matrix<float,EKF_NUM_CLOCK_STATES,1> MatrixCx1;
    typedef Eigen::Quaterniond Quaternion;
    typedef int8_t clock_state_idx_t;

    typedef enum {
        STATE_TYPE_NONE = 0,
        STATE_TYPE_TSCA_ERR,
        STATE_TYPE_TOFS
    } StateType;

    typedef enum {
        NUM_PARAM_IDS
    } ParamIdentifier;

    class ParamContainer {
    public:
        virtual float get_param(ParamIdentifier id) = 0;
    };

    typedef struct {
        float tx_timestamp_sigma_m;
        float rx_timestamp_sigma_m;
        float tsca_err_pnoise;
        float tofs_pnoise;
        float accel_sigma;
        float gyro_sigma;
        float gbias_pnoise;
        float abias_pnoise_xy;
        float abias_pnoise_z;
        float gscale_pnoise;
    } Params;

    typedef struct {
        StateType type;
        clock_state_idx_t tsca_idx;
    } StateInfo;

    typedef struct {
        clock_state_idx_t rx_tofs_idx;
        Vector3f rx_pos;
        bool rx_is_tag;
        uint64_t rx_timestamp;
    } UWBObservationRxTimestamp;

    typedef struct {
        clock_state_idx_t tx_tofs_idx;
        Vector3f tx_pos;
        uint64_t tx_timestamp;
        bool tx_is_tag;
        uint8_t num_rx_timestamps;
        UWBObservationRxTimestamp rx_timestamps[MAX_OBS_RX_TIMESTAMPS];
    } UWBObservation;

    // Constants
    static const clock_state_idx_t STATE_IDX_NONE = -1;

    UWBEKF(Params params) : _params(params) {}

    void initialize(const Vector3f& init_accel_vec, const Vector3d& ecef_pos, const Vector3f& ecef_vel, float yaw_angle, float yaw_sigma);
    void predict(float uwb_dt, float imu_dt, Vector3f del_ang, Vector3f del_vel);
    bool fuse_uwb(const UWBObservation& obs, float& pdf_ret);
//     bool fuse_pos_element_ecef(const double pos_obs, const float obs_var, uint8_t element_idx, float& pdf_ret);
//     bool fuse_vel_element_ecef(const float vel_obs, const float obs_var, uint8_t element_idx, float& pdf_ret);

    clock_state_idx_t add_clock_scale_state();
    clock_state_idx_t add_clock_offset_state(clock_state_idx_t clock_scale_state, float init_sigma);

    void print_info();

    const MatrixSx1& get_state() {
        return x;
    }

    const MatrixSxS& get_covariance() {
        return P;
    }

    void get_state_and_covariance_different_quat(const Quaternion& new_quat, MatrixSx1& x_ret, MatrixSxS& P_ret);
    void zeroRotErr();

    const Quaternion& get_quat() {
        return quat;
    }

    Eigen::Vector3d get_pos() {
        return (Eigen::Vector3d)x.block<3,1>(STATE_IDX_POS0,0).cast<double>();
    }

    float get_heading() {
        const float& qi = quat.coeffs()(0);
        const float& qj = quat.coeffs()(1);
        const float& qk = quat.coeffs()(2);
        const float& qr = quat.coeffs()(3);
        return atan2(2*(qr*qk+qi*qj), 1-2*(qj*qj+qk*qk));
    }
    float get_heading_sigma();

    // EKF state
    MatrixSx1 x;
    MatrixSxS P;
    Quaternion quat;

private:
    Vector3d _ecef_origin;
    StateInfo _clock_state_map[EKF_NUM_CLOCK_STATES] {};
    uint64_t _clock_offset[EKF_NUM_CLOCK_STATES] {};

    bool is_valid_time_offset_state(clock_state_idx_t idx);
    bool is_valid_time_scale_state(clock_state_idx_t idx);

    void addProcessNoise(float uwb_dt, float imu_dt, Vector3f del_ang, Vector3f del_vel);

    void update_clock_offset();

    Params _params;
};
