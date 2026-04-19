/**
 * @file exo.hpp
 * @author zzz
 * @brief 
 * @version 0.1
 * @date 2025-12-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef EXO_HPP
#define EXO_HPP

#include <cstdint>
#include "status_led.hpp"
#include "robstride.hpp"
#include "dm_motor.hpp"
#include "force_profile_generator.hpp"
#include "pid.hpp"
#include "disturbance_observer.hpp"
#include "shell.hpp"
#include "utils.h"
#include "mag_encoder.hpp"
#include "dji_esc.hpp"

/** Forward declarations */
class IMUData;
class JointData;
class AnkleData;
class KneeSeaJointData;
class FsrGaitData;
class SideData;
class ExoData;
class AnkleJoint;
class KneeJoint;
class KneeSeaJoint;
class FsrGaitEstimator;
class AdaptiveOscillator;
class Side;
class ExoShell;
class Exo;

/** for bitwise operations */
#include <type_traits>
#define DEFINE_ENUM_CLASS_BITWISE_OPS(EnumType) \
    inline constexpr EnumType operator|(EnumType a, EnumType b) { \
        return static_cast<EnumType>(static_cast<std::underlying_type_t<EnumType>>(a) | static_cast<std::underlying_type_t<EnumType>>(b)); \
    } \
    inline constexpr EnumType& operator|=(EnumType& a, EnumType b) { \
        a = a | b; return a; \
    } \
    inline constexpr bool operator&(EnumType a, EnumType b) { \
        return (static_cast<std::underlying_type_t<EnumType>>(a) & static_cast<std::underlying_type_t<EnumType>>(b)) != 0; \
    } \
    inline constexpr EnumType operator~(EnumType a) { \
        return static_cast<EnumType>(~static_cast<std::underlying_type_t<EnumType>>(a)); \
    } \
    inline constexpr EnumType& operator&=(EnumType& a, EnumType b) { \
        a = static_cast<EnumType>(static_cast<std::underlying_type_t<EnumType>>(a) & static_cast<std::underlying_type_t<EnumType>>(b)); \
        return a; \
    }

/** 应该与NRF54代码中的一致 */
typedef struct foot_sensor_packet_t
{
    int32_t mV_heel;
    int32_t mV_toe;
    float mV_pull;
    float quatI;
    float quatJ;
    float quatK;
    float quatReal;
} foot_sensor_packet_t;

typedef struct exo_sensor_packet_t
{
    foot_sensor_packet_t left_foot;
    foot_sensor_packet_t right_foot;
} exo_sensor_packet_t;

class ImuData
{
public:
    explicit ImuData(bool is_left = true) : is_left_(is_left) {}
    virtual ~ImuData() = default;

    float quat_i_ = 0.0f;
    float quat_j_ = 0.0f;
    float quat_k_ = 0.0f;
    float quat_real_ = 1.0f;

    bool is_left_ = true;
    bool is_used_ = false;
    bool is_calibrated_ = true;
};

class JointData
{
public:
    explicit JointData(bool is_left = true) : is_left_(is_left) {}
    virtual ~JointData() = default;

    float pos_ref_rad_ = 0.0;       /** 人体关节角度参考 */
    float pos_rad_ = 0.0f;          /** 人体关节角度反馈 */
    float vel_ref_radps_ = 0.0f;    /** 人体关节角速度参考 */
    float vel_radps_ = 0.0f;        /** 人体关节角速度反馈 */
    float tor_interact_ref_Nm_ = 0.0f;  /** 人机交互力矩参考 */
    float tor_interact_Nm_ = 0.0f;
    float tor_ref_Nm_ = 0.0f;       /** 还没想好用于表示什么力矩 */
    float tor_Nm_ = 0.0f;           /** 还没想好用于表示什么力矩 */

    bool is_left_;              /** 表示该数据是左侧关节的还是右侧的 */
    bool is_used_ = false;      /** 表示该关节是否使用 */
    bool is_calibrated_ = true;    /** 上述关节数据暂不需要标定, 先标注为已标定 */
};

class AnkleData : public JointData
{
public:
    explicit AnkleData(bool is_left = true) : JointData(is_left) {}
    virtual ~AnkleData() = default;

    float plantarflexion_force_N_ = 0.0f; 
};

class KneeSeaJointData : public JointData
{
public:
    explicit KneeSeaJointData(bool is_left = true) : JointData(is_left) {}
    virtual ~KneeSeaJointData() = default;

    float pos_slider_mm_ = 0.0f;    /** 滑块位移 */
    float pos_slider_offset_mm_ = 0.0f;  /** 滑块在最大膝伸展(暂且当作0度)时的位置 */
    float vel_slider_mmps_ = 0.0f; /** 滑块线速度 */
    float screw_lead_rad2mm_ = 2.0f/ _2PI;  /** 滑块从旋转到直线位移的系数 */

    float pos_linear_encoder_mm_ = 0.0f; /** 外框位移 */
    float pos_linear_encoder_offset_mm_ = 13587.649414f;  /** 外框在最大膝伸展(暂且当作0度)时的位置, 暂且当作0度 */
    float max_pos_linear_encoder_mm_ = 13636.950195f;   /** 当前最大膝弯曲(暂且当作90度)时的位置 */
    float vel_linear_encoder_mmps_ = 0.0f;   /** 外框线速度 */

    float pos_bias_mm_ = 0.0f;      /** 滑块与外框位移之差 */
    float force_spring_ref_N_ = 0.0f;  /** 弹簧参考力 */
    float force_spring_N_ = 0.0f;      /** 弹簧反馈力(位移差xspring_stiffness_Npmm_) */
    float spring_stiffness_Npmm_ = 2 * 15.637f;   /** 两根弹簧的"理论"刚度 */
};

struct FsrSensorData
{
    static constexpr uint32_t kCalibrationDurationMs = 5000u;
    static constexpr uint8_t kNumRefinementSteps = 7u;
    static constexpr float kSchmittLowerThresholdRefinement = 0.33f;
    static constexpr float kSchmittUpperThresholdRefinement = 0.66f;

    float raw_reading = 0.0f;
    float calibrated_reading = 0.0f;
    float calibration_min = 0.0f;
    float calibration_max = 0.0f;

    float step_max_sum = 0.0f;
    float step_max = 0.0f;
    float step_min_sum = 0.0f;
    float step_min = 0.0f;
    float calibration_refinement_min = 0.0f;
    float calibration_refinement_max = 0.0f;

    float schmitt_lower_threshold_calc_contact = 0.15f; //
    float schmitt_upper_threshold_calc_contact = 0.25f; //

    uint32_t calibration_start_sys_ms = 0;
    uint8_t refinement_step_count = 0;

    bool last_do_calibrate = false;
    bool last_do_refinement = false;
    bool ground_contact_during_refinement = false;
    bool ground_contact = false;
};

class FsrGaitData
{
public:
    explicit FsrGaitData(bool is_left = true) : is_left_(is_left) {}
    virtual ~FsrGaitData() = default;

    static constexpr uint8_t kNumStepsAvg = 3;

    FsrSensorData heel_;
    FsrSensorData toe_;

    uint32_t step_times_[kNumStepsAvg] = {0};
    uint32_t stance_times_[kNumStepsAvg] = {0};
    uint32_t swing_times_[kNumStepsAvg] = {0};

    uint32_t ground_strike_timestamp_ = 0;
    uint32_t prev_ground_strike_timestamp_ = 0;
    uint32_t toe_strike_timestamp_ = 0;
    uint32_t prev_toe_strike_timestamp_ = 0;
    uint32_t toe_off_timestamp_ = 0;
    uint32_t prev_toe_off_timestamp_ = 0;

    float percent_gait_ = -1.0F;
    float percent_stance_ = -1.0f;
    float percent_swing_ = -1.0f;
    float expected_step_duration_ = -1.0f;
    float expected_stance_duration_ = -1.0f;
    float expected_swing_duration_ = -1.0f;
    float expected_duration_window_upper_coeff_ = 1.75;
    float expected_duration_window_lower_coeff_ = 0.25f;

    bool ground_strike_ = false;
    bool toe_strike_ = false;
    bool toe_off_ = false;
    bool toe_on_ = false;
    bool heel_contact_state_ = false;
    bool toe_contact_state_ = false;
    bool prev_heel_contact_state_ = true;
    bool prev_toe_contact_state_ = true;

    bool do_calibration_toe_fsr_ = false;
    bool do_calibration_refinement_toe_fsr_ = false;
    bool do_calibration_heel_fsr_ = false;
    bool do_calibration_refinement_heel_fsr_ = false;

    bool is_left_;
    bool is_used_ = true;
    bool is_calibrated_ = false;
};

class AoData
{
public:
    explicit AoData() = default;
    virtual ~AoData() = default;

    uint32_t left_event_cnt_ = 0;
    uint32_t right_event_cnt_ = 0;
    float left_phi_comp_rad_ = 0.0f;
    float right_phi_comp_rad_ = 0.0f;

    bool is_used_ = true;
};

class SideData
{
public:
    explicit SideData(bool is_left = true) : hip_joint_(is_left), knee_joint_(is_left), knee_sea_joint_(is_left), ankle_joint_(is_left), fsr_gait_data_(is_left), foot_imu_(is_left), is_left_(is_left) {}
    virtual ~SideData() = default;

    JointData hip_joint_;
    JointData knee_joint_;
    KneeSeaJointData knee_sea_joint_;
    AnkleData ankle_joint_;
    FsrGaitData fsr_gait_data_;
    ImuData foot_imu_;

    bool is_left_;
    bool is_used_ = true;
    bool is_calibrated_ = false;
};

class ExoData
{
public:
    /*  9 RGB Color Codes
    const uint8_t kRGBColors[9][3] = {
        {0x00, 0x00, 0x00}, // Off       #000000
        {0x00, 0xFF, 0x00}, // Green     #00FF00
        {0x00, 0x00, 0xFF}, // Blue      #0000FF
        {0xFF, 0xFF, 0x00}, // Yellow    #FFFF00
        {0xFF, 0x00, 0xFF}, // Magenta   #FF00FF
        {0x00, 0xFF, 0xFF}, // Cyan      #00FFFF
        {0xFF, 0x8C, 0x00}, // Orange    #FF8C00
        {0x80, 0x00, 0x80}, // Purple    #800080
        {0xFF, 0x00, 0x00}, // Red       #FF0000
    };
    */
    enum class State : uint8_t
    {
        kSleep = 0U,
        kWaitMotorComm,
        kCalibrating,
        kReady,
        kAssisting,
        kFaultLowBattery,
        kFaultSystem,
    };

    enum class Error : uint32_t
    {
        kNone            = 0 << 0,
        kBatteryLow      = 1 << 0,
        kLeftHipFault    = 1 << 1,
        kRightHipFault   = 1 << 2,
        kLeftKneeFault   = 1 << 3,
        kRightKneeFault  = 1 << 4,
        kLeftAnkleFault  = 1 << 5,
        kRightAnkleFault = 1 << 6,
        kCanBusOff       = 1 << 7,
        kImuFault        = 1 << 8,
    };

    enum class SysEvent : uint32_t
    {
        kNone            = 0 << 0,
        kEmergencyStop   = 1 << 0,
        kWakeup          = 1 << 1,
        kStartCalibrate  = 1 << 2,
        kStartAssist     = 1 << 3,
        kStopAssist      = 1 << 4,
        kEnterSleep      = 1 << 5,
        kClearFaults     = 1 << 6,
    };

    enum class LocoMode : uint8_t
    {
        kSitting = 0,
        kSitToStand,
        kStanding,
        kStandToSit,
        kWalking,
        kRampAscent,
        kRampDescent,
        kStairAscent,
        kStairDescent,
    };

    struct ArbiterOverride {
        LocoMode forced_locomode = LocoMode::kWalking;
        bool enable_locomode_override = false;
    };

    struct TelemetryConfig {
        bool enable = false;
        uint32_t pause_until_ms = 0;
    };

    ExoData() : left_side_(true), right_side_(false) {}
    ~ExoData() = default;

    /**< Two Side */
    SideData left_side_;
    SideData right_side_;
    AoData ao_data_;

    State state_ = State::kSleep;
    Error error_code_ = Error::kNone;
    SysEvent pending_events_ = SysEvent::kNone;
    LocoMode loco_mode_ = LocoMode::kWalking;
    ArbiterOverride override_usr_ = {.forced_locomode = LocoMode::kWalking, .enable_locomode_override = false};
    TelemetryConfig telemetry_config_ = {.enable = false, .pause_until_ms = 0};

    float user_weight_kg_ = 60.0f;
    float battery_voltage_ = 24.0f;
    bool do_test = false;
};

DEFINE_ENUM_CLASS_BITWISE_OPS(ExoData::Error)
DEFINE_ENUM_CLASS_BITWISE_OPS(ExoData::SysEvent)

struct ExoHardware
{
    FDCAN_HandleTypeDef &motor_can;           // 用于大疆/Robstride电机通信
    UART_HandleTypeDef &sensor_uart;          // 用于接收无线传感器数据
    UART_HandleTypeDef &shell_uart;           // 用于命令行 Shell / VOFA 调试
    UART_HandleTypeDef &left_mag_encoder_uart;    // 用于磁栅尺编码器1
    UART_HandleTypeDef &right_mag_encoder_uart;   // 用于磁栅尺编码器2
};

enum ExoJointCanID : uint8_t
{
    kLeftHip = 0x01,
    kRightHip = 0x02,
    kLeftKnee = 0x2A,
    kRightKnee = 0x55,
    kLeftAnkle = 0x2A,
    kRightAnkle = 0x55,
};

class AnkleJoint
{
public:
    explicit AnkleJoint(bool is_left, ExoData &pe)  : pe_(pe), ps_(is_left ? pe_.left_side_ : pe_.right_side_), pj_(is_left ? pe_.left_side_.ankle_joint_ : pe_.right_side_.ankle_joint_), force_profile_generator_(), motor_(is_left ? ExoJointCanID::kLeftAnkle : ExoJointCanID::kRightAnkle), pid_(0.2f, 0.4f, 0.0f, -1.0f, 10.0f) {}
    virtual ~AnkleJoint() = default;
    void Calibrate();
    void Read();
    bool IsMotorConnect();
    void Shutdown();
    void Standby();
    void Assist();

    ExoData &pe_;
    SideData &ps_;
    JointData &pj_;
    AnkleForceProfileGenerator force_profile_generator_;
    Robstride motor_;
    PIDController pid_;

    float cable_pre_tensioned_position_ = 0.0f;
    float cable_tensioned_position_ = 6.0f;
    float cable_released_position_ = 0.0f;
    float assistance_start_phase_percent_ = 35.0f;
    float assistance_end_phase_percent_ = 65.0f;
};

class KneeJoint
{
public:
    explicit KneeJoint(bool is_left, ExoData &pe) : pe_(pe), ps_(is_left ? pe.left_side_ : pe.right_side_), pj_(is_left ? pe.left_side_.knee_joint_ : pe.right_side_.knee_joint_), force_profile_generator_(), motor_(is_left ? ExoJointCanID::kLeftKnee : ExoJointCanID::kRightKnee), disturbance_observer_(5.0f) {}
    virtual ~KneeJoint() = default;
    void Calibrate();
    void Read();
    bool IsMotorConnect();
    void Shutdown();
    void Standby();
    void Assist();

    ExoData &pe_;
    SideData &ps_;
    JointData &pj_;
    KneeForceProfileGenerator force_profile_generator_;
    Robstride motor_;
    DisturbanceObserver disturbance_observer_;
    void ImpedanceControl();
};

class KneeSeaJoint
{
public:
    enum class CtrlMode : uint8_t
    {
        kJointPos,
        kSpringForce,
    };


    explicit KneeSeaJoint(bool is_left,  ExoData &pe, DjiEscHub &dji_esc_hub, UART_HandleTypeDef &huart) : pe_(pe), ps_(is_left ? pe.left_side_ : pe.right_side_), pj_(is_left ? pe.left_side_.knee_sea_joint_ : pe.right_side_.knee_sea_joint_), motor_(dji_esc_hub, is_left ? DjiEsc::EscId::kId1 : DjiEsc::EscId::kId2), mag_encoder_(huart), 
    joint_pos_pid_(5.0f, 1.0f, 0.0f, -100.0f, motor_.max_iqref_amp_),
    spring_force_pid_(1.0f, 0.1, 0.0, -100.0f, motor_.max_iqref_amp_) {}
    virtual ~KneeSeaJoint() = default;

    void Calibrate();
    void Read();
    bool IsMotorConnect();
    void Shutdown();
    void Standby();
    void Assist();

    void JointPosControl();
    void SpringForceControl();

    ExoData &pe_;
    SideData &ps_;
    KneeSeaJointData &pj_;
    KneeForceProfileGenerator force_profile_generator_;
    DjiEsc motor_;   /** 电调ID固定: 左膝id=1, 右膝id=2 */
    MagEncoder mag_encoder_;
    PIDController joint_pos_pid_; /** 必须放在motor_后面, 因为依赖其进行构造 */
    PIDController spring_force_pid_;  /** 必须放在motor_后面, 因为依赖其进行构造 */
    float force_test_sin_freq = 0.1f;
private:
    CtrlMode ctrl_mode_ = CtrlMode::kSpringForce;
};

class HipJoint
{
public:
    explicit HipJoint(bool is_left, ExoData &pe) : pe_(pe), ps_(is_left ? pe_.left_side_ : pe_.right_side_), pj_(is_left ? pe_.left_side_.hip_joint_ : pe_.right_side_.hip_joint_), motor_(is_left ? ExoJointCanID::kLeftHip : ExoJointCanID::kRightHip) {}
    virtual ~HipJoint() = default;
    void Calibrate();
    void Read();
    bool IsMotorConnect();
    void Shutdown();
    void Standby();
    void Assist();
    ExoData &pe_;
    SideData &ps_;
    JointData &pj_;
    DMMotor motor_;
};

class FsrGaitEstimator
{
public:
    explicit FsrGaitEstimator(FsrGaitData &gait_data) : gait_data_(gait_data) {}
    virtual ~FsrGaitEstimator() = default;

    void Calibrate();
    void Update();
    void Reset();
private:
    void ProcessCalibration(FsrSensorData& sensor, bool& do_calibrate, bool& do_refinement);
    void ProcessSensorUpdate(FsrSensorData& sensor);

    float UpdateExpectedDuration();
    float UpdateExpectedStanceDuration();
    float UpdateExpectedSwingDuration();
    static inline bool SchmittTrigger(float value, bool is_last_high, float lower, float upper)
    {
        if (is_last_high)
        {
            return value < lower ? false : true;
        }
        else
        {
            return value > upper ? true : false;
        }
    }

    FsrGaitData &gait_data_;
};

class AdaptiveOscillator
{
public:
    explicit AdaptiveOscillator(ExoData &pe) : pe_(pe) {}
    virtual ~AdaptiveOscillator() = default;

    void Update();
    void Reset();
private:
    ExoData &pe_;

    static constexpr uint64_t kMaxTstrideUs = 3.0 * 1000000;
    static constexpr uint64_t kMinTstrideUs = 0.1 * 1000000;
    static constexpr uint64_t kMaxStoppingDurationUs = 0.5 * 1000000;
    static constexpr float kEmaTauS = 0.2f;
    static constexpr uint8_t kNumAOs = 3;

    uint64_t tprev_sys_us_ = 0;
    uint64_t left_tk_sys_us_ = 0;
    uint64_t right_tk_sys_us_ = 0;

    float v_phi_ = 10.0f;
    float v_omega_= 10.0f;
    float eta_ = 1.0f;
    float kp_ = 1.0f;
    float rho_ = -0.7f;

    float hat_x_ = 0.0f;
    float omega_ = _2PI * 1.0f;
    float phi_[kNumAOs] = {0.0f};
    float alpha_[kNumAOs] = {0.2f};
    float alpha0_ = 0.0f;
    float vel_energy_ema_ = 0.0f;

    float left_Pe_tilde_tk_ = 0.0f;
    float right_Pe_tilde_tk_ = 0.0f;
    float left_epsilon_phi_tk_ = 0.0f;
    float right_epsilon_phi_tk_ = 0.0f;
    float left_phi_e_ = 0.0f;
    float right_phi_e_ = 0.0f;
};

class Side
{
public:
    explicit Side(bool is_left, ExoData &pe, DjiEscHub &dji_esc_hub, UART_HandleTypeDef &huart) : pe_(pe), ps_(is_left ? pe_.left_side_ : pe_.right_side_), hip_joint_(is_left, pe), knee_joint_(is_left, pe), knee_sea_joint_(is_left, pe, dji_esc_hub, huart), ankle_joint_(is_left, pe), fsr_gait_estimator_(ps_.fsr_gait_data_) {}
    virtual ~Side() = default;
    void Calibrate();
    void Read();
    void Assist();
    bool IsMotorConnect();
    void Standby();
    void Shutdown();

    ExoData &pe_;
    SideData &ps_;
    HipJoint hip_joint_;
    KneeJoint knee_joint_;
    KneeSeaJoint knee_sea_joint_;
    AnkleJoint ankle_joint_;
    FsrGaitEstimator fsr_gait_estimator_;
};

class ExoShell : public Shell
{
public:
    explicit ExoShell(UART_HandleTypeDef &huart, Exo &exo);
    ~ExoShell() = default;

    void OnCmdSetLed(int argc, char **argv);
    void OnCmdSetLocoMode(int argc, char **argv);
private:
    Exo &exo_;
};

class Exo
{
public:
    explicit Exo(ExoData &pe, ExoHardware &hw) : pe_(pe), hw_(hw), dji_esc_hub_(hw.motor_can), ao_(pe), left_side_(true, pe, dji_esc_hub_, hw.left_mag_encoder_uart), right_side_(false, pe, dji_esc_hub_, hw.right_mag_encoder_uart), shell_(hw.shell_uart, *this) {}
    ~Exo() = default;

    void Initialize();
    void Run();
    void Read();
    void Calibrate();
    void ResetCalibrationFlags();
    void Estimate();
    void Standby();
    void Assist();
    void Shutdown();

    void CheckSystemHealth();
    void VofaSendTelemetry();
    bool IsMotorConnect();
    bool IsCalibrateDone();
    bool IsStopWalking();

    void CanRxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, const uint8_t *data);
    void UartRxCallback(UART_HandleTypeDef *huart, uint16_t data_size);

    ExoData &pe_;
    ExoHardware &hw_;

    DjiEscHub dji_esc_hub_;
    AdaptiveOscillator ao_;
    Side left_side_;
    Side right_side_;
    ExoShell shell_;
    StateLed state_led_;

private:
    void SensorUartRxCallback(const uint8_t *data, uint16_t data_size);
    void UsrShellUartRxCallback(const uint8_t *data, uint16_t data_size);
    static ExoData::SysEvent AllowedEventsForState(ExoData::State s);
    static inline void ClearNonCriticalEvents(ExoData &pe)
    {
        pe.pending_events_ &= ~ExoData::SysEvent::kWakeup;
        pe.pending_events_ &= ~ExoData::SysEvent::kStartCalibrate;
        pe.pending_events_ &= ~ExoData::SysEvent::kStartAssist;
        pe.pending_events_ &= ~ExoData::SysEvent::kStopAssist;
        pe.pending_events_ &= ~ExoData::SysEvent::kEnterSleep;
    }
};

extern "C" {
void CallExoCanRxCallBack(Exo *exo, FDCAN_HandleTypeDef *hfdcan, uint32_t can_ext_id, const uint8_t *rx_data);
void CallExoUartRxCallback(Exo *exo, UART_HandleTypeDef *huart, uint16_t data_size);
}


#endif