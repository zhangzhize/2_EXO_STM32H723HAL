#ifndef EXO_DATA_HPP
#define EXO_DATA_HPP

#include <cstdint>

/*  RGB Color Codes
    0: {  0,   0,   0}, // Off       #000000
    1: {  0, 255,   0}, // Green     #00FF00
    2: {  0,   0, 255}, // Blue      #0000FF
    3: {255, 255,   0}, // Yellow    #FFFF00
    4: {255,   0, 255}, // Magenta   #FF00FF
    5: {  0, 255, 255}, // Cyan      #00FFFF
    6: {255, 140,   0}, // Orange    #FF8C00
    7: {128,   0, 128}, // Purple    #800080
    8: {255,   0,   0}, // Red       #FF0000
    */
enum class ExoStatus : uint8_t
{
    kOff = 0U,
    kCalibration = 1U,
    kReady = 2U,
    kAssisting = 3U,
    kErrorBatteryLowVoltage = 8U,
    kErrorMotor = 9U,
};

/** from nrf54 */
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
    ImuData(bool is_left_ = true);
    ~ImuData() = default;

    bool is_left_;
    bool is_used_;

    float quat_i_;
    float quat_j_;
    float quat_k_;
    float quat_real_;
};

class JointData
{
public:
    JointData(bool is_left = true);
    ~JointData() = default;

    bool is_left_;
    bool is_used_;

    float rom_rad_;
    float pos_rad_;
    float vel_radps_;
    float tor_Nm_;
};

class SideData
{
public:
    SideData(bool is_left = true);
    ~SideData() = default;

    bool is_left_;
    bool is_used_;
    JointData hip_joint_;
    JointData knee_joint_;
    JointData ankle_joint_;

    ImuData foot_imu_;
    float ankle_plantarflexion_force_N_;

    static const uint8_t kNumStepsAvg = 3;   
    uint32_t step_times_[kNumStepsAvg];
    uint32_t stance_times_[kNumStepsAvg];
    uint32_t swing_times_[kNumStepsAvg];

    uint32_t ground_strike_timestamp_;
    uint32_t prev_ground_strike_timestamp_;
    uint32_t toe_strike_timestamp_;
    uint32_t prev_toe_strike_timestamp_;
    uint32_t toe_off_timestamp_;
    uint32_t prev_toe_off_timestamp_;

    float percent_gait_;
    float percent_stance_;
    float percent_swing_;
    float expected_step_duration_;
    float expected_stance_duration_;
    float expected_swing_duration_;
    float expected_duration_window_upper_coeff_;
    float expected_duration_window_lower_coeff_;

    bool ground_strike_;
    bool toe_strike_;
    bool toe_off_;
    bool toe_on_;
    bool heel_contact_state_;
    bool toe_contact_state_;
    bool prev_heel_contact_state_;
    bool prev_toe_contact_state_;

    bool is_calibration_done_;
    bool do_calibration_toe_fsr_;
    bool do_calibration_refinement_toe_fsr_;
    bool do_calibration_heel_fsr_;
    bool do_calibration_refinement_heel_fsr_;
};

class ExoData
{
public:
    ExoData();
    ~ExoData() = default;
    /**< Common */
    float user_weight_kg_;
    float battery_voltage_;
    ExoStatus exo_status_;
    /**< Two Side */
    SideData left_side_;
    SideData right_side_;
    /**< For AO */
    float ao_left_phase_rad_;
    float ao_right_phase_rad_;
    uint32_t ao_left_event_cnt_;
    uint32_t ao_right_event_cnt_;
};


#endif // EXO_DATA_HPP
