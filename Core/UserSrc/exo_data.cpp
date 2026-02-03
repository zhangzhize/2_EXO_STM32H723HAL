#include "exo_data.hpp"

JointData::JointData(bool is_left)
{
    is_left_ = is_left;
    is_used_ = true;

    rom_rad_ = 0.0f;
    pos_rad_ = 0.0f;
    vel_radps_ = 0.0f;
    tor_Nm_ = 0.0f;
}

SideData::SideData(bool is_left) : hip_joint_(is_left), knee_joint_(is_left), ankle_joint_(is_left)
{
    is_left_ = is_left;
    is_used_ = true;

    /**< from Side */
    prev_heel_contact_state_ = true;
    prev_toe_contact_state_ = true;

    for (int i = 0; i<kNumStepsAvg; i++)
    {
        step_times_[i] = 0;
        stance_times_[i] = 0;
        swing_times_[i] = 0;
    }

    ground_strike_timestamp_ = 0;
    prev_ground_strike_timestamp_ = 0;
    toe_strike_timestamp_ = 0;
    prev_toe_strike_timestamp_ = 0;
    toe_off_timestamp_ = 0;
    prev_toe_off_timestamp_ = 0;

    /**< from SideData */
    percent_gait_ = -1.0f; 
    percent_stance_ = -1.0f;
    percent_swing_ = -1.0f;
    expected_step_duration_ = -1.0f; 
    expected_swing_duration_ = -1.0f;
    expected_stance_duration_ = -1.0f;

    ground_strike_ = false; 
    toe_off_ = false;
    toe_strike_ = false;
    toe_on_ = false;
    heel_contact_state_ = false;
    toe_contact_state_ = false;

    is_calibration_done_ = false;
    do_calibration_toe_fsr_ = true;
    do_calibration_refinement_toe_fsr_ = true;
    do_calibration_heel_fsr_ = true;
    do_calibration_refinement_heel_fsr_ = true;

    expected_duration_window_upper_coeff_ = 1.75f;
    expected_duration_window_lower_coeff_ = 0.25f;
}

ExoData::ExoData() : left_side_(true), right_side_(false)
{
    user_weight_kg_ = 70.0f;
    battery_voltage_ = 24.0f;
    exo_status_ = ExoStatus::kOff;
    ao_left_phase_rad_ = 0.0f;
    ao_right_phase_rad_ = 0.0f;
    ao_left_event_cnt_ = 0;
    ao_right_event_cnt_ = 0;
}