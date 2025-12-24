#ifndef ADAPTIVE_HPP
#define ADAPTIVE_HPP

#include <cstdint>

enum class GaitEventDetectMode : uint8_t
{
    kPressureSensor = 0x00,
    kKneeAngleDiff = 0x01,
};

class AdaptiveOscillator
{
public:
    AdaptiveOscillator();
    ~AdaptiveOscillator() = default;

    void UpdateGaitPhase(float left_angle_rad, float right_angle_rad, float left_velocity, float right_velocity, bool left_heel_contact_state, bool right_heel_contact_state);
    void ResetGaitPhase(void);

    static const uint8_t kNumAOs = 3;
    const uint64_t kMaxTstrideUs = 3.0 * 1000000;
    const uint64_t kMinTstrideUs = 0.1 * 1000000;
    const float kHeelPresVolThreshold = 1.0f;
    const uint64_t kMaxStoppingDurationUs = 0.5 * 1000000;
    const float KEmaTauS = 0.2f;

    uint64_t tprev_sys_us_;

    float v_phi_;
    float v_omega_;
    float eta_;
    float kp_;
    float rho_;

    float hat_x_;
    float omega_;
    float phi_[kNumAOs];
    float alpha_[kNumAOs];
    float alpha0_;

    float left_phi_comp_rad_;
    float right_phi_comp_rad_;

    float left_tk_sys_us_;
    float right_tk_sys_us_;
    float left_Pe_tilde_tk_;
    float right_Pe_tilde_tk_;
    float left_epsilon_phi_tk_;
    float right_epsilon_phi_tk_;
    float left_phi_e_;
    float right_phi_e_;

    GaitEventDetectMode gait_event_detect_mode_;
    uint32_t left_event_cnt_;
    uint32_t right_event_cnt_;
    float ema_vel_rms_;
};



#endif // __ADAPTIVE_HPP