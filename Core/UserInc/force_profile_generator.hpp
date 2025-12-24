#ifndef FORCE_PROFILE_GENERATOR_HPP
#define FORCE_PROFILE_GENERATOR_HPP

#include <cstdint>
#include "piecewise_Hermite_interp.hpp"

class KneeForceProfileGenerator
{
public:
    KneeForceProfileGenerator();
    ~KneeForceProfileGenerator() = default;

    float GetForceProfile(float gait_phase_rad, float knee_angle_rad, float knee_velocity);

    float stiffness_onset_phase_rad_;
    float stiffness_offset_phase_rad_;
    float stiffness_;

    HermiteInterp force_profile_interp_;
    float peak_time_phase_rad_;
    float peak_torque_Nmkg_;
    float rise_time_phase_rad_;
    float fall_time_phase_rad_;

    float damping_onset_phase_rad_;
    float damping_offset_phase_rad_;
    float damping_;
};

class AnkleForceProfileGenerator
{
public:
    AnkleForceProfileGenerator();
    ~AnkleForceProfileGenerator() = default;

    float GetForceProfile(float gait_phase_rad);

    float start_time_phase_rad_;
    float end_time_phase_rad_;

    float peak_time_phase_rad_;
    float peak_torque_Nmkg_;
    float rise_time_phase_rad_;
    float fall_time_phase_rad_;
    HermiteInterp force_profile_interp_;
};


#endif
