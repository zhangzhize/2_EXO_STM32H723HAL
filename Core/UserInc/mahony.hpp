//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 02/04/2026   Zhize Zhang     Ported to STM32H723
//=============================================================================================
#ifndef MAHONY_HPP
#define MAHONY_HPP

class Mahony
{
public:
    Mahony();
    ~Mahony() = default;

    void Begin(float sample_freq) { inv_sample_freq_ = 1.0f / sample_freq; }
    void FirstUpdate(float ax, float ay, float az, float mx, float my, float mz);
	void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void GetEulerAnglesDeg(float& roll_deg, float& pitch_deg, float& yaw_deg)
    {
        if (!is_angle_computed_) ComputeAngles();
        roll_deg = roll_rad_ * 57.29578f;
        pitch_deg = pitch_rad_ * 57.29578f;
        yaw_deg = yaw_rad_ * 57.29578f;
    }
    void GetEulerAnglesRad(float& roll_rad, float& pitch_rad, float& yaw_rad)
    {
        if (!is_angle_computed_) ComputeAngles();
        roll_rad = roll_rad_;
        pitch_rad = pitch_rad_;
        yaw_rad = yaw_rad_;
    }
private:
    float two_Kp_;  // 2 * proportional gain (Kp)
    float two_Ki_;  // 2 * integral gain (Ki)
    float q0_, q1_, q2_, q3_; // quaternion of sensor frame relative to auxiliary frame
    float integral_FBx_, integral_FBy_, integral_FBz_;  // integral error terms scaled by Ki
    float inv_sample_freq_;
    float roll_rad_, pitch_rad_, yaw_rad_;
    bool is_angle_computed_;

	static float InvSqrt(float x);
	void ComputeAngles();

    static constexpr float kDefaultSampleFreq = 512.0f;	// sample frequency in Hz
    static constexpr float kTwoKpDef = (2.0f * 6.0f);	// 2 * proportional gain
    static constexpr float kTwoKiDef = (2.0f * 0.1f);	// 2 * integral gain
};


#endif
