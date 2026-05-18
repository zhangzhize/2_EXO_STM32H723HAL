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
    Mahony(float sample_freq) : inv_sample_freq_(1.0f / sample_freq) {};
    ~Mahony() = default;

	void Update9Axis(float gyro_degps_[3], float accel_mps2_[3], float magnet_uT_[3]);
    void Update6Axis(float gyro_degps_[3], float accel_mps2_[3]);
    
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
    void GetQuaternion(float q[4]) const
    {
        q[0] = q_[0];
        q[1] = q_[1];
        q[2] = q_[2];
        q[3] = q_[3];
    }
private:
    float inv_sample_freq_;
    float two_Kp_ = (2.0f * 6.0f);  // 2 * proportional gain (Kp)
    float two_Ki_ = (2.0f * 0.1f);  // 2 * integral gain (Ki)
    float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // quaternion of sensor frame relative to auxiliary frame
    float integral_FBx_ = 0.0f;
    float integral_FBy_ = 0.0f;
    float integral_FBz_ = 0.0f;  // integral error terms scaled by Ki
    float roll_rad_ = 0.0f;
    float pitch_rad_ = 0.0f;
    float yaw_rad_ = 0.0f;
    bool is_angle_computed_ = false;

	static float InvSqrt(float x);
	void ComputeAngles();
};


#endif
