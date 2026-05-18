#include "mahony.hpp"
#include "math.h"
// #include "dsp/fast_math_functions.h"    //ZZZ: Need ARMCLANG
// #include "arm_math.h"       //ZZZ: Not Used

void Mahony::Update9Axis(float gyro_degps_[3], float accel_mps2_[3], float magnet_uT_[3])
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

    float mx = magnet_uT_[0];
    float my = magnet_uT_[1];
    float mz = magnet_uT_[2];

	// Use IMU algorithm if magnetometer measurement invalid
	// (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		Update6Axis(gyro_degps_, accel_mps2_);
		return;
	}

	// Convert gyroscope degrees/sec to radians/sec
	float gx = 0.0174533f * gyro_degps_[0];
	float gy = 0.0174533f * gyro_degps_[1];
	float gz = 0.0174533f * gyro_degps_[2];

    float ax = accel_mps2_[0];
    float ay = accel_mps2_[1];
    float az = accel_mps2_[2];

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = InvSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q_[0] * q_[0];
		q0q1 = q_[0] * q_[1];
		q0q2 = q_[0] * q_[2];
		q0q3 = q_[0] * q_[3];
		q1q1 = q_[1] * q_[1];
		q1q2 = q_[1] * q_[2];
		q1q3 = q_[1] * q_[3];
		q2q2 = q_[2] * q_[2];
		q2q3 = q_[2] * q_[3];
		q3q3 = q_[3] * q_[3];

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction
		// and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(two_Ki_ > 0.0f) {
			// integral error scaled by Ki
			integral_FBx_ += two_Ki_ * halfex * inv_sample_freq_;
			integral_FBy_ += two_Ki_ * halfey * inv_sample_freq_;
			integral_FBz_ += two_Ki_ * halfez * inv_sample_freq_;
			gx += integral_FBx_;	// apply integral feedback
			gy += integral_FBy_;
			gz += integral_FBz_;
		} else {
			integral_FBx_ = 0.0f;	// prevent integral windup
			integral_FBy_ = 0.0f;
			integral_FBz_ = 0.0f;
		}

		// Apply proportional feedback
		gx += two_Kp_ * halfex;
		gy += two_Kp_ * halfey;
		gz += two_Kp_ * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * inv_sample_freq_);		// pre-multiply common factors
	gy *= (0.5f * inv_sample_freq_);
	gz *= (0.5f * inv_sample_freq_);
	qa = q_[0];
	qb = q_[1];
	qc = q_[2];
	q_[0] += (-qb * gx - qc * gy - q_[3] * gz);
	q_[1] += (qa * gx + qc * gz - q_[3] * gy);
	q_[2] += (qa * gy - qb * gz + q_[3] * gx);
	q_[3] += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = InvSqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
	q_[0] *= recipNorm;
	q_[1] *= recipNorm;
	q_[2] *= recipNorm;
	q_[3] *= recipNorm;
	is_angle_computed_ = false;
}

void Mahony::Update6Axis(float gyro_degps_[3], float accel_mps2_[3])
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Convert gyroscope degrees/sec to radians/sec
	float gx = 0.0174533f * gyro_degps_[0];
	float gy = 0.0174533f * gyro_degps_[1];
	float gz = 0.0174533f * gyro_degps_[2];

    float ax = accel_mps2_[0];
    float ay = accel_mps2_[1];
    float az = accel_mps2_[2];
    
	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q_[1] * q_[3] - q_[0] * q_[2];
		halfvy = q_[0] * q_[1] + q_[2] * q_[3];
		halfvz = q_[0] * q_[0] - 0.5f + q_[3] * q_[3];

		// Error is sum of cross product between estimated
		// and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(two_Ki_ > 0.0f) {
			// integral error scaled by Ki
			integral_FBx_ += two_Ki_ * halfex * inv_sample_freq_;
			integral_FBy_ += two_Ki_ * halfey * inv_sample_freq_;
			integral_FBz_ += two_Ki_ * halfez * inv_sample_freq_;
			gx += integral_FBx_;	// apply integral feedback
			gy += integral_FBy_;
			gz += integral_FBz_;
		} else {
			integral_FBx_ = 0.0f;	// prevent integral windup
			integral_FBy_ = 0.0f;
			integral_FBz_ = 0.0f;
		}

		// Apply proportional feedback
		gx += two_Kp_ * halfex;
		gy += two_Kp_ * halfey;
		gz += two_Kp_ * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * inv_sample_freq_);		// pre-multiply common factors
	gy *= (0.5f * inv_sample_freq_);
	gz *= (0.5f * inv_sample_freq_);
	qa = q_[0];
	qb = q_[1];
	qc = q_[2];
	q_[0] += (-qb * gx - qc * gy - q_[3] * gz);
	q_[1] += (qa * gx + qc * gz - q_[3] * gy);
	q_[2] += (qa * gy - qb * gz + q_[3] * gx);
	q_[3] += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = InvSqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
	q_[0] *= recipNorm;
	q_[1] *= recipNorm;
	q_[2] *= recipNorm;
	q_[3] *= recipNorm;
	is_angle_computed_ = false;
}

float Mahony::InvSqrt(float x)
{
	float halfx = 0.5f * x;
	union { float f; long l; } i;
	i.f = x;
	i.l = 0x5f3759df - (i.l >> 1);
	float y = i.f;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
    //zzz: Need ARMCLANG
    // volatile float tmp = 1.0f;
	// tmp /= __sqrtf(x);
	// return tmp;
}

void Mahony::ComputeAngles()
{
	roll_rad_ = atan2f(q_[0] * q_[1] + q_[2] * q_[3], 0.5f - q_[1] * q_[1] - q_[2] * q_[2]);
	pitch_rad_ = asinf(-2.0f * (q_[1] * q_[3] - q_[0] * q_[2]));
	yaw_rad_ = atan2f(q_[1] * q_[2] + q_[0] * q_[3], 0.5f - q_[2] * q_[2] - q_[3] * q_[3]);
	is_angle_computed_ = true;

    // arm_atan2_f32(q_[0] * q_[1] + q_[2] * q_[3], 0.5f - q_[1] * q_[1] - q_[2] * q_[2], &roll_rad_);  //zzz: arm_atan2_f32 Need CMSIS-DSP
	// pitch_rad_ = asinf(-2.0f * (q_[1] * q_[3] - q_[0] * q_[2]));
	// arm_atan2_f32(q_[1] * q_[2] + q_[0] * q_[3], 0.5f - q_[2] * q_[2] - q_[3] * q_[3], &yaw_rad_); 
	// is_angle_computed_ = true;
}