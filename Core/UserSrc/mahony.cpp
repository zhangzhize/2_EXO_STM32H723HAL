#include "mahony.hpp"
#include "math.h"
// #include "dsp/fast_math_functions.h"    //ZZZ: Need ARMCLANG
// #include "arm_math.h"       //ZZZ: Not Used

Mahony::Mahony()
{
	two_Kp_ = kTwoKpDef;	// 2 * proportional gain (Kp)
	two_Ki_ = kTwoKiDef;	// 2 * integral gain (Ki)
	q0_ = 1.0f;
	q1_ = 0.0f;
	q2_ = 0.0f;
	q3_ = 0.0f;
	integral_FBx_ = 0.0f;
	integral_FBy_ = 0.0f;
	integral_FBz_ = 0.0f;
	is_angle_computed_ = false;
	inv_sample_freq_ = 1.0f / kDefaultSampleFreq;
}

void Mahony::FirstUpdate(float ax, float ay, float az, float mx, float my, float mz)
{
    float recipNorm;
    float init_yaw, init_pitch, init_roll;
    float cr2, cp2, cy2, sr2, sp2, sy2;
    float sin_roll, cos_roll, sin_pitch, cos_pitch;
    float magX, magY;

    recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) 
    {
	    recipNorm = InvSqrt(mx * mx + my * my + mz * mz);
	    mx *= recipNorm;
	    my *= recipNorm;
	    mz *= recipNorm;
	}

    init_pitch = atan2f(-ax, az);
    init_roll = atan2f(ay, az);

    sin_roll  = sinf(init_roll);
    cos_roll  = cosf(init_roll);
    cos_pitch = cosf(init_pitch);
    sin_pitch = sinf(init_pitch);

    if((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f))
    {
    	magX = mx * cos_pitch + my * sin_pitch * sin_roll + mz * sin_pitch * cos_roll;
    	magY = my * cos_roll - mz * sin_roll;
        init_yaw  = atan2f(-magY, magX);
    }
    else
    {
        init_yaw = 0.0f;
    }

    cr2 = cosf(init_roll * 0.5f);
    cp2 = cosf(init_pitch * 0.5f);
    cy2 = cosf(init_yaw * 0.5f);
    sr2 = sinf(init_roll * 0.5f);
    sp2 = sinf(init_pitch * 0.5f);
    sy2 = sinf(init_yaw * 0.5f);

    q0_ = cr2 * cp2 * cy2 + sr2 * sp2 * sy2;
    q1_ = sr2 * cp2 * cy2 - cr2 * sp2 * sy2;
    q2_ = cr2 * sp2 * cy2 + sr2 * cp2 * sy2;
    q3_ = cr2 * cp2 * sy2 - sr2 * sp2 * cy2;

    // Normalise quaternion
    recipNorm = InvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= recipNorm;
    q1_ *= recipNorm;
    q2_ *= recipNorm;
    q3_ *= recipNorm;
}

void Mahony::Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid
	// (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		UpdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

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
		q0q0 = q0_ * q0_;
		q0q1 = q0_ * q1_;
		q0q2 = q0_ * q2_;
		q0q3 = q0_ * q3_;
		q1q1 = q1_ * q1_;
		q1q2 = q1_ * q2_;
		q1q3 = q1_ * q3_;
		q2q2 = q2_ * q2_;
		q2q3 = q2_ * q3_;
		q3q3 = q3_ * q3_;

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
	qa = q0_;
	qb = q1_;
	qc = q2_;
	q0_ += (-qb * gx - qc * gy - q3_ * gz);
	q1_ += (qa * gx + qc * gz - q3_ * gy);
	q2_ += (qa * gy - qb * gz + q3_ * gx);
	q3_ += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = InvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
	q0_ *= recipNorm;
	q1_ *= recipNorm;
	q2_ *= recipNorm;
	q3_ *= recipNorm;
	is_angle_computed_ = false;
}

void Mahony::UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q1_ * q3_ - q0_ * q2_;
		halfvy = q0_ * q1_ + q2_ * q3_;
		halfvz = q0_ * q0_ - 0.5f + q3_ * q3_;

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
	qa = q0_;
	qb = q1_;
	qc = q2_;
	q0_ += (-qb * gx - qc * gy - q3_ * gz);
	q1_ += (qa * gx + qc * gz - q3_ * gy);
	q2_ += (qa * gy - qb * gz + q3_ * gx);
	q3_ += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = InvSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
	q0_ *= recipNorm;
	q1_ *= recipNorm;
	q2_ *= recipNorm;
	q3_ *= recipNorm;
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
	roll_rad_ = atan2f(q0_ * q1_ + q2_ * q3_, 0.5f - q1_ * q1_ - q2_ * q2_);
	pitch_rad_ = asinf(-2.0f * (q1_ * q3_ - q0_ * q2_));
	yaw_rad_ = atan2f(q1_ * q2_ + q0_ * q3_, 0.5f - q2_ * q2_ - q3_ * q3_);
	is_angle_computed_ = true;

    // arm_atan2_f32(q0_ * q1_ + q2_ * q3_, 0.5f - q1_ * q1_ - q2_ * q2_, &roll_rad_);  //zzz: arm_atan2_f32 Need CMSIS-DSP
	// pitch_rad_ = asinf(-2.0f * (q1_ * q3_ - q0_ * q2_));
	// arm_atan2_f32(q1_ * q2_ + q0_ * q3_, 0.5f - q2_ * q2_ - q3_ * q3_, &yaw_rad_); 
	// is_angle_computed_ = true;
}