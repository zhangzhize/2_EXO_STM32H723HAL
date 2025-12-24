#include "pid.hpp"
#include "utils.h"

PIDController::PIDController(float kp, float ki, float kd, float output_ramp, float output_limit)
    : kp_(kp), ki_(ki), kd_(kd), output_ramp_(output_ramp), output_limit_(output_limit)
    , error_prev_(0.0f), output_prev_(0.0f), integral_prev_(0.0f), timestamp_prev_(GetSysTimeUs())
{
}

float PIDController::operator()(float error)
{
    // calculate the time from the last call
    unsigned long timestamp_now = GetSysTimeUs();
    float ts = (timestamp_now - timestamp_prev_) * 1e-6f;
    // quick fix for strange cases (GetSysTimeUs overflow)
    if (ts <= 0 || ts > 0.5f) ts = 1e-3f;

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
    float proportional_out = kp_ * error;
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    float integral_out = integral_prev_ + ki_ * ts * 0.5f * (error + error_prev_);
    // antiwindup - limit the output
    integral_out = _constrain(integral_out, -output_limit_, output_limit_);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float derivative_out = kd_ * (error - error_prev_) / ts;

    // sum all the components
    float output = proportional_out + integral_out + derivative_out;
    // antiwindup - limit the output variable
    output = _constrain(output, -output_limit_, output_limit_);

    // if output ramp defined
    if (output_ramp_ > 0)
    {
        // limit the acceleration by ramping the output
        float output_rate = (output - output_prev_) / ts;
        if (output_rate > output_ramp_)
            output = output_prev_ + output_ramp_ * ts;
        else if (output_rate < -output_ramp_)
            output = output_prev_ - output_ramp_ * ts;
    }
    // saving for the next pass
    integral_prev_ = integral_out;
    output_prev_ = output;
    error_prev_ = error;
    timestamp_prev_ = timestamp_now;
    return output;
}

void PIDController::ResetError(void)
{
    integral_prev_ = 0.0f;
    output_prev_ = 0.0f;
    error_prev_ = 0.0f;
    timestamp_prev_ = GetSysTimeUs();
}
