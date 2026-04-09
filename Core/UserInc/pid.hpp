#ifndef PID_HPP
#define PID_HPP

#include <cstdint>

class PIDController
{
public:
    explicit PIDController(float kp, float ki, float kd, float output_ramp, float output_limit)
    : kp_(kp), ki_(ki), kd_(kd), output_ramp_(output_ramp), output_limit_(output_limit) {}
    virtual ~PIDController() = default;

    float operator()(float error);
    void ResetError(void);
    
    float kp_;
    float ki_;
    float kd_; 
    float output_ramp_; //!< Maximum speed of change of the output value
    float output_limit_; //!< Maximum output value
private:
    float error_prev_ = 0.0f; //!< last tracking error value
    float output_prev_ = 0.0f;  //!< last pid output value
    float integral_prev_ = 0.0f; //!< last integral component value
    unsigned long timestamp_prev_ = 0; //!< Last execution timestamp
};

#endif