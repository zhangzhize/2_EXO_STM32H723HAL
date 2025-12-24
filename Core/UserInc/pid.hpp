#ifndef PID_HPP
#define PID_HPP

#include <cstdint>

class PIDController
{
public:
    PIDController(float kp, float ki, float kd, float output_ramp, float output_limit);
    ~PIDController() = default;

    float operator()(float error);
    void ResetError(void);
    
    float kp_;
    float ki_;
    float kd_; 
    float output_ramp_; //!< Maximum speed of change of the output value
    float output_limit_; //!< Maximum output value
private:
    float error_prev_; //!< last tracking error value
    float output_prev_;  //!< last pid output value
    float integral_prev_; //!< last integral component value
    unsigned long timestamp_prev_; //!< Last execution timestamp
};

#endif