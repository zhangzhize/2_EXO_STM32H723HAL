#include "disturbance_observer.hpp"
#include "utils.h"


/**
 * @brief Construct a new Disturbance Observer:: Disturbance Observer object
 * 
 * @param eso_w 
 */
DisturbanceObserver::DisturbanceObserver(float eso_w) : hat_x1_(0.0f), hat_x2_(0.0f), eso_w_(eso_w), tprev_sys_us_(GetSysTimeUs())
{
}

void DisturbanceObserver::UpdateObserver(float x1, float u)
{
    float ts = 0.0f;
    uint64_t tnow_sys_us = GetSysTimeUs();
    if (tnow_sys_us - tprev_sys_us_ > 1000000)
    {
        ts = 0.01f;
        tprev_sys_us_ = tnow_sys_us;
    }
    else
    {
        ts = (tnow_sys_us - tprev_sys_us_) * 0.000001;
        tprev_sys_us_ = tnow_sys_us;
    }

    float dot_hat_x1 = hat_x2_ + 2 * eso_w_ * (x1 - hat_x1_) + u;
    float dot_hat_x2 = eso_w_ * eso_w_ * (x1 - hat_x1_);
    hat_x1_ = hat_x1_ + dot_hat_x1 * ts;
    hat_x2_ = hat_x2_ + dot_hat_x2 * ts;
}

void DisturbanceObserver::ResetObserver(void)
{
    hat_x1_ = 0.0f;
    hat_x2_ = 0.0f;
}