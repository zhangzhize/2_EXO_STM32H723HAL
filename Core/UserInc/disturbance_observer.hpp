#ifndef DISTURBANCE_OBSERVER_HPP
#define DISTURBANCE_OBSERVER_HPP

#include <cstdint>

class DisturbanceObserver
{
public:
    DisturbanceObserver(float eso_w);
    ~DisturbanceObserver() = default;

    void UpdateObserver(float x1, float u);
    void ResetObserver(void);

    float hat_x1_, hat_x2_;
    float eso_w_;
    uint64_t tprev_sys_us_;
};

#endif