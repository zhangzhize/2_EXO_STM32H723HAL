#ifndef PIECEWISE_HERMLITE_INTERP_HPP
#define PIECEWISE_HERMLITE_INTERP_HPP

#include <cstdint>

class HermiteInterp
{
public:
    HermiteInterp(float* ptr_xs, float* ptr_ys, float *ptr_dys, uint16_t num_xs);
    ~HermiteInterp() = default;

    void Interp(float x_interp_interval);

    float *ptr_xs_;
    float *ptr_ys_;
    float *ptr_dys_;
    float *ptr_coef_a_;
    float *ptr_coef_b_;
    float *ptr_coef_c_;
    float *ptr_coef_d_;
    uint16_t num_xs_;

    float x_interp_start_;
    float x_interp_interval_;
    uint16_t num_interp_;
    float *ptr_y_interp_;
};


#endif