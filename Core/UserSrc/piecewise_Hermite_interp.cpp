#include "piecewise_Hermite_interp.hpp"
#include <cstring>

HermiteInterp::HermiteInterp() :
    ptr_xs_(nullptr), ptr_ys_(nullptr), ptr_dys_(nullptr), 
    ptr_coef_a_(nullptr), ptr_coef_b_(nullptr), ptr_coef_c_(nullptr), ptr_coef_d_(nullptr),
    num_xs_(0), x_interp_start_(0.0f), x_interp_interval_(0.0f),
    num_interp_(0), ptr_y_interp_(nullptr)
{
}

HermiteInterp::~HermiteInterp()
{
    delete[] ptr_xs_; ptr_xs_ = nullptr;
    delete[] ptr_ys_; ptr_ys_ = nullptr;
    delete[] ptr_dys_; ptr_dys_ = nullptr;
    delete[] ptr_coef_a_; ptr_coef_a_ = nullptr;
    delete[] ptr_coef_b_; ptr_coef_b_ = nullptr;
    delete[] ptr_coef_c_; ptr_coef_c_ = nullptr;
    delete[] ptr_coef_d_; ptr_coef_d_ = nullptr;
    delete[] ptr_y_interp_; ptr_y_interp_ = nullptr;
}

void HermiteInterp::CalCoeffs(float* ptr_xs, float* ptr_ys, float *ptr_dys, uint16_t num_xs)
{
    delete[] ptr_xs_;
    delete[] ptr_ys_;
    delete[] ptr_dys_;
    delete[] ptr_coef_a_;
    delete[] ptr_coef_b_;
    delete[] ptr_coef_c_;
    delete[] ptr_coef_d_;
    ptr_xs_ = nullptr;
    ptr_ys_ = nullptr;
    ptr_dys_ = nullptr;
    ptr_coef_a_ = nullptr;
    ptr_coef_b_ = nullptr;
    ptr_coef_c_ = nullptr;
    ptr_coef_d_ = nullptr;

    if (ptr_xs == nullptr || ptr_ys == nullptr || ptr_dys == nullptr || num_xs < 2)
    {
        return;
    }

    // validate xs are strictly increasing and not too close
    const float min_spacing = 1e-6f;
    for (uint16_t i = 0; i < (uint16_t)(num_xs - 1); ++i)
    {
        float h = ptr_xs[i + 1] - ptr_xs[i];
        if (!(h > min_spacing))
        {
            return;
        }
    }

    num_xs_ = num_xs;
    ptr_xs_ = new float[num_xs];
    ptr_ys_ = new float[num_xs];
    ptr_dys_ = new float[num_xs];
    ptr_coef_a_ = new float[num_xs - 1];
    ptr_coef_b_ = new float[num_xs - 1];
    ptr_coef_c_ = new float[num_xs - 1];
    ptr_coef_d_ = new float[num_xs - 1];
    if (ptr_xs_ == nullptr || ptr_ys_ == nullptr || ptr_dys_ == nullptr || ptr_coef_a_ == nullptr || ptr_coef_b_ == nullptr || ptr_coef_c_ == nullptr || ptr_coef_d_ == nullptr)
    {
        return;
    }

    memcpy(ptr_xs_, ptr_xs, num_xs * sizeof(float));
    memcpy(ptr_ys_, ptr_ys, num_xs * sizeof(float));
    memcpy(ptr_dys_, ptr_dys, num_xs * sizeof(float));

    // compute cubic Hermite coefficients for each interval
    // polynomial on interval i: P(s) = a*s^3 + b*s^2 + c*s + d,  s = x - x_i
    for (uint16_t i = 0; i < (uint16_t)(num_xs - 1); ++i)
    {
        float x0 = ptr_xs_[i];
        float x1 = ptr_xs_[i + 1];
        float y0 = ptr_ys_[i];
        float y1 = ptr_ys_[i + 1];
        float m0 = ptr_dys_[i];
        float m1 = ptr_dys_[i + 1];
        float h = x1 - x0;
        float dy = y1 - y0;

        ptr_coef_d_[i] = y0;
        ptr_coef_c_[i] = m0;
        ptr_coef_b_[i] = (3.0f * dy / h - 2.0f * m0 - m1) / h;
        ptr_coef_a_[i] = (m0 + m1 - 2.0f * dy / h) / (h * h);
    }
}

void HermiteInterp::Interp(float x_interp_interval)
{
    delete[] ptr_y_interp_;
    ptr_y_interp_ = nullptr;
    
    if (ptr_xs_ == nullptr || ptr_ys_ == nullptr || ptr_dys_ == nullptr || ptr_coef_a_ == nullptr || ptr_coef_b_ == nullptr || ptr_coef_c_ == nullptr || ptr_coef_d_ == nullptr || x_interp_interval <= 1e-4f)
    {
        return;
    }
    
    x_interp_start_ = ptr_xs_[0];
    x_interp_interval_ = x_interp_interval;
    float x_interp_end = ptr_xs_[num_xs_ - 1];

    num_interp_ = static_cast<uint16_t>((x_interp_end - x_interp_start_) / x_interp_interval_) + 1;
    ptr_y_interp_ = new float[num_interp_];

    if (ptr_y_interp_ == nullptr)
    {
        return;
    }

    // use advancing interval index to avoid repeated linear search
    uint16_t interval_index = 0;
    for (uint16_t i = 0; i < num_interp_; ++i)
    {
        float x = x_interp_start_ + i * x_interp_interval_;

        // advance interval index while x is beyond next knot
        while (interval_index + 1 < (uint16_t)(num_xs_ - 1) && x >= ptr_xs_[interval_index + 1])
        {
            ++interval_index;
        }

        if (x >= ptr_xs_[num_xs_ - 1])
        {
            interval_index = num_xs_ - 2;
        }

        float s = x - ptr_xs_[interval_index];
        float a = ptr_coef_a_[interval_index];
        float b = ptr_coef_b_[interval_index];
        float c = ptr_coef_c_[interval_index];
        float d = ptr_coef_d_[interval_index];

        // evaluate cubic polynomial
        ptr_y_interp_[i] = ((a * s + b) * s + c) * s + d;
    }
}
