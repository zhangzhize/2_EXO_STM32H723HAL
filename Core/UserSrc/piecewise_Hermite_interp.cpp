#include "piecewise_Hermite_interp.hpp"
#include <cstring>

HermiteInterp::HermiteInterp() :
    ptr_xs_(nullptr), ptr_ys_(nullptr), ptr_dys_(nullptr),
    ptr_coef_a_(nullptr), ptr_coef_b_(nullptr), ptr_coef_c_(nullptr), ptr_coef_d_(nullptr),
    num_xs_(0), x_interp_start_(0.0f), x_interp_interval_(0.0f),
    num_interp_(0), ptr_y_interp_(nullptr)
{
}

/**
 * @brief 析构 — 释放所有动态分配的数组内存
 */
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

/**
 * @brief 计算分段三次 Hermite 多项式系数
 * @param ptr_xs  输入 x 坐标 (需严格递增，相邻间距 > 1e-6)
 * @param ptr_ys  输入 y 坐标
 * @param ptr_dys 输入导数值
 * @param num_xs  数据点数量 (≥ 2)
 */
void HermiteInterp::CalCoeffs(float* ptr_xs, float* ptr_ys, float *ptr_dys, uint16_t num_xs)
{
    /* 释放旧数据，准备接收新插值 */
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

    /* 校验 x 坐标严格递增，间距阈值 1e-6 防止除零和数值不稳定 */
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

    /* 逐段计算三次 Hermite 系数
       P(s) = a·s³ + b·s² + c·s + d,  s = x - x_i, s ∈ [0, h] */
    for (uint16_t i = 0; i < (uint16_t)(num_xs - 1); ++i)
    {
        float x0 = ptr_xs_[i];
        float x1 = ptr_xs_[i + 1];
        float y0 = ptr_ys_[i];
        float y1 = ptr_ys_[i + 1];
        float m0 = ptr_dys_[i];      /* P'(x_i) */
        float m1 = ptr_dys_[i + 1];  /* P'(x_{i+1}) */
        float h = x1 - x0;           /* 段宽 */
        float dy = y1 - y0;          /* 段内 y 增量 */

        ptr_coef_d_[i] = y0;
        ptr_coef_c_[i] = m0;
        ptr_coef_b_[i] = (3.0f * dy / h - 2.0f * m0 - m1) / h;
        ptr_coef_a_[i] = (m0 + m1 - 2.0f * dy / h) / (h * h);
    }
}

/**
 * @brief 以固定步长预计算插值表格
 * @param x_interp_interval  采样步长 (> 1e-4)
 */
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

    /* 前进段索引: interval_index 只增不减，因为 x 递增 */
    uint16_t interval_index = 0;
    for (uint16_t i = 0; i < num_interp_; ++i)
    {
        float x = x_interp_start_ + i * x_interp_interval_;

        /* x 跨过下一个节点 → 段索引前进 */
        while (interval_index + 1 < (uint16_t)(num_xs_ - 1) && x >= ptr_xs_[interval_index + 1])
        {
            ++interval_index;
        }

        /* x 超出最后一个节点 → 钳位到最后一段 */
        if (x >= ptr_xs_[num_xs_ - 1])
        {
            interval_index = num_xs_ - 2;
        }

        float s = x - ptr_xs_[interval_index];
        float a = ptr_coef_a_[interval_index];
        float b = ptr_coef_b_[interval_index];
        float c = ptr_coef_c_[interval_index];
        float d = ptr_coef_d_[interval_index];

        /* 三次多项式求值: P(s) = ((a·s + b)·s + c)·s + d (Horner 法) */
        ptr_y_interp_[i] = ((a * s + b) * s + c) * s + d;
    }
}

/**
 * @brief 在预计算表格中采样 y 值
 *
 * 对任意 x 在预计算表格 ptr_y_interp_ 中查找：
 *   idx = (x - x_start) / interval → 取整数部分
 *   在 idx 和 idx+1 之间做线性插值
 *
 * 越界处理: x 超出表格范围时返回最近端点的值 (饱和外推)
 *
 * @param x  查询点坐标
 * @return 线性插值结果
 */
float HermiteInterp::Sample(float x) const
{
    if (ptr_y_interp_ == nullptr || num_interp_ == 0)
    {
        return 0.0f;
    }

    float x0 = x_interp_start_;
    float dx = x_interp_interval_;
    float idxf = (x - x0) / dx;

    /* 越界: 返回边界值 */
    if (idxf <= 0.0f)
    {
        return ptr_y_interp_[0];
    }
    if (idxf >= (float)(num_interp_ - 1))
    {
        return ptr_y_interp_[num_interp_ - 1];
    }

    /* 在相邻表项之间做线性插值 */
    int i0 = (int)idxf;
    int i1 = i0 + 1;
    float y0 = ptr_y_interp_[i0];
    float y1 = ptr_y_interp_[i1];
    float frac = idxf - (float)i0;
    return y0 + frac * (y1 - y0);
}

uint16_t HermiteInterp::GetNumInterp() const
{
    return num_interp_;
}

float HermiteInterp::GetXInterpStart() const
{
    return x_interp_start_;
}

float HermiteInterp::GetXInterpInterval() const
{
    return x_interp_interval_;
}

const float* HermiteInterp::GetYInterp() const
{
    return ptr_y_interp_;
}
