#include "semg.hpp"
#include "math.h"

/* ==========================================================================
 * sEmg — 基于 ARM CMSIS-DSP 的 sEMG 信号处理器
 * ========================================================================== */

sEmg::sEmg()
{
    // 系数和状态已在类声明中初始化，这里只需配置 CMSIS-DSP 实例
    arm_biquad_cascade_df2T_init_f32(&biquad_inst_,
                                     (uint8_t)kNumBiquadStages,
                                     &biquad_coeffs_[0][0],
                                     &biquad_state_[0][0]);
}

void sEmg::Update(int32_t raw_adc_value)
{
    float32_t input_f32 = (float32_t)raw_adc_value;
    float32_t filtered;

    semg_raw_value_ = raw_adc_value;

    // 1. CMSIS-DSP 双二阶级联巴特沃斯带通滤波
    arm_biquad_cascade_df2T_f32(&biquad_inst_, &input_f32, &filtered, 1);

    semg_filtered_value_ = filtered;

    // 2. 全波整流 + 滑动平均包络
    semg_envelope_value_ = CalcEnvelope((int32_t)fabsf(filtered));
}

void sEmg::SetFilterCoeffs(const float32_t *coeffs, uint32_t num_stages)
{
    if (num_stages > kNumBiquadStages) return;

    for (uint32_t i = 0; i < num_stages; ++i)
        for (int j = 0; j < 5; ++j)
            biquad_coeffs_[i][j] = coeffs[i * 5 + j];

    arm_fill_f32(0.0f, &biquad_state_[0][0], 2 * kNumBiquadStages);
    arm_biquad_cascade_df2T_init_f32(&biquad_inst_,
                                     (uint8_t)num_stages,
                                     &biquad_coeffs_[0][0],
                                     &biquad_state_[0][0]);
}

void sEmg::Reset()
{
    buffer_index_ = 0;
    buffer_sum_   = 0;

    arm_fill_f32(0.0f, &biquad_state_[0][0], 2 * kNumBiquadStages);
    for (int i = 0; i < kBufferSize; ++i) circular_buffer_[i] = 0;

    semg_raw_value_      = 0;
    semg_filtered_value_ = 0.0f;
    semg_envelope_value_ = 0;
}

int32_t sEmg::CalcEnvelope(int32_t semg_abs_value)
{
    // O(1) 增量滑动平均，每步减旧值、加新值，无需遍历
    buffer_sum_ -= circular_buffer_[buffer_index_];
    buffer_sum_ += semg_abs_value;
    circular_buffer_[buffer_index_] = semg_abs_value;
    buffer_index_ = (buffer_index_ + 1) % kBufferSize;

    return (buffer_sum_ / kBufferSize) * 2;
}

