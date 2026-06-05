#include "semg.hpp"

#include "math.h"

sEmg::sEmg()
{
    /* 双二阶系数和状态数组已在类声明中零初始化 (biquad_state_ = {})
       此处仅需初始化 CMSIS-DSP 实例句柄，建立系数/状态指针关联 */
    arm_biquad_cascade_df2T_init_f32(&biquad_inst_, (uint8_t)kNumBiquadStages, &biquad_coeffs_[0][0], &biquad_state_[0][0]);
}

/**
 * @brief 处理一个原始 ADC 采样点
 * @param raw_adc_value ADC 原始采样值 (int32)
 */
void sEmg::Update(int32_t raw_adc_value)
{
    float32_t input_f32 = (float32_t)raw_adc_value;
    float32_t filtered;

    semg_raw_value_ = raw_adc_value;

    /* 步骤 1: CMSIS-DSP 双二阶级联巴特沃斯带通滤波 arm_biquad_cascade_df2T_f32 执行 4 阶 DF2T 滤波，输入/输出均为 float32 */
    arm_biquad_cascade_df2T_f32(&biquad_inst_, &input_f32, &filtered, 1);

    semg_filtered_value_ = filtered;

    /* 步骤 2-3: 全波整流 (fabsf) → 滑动平均包络检测 */
    semg_envelope_value_ = CalcEnvelope((int32_t)fabsf(filtered));
}

/**
 * @brief 运行时替换滤波器系数
 * @param coeffs     扁平系数数组，每节 5 个值: {b0, b1, b2, a1, a2}
 * @param num_stages 新的级联节数 (≤ kNumBiquadStages)
 */
void sEmg::SetFilterCoeffs(const float32_t *coeffs, uint32_t num_stages)
{
    if (num_stages > kNumBiquadStages) return;

    for (uint32_t i = 0; i < num_stages; ++i)
        for (int j = 0; j < 5; ++j)
            biquad_coeffs_[i][j] = coeffs[i * 5 + j];

    /* 清零状态缓冲区 — 系数替换后旧状态无效，必须归零防止瞬态冲击 */
    arm_fill_f32(0.0f, &biquad_state_[0][0], 2 * kNumBiquadStages);
    arm_biquad_cascade_df2T_init_f32(&biquad_inst_, (uint8_t)num_stages, &biquad_coeffs_[0][0], &biquad_state_[0][0]);
}

/**
 * @brief 完全复位 — 滤波器状态 + 包络缓冲区 + 输出值全部归零
 */
void sEmg::Reset()
{
    buffer_index_ = 0;
    buffer_sum_ = 0;

    arm_fill_f32(0.0f, &biquad_state_[0][0], 2 * kNumBiquadStages);
    for (int i = 0; i < kBufferSize; ++i)
        circular_buffer_[i] = 0;

    semg_raw_value_ = 0;
    semg_filtered_value_ = 0.0f;
    semg_envelope_value_ = 0;
}

/**
 * @brief 增量式滑动平均包络计算 — O(1) 复杂度
 *
 * 算法: 维护 32 点环形缓冲区和元素总和。
 * 每步: sum = sum - 旧值 + 新值 → 新值写入缓冲区 → 索引 +1 取模。
 *
 * 最终输出 = (sum / 32) × 2，乘 2 因子用于校准包络幅值匹配原始肌电幅度。
 *
 * @param  semg_abs_value  全波整流后的绝对值
 * @return 包络值
 */
int32_t sEmg::CalcEnvelope(int32_t semg_abs_value)
{
    buffer_sum_ -= circular_buffer_[buffer_index_];
    buffer_sum_ += semg_abs_value;
    circular_buffer_[buffer_index_] = semg_abs_value;
    buffer_index_ = (buffer_index_ + 1) % kBufferSize;

    return (buffer_sum_ / kBufferSize) * 2;
}
