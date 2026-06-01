/**
 * @file semg.hpp
 * @author Zhize Zhang (zhangzhize1@foxmail.com)
 * @brief sEMG 表面肌电信号处理 — CMSIS-DSP 巴特沃斯带通滤波 + 包络检测
 *
 * 信号链: 原始 ADC → 4 阶巴特沃斯带通 (CMSIS-DSP arm_biquad_cascade_df2T_f32) →
 * 全波整流 → 32 点滑动平均包络
 *
 * 滤波系数用 test/design_biquad_filter.py 设计并导出。
 *
 * @version 0.3
 * @date 2026-05-30
 *
 * @copyright Copyright (c) 2026
 */

#ifndef SEMG_HPP
#define SEMG_HPP

#include <cstdint>
extern "C" {
#include "arm_math.h"
}

/**
 * @brief 基于 ARM CMSIS-DSP 的 sEMG 信号处理器
 *
 * 信号链: 原始 ADC → 4 阶巴特沃斯带通 (CMSIS-DSP DF2T 双二阶级联) →
 * 全波整流 → 32 点滑动平均包络
 *
 * 重要: CMSIS-DSP DF2T 的 a 系数符号约定与 scipy signal.butter 的 SOS 格式相反!
 *   - scipy SOS:    H(z) = N(z) / (1 + a₁·z⁻¹ + a₂·z⁻²)
 *   - CMSIS DF2T:   H(z) = N(z) / (1 - a₁·z⁻¹ - a₂·z⁻²)  (内部用 d = b·x + a·y + d)
 *   design_biquad_filter.py 已自动取反，直接粘贴输出即可。
 *
 * 默认系数: fs=250 Hz, 35-55 Hz 带通, 4 阶 (CMSIS-DSP DF2T 格式, a 已取反)
 */
class sEmg
{
public:
    sEmg();
    ~sEmg() = default;

    /** 处理一个原始 ADC 采样，更新滤波值和包络值 */
    void Update(int32_t raw_adc_value);

    /**
     * @brief 运行时替换滤波器系数
     * @param coeffs  扁平数组，长度 5 × num_stages，CMSIS-DSP DF2T 格式:
     *                {b0, b1, b2, a1, a2} 每节 (a 已取反)
     * @param num_stages  双二阶节数 (≤ kNumBiquadStages)
     */
    void SetFilterCoeffs(const float32_t *coeffs, uint32_t num_stages);

    /** 重置所有滤波器状态和包络缓冲区为零 */
    void Reset();

    // --- 访问器 ---
    int32_t semg_raw_value()      const { return semg_raw_value_; }
    float   semg_filtered_value() const { return semg_filtered_value_; }
    int32_t semg_envelope_value() const { return semg_envelope_value_; }

private:
    static constexpr int kNumBiquadStages = 4;
    static constexpr int kBufferSize      = 32;

    // 默认双二阶系数 (fs=250 Hz, 35-55 Hz 带通, 4 阶)
    // 注意: 这些系数是 CMSIS-DSP DF2T 格式 (a1,a2 已取反)
    // 用 design_biquad_filter.py 生成:
    //   python test/design_biquad_filter.py --fs 250 --low 35 --high 55 --order 4 --var bq
    // clang-format off
    // 每行一个 biquad 节: {b0, b1, b2, a1_cmsis, a2_cmsis}
    // CMSIS DF2T 内部: y = b0·x + d1, d1' = b1·x + a1·y + d2, d2' = b2·x + a2·y
    float32_t biquad_coeffs_[kNumBiquadStages][5] = {
        {0.00223489f,  0.00446978f,  0.00223489f,  0.55195385f, -0.60461714f},  // 第0节: 低频增益
        {1.0f,         2.0f,         1.0f,          0.86036562f, -0.63511954f},  // 第1节: 谐振器 (b = [1,+2,1])
        {1.0f,        -2.0f,         1.0f,          0.37367240f, -0.81248708f},  // 第2节: 隔直 (b = [1,-2,1])
        {1.0f,        -2.0f,         1.0f,          1.15601175f, -0.84761589f},  // 第3节: 隔直 (b = [1,-2,1])
    };
    // clang-format on

    // 滤波器状态 — 每节 2 个延迟单元，零初始化
    float32_t biquad_state_[kNumBiquadStages][2] = {};

    // 包络检测: 环形缓冲区滑动平均
    int32_t circular_buffer_[kBufferSize] = {};
    int     buffer_index_ = 0;
    int32_t buffer_sum_   = 0;

    // 输出值
    int32_t semg_raw_value_      = 0;
    float   semg_filtered_value_ = 0.0f;
    int32_t semg_envelope_value_ = 0;

    // CMSIS-DSP 实例句柄 (构造函数中初始化)
    arm_biquad_cascade_df2T_instance_f32 biquad_inst_;

    /** 滑动平均包络: 最近 N 个整流采样点的均值 × 2 */
    int32_t CalcEnvelope(int32_t semg_abs_value);
};

#endif // SEMG_HPP