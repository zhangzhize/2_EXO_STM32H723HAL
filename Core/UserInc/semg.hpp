/**
 * @file semg.hpp
 * @author Zhize Zhang (zhangzhize1@foxmail.com)
 * @brief sEMG 表面肌电信号处理 — CMSIS-DSP 巴特沃斯带通滤波 + 包络检测
 *
 * 信号链: 原始 ADC → 4 阶巴特沃斯带通 (CMSIS-DSP arm_biquad_cascade_df2T_f32) →
 * 全波整流 → 32 点滑动平均包络
 *
 * 滤波系数用 Python 脚本设计: test/design_biquad_filter.py
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
 * 信号处理流程:
 *   1. 原始 ADC 采样值 → float32 转换
 *   2. 4 阶巴特沃斯带通滤波 (CMSIS-DSP arm_biquad_cascade_df2T_f32)
 *   3. 全波整流 (fabsf)
 *   4. 32 点滑动平均包络检测 (O(1) 增量算法)
 *
 * 重要 — CMSIS-DSP DF2T 系数符号约定:
 *   标准 scipy SOS 格式:   H(z) = N(z) / (1 + a1·z⁻¹ + a2·z⁻²)
 *   CMSIS-DSP DF2T 格式:   H(z) = N(z) / (1 - a1·z⁻¹ - a2·z⁻²)
 *   (CMSIS 内部状态更新: d1' = b1·x + a1·y + d2, d2' = b2·x + a2·y)
 *   design_biquad_filter.py 已自动对 a1/a2 取反，生成的系数可直接使用。
 *
 * 默认系数: 采样率 fs=250 Hz, 通带 35-55 Hz, 4 阶巴特沃斯
 *   (CMSIS-DSP DF2T 格式, a1/a2 已取反)
 */
class sEmg
{
public:
    sEmg();
    ~sEmg() = default;

    /** @brief 处理一个原始 ADC 采样值，依次完成带通滤波 → 整流 → 包络检测 */
    void Update(int32_t raw_adc_value);

    /**
     * @brief 运行时动态替换滤波器系数 (用于在线调参或切换滤波器配置)
     * @param coeffs  扁平化系数数组，长度 = 5 × num_stages
     *                每节格式: {b0, b1, b2, a1, a2} (CMSIS-DSP DF2T 格式, a 已取反)
     * @param num_stages  双二阶级联节数，不得超过 kNumBiquadStages
     * @note 替换系数时会同时清零状态缓冲区，避免瞬态振荡
     */
    void SetFilterCoeffs(const float32_t *coeffs, uint32_t num_stages);

    /** @brief 重置滤波器状态、包络环形缓冲区和输出值 */
    void Reset();

    /* 访问器 — 对外暴露最新的信号处理结果 */
    int32_t semg_raw_value()      const { return semg_raw_value_; }
    float   semg_filtered_value() const { return semg_filtered_value_; }
    int32_t semg_envelope_value() const { return semg_envelope_value_; }

private:
    static constexpr int kNumBiquadStages = 4;  /*!< 双二阶级联节数 (4 阶 = 2 节 × 2 阶/节 = 4 个一阶节) */
    static constexpr int kBufferSize      = 32; /*!< 包络滑动平均窗口长度 (采样点) */

    /**
     * 默认双二阶系数 — 采样率 250 Hz, 35-55 Hz 带通, 4 阶巴特沃斯
     *
     * CMSIS-DSP DF2T 格式: a1/a2 已从 scipy SOS 标准取反。
     * 生成命令:
     *   python test/design_biquad_filter.py --fs 250 --low 35 --high 55 --order 4 --var bq
     *
     * 每节含义:
     *   第 0 节: 低频增益级 (Normalized LPF 原型 → BPF 变换)
     *   第 1 节: 谐振级 (b = [1,+2,1])
     *   第 2 节: 低频隔直级 (b = [1,-2,1])
     *   第 3 节: 高频隔直级 (b = [1,-2,1])
     */
    // clang-format off
    float32_t biquad_coeffs_[kNumBiquadStages][5] = {
        {0.00223489f,  0.00446978f,  0.00223489f,  0.55195385f, -0.60461714f},
        {1.0f,         2.0f,         1.0f,          0.86036562f, -0.63511954f},
        {1.0f,        -2.0f,         1.0f,          0.37367240f, -0.81248708f},
        {1.0f,        -2.0f,         1.0f,          1.15601175f, -0.84761589f},
    };
    // clang-format on

    float32_t biquad_state_[kNumBiquadStages][2] = {}; /*!< DF2T 状态: 每节 2 个延迟单元 (d1, d2)，零初始化 */

    /* 包络检测 — 环形缓冲区滑动平均 (O(1) 增量更新) */
    int32_t circular_buffer_[kBufferSize] = {};  /*!< 32 点全波整流值环形缓冲区 */
    int     buffer_index_ = 0;                   /*!< 缓冲区当前写入位置 */
    int32_t buffer_sum_   = 0;                   /*!< 缓冲区元素总和 (增量维护，避免每次遍历) */

    /* 最新输出值 */
    int32_t semg_raw_value_      = 0;     /*!< 原始 ADC 采样值 */
    float   semg_filtered_value_ = 0.0f;  /*!< 带通滤波输出 (float32) */
    int32_t semg_envelope_value_ = 0;     /*!< 包络值 (滤波后全波整流 + 滑动平均 × 2) */

    arm_biquad_cascade_df2T_instance_f32 biquad_inst_; /*!< CMSIS-DSP 双二阶级联滤波器实例句柄 */

    /**
     * @brief 增量式滑动平均包络计算 (O(1) 复杂度)
     * @param  semg_abs_value  全波整流后的绝对值
     * @return 包络值 = (最近 N 个整流值的均值) × 2
     *
     * 算法: 维护 32 点环形缓冲区和元素总和。每次更新:
     *   sum = sum - old + new;  buffer[idx] = new;  idx = (idx + 1) % N
     * 相比 O(N) 遍历方案显著节省 CPU，适合 1 kHz 实时肌电采集。
     */
    int32_t CalcEnvelope(int32_t semg_abs_value);
};

#endif // SEMG_HPP