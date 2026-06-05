/**
 * @file chirp_generator.hpp
 * @brief 线性扫频信号发生器 (Chirp Generator)
 *
 * 生成频率随时间线性变化的正弦信号，用于外骨骼系统辨识激励。
 * 频率从 start_freq_Hz 线性变化到 end_freq_Hz，持续 duration_s 秒。
 *
 * 信号模型: y(t) = sin(2π · (f_start·t + 0.5·k·t²))
 * 其中 k = (f_end - f_start) / duration 为频率变化率 (Hz/s)
 *
 * 特性:
 *   - 自动对齐完整周期: SetParams 中微调 duration 使起始/结束相位连续
 *   - 单向运行: 一次扫频完成后自动停振 (is_finished_ = true)
 *   - 浮点相位增量: 使用小数圈数 (frac_revs) 而非累积相位，避免大数精度下降
 *   - 调用 CMSIS-DSP arm_sin_f32 计算正弦值
 */
#ifndef CHIRP_GENERATOR_HPP
#define CHIRP_GENERATOR_HPP

#include <cstdint>

class ChirpGenerator {
public:
    /**
     * @brief 构造扫频信号发生器
     * @param start_freq_Hz  起始频率 (Hz)
     * @param end_freq_Hz    终止频率 (Hz)
     * @param duration_s     扫频持续时间 (s)，会被微调以对齐整周期
     */
    ChirpGenerator(float start_freq_Hz, float end_freq_Hz, float duration_s);

    /**
     * @brief 推进一个时间步，返回当前信号值
     * @param  sys_us  系统微秒时间戳
     * @return 当前正弦值 ∈ [-1, 1]，扫频完成后返回 0
     */
    float Update(uint64_t sys_us);

    /** @brief 复位扫频状态，准备新一轮扫频 */
    void Reset();

    /**
     * @brief 在线修改扫频参数 (频率范围、时长)
     * @note 修改参数后自动 Reset，需重新开始扫频
     */
    void SetParams(float start_freq_Hz, float end_freq_Hz, float duration_s);

private:
    float start_freq_Hz_;    /*!< 起始频率 (Hz) */
    float end_freq_Hz_;      /*!< 终止频率 (Hz) */
    float duration_s_;       /*!< 扫频总时长 (s)，已对齐整周期 */
    float k_Hzps_;           /*!< 频率变化率 (Hz/s), k = (f_end - f_start) / duration */

    uint64_t tstart_us_ = 0;       /*!< 扫频起始时间戳 (us) */
    bool is_first_update_ = true;  /*!< 首次调用标记，用于记录起始时间 */
    bool is_finished_ = false;     /*!< 扫频完成标记，完成后输出恒为 0 */
};

#endif // CHIRP_GENERATOR_HPP