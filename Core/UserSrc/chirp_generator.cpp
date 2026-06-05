extern "C" {
    #include "arm_math.h"
}
#include "chirp_generator.hpp"
#include <cmath>
#include "utils.h"

/**
 * @brief 构造并初始化参数 (委托 SetParams 完成整周期对齐)
 */
ChirpGenerator::ChirpGenerator(float start_freq_Hz, float end_freq_Hz, float duration_s)
    : start_freq_Hz_(start_freq_Hz), end_freq_Hz_(end_freq_Hz), duration_s_(duration_s), k_Hzps_(0.0f)
{
    SetParams(start_freq_Hz, end_freq_Hz, duration_s);
}

/**
 * @brief 推进扫频时间步
 * @param sys_us  当前系统微秒时间戳 (用于计算已流逝时间)
 * @return 正弦信号值 ∈ [-1.0, 1.0]，扫频结束返回 0
 *
 * 执行逻辑:
 *   1. 若已完成扫频 → 返回 0 (不自动重启)
 *   2. 首次调用 → 记录起始时间 tstart_us_
 *   3. 计算已流逝时间 telaps_s = (now - tstart) / 1e6
 *   4. 若超过 duration_s_ → 标记完成
 *   5. 计算当前时刻的累积圈数 revs，提取小数部分
 *   6. 由小数圈数计算正弦值
 */
float ChirpGenerator::Update(uint64_t sys_us)
{
    if (is_finished_)
    {
        return 0.0f;  /* 扫频已完成，输出 0 避免持续振荡 */
    }

    if (is_first_update_)
    {
        tstart_us_ = sys_us;
        is_first_update_ = false;
    }

    double telasp_s = (sys_us - tstart_us_) * 1e-6;
    if (telasp_s >= duration_s_)
    {
        is_finished_ = true;
    }

    /* 累积圈数: revs = f_start·t + 0.5·k·t²
       提取小数部分避免大数精度损失，用于计算 sin(2π·frac) */
    double revs = (double)start_freq_Hz_ * telasp_s + 0.5 * (double)k_Hzps_ * telasp_s * telasp_s;
    float frac_revs = (float)(revs - (int32_t)revs);

    return arm_sin_f32(_2PI * frac_revs);
}

/**
 * @brief 复位扫频状态
 *
 * 将起始时间戳和完成标记归位，is_first_update_ 设为 true
 * 表示下一次 Update 调用将记录新的起始时间。
 */
void ChirpGenerator::Reset()
{
    tstart_us_ = 0;
    is_first_update_ = true;
    is_finished_ = false;
}

/**
 * @brief 修改扫频参数并自动复位
 *
 * 整周期对齐逻辑:
 *   1. 计算平均频率 f_avg = (f_start + f_end) / 2
 *   2. 计算目标总周期数 N_target = f_avg × duration
 *   3. 四舍五入到最近整数 N_exact
 *   4. 反推实际持续时间: duration = N_exact / f_avg
 *
 * 这样可以保证扫频从 sin(0)=0 开始，在 sin(2π·N)=0 结束，
 * 避免相位不连续导致的信号突变。
 */
void ChirpGenerator::SetParams(float start_freq_Hz, float end_freq_Hz, float duration_s)
{
    start_freq_Hz_ = start_freq_Hz;
    end_freq_Hz_ = end_freq_Hz;

    /* 整周期对齐: 将总周期数圆整到最近整数 */
    float f_avg = (start_freq_Hz + end_freq_Hz) / 2.0f;
    float target_cycles = f_avg * duration_s;
    uint32_t exact_cycles = (uint32_t)(target_cycles + 0.5f);

    if (f_avg > 0.0001f)
    {
        duration_s_ = (float)exact_cycles / f_avg;  /* 反推实际 duration */
    }
    else
    {
        duration_s_ = duration_s;  /* f_avg ≈ 0 时保持原值，避免除零 */
    }

    /* 计算频率变化率 k = Δf / Δt */
    if (duration_s_ > 0.0001f)
    {
        k_Hzps_ = (end_freq_Hz_ - start_freq_Hz_) / duration_s_;
    }
    else
    {
        k_Hzps_ = 0.0f;  /* duration 过短时视为定频输出 */
    }

    Reset();
}