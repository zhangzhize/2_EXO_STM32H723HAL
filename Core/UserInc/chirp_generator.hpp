#ifndef CHIRP_GENERATOR_HPP
#define CHIRP_GENERATOR_HPP

#include <cstdint>

class ChirpGenerator {
public:
    ChirpGenerator(float start_freq_Hz, float end_freq_Hz, float duration_s);
    
    float Update(uint64_t sys_us);
    void Reset();
    void SetParams(float start_freq_Hz, float end_freq_Hz, float duration_s);

private:
    float start_freq_Hz_;        // 起始频率 (Hz)
    float end_freq_Hz_;         // 终止频率 (Hz)
    float duration_s_;          // 扫频总时间 (s)
    float k_Hzps_;               // 频率变化率 (Hz/s)
    uint64_t tstart_us_ = 0;             // 当前运行时间 (us)
    bool is_first_update_ = true;
    bool is_finished_ = false;          // 标记是否已完成一次扫频，防止自动重启
};

#endif // CHIRP_GENERATOR_HPP