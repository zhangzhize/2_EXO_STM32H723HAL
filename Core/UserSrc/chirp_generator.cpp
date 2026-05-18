extern "C" {
    #include "arm_math.h"
}
#include "chirp_generator.hpp"
#include <cmath>
#include "utils.h"

ChirpGenerator::ChirpGenerator(float start_freq_Hz, float end_freq_Hz, float duration_s)
    : start_freq_Hz_(start_freq_Hz), end_freq_Hz_(end_freq_Hz), duration_s_(duration_s), k_Hzps_(0.0f)
{
    SetParams(start_freq_Hz, end_freq_Hz, duration_s);
}

float ChirpGenerator::Update(uint64_t sys_us)
{
    if (is_finished_)
    {
        return 0.0f;
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

    double revs = (double)start_freq_Hz_ * telasp_s + 0.5 * (double)k_Hzps_ * telasp_s * telasp_s;
    float frac_revs = (float)(revs - (int32_t)revs);

    return arm_sin_f32(_2PI * frac_revs);
}

void ChirpGenerator::Reset()
{
    tstart_us_ = 0;
    is_first_update_ = true;
    is_finished_ = false;
}

void ChirpGenerator::SetParams(float start_freq_Hz, float end_freq_Hz, float duration_s)
{
    start_freq_Hz_ = start_freq_Hz;
    end_freq_Hz_ = end_freq_Hz;

    float f_avg = (start_freq_Hz + end_freq_Hz) / 2.0f;
    float target_cycles = f_avg * duration_s;
    uint32_t exact_cycles = (uint32_t)(target_cycles + 0.5f);

    if (f_avg > 0.0001f)
    {
        duration_s_ = (float)exact_cycles / f_avg;
    }
    else
    {
        duration_s_ = duration_s;
    }

    if (duration_s_ > 0.0001f)
    {
        k_Hzps_ = (end_freq_Hz_ - start_freq_Hz_) / duration_s_;
    }
    else
    {
        k_Hzps_ = 0.0f;
    }
    
    Reset();
}