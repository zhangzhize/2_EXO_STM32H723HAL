/**
 * @file emg.hpp
 * @author Zhize Zhang (zhangzhize1@foxmail.com)
 * @brief 移植CheezsEMG
 * @version 0.1
 * @date 2026-04-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef SEMG_HPP
#define SEMG_HPP

#include <cstdint>

class Emg
{
public:
    Emg() {};
    ~Emg() = default;
    void Update(int32_t raw_adc_value);

    int32_t semg_raw_value() { return semg_raw_value_; };
    float semg_filtered_value() { return semg_filtered_value_; };
    int32_t semg_envelope_value() { return semg_envelope_value_; };

private:
    static const int kBufferSize = 32;
    
    int32_t circular_buffer_[kBufferSize] = {0};
    int buffer_index_ = 0;
    int32_t buffer_sum_ = 0;

    int32_t semg_raw_value_ = 0;
    int32_t semg_envelope_value_ = 0;
    float semg_filtered_value_ = 0.0f;

    float z1_[4] = {0.0f};
    float z2_[4] = {0.0f};

    float ButterWorthFilter(float semg_raw_value);
    int32_t CalculateEnvelope(int32_t semg_abs_value);
};



#endif // SEMG_HPP