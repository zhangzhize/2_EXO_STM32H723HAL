#include "semg.hpp"
#include "math.h"

void Emg::Update(int32_t raw_adc_value)
{
    semg_raw_value_ = raw_adc_value;
    semg_filtered_value_ = ButterWorthFilter((float)semg_raw_value_);
    semg_envelope_value_ = CalculateEnvelope((int32_t)fabsf(semg_filtered_value_));
}

float Emg::ButterWorthFilter(float semg_raw_value)
{
    float output = semg_raw_value;
    {
        float x = output - (-0.55195385f * z1_[0]) - (0.60461714f * z2_[0]);
        output = 0.00223489f * x + (0.00446978f * z1_[0]) + (0.00223489f * z2_[0]);
        z2_[0] = z1_[0];
        z1_[0] = x;
    }
    {
        float x = output - (-0.86036562f * z1_[1]) - (0.63511954f * z2_[1]);
        output = 1.00000000f * x + (2.00000000f * z1_[1]) + (1.00000000f * z2_[1]);
        z2_[1] = z1_[1];
        z1_[1] = x;
    }
    {
        float x = output - (-0.37367240f * z1_[2]) - (0.81248708f * z2_[2]);
        output = 1.00000000f * x + (-2.00000000f * z1_[2]) + (1.00000000f * z2_[2]);
        z2_[2] = z1_[2];
        z1_[2] = x;
    }
    {
        float x = output - (-1.15601175f * z1_[3]) - (0.84761589f * z2_[3]);
        output = 1.00000000f * x + (-2.00000000f * z1_[3]) + (1.00000000f * z2_[3]);
        z2_[3] = z1_[3];
        z1_[3] = x;
    }
    return output;
}

int32_t Emg::CalculateEnvelope(int32_t semg_abs_value)
{
    buffer_sum_ -= circular_buffer_[buffer_index_];
    buffer_sum_ += semg_abs_value;
    circular_buffer_[buffer_index_] = semg_abs_value;
    buffer_index_ = (buffer_index_ + 1) % kBufferSize;

    return (buffer_sum_ / kBufferSize) * 2;
}

