#include "buzzer.hpp"
#include "utils.h"

void Buzzer::Init()
{
    DeInit();
    HAL_TIM_PWM_Start(&htim_, TIM_CHANNEL_2);
}

void Buzzer::DeInit()
{
    HAL_TIM_PWM_Stop(&htim_, TIM_CHANNEL_2);
    htim_.Instance->CCR2 = 0;
    is_on_ = false;
    q_head_ = 0;
    q_tail_ = 0;
}

void Buzzer::Tick()
{
    if (q_head_ == q_tail_) return;  // 队列空

    Tone &cur = queue_[q_head_];
    uint32_t now_ms = GetSysTimeMs();

    if (!is_on_)
    {
        // 新音调开始
        if (cur.freq_hz > 0)
        {
            SetFreqAndStart(cur.freq_hz);
            is_on_ = true;
        }
        tone_start_ms_ = now_ms;
    }

    if (now_ms - tone_start_ms_ >= cur.duration_ms)
    {
        // 当前音调结束, 切换到下一个
        q_head_ = (q_head_ + 1) % kQueueSize;
        is_on_ = false;

        if (q_head_ != q_tail_)
        {
            // 还有下一个, 立即开始
            Tone &next = queue_[q_head_];
            if (next.freq_hz > 0)
            {
                SetFreqAndStart(next.freq_hz);
                is_on_ = true;
            }
            tone_start_ms_ = now_ms;
        }
        else
        {
            Mute();
        }
    }
}

void Buzzer::OnStateChange(uint8_t new_state)
{
    // 清空队列
    Mute();
    q_head_ = 0;
    q_tail_ = 0;
    is_on_ = false;

    // new_state 对应 ExoData::State 枚举值
    switch (new_state)
    {
    case 0:  // kSleep — 进入休眠
        queue_[q_tail_++] = {2000, 80};
        q_tail_ %= kQueueSize;
        break;

    case 2:  // kCalibrating — 无声音
    case 1:  // kWaitMotorComm — 无声音
        break;

    case 3:  // kReady — 就绪
        queue_[q_tail_++] = {3000, 60};
        q_tail_ %= kQueueSize;
        break;

    case 4:  // kAssisting — 开始助力
        queue_[q_tail_++] = {3000, 60};
        q_tail_ %= kQueueSize;
        queue_[q_tail_++] = {0, 50};
        q_tail_ %= kQueueSize;
        queue_[q_tail_++] = {3000, 60};
        q_tail_ %= kQueueSize;
        break;

    case 5:  // kFaultLowBattery — 周期性短鸣 (Tick 中维护)
        // 初始化一声音调, tick 会周期性补充
        queue_[q_tail_++] = {1000, 150};
        q_tail_ %= kQueueSize;
        queue_[q_tail_++] = {0, 2000};
        q_tail_ %= kQueueSize;
        break;

    case 6:  // kFaultSystem — 长鸣
        queue_[q_tail_++] = {1200, 600};
        q_tail_ %= kQueueSize;
        break;

    default:
        break;
    }
}

void Buzzer::PlayTone(const Tone &tone)
{
    if (tone.freq_hz == 0)
    {
        Mute();
    }
    else
    {
        SetFreqAndStart(tone.freq_hz);
        is_on_ = true;
    }
}

void Buzzer::Mute()
{
    htim_.Instance->CCR2 = 0;
    is_on_ = false;
}

void Buzzer::SetFreqAndStart(uint16_t freq_hz)
{
    if (freq_hz < 20) freq_hz = 20;       // 下限 20Hz
    if (freq_hz > 5000) freq_hz = 5000;   // 上限 5kHz

    uint16_t arr = static_cast<uint16_t>(kTimerClkHz / freq_hz) - 1;
    if (arr < 10) arr = 10;
    if (arr > 65535) arr = 65535;

    __HAL_TIM_SET_AUTORELOAD(&htim_, arr);
    __HAL_TIM_SET_COMPARE(&htim_, TIM_CHANNEL_2, arr / 2);

    // 如果之前没开 PWM, 启动
    HAL_TIM_PWM_Start(&htim_, TIM_CHANNEL_2);
}
