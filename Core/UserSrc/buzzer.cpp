#include "buzzer.hpp"
#include "utils.h"

/**
 * @brief 初始化蜂鸣器
 *
 * 先执行 DeInit 清理状态，再启动 PWM 输出通道。
 * 启动后蜂鸣器处于静音状态 (CCR2 = 0)，等待 OnStateChange 排入音调。
 */
void Buzzer::Init()
{
    DeInit();
    HAL_TIM_PWM_Start(&htim_, TIM_CHANNEL_2);
}

/**
 * @brief 停用蜂鸣器
 *
 * 停止 PWM 输出，重置 CCR2 为 0 (彻底静音)，清空队列状态。
 */
void Buzzer::DeInit()
{
    HAL_TIM_PWM_Stop(&htim_, TIM_CHANNEL_2);
    htim_.Instance->CCR2 = 0;    /* 直接写寄存器确保输出低电平 */
    is_on_ = false;
    q_head_ = 0;
    q_tail_ = 0;
}

/**
 * @brief 音调调度 Tick (每 1ms 调用)
 *
 * 工作流程：
 * 1. 若队列为空则直接返回
 * 2. 若当前 Tone 尚未开始 (is_on_ == false)，设置频率并记录开始时间
 * 3. 检查当前 Tone 是否已经到时 (now_ms - tone_start_ms_ >= duration_ms)
 * 4. 到时时切换到队列下一个 Tone，若队列结束则静音
 *
 * 注意：使用无符号减法 (now_ms - tone_start_ms_) 处理 32 位毫秒回绕，
 * 只要单次 Tone 时长不超过 ~49.7 天就安全。
 */
void Buzzer::Tick()
{
    if (q_head_ == q_tail_) return;   /* 队列空，无音调待播放 */

    Tone &cur = queue_[q_head_];
    uint32_t now_ms = GetSysTimeMs();

    if (!is_on_)
    {
        /* 新音调开始：设置 PWM 频率并记录起始时刻 */
        if (cur.freq_hz > 0)
        {
            SetFreqAndStart(cur.freq_hz);
            is_on_ = true;
        }
        tone_start_ms_ = now_ms;
    }

    if (now_ms - tone_start_ms_ >= cur.duration_ms)
    {
        /* 当前音调到时，切换到下一个 */
        q_head_ = (q_head_ + 1) % kQueueSize;
        is_on_ = false;

        if (q_head_ != q_tail_)
        {
            /* 队列中还有下一个 Tone，立即开始播放 */
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
            Mute();   /* 队列播放完毕，静音 */
        }
    }
}

/**
 * @brief 系统状态切换回调
 *
 * 根据 ExoData::State 枚举值清空旧队列并排入对应的音调序列。
 * 各状态声音定义：
 * - kSleep (0): 短促低音 2000Hz/80ms，提示进入休眠
 * - kWaitMotorComm (1) / kCalibrating (2): 无声音
 * - kReady (3): 短促高音 3000Hz/60ms，提示就绪
 * - kAssisting (4): 两短一长 3000Hz，提示开始助力
 * - kFaultLowBattery (5): 周期性 1000Hz/150ms + 2000ms 静音，低电量告警
 * - kFaultSystem (6): 长鸣 1200Hz/600ms，系统故障告警
 *
 * @param new_state  ExoData::State 枚举值
 */
void Buzzer::OnStateChange(uint8_t new_state)
{
    /* 清空旧队列并立即静音 */
    Mute();
    q_head_ = 0;
    q_tail_ = 0;
    is_on_ = false;

    switch (new_state)
    {
    case 0:  /* kSleep —— 进入休眠：短促低音 */
        queue_[q_tail_++] = {2000, 80};
        q_tail_ %= kQueueSize;
        break;

    case 2:  /* kCalibrating —— 无声音 */
    case 1:  /* kWaitMotorComm —— 无声音 */
        break;

    case 3:  /* kReady —— 就绪：短促高音 */
        queue_[q_tail_++] = {3000, 60};
        q_tail_ %= kQueueSize;
        break;

    case 4:  /* kAssisting —— 开始助力：两短一长 (嘟-嘟-嘟嘟) */
        queue_[q_tail_++] = {3000, 60};
        q_tail_ %= kQueueSize;
        queue_[q_tail_++] = {0, 50};
        q_tail_ %= kQueueSize;
        queue_[q_tail_++] = {3000, 60};
        q_tail_ %= kQueueSize;
        break;

    case 5:  /* kFaultLowBattery —— 低电量：周期性短鸣 */
        /* 初始化一声音调，Tick 中会周期性补充后续 Tone */
        queue_[q_tail_++] = {1000, 150};
        q_tail_ %= kQueueSize;
        queue_[q_tail_++] = {0, 2000};
        q_tail_ %= kQueueSize;
        break;

    case 6:  /* kFaultSystem —— 系统故障：长鸣 600ms */
        queue_[q_tail_++] = {1200, 600};
        q_tail_ %= kQueueSize;
        break;

    default:
        break;
    }
}

/**
 * @brief 播放单个音调 (直接控制，不经过队列)
 *
 * freq_hz == 0 时静音，否则设置频率并开始输出。
 */
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

/**
 * @brief 静音：将 PWM 占空比置零
 *
 * 直接写 CCR2 寄存器为 0，不停止 PWM 通道 (保持定时器运行以维持 ARR 设置)。
 */
void Buzzer::Mute()
{
    htim_.Instance->CCR2 = 0;
    is_on_ = false;
}

/**
 * @brief 设置 PWM 频率并启动输出
 *
 * 计算流程：
 * 1. 频率安全限幅至 [20, 5000] Hz
 * 2. 计算 ARR = kTimerClkHz / freq_hz - 1
 * 3. ARR 限幅至 [10, 65535] (16 位定时器限制 + 防止分频为 0)
 * 4. 更新 ARR 和 CCR2 (50% 占空比)
 * 5. 启动 PWM 通道
 *
 * @param freq_hz  目标频率 (Hz)
 */
void Buzzer::SetFreqAndStart(uint16_t freq_hz)
{
    if (freq_hz < 20) freq_hz = 20;       /* 下限 20Hz：防止人耳听不见的低频，同时防止 ARR 过大溢出 */
    if (freq_hz > 5000) freq_hz = 5000;   /* 上限 5kHz：常用蜂鸣器谐振频率附近，无需更高 */

    /* 计算自动重载值：ARR = f_timer / f_pwm - 1，并限幅 */
    uint16_t arr = static_cast<uint16_t>(kTimerClkHz / freq_hz) - 1;
    if (arr < 10) arr = 10;               /* 防止分频系数过小导致波形失真 */
    if (arr > 65535) arr = 65535;         /* 16 位定时器上限 */

    /* 直接操作定时器寄存器以快速更新频率 */
    __HAL_TIM_SET_AUTORELOAD(&htim_, arr);
    __HAL_TIM_SET_COMPARE(&htim_, TIM_CHANNEL_2, arr / 2);  /* 50% 占空比声音最响 */

    /* 确保 PWM 通道已启动 */
    HAL_TIM_PWM_Start(&htim_, TIM_CHANNEL_2);
}
