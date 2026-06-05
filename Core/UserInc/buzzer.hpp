/**
 * @file    buzzer.hpp
 * @brief   有源蜂鸣器音调控制模块
 *
 * 使用定时器 PWM 通道驱动无源蜂鸣器，通过频率和时长组合实现不同提示音。
 *
 * 设计要点：
 * - 非阻塞驱动：Tick() 每 1ms 调用一次，在当前音调到时后自动切换下一音调
 * - 环形音调队列：最多 8 个 Tone 的 FIFO 队列，支持连续的频率-静音编排
 * - 状态驱动：OnStateChange() 根据系统状态枚举值清空旧队列并排入对应的声音序列
 * - 频率安全限幅：20Hz ~ 5kHz 硬件限幅，ARR 寄存器限幅 10 ~ 65535
 */
#ifndef BUZZER_HPP
#define BUZZER_HPP

#include <cstdint>
#include "tim.h"

/**
 * @class Buzzer
 * @brief 蜂鸣器音调队列控制器
 *
 * 工作流程：
 * 1. Init() 初始化 PWM 并启动定时器通道
 * 2. OnStateChange(state) 清空队列并根据状态排入对应音调序列
 * 3. Tick() 在主循环中每 1ms 调用，检查当前音调是否到期并切换到下一个
 * 4. DeInit() 停止 PWM 输出，关闭蜂鸣器
 */
class Buzzer
{
public:
    explicit Buzzer(TIM_HandleTypeDef &htim) : htim_(htim) {}
    ~Buzzer() = default;

    /* 初始化 PWM 输出通道 (启动蜂鸣器驱动) */
    void Init();
    /* 停用 PWM 输出通道并清空队列 (关闭蜂鸣器) */
    void DeInit();
    /* 每 1ms 调一次：检查当前音调是否到时并自动切换到下一个 (非阻塞) */
    void Tick();

    /* 系统状态切换时调用，根据新状态清空并排入对应的音调序列 */
    void OnStateChange(uint8_t new_state);

private:
    /* 定时器输入时钟频率 (Hz): 240MHz PCLK / 24 预分频 = 10MHz */
    static constexpr float kTimerClkHz = 10e6f;

    /* 单个音调定义 */
    struct Tone {
        uint16_t freq_hz;       /*!< 频率 (Hz)，0 表示静音 */
        uint16_t duration_ms;   /*!< 持续时长 (ms) */
    };

    /* 播放单个 Tone (设置频率并记录开始时间) */
    void PlayTone(const Tone &tone);
    /* 静音：将 PWM 占空比设为 0 */
    void Mute();
    /* 设置 PWM 频率并启动输出 */
    void SetFreqAndStart(uint16_t freq_hz);

    TIM_HandleTypeDef &htim_;       /*!< 驱动蜂鸣器的定时器句柄引用 */
    bool is_on_ = false;            /*!< 当前是否正在发声 */

    /* 环形音调队列容量 */
    static constexpr uint8_t kQueueSize = 8;
    Tone queue_[kQueueSize] = {};   /*!< 音调环形队列 */
    uint8_t q_head_ = 0;            /*!< 队列头索引 (当前播放位置) */
    uint8_t q_tail_ = 0;            /*!< 队列尾索引 (下一个入队位置) */
    uint32_t tone_start_ms_ = 0;    /*!< 当前音调开始时刻 (ms) */
};

#endif
