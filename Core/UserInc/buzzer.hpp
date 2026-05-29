#ifndef BUZZER_HPP
#define BUZZER_HPP

#include <cstdint>
#include "tim.h"

class Buzzer
{
public:
    explicit Buzzer(TIM_HandleTypeDef &htim) : htim_(htim) {}
    ~Buzzer() = default;

    void Init();
    void DeInit();
    void Tick();  // 非阻塞, 每 1ms 调一次

    /// 状态切换时调用, 根据新状态排队对应声音
    void OnStateChange(uint8_t new_state);

private:
    static constexpr float kTimerClkHz = 10e6f;   // 240MHz / 24

    struct Tone {
        uint16_t freq_hz;     // 0 表示静音
        uint16_t duration_ms;
    };

    void PlayTone(const Tone &tone);
    void Mute();
    void SetFreqAndStart(uint16_t freq_hz);

    TIM_HandleTypeDef &htim_;
    bool is_on_ = false;

    // 音调队列 (简单双缓冲: 当前 + 下一个, 足够用)
    static constexpr uint8_t kQueueSize = 8;
    Tone queue_[kQueueSize] = {};
    uint8_t q_head_ = 0;
    uint8_t q_tail_ = 0;
    uint32_t tone_start_ms_ = 0;
};

#endif
