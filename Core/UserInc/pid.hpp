/**
 * @file pid.hpp
 * @brief PID 控制器 — 位置式 PID + Tustin 积分 + 输出斜坡限制 + 抗积分饱和
 *
 * 采用 Tustin（双线性变换）离散化积分项，相比前向/后向欧拉法具有更高的精度。
 * 同时提供输出变化率斜坡限制（output_ramp），防止输出阶跃对执行器造成冲击。
 *
 * 参考: STM32 Motor Control SDK 的 PID 实现风格
 */
#ifndef PID_HPP
#define PID_HPP

#include <cstdint>

/**
 * @brief 增量式位置型 PID 控制器
 *
 * 控制律: u(k) = Kp·e(k) + Ki·Ts/2·(e(k)+e(k-1)) + Kd·(e(k)-e(k-1))/Ts
 * - 积分项使用 Tustin 双线性变换离散化，精度优于欧拉法
 * - 输出限幅 (output_limit) 防止积分饱和（anti-windup clamping）
 * - 输出变化率限幅 (output_ramp) 提供平滑的输出过渡
 */
class PIDController
{
public:
    /**
     * @brief 构造 PID 控制器
     * @param kp           比例增益
     * @param ki           积分增益
     * @param kd           微分增益
     * @param output_ramp  输出最大变化速率（单位/s），0 表示不限幅
     * @param output_limit 输出绝对值上限，同时用于积分抗饱和钳位
     */
    explicit PIDController(float kp, float ki, float kd, float output_ramp, float output_limit)
    : kp_(kp), ki_(ki), kd_(kd), output_ramp_(output_ramp), output_limit_(output_limit) {}
    virtual ~PIDController() = default;

    /**
     * @brief 执行一次 PID 计算
     * @param error 当前误差 = 目标值 - 测量值
     * @return PID 输出值
     */
    float operator()(float error);

    /** @brief 复位积分器及历史状态，用于模式切换或初始化 */
    void ResetError(void);

    float kp_;                       /*!< 比例增益 Kp */
    float ki_;                       /*!< 积分增益 Ki */
    float kd_;                       /*!< 微分增益 Kd */
    float output_ramp_;              /*!< 输出最大变化速率 (单位/s)，0 表示不限制 */
    float output_limit_;             /*!< 输出绝对值上限，同时钳位积分分量 */
private:
    float error_prev_ = 0.0f;        /*!< 上一次的跟踪误差 e(k-1) */
    float output_prev_ = 0.0f;       /*!< 上一次 PID 输出 u(k-1) */
    float integral_prev_ = 0.0f;     /*!< 上一次积分分量 u_i(k-1) */
    unsigned long timestamp_prev_ = 0; /*!< 上一次调用的系统时间戳 (us) */
};

#endif