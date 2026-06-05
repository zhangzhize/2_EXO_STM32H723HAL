/**
 * @file mahony.hpp
 * @brief Mahony AHRS 姿态解算算法 — 基于四元数的互补滤波
 *
 * 使用陀螺仪角速度积分 + 加速度计/磁力计修正的互补滤波框架估计空间姿态。
 * 支持 6 轴 (陀螺仪+加速度计) 和 9 轴 (陀螺仪+加速度计+磁力计) 两种模式。
 * 输出四元数及欧拉角 (roll/pitch/yaw)，适用于外骨骼 IMU 姿态估计。
 *
 * 原始算法: SOH Madgwick, 2011 — http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 * 移植: Zhize Zhang, 2026/04/02 — 适配 STM32H723
 *
 * 关键参数:
 *   - Kp (比例增益): 决定陀螺仪漂移修正的速度，默认 6.0
 *   - Ki (积分增益): 决定陀螺仪零偏估计的速度，默认 0.1
 *
 * 快速反平方根: 使用 Quake III 经典算法 InvSqrt，避免浮点除法开销
 */
#ifndef MAHONY_HPP
#define MAHONY_HPP

class Mahony
{
public:
    /**
     * @brief 构造 Mahony 滤波器
     * @param sample_freq 传感器采样频率 (Hz)，用于积分步长计算
     */
    Mahony(float sample_freq) : inv_sample_freq_(1.0f / sample_freq) {};
    ~Mahony() = default;

    /**
     * @brief 9 轴姿态更新 (陀螺仪 + 加速度计 + 磁力计)
     * @param gyro_degps_  陀螺仪三轴角速度 (deg/s)
     * @param accel_mps2_  加速度计三轴加速度 (m/s²)
     * @param magnet_uT_   磁力计三轴磁场强度 (uT)
     * @note 磁力计数据全零时自动降级为 6 轴模式
     */
    void Update9Axis(float gyro_degps_[3], float accel_mps2_[3], float magnet_uT_[3]);

    /**
     * @brief 6 轴姿态更新 (陀螺仪 + 加速度计)
     * @param gyro_degps_  陀螺仪三轴角速度 (deg/s)
     * @param accel_mps2_  加速度计三轴加速度 (m/s²)
     * @note 仅使用重力矢量修正 pitch/roll，yaw 无绝对参考
     */
    void Update6Axis(float gyro_degps_[3], float accel_mps2_[3]);

    /** @brief 获取欧拉角 (度)，内部惰性计算四元数→欧拉角转换 */
    void GetEulerAnglesDeg(float& roll_deg, float& pitch_deg, float& yaw_deg)
    {
        if (!is_angle_computed_) ComputeAngles();
        roll_deg = roll_rad_ * 57.29578f;
        pitch_deg = pitch_rad_ * 57.29578f;
        yaw_deg = yaw_rad_ * 57.29578f;
    }

    /** @brief 获取欧拉角 (弧度)，内部惰性计算四元数→欧拉角转换 */
    void GetEulerAnglesRad(float& roll_rad, float& pitch_rad, float& yaw_rad)
    {
        if (!is_angle_computed_) ComputeAngles();
        roll_rad = roll_rad_;
        pitch_rad = pitch_rad_;
        yaw_rad = yaw_rad_;
    }

    /** @brief 获取当前姿态四元数 (w, x, y, z)，传感器坐标系相对于世界坐标系 */
    void GetQuaternion(float q[4]) const
    {
        q[0] = q_[0];
        q[1] = q_[1];
        q[2] = q_[2];
        q[3] = q_[3];
    }

private:
    float inv_sample_freq_;                          /*!< 采样频率的倒数 (积分步长) */

    float two_Kp_ = (2.0f * 6.0f);                   /*!< 2 × 比例增益 (Kp)，控制互补滤波修正强度 */
    float two_Ki_ = (2.0f * 0.1f);                   /*!< 2 × 积分增益 (Ki)，控制陀螺零偏修正强度 */

    float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};         /*!< 姿态四元数 (w, x, y, z)，初值为单位四元数 */

    float integral_FBx_ = 0.0f;                      /*!< 陀螺仪 x 轴零偏积分项 */
    float integral_FBy_ = 0.0f;                      /*!< 陀螺仪 y 轴零偏积分项 */
    float integral_FBz_ = 0.0f;                      /*!< 陀螺仪 z 轴零偏积分项 */

    float roll_rad_ = 0.0f;                          /*!< 缓存: Roll 角 (rad) */
    float pitch_rad_ = 0.0f;                         /*!< 缓存: Pitch 角 (rad) */
    float yaw_rad_ = 0.0f;                           /*!< 缓存: Yaw 角 (rad) */
    bool is_angle_computed_ = false;                 /*!< 欧拉角是否已从四元数计算，惰性求值标记 */

    /** @brief 快速反平方根 (Quake III 经典算法)，替代 1/sqrt(x) 避免除法开销 */
    static float InvSqrt(float x);

    /** @brief 由四元数计算欧拉角 roll/pitch/yaw (Tait-Bryan, ZYX 顺序) */
    void ComputeAngles();
};


#endif
