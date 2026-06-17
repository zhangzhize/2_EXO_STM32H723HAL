#include "mahony.hpp"
#include "math.h"
// #include "arm_math.h"       //ZZZ: Not Used


void Mahony::Update9Axis(float gyro_radps_[3], float accel_mps2_[3], float magnet_uT_[3])
{
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  float mx = magnet_uT_[0];
  float my = magnet_uT_[1];
  float mz = magnet_uT_[2];

  /* 磁力计数据全零 → 降级为 6 轴模式，避免归一化时出现 NaN */
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
  {
    Update6Axis(gyro_radps_, accel_mps2_);
    return;
  }

  /* 陀螺仪 deg/s → rad/s 单位转换 (π/180) */
  float gx = gyro_radps_[0];
  float gy = gyro_radps_[1];
  float gz = gyro_radps_[2];

  float ax = accel_mps2_[0];
  float ay = accel_mps2_[1];
  float az = accel_mps2_[2];

  /* 仅当加速度计数据有效 (非全零) 时计算反馈修正，避免归一化 NaN */
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    /* 加速度计归一化 → 得到测量重力方向单位向量 */
    recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* 磁力计归一化 → 得到测量地磁方向单位向量 */
    recipNorm = InvSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    /* 预计算四元数乘积项，减少重复运算 */
    q0q0 = q_[0] * q_[0];
    q0q1 = q_[0] * q_[1];
    q0q2 = q_[0] * q_[2];
    q0q3 = q_[0] * q_[3];
    q1q1 = q_[1] * q_[1];
    q1q2 = q_[1] * q_[2];
    q1q3 = q_[1] * q_[3];
    q2q2 = q_[2] * q_[2];
    q2q3 = q_[2] * q_[3];
    q3q3 = q_[3] * q_[3];

    /* 地磁场参考方向: 将磁力计测量旋转到水平面，得到北向(bx)和垂向(bz)分量 */
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    /* 由当前四元数估计的重力方向 (halfv) 和地磁方向 (halfw) */
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    /* 姿态误差 = 估计方向与测量方向的叉积之和
       陀螺仪修正量 proportional to this error (互补滤波核心) */
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    /* 积分反馈: 累积陀螺仪零偏估计，Ki > 0 时启用 */
    if (two_Ki_ > 0.0f)
    {
      integral_FBx_ += two_Ki_ * halfex * inv_sample_freq_;
      integral_FBy_ += two_Ki_ * halfey * inv_sample_freq_;
      integral_FBz_ += two_Ki_ * halfez * inv_sample_freq_;
      gx += integral_FBx_;
      gy += integral_FBy_;
      gz += integral_FBz_;
    }
    else
    {
      integral_FBx_ = 0.0f; /* Ki=0 时积分清零，防止积分饱和 */
      integral_FBy_ = 0.0f;
      integral_FBz_ = 0.0f;
    }

    /* 比例反馈: 瞬时姿态误差修正 */
    gx += two_Kp_ * halfex;
    gy += two_Kp_ * halfey;
    gz += two_Kp_ * halfez;
  }

  /* 四元数积分: q(k+1) = q(k) + 0.5·q(k)⊗ω·dt
     预先提取公共因子 (0.5 * dt) 减少乘法次数 */
  gx *= (0.5f * inv_sample_freq_);
  gy *= (0.5f * inv_sample_freq_);
  gz *= (0.5f * inv_sample_freq_);
  qa = q_[0];
  qb = q_[1];
  qc = q_[2];
  q_[0] += (-qb * gx - qc * gy - q_[3] * gz);
  q_[1] += (qa * gx + qc * gz - q_[3] * gy);
  q_[2] += (qa * gy - qb * gz + q_[3] * gx);
  q_[3] += (qa * gz + qb * gy - qc * gx);

  /* 四元数归一化: 防止数值积分导致的模长漂移 */
  recipNorm = InvSqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
  q_[0] *= recipNorm;
  q_[1] *= recipNorm;
  q_[2] *= recipNorm;
  q_[3] *= recipNorm;
  is_angle_computed_ = false;
}

void Mahony::Update6Axis(float gyro_radps_[3], float accel_mps2_[3])
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  /* 陀螺仪 deg/s → rad/s 单位转换 */
  float gx = gyro_radps_[0];
  float gy = gyro_radps_[1];
  float gz = gyro_radps_[2];

  float ax = accel_mps2_[0];
  float ay = accel_mps2_[1];
  float az = accel_mps2_[2];

  /* 仅当加速度计数据有效时计算反馈修正 */
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
    /* 加速度计归一化 */
    recipNorm = InvSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* 由当前四元数估计的重力方向 (传感器坐标下的地球重力矢量) */
    halfvx = q_[1] * q_[3] - q_[0] * q_[2];
    halfvy = q_[0] * q_[1] + q_[2] * q_[3];
    halfvz = q_[0] * q_[0] - 0.5f + q_[3] * q_[3];

    /* 姿态误差 = 估计重力方向 × 测量重力方向 (叉积) */
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    /* 积分反馈: Ki > 0 时累积陀螺仪零偏 */
    if (two_Ki_ > 0.0f)
    {
      integral_FBx_ += two_Ki_ * halfex * inv_sample_freq_;
      integral_FBy_ += two_Ki_ * halfey * inv_sample_freq_;
      integral_FBz_ += two_Ki_ * halfez * inv_sample_freq_;
      gx += integral_FBx_;
      gy += integral_FBy_;
      gz += integral_FBz_;
    }
    else
    {
      integral_FBx_ = 0.0f; /* Ki=0 时清空积分，防止积分饱和 */
      integral_FBy_ = 0.0f;
      integral_FBz_ = 0.0f;
    }

    /* 比例反馈 */
    gx += two_Kp_ * halfex;
    gy += two_Kp_ * halfey;
    gz += two_Kp_ * halfez;
  }

  /* 四元数积分: q(k+1) = q(k) + 0.5·q(k)⊗ω·dt */
  gx *= (0.5f * inv_sample_freq_);
  gy *= (0.5f * inv_sample_freq_);
  gz *= (0.5f * inv_sample_freq_);
  qa = q_[0];
  qb = q_[1];
  qc = q_[2];
  q_[0] += (-qb * gx - qc * gy - q_[3] * gz);
  q_[1] += (qa * gx + qc * gz - q_[3] * gy);
  q_[2] += (qa * gy - qb * gz + q_[3] * gx);
  q_[3] += (qa * gz + qb * gy - qc * gx);

  /* 四元数归一化 */
  recipNorm = InvSqrt(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
  q_[0] *= recipNorm;
  q_[1] *= recipNorm;
  q_[2] *= recipNorm;
  q_[3] *= recipNorm;
  is_angle_computed_ = false;
}

/**
 * @brief 快速反平方根 (Quake III Arena 经典算法)
 * @param x 输入正数
 * @return 1/sqrt(x) 的近似值
 */
float Mahony::InvSqrt(float x)
{
  float halfx = 0.5f * x;
  union
  {
    float f;
    long l;
  } i;
  i.f = x;
  i.l = 0x5f3759df - (i.l >> 1); /* 位操作: 魔数 - 指数/2，得到初始近似 */
  float y = i.f;
  y = y * (1.5f - (halfx * y * y)); /* 牛顿迭代法第 1 次精化 */
  y = y * (1.5f - (halfx * y * y)); /* 牛顿迭代法第 2 次精化 */
  return y;
}

/**
 * @brief 四元数 → 欧拉角转换 (Tait-Bryan 角, ZYX 顺序)
 */
void Mahony::ComputeAngles()
{
  roll_rad_ = atan2f(q_[0] * q_[1] + q_[2] * q_[3], 0.5f - q_[1] * q_[1] - q_[2] * q_[2]);
  pitch_rad_ = asinf(-2.0f * (q_[1] * q_[3] - q_[0] * q_[2]));
  yaw_rad_ = atan2f(q_[1] * q_[2] + q_[0] * q_[3], 0.5f - q_[2] * q_[2] - q_[3] * q_[3]);

  is_angle_computed_ = true;
}