#include "exo.hpp"
#include <algorithm>
#include <cstring>
extern "C" {
#include "arm_math.h"
}
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_usart.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "gpio.h"
#include "bmi088_driver.h"
#include "usbd_cdc_if.h"

extern uint32_t g_adc_data[3];  /* definition in alt_main.cpp */
static uint32_t exo_run_time_us = 0; /* 用于统计 Exo::Run() 的运行时间, 单位微秒 */

namespace {
float Clamp01Value(float value)
{
    return std::max(0.0f, std::min(1.0f, value));
}

float TimeAlpha(uint32_t delta_ms, float tau_ms)
{
    if (tau_ms <= 1.0f) return 1.0f;
    return Clamp01Value((float)delta_ms / tau_ms);
}

bool IsImuUsable(const ImuData &imu)
{
    return imu.is_used_ && imu.is_calibrated_;
}

enum class SagittalAxis : uint8_t
{
    kRoll,
    kPitch,
};

struct RelativeEulerRad
{
    float roll = 0.0f;
    float pitch = 0.0f;
    float yaw = 0.0f;
};

void RefreshImuRadiansFromDegrees(ImuData &imu)
{
    imu.roll_rad_ = imu.roll_deg_ * DEG_TO_RAD;
    imu.pitch_rad_ = imu.pitch_deg_ * DEG_TO_RAD;
    imu.yaw_rad_ = imu.yaw_deg_ * DEG_TO_RAD;
}

void SetImuSagittalFromAxis(ImuData &imu, SagittalAxis axis)
{
    RefreshImuRadiansFromDegrees(imu);
    if (!imu.quaternion_valid_)
    {
        EulerRad2Quaternion(imu.roll_rad_, imu.pitch_rad_, imu.yaw_rad_, imu.q_);
    }

    if (axis == SagittalAxis::kRoll)
    {
        imu.sagittal_angle_rad_ = imu.roll_rad_;
        imu.sagittal_angle_deg_ = imu.roll_deg_;
        imu.sagittal_gyro_degps_ = imu.gyro_degps_[0];
    }
    else
    {
        imu.sagittal_angle_rad_ = imu.pitch_rad_;
        imu.sagittal_angle_deg_ = imu.pitch_deg_;
        imu.sagittal_gyro_degps_ = imu.gyro_degps_[1];
    }
    imu.sagittal_gyro_radps_ = imu.sagittal_gyro_degps_ * DEG_TO_RAD;
}

void QuaternionToRotationMatrix(const float q[4], float r[3][3])
{
    float w = q[0];
    float x = q[1];
    float y = q[2];
    float z = q[3];
    float norm = 0.0f;
    (void)arm_sqrt_f32(w * w + x * x + y * y + z * z, &norm);
    if (norm < 1.0e-6f)
    {
        w = 1.0f;
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }
    else
    {
        float inv_norm = 1.0f / norm;
        w *= inv_norm;
        x *= inv_norm;
        y *= inv_norm;
        z *= inv_norm;
    }

    r[0][0] = 1.0f - 2.0f * (y * y + z * z);
    r[0][1] = 2.0f * (x * y - z * w);
    r[0][2] = 2.0f * (x * z + y * w);
    r[1][0] = 2.0f * (x * y + z * w);
    r[1][1] = 1.0f - 2.0f * (x * x + z * z);
    r[1][2] = 2.0f * (y * z - x * w);
    r[2][0] = 2.0f * (x * z - y * w);
    r[2][1] = 2.0f * (y * z + x * w);
    r[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

RelativeEulerRad RelativeEulerFromRotationMatrix(const ImuData &proximal, const ImuData &distal)
{
    float r_proximal[3][3];
    float r_distal[3][3];
    float r_rel[3][3] = {{0.0f}};
    QuaternionToRotationMatrix(proximal.q_, r_proximal);
    QuaternionToRotationMatrix(distal.q_, r_distal);

    /* R_rel = R_proximal^T * R_distal */
    for (uint8_t i = 0; i < 3; ++i)
    {
        for (uint8_t j = 0; j < 3; ++j)
        {
            for (uint8_t k = 0; k < 3; ++k)
            {
                r_rel[i][j] += r_proximal[k][i] * r_distal[k][j];
            }
        }
    }

    RelativeEulerRad euler;
    euler.roll = atan2f(r_rel[2][1], r_rel[2][2]);
    euler.pitch = asinf(_constrain(-r_rel[2][0], -1.0f, 1.0f));
    euler.yaw = atan2f(r_rel[1][0], r_rel[0][0]);
    return euler;
}

void SetJointSagittalState(JointData &joint, float raw_pos_rad, float raw_vel_radps, bool is_valid)
{
    joint.sagittal_valid_ = is_valid;
    if (!is_valid) return;

    joint.sagittal_pos_rad_ = raw_pos_rad - joint.sagittal_pos_offset_rad_;
    joint.sagittal_vel_radps_ = raw_vel_radps;
}

void CaptureJointSagittalOffset(JointData &joint, float raw_pos_rad, bool is_valid)
{
    if (!is_valid) return;
    joint.sagittal_pos_offset_rad_ = raw_pos_rad;
    joint.sagittal_valid_ = true;
}

void UpdateSegmentSagittalAxes(ExoData &pe)
{
    SetImuSagittalFromAxis(pe.body_imu_, SagittalAxis::kPitch);

    SetImuSagittalFromAxis(pe.left_side_.foot_imu_, SagittalAxis::kPitch);
    SetImuSagittalFromAxis(pe.right_side_.foot_imu_, SagittalAxis::kPitch);

    SetImuSagittalFromAxis(pe.left_side_.thigh_imu_, SagittalAxis::kRoll);
    SetImuSagittalFromAxis(pe.left_side_.shank_imu_, SagittalAxis::kRoll);
    SetImuSagittalFromAxis(pe.right_side_.thigh_imu_, SagittalAxis::kRoll);
    SetImuSagittalFromAxis(pe.right_side_.shank_imu_, SagittalAxis::kRoll);
}

bool ComputeRawJointSagittal(const ExoData &pe, const SideData &side,
                             float &hip_raw_rad, float &hip_vel_radps,
                             float &knee_raw_rad, float &knee_vel_radps,
                             float &ankle_raw_rad, float &ankle_vel_radps)
{
    const bool hip_ok = IsImuUsable(pe.body_imu_) && IsImuUsable(side.thigh_imu_);
    const bool knee_ok = IsImuUsable(side.thigh_imu_) && IsImuUsable(side.shank_imu_);
    const bool ankle_ok = IsImuUsable(side.shank_imu_) && IsImuUsable(side.foot_imu_);

    if (hip_ok)
    {
        RelativeEulerRad hip_rel = RelativeEulerFromRotationMatrix(pe.body_imu_, side.thigh_imu_);
        hip_raw_rad = -hip_rel.roll;
        hip_vel_radps = pe.body_imu_.sagittal_gyro_radps_ - side.thigh_imu_.sagittal_gyro_radps_;
    }
    if (knee_ok)
    {
        RelativeEulerRad knee_rel = RelativeEulerFromRotationMatrix(side.thigh_imu_, side.shank_imu_);
        knee_raw_rad = -knee_rel.roll;
        knee_vel_radps = side.thigh_imu_.sagittal_gyro_radps_ - side.shank_imu_.sagittal_gyro_radps_;
    }
    if (ankle_ok)
    {
        RelativeEulerRad ankle_rel = RelativeEulerFromRotationMatrix(side.shank_imu_, side.foot_imu_);
        ankle_raw_rad = ankle_rel.pitch;
        ankle_vel_radps = side.foot_imu_.sagittal_gyro_radps_ - side.shank_imu_.sagittal_gyro_radps_;
    }

    return hip_ok || knee_ok || ankle_ok;
}

void CaptureJointStandingOffsets(ExoData &pe)
{
    UpdateSegmentSagittalAxes(pe);

    auto capture_side = [&](SideData &side) {
        float hip_raw = 0.0f;
        float hip_vel = 0.0f;
        float knee_raw = 0.0f;
        float knee_vel = 0.0f;
        float ankle_raw = 0.0f;
        float ankle_vel = 0.0f;
        (void)ComputeRawJointSagittal(pe, side, hip_raw, hip_vel, knee_raw, knee_vel, ankle_raw, ankle_vel);

        CaptureJointSagittalOffset(side.hip_joint_, hip_raw, IsImuUsable(pe.body_imu_) && IsImuUsable(side.thigh_imu_));
        CaptureJointSagittalOffset(side.knee_joint_, knee_raw, IsImuUsable(side.thigh_imu_) && IsImuUsable(side.shank_imu_));
        CaptureJointSagittalOffset(side.ankle_joint_, ankle_raw, IsImuUsable(side.shank_imu_) && IsImuUsable(side.foot_imu_));
    };

    capture_side(pe.left_side_);
    capture_side(pe.right_side_);
}

float GetStandingReferencedKneePitchDeg(const ExoData &pe, const SideData &side)
{
    if (side.knee_joint_.sagittal_valid_)
    {
        return side.knee_joint_.sagittal_pos_rad_ * RAD_TO_DEG;
    }

    const ImuData &thigh = side.thigh_imu_;
    const ImuData &shank = side.shank_imu_;
    float raw_knee_deg = thigh.sagittal_angle_deg_ - shank.sagittal_angle_deg_;
    if (!pe.intention_data_.standing_posture_ref_valid_)
    {
        return raw_knee_deg;
    }

    float thigh_ref = side.is_left_ ?
        pe.intention_data_.left_thigh_pitch_standing_ref_deg_ :
        pe.intention_data_.right_thigh_pitch_standing_ref_deg_;
    float shank_ref = side.is_left_ ?
        pe.intention_data_.left_shank_pitch_standing_ref_deg_ :
        pe.intention_data_.right_shank_pitch_standing_ref_deg_;
    return raw_knee_deg - (thigh_ref - shank_ref);
}

bool CapturePostureReference(ExoData &pe, bool is_sitting, uint32_t now_ms)
{
    float thigh_sum = 0.0f;
    float shank_sum = 0.0f;
    uint8_t posture_count = 0;
    UpdateSegmentSagittalAxes(pe);

    auto capture_side = [&](SideData &side) {
        const bool has_posture_imus = IsImuUsable(side.thigh_imu_) && IsImuUsable(side.shank_imu_);
        if (!has_posture_imus)
        {
            return;
        }

        if (side.is_left_)
        {
            if (is_sitting)
            {
                pe.intention_data_.left_thigh_pitch_sitting_ref_deg_ = side.thigh_imu_.sagittal_angle_deg_;
                pe.intention_data_.left_shank_pitch_sitting_ref_deg_ = side.shank_imu_.sagittal_angle_deg_;
            }
            else
            {
                pe.intention_data_.left_thigh_pitch_standing_ref_deg_ = side.thigh_imu_.sagittal_angle_deg_;
                pe.intention_data_.left_shank_pitch_standing_ref_deg_ = side.shank_imu_.sagittal_angle_deg_;
            }
        }
        else
        {
            if (is_sitting)
            {
                pe.intention_data_.right_thigh_pitch_sitting_ref_deg_ = side.thigh_imu_.sagittal_angle_deg_;
                pe.intention_data_.right_shank_pitch_sitting_ref_deg_ = side.shank_imu_.sagittal_angle_deg_;
            }
            else
            {
                pe.intention_data_.right_thigh_pitch_standing_ref_deg_ = side.thigh_imu_.sagittal_angle_deg_;
                pe.intention_data_.right_shank_pitch_standing_ref_deg_ = side.shank_imu_.sagittal_angle_deg_;
            }
        }

        if (!is_sitting)
        {
            side.thigh_imu_.sagittal_ref_rad_ = side.thigh_imu_.sagittal_angle_rad_;
            side.thigh_imu_.sagittal_ref_deg_ = side.thigh_imu_.sagittal_angle_deg_;
            side.thigh_imu_.sagittal_ref_valid_ = true;
            side.shank_imu_.sagittal_ref_rad_ = side.shank_imu_.sagittal_angle_rad_;
            side.shank_imu_.sagittal_ref_deg_ = side.shank_imu_.sagittal_angle_deg_;
            side.shank_imu_.sagittal_ref_valid_ = true;
            side.fsr_gait_data_.shank_standing_ref_deg_ = side.shank_imu_.sagittal_ref_deg_;
        }

        thigh_sum += side.thigh_imu_.sagittal_angle_deg_;
        shank_sum += side.shank_imu_.sagittal_angle_deg_;
        posture_count++;
    };

    capture_side(pe.left_side_);
    capture_side(pe.right_side_);
    if (posture_count == 0u)
    {
        return false;
    }

    float inv_count = 1.0f / (float)posture_count;
    if (is_sitting)
    {
        pe.intention_data_.thigh_pitch_sitting_ref_deg_ = thigh_sum * inv_count;
        pe.intention_data_.shank_pitch_sitting_ref_deg_ = shank_sum * inv_count;
        pe.intention_data_.sitting_posture_ref_valid_ = true;
    }
    else
    {
        pe.intention_data_.thigh_pitch_standing_ref_deg_ = thigh_sum * inv_count;
        pe.intention_data_.shank_pitch_standing_ref_deg_ = shank_sum * inv_count;
        pe.intention_data_.standing_posture_ref_valid_ = true;
        if (IsImuUsable(pe.body_imu_))
        {
            pe.body_imu_.sagittal_ref_rad_ = pe.body_imu_.sagittal_angle_rad_;
            pe.body_imu_.sagittal_ref_deg_ = pe.body_imu_.sagittal_angle_deg_;
            pe.body_imu_.sagittal_ref_valid_ = true;
        }
        CaptureJointStandingOffsets(pe);

        if (IsImuUsable(pe.left_side_.foot_imu_) &&
            pe.left_side_.fsr_gait_data_.IsPacketFresh(now_ms))
        {
            pe.intention_data_.left_foot_pitch_level_ref_deg_ = pe.left_side_.foot_imu_.sagittal_angle_deg_;
            pe.intention_data_.left_foot_pitch_ref_valid_ = true;
            pe.left_side_.foot_imu_.sagittal_ref_rad_ = pe.left_side_.foot_imu_.sagittal_angle_rad_;
            pe.left_side_.foot_imu_.sagittal_ref_deg_ = pe.left_side_.foot_imu_.sagittal_angle_deg_;
            pe.left_side_.foot_imu_.sagittal_ref_valid_ = true;
        }
        if (IsImuUsable(pe.right_side_.foot_imu_) &&
            pe.right_side_.fsr_gait_data_.IsPacketFresh(now_ms))
        {
            pe.intention_data_.right_foot_pitch_level_ref_deg_ = pe.right_side_.foot_imu_.sagittal_angle_deg_;
            pe.intention_data_.right_foot_pitch_ref_valid_ = true;
            pe.right_side_.foot_imu_.sagittal_ref_rad_ = pe.right_side_.foot_imu_.sagittal_angle_rad_;
            pe.right_side_.foot_imu_.sagittal_ref_deg_ = pe.right_side_.foot_imu_.sagittal_angle_deg_;
            pe.right_side_.foot_imu_.sagittal_ref_valid_ = true;
        }
    }

    pe.intention_data_.is_sts_ref_valid_ = pe.intention_data_.standing_posture_ref_valid_;
    return true;
}

void CaptureFootPitchLevelReference(ExoData &pe, uint32_t now_ms)
{
    UpdateSegmentSagittalAxes(pe);

    if (!pe.intention_data_.left_foot_pitch_ref_valid_ &&
        IsImuUsable(pe.left_side_.foot_imu_) &&
        pe.left_side_.fsr_gait_data_.IsPacketFresh(now_ms))
    {
        pe.intention_data_.left_foot_pitch_level_ref_deg_ = pe.left_side_.foot_imu_.sagittal_angle_deg_;
        pe.intention_data_.left_foot_pitch_ref_valid_ = true;
        pe.left_side_.foot_imu_.sagittal_ref_rad_ = pe.left_side_.foot_imu_.sagittal_angle_rad_;
        pe.left_side_.foot_imu_.sagittal_ref_deg_ = pe.left_side_.foot_imu_.sagittal_angle_deg_;
        pe.left_side_.foot_imu_.sagittal_ref_valid_ = true;
    }

    if (!pe.intention_data_.right_foot_pitch_ref_valid_ &&
        IsImuUsable(pe.right_side_.foot_imu_) &&
        pe.right_side_.fsr_gait_data_.IsPacketFresh(now_ms))
    {
        pe.intention_data_.right_foot_pitch_level_ref_deg_ = pe.right_side_.foot_imu_.sagittal_angle_deg_;
        pe.intention_data_.right_foot_pitch_ref_valid_ = true;
        pe.right_side_.foot_imu_.sagittal_ref_rad_ = pe.right_side_.foot_imu_.sagittal_angle_rad_;
        pe.right_side_.foot_imu_.sagittal_ref_deg_ = pe.right_side_.foot_imu_.sagittal_angle_deg_;
        pe.right_side_.foot_imu_.sagittal_ref_valid_ = true;
    }
}

void InjectReplayStairSide(SideData &side, const float *values, uint8_t base)
{
    FsrGaitData &fsr = side.fsr_gait_data_;
    bool heel_contact = values[base + 4u] > 0.5f;
    bool toe_contact = values[base + 5u] > 0.5f;

    fsr.is_used_ = true;
    fsr.is_calibrated_ = true;
    fsr.is_data_fresh_ = true;
    fsr.heel_contact_state_ = heel_contact;
    fsr.toe_contact_state_ = toe_contact;

    side.thigh_imu_.is_used_ = true;
    side.thigh_imu_.is_calibrated_ = true;
    side.thigh_imu_.roll_deg_ = values[base + 15u];
    side.thigh_imu_.gyro_degps_[0] = values[base + 17u];
    side.thigh_imu_.quaternion_valid_ = false;

    side.shank_imu_.is_used_ = true;
    side.shank_imu_.is_calibrated_ = true;
    side.shank_imu_.roll_deg_ = values[base + 16u];
    side.shank_imu_.gyro_degps_[0] = values[base + 18u];
    side.shank_imu_.quaternion_valid_ = false;
}

ExoData::LocoMode ParseReplayLocoMode(float value)
{
    uint8_t raw = value < 0.0f ? 0u : static_cast<uint8_t>(value + 0.5f);
    if (raw > static_cast<uint8_t>(ExoData::LocoMode::kStairDescent))
    {
        return ExoData::LocoMode::kWalking;
    }
    return static_cast<ExoData::LocoMode>(raw);
}

bool IsReplaySlopeMode(ExoData::LocoMode mode)
{
    return mode == ExoData::LocoMode::kWalking ||
           mode == ExoData::LocoMode::kRampAscent ||
           mode == ExoData::LocoMode::kRampDescent;
}

void InjectReplaySlopeSide(SideData &side, const float *values, uint8_t base, uint32_t now_ms)
{
    FsrGaitData &fsr = side.fsr_gait_data_;
    ImuData &foot = side.foot_imu_;
    ImuData &shank = side.shank_imu_;
    ImuData &thigh = side.thigh_imu_;

    fsr.is_used_ = true;
    fsr.is_calibrated_ = true;
    fsr.is_data_fresh_ = true;
    fsr.heel_.raw_reading = values[base + 0u];
    fsr.toe_.raw_reading = values[base + 1u];
    fsr.heel_.calibrated_reading = values[base + 2u];
    fsr.toe_.calibrated_reading = values[base + 3u];
    fsr.heel_contact_state_ = values[base + 4u] > 0.5f;
    fsr.toe_contact_state_ = values[base + 5u] > 0.5f;
    fsr.is_valid_ = values[base + 7u] > 0.5f;
    fsr.heel_.last_update_ms_ = now_ms;
    fsr.toe_.last_update_ms_ = now_ms;

    foot.is_used_ = true;
    foot.is_calibrated_ = true;
    foot.roll_deg_ = values[base + 8u];
    foot.pitch_deg_ = values[base + 9u];
    foot.yaw_deg_ = values[base + 10u];
    foot.quaternion_valid_ = false;

    shank.is_used_ = true;
    shank.is_calibrated_ = true;
    shank.roll_deg_ = values[base + 11u];
    shank.pitch_deg_ = values[base + 12u];
    shank.yaw_deg_ = values[base + 13u];
    shank.gyro_degps_[0] = values[base + 14u];
    shank.gyro_degps_[1] = values[base + 15u];
    shank.gyro_degps_[2] = values[base + 16u];
    shank.quaternion_valid_ = false;

    thigh.is_used_ = true;
    thigh.is_calibrated_ = true;
    thigh.roll_deg_ = values[base + 17u];
    thigh.pitch_deg_ = values[base + 18u];
    thigh.yaw_deg_ = values[base + 19u];
    thigh.gyro_degps_[0] = values[base + 20u];
    thigh.gyro_degps_[1] = values[base + 21u];
    thigh.gyro_degps_[2] = values[base + 22u];
    thigh.quaternion_valid_ = false;
}

void UpdateReplaySlopeReferences(ExoData &pe, const float *values, uint8_t count)
{
    UpdateSegmentSagittalAxes(pe);

    if (count >= 62u)
    {
        pe.intention_data_.left_foot_pitch_level_ref_deg_ = values[60];
        pe.intention_data_.right_foot_pitch_level_ref_deg_ = values[61];
        pe.intention_data_.left_foot_pitch_ref_valid_ = true;
        pe.intention_data_.right_foot_pitch_ref_valid_ = true;
        return;
    }

    if (!pe.intention_data_.label_is_valid_ || fabsf(pe.intention_data_.label_slope_deg_) > 0.5f)
    {
        return;
    }

    const FsrGaitData &left_fsr = pe.left_side_.fsr_gait_data_;
    if (!pe.intention_data_.left_foot_pitch_ref_valid_ &&
        left_fsr.heel_contact_state_ && left_fsr.toe_contact_state_)
    {
        pe.intention_data_.left_foot_pitch_level_ref_deg_ = pe.left_side_.foot_imu_.sagittal_angle_deg_;
        pe.intention_data_.left_foot_pitch_ref_valid_ = true;
    }

    const FsrGaitData &right_fsr = pe.right_side_.fsr_gait_data_;
    if (!pe.intention_data_.right_foot_pitch_ref_valid_ &&
        right_fsr.heel_contact_state_ && right_fsr.toe_contact_state_)
    {
        pe.intention_data_.right_foot_pitch_level_ref_deg_ = pe.right_side_.foot_imu_.sagittal_angle_deg_;
        pe.intention_data_.right_foot_pitch_ref_valid_ = true;
    }
}

void VofaTransmitJustFloat(DmaUnionBuffer &buf, uint16_t float_count)
{
    uint16_t count = 4u * float_count;
    buf.u8_data[count++] = 0x00;
    buf.u8_data[count++] = 0x00;
    buf.u8_data[count++] = 0x80;
    buf.u8_data[count++] = 0x7f;
    CDC_Transmit_HS(buf.u8_data, count);
}
}


extern "C" {
static void ExoCanRxBridge(void *ctx, FDCAN_HandleTypeDef *hfdcan, uint32_t can_ext_id, const uint8_t *rx_data)
{
    static_cast<Exo *>(ctx)->CanRxCallback(hfdcan, can_ext_id, rx_data);
}

static void ExoUartRxBridge(void *ctx, UART_HandleTypeDef *huart, uint16_t data_size)
{
    static_cast<Exo *>(ctx)->UartRxCallback(huart, data_size);
}

static void ExoUartErrorBridge(void *ctx, UART_HandleTypeDef *huart)
{
    static_cast<Exo *>(ctx)->UartErrorCallback(huart);
}

static void ExoSpiErrorBridge(void *ctx, SPI_HandleTypeDef *hspi)
{
    static_cast<Exo *>(ctx)->SpiErrorCallback(hspi);
}

static void ExoGpioExtiBridge(void *ctx, uint16_t GPIO_Pin)
{
    Exo *exo = static_cast<Exo *>(ctx);
    if (GPIO_Pin == NRF54_CS_INT_Pin)
    {
        if (HAL_GPIO_ReadPin(NRF54_CS_INT_GPIO_Port, NRF54_CS_INT_Pin) == GPIO_PIN_RESET)
        {
            exo->SpiRxStart();
        }
        else if (HAL_GPIO_ReadPin(NRF54_CS_INT_GPIO_Port, NRF54_CS_INT_Pin) == GPIO_PIN_SET)
        {
            exo->SpiRxCallback();
        }
    }
}
}

#define SENSOR_SPI_RX_BUF_SIZE              128
#define SENSOR_UART_RX_BUF_SIZE             256
__attribute__((section(".dma_buf"), aligned(32))) uint8_t spi_rx_dma_buf[2][SENSOR_SPI_RX_BUF_SIZE];
__attribute__((section(".dma_buf"), aligned(32))) uint8_t sensor_uart_rx_buffer[SENSOR_UART_RX_BUF_SIZE];


void AnkleJoint::Calibrate()
{
    if (!pj_.is_used_ || pj_.is_calibrated_) return;
}

bool AnkleJoint::IsMotorConnect()
{
    if (!pj_.is_used_) return true;

    motor_.EnableMotor();
    if (motor_.status_feedback_cnt_ > 10)
    {
        return true;
    }
    return false;
}

void AnkleJoint::Shutdown()
{
    motor_.DisableMotor(0);
}

void AnkleJoint::Standby()
{
    if (!pj_.is_used_) return;

    // motor_.torque_forward_ = 0.0f;
    // motor_.position_ref_ = 0.1f;
    // motor_.speed_ref_ = 0.0f;
    // motor_.motion_mode_kp_ = 0.0f;
    // motor_.motion_mode_kd_ = 0.0f;
    // motor_.MotionControl();
    motor_.EnableMotor();
}

void AnkleJoint::Read()
{
    if (!pj_.is_used_) return;

    if (pj_.is_left_)
    {
        pj_.link_rad_ = motor_.position_;
        pj_.link_vel_radps_ = motor_.speed_;
        pj_.tor_output_Nm_ = motor_.torque_;
    }
    else
    {
        pj_.link_rad_ = -motor_.position_;
        pj_.link_vel_radps_ = -motor_.speed_;
        pj_.tor_output_Nm_ = -motor_.torque_;
    }
}

void AnkleJoint::Assist()
{
    if (!pj_.is_used_) return;

    /* 1. 获取 FSR 算出的相位百分比 (0.0f~100.0f) */
    const FsrGaitData &fsr = pj_.is_left_ ? pe_.left_side_.fsr_gait_data_ : pe_.right_side_.fsr_gait_data_;
    float phase_percent = fsr.is_valid_ ? fsr.percent_gait_ : -1.0f;

    /* 2. 根据步态相位设置参考位置 */
    float cable_position_ref = cable_released_position_;
    if (phase_percent >= 0.0f && phase_percent < 100.0f) 
    {
        if (phase_percent < assistance_start_phase_percent_) 
        {
            cable_position_ref = cable_pre_tensioned_position_;
        }
        else if (phase_percent >= assistance_start_phase_percent_ && phase_percent < assistance_end_phase_percent_) 
        {
            cable_position_ref = cable_tensioned_position_;
        }
        else 
        {
            cable_position_ref = cable_released_position_;
        }
    }

    if (!pj_.is_left_)
    {
        cable_position_ref = -cable_position_ref;
    }

    /* 3. 发送电机控制指令 */
    motor_.position_ref_ = cable_position_ref;
    motor_.torque_forward_ = 0.0f;
    motor_.speed_ref_ = 0.0f;
    motor_.motion_mode_kp_ = 15.0f;
    motor_.motion_mode_kd_ = 0.5f;
    motor_.MotionControl();
}

void KneeJoint::Calibrate()
{
    if (!pj_.is_used_ || pj_.is_calibrated_) return;
    pj_.link_offset_rad_ = motor_.position_;
    pj_.is_calibrated_ = true;
}

bool KneeJoint::IsMotorConnect()
{
    if (!pj_.is_used_) return true;

    motor_.EnableMotor();
    if (motor_.status_feedback_cnt_ > 10)
    {
        return true;
    }
    return false;
}

void KneeJoint::Shutdown()
{
    motor_.DisableMotor(0);
}

void KneeJoint::Standby()
{
    if (!pj_.is_used_) return;

    motor_.torque_forward_ = 0.0f;
    motor_.position_ref_ = 0.0f;
    motor_.speed_ref_ = 0.0f;
    motor_.motion_mode_kp_ = 0.0f;
    motor_.motion_mode_kd_ = 0.0f;
    motor_.MotionControl();
    // motor_.EnableMotor();
}

void KneeJoint::Read()
{
    if (!pj_.is_used_) return;

    /* 左侧正转为伸展, 即负角速度方向 */
    if (!pj_.is_left_)
    {
        pj_.link_rad_ = motor_.position_ - pj_.link_offset_rad_;
        pj_.link_vel_radps_ = motor_.speed_;
        pj_.tor_output_Nm_ = motor_.torque_;
    }
    else
    {
        pj_.link_rad_ = -(motor_.position_ - pj_.link_offset_rad_);
        pj_.link_vel_radps_ = -motor_.speed_;
        pj_.tor_output_Nm_ = -motor_.torque_;
    }
}

void KneeJoint::Assist()
{
    if (!pj_.is_used_) return;

    if (pe_.intention_data_.current_mode_ == ExoData::LocoMode::kSitToStand)
    {
        motor_.torque_forward_ = 3.0f;
        OpenLoopTorqueControl();
        return;
    }
    
    float force_profile = 0.0f;

    if (ps_.fsr_gait_data_.is_valid_)
    {
        float phase_percent = ps_.fsr_gait_data_.percent_gait_;
        force_profile = force_profile_generator_.GetForceProfile(phase_percent,
                                                                 pj_.sagittal_pos_rad_,
                                                                 pj_.sagittal_vel_radps_);
    }
    
    /* 左侧正转为伸展, 即负角速度方向 */
    if (pj_.is_left_)
    {
        force_profile = -force_profile;
    }

    /* 暂时使用开环控制, 并限幅5 Nm */
    pj_.tor_output_ref_Nm_ = _constrain(force_profile * pe_.user_weight_kg_, -5.0f, 5.0f);
    OpenLoopTorqueControl();
}


void KneeJoint::ImpedanceControl()
{
    static float q_ref_ = 0.0f;
    static float dot_q_ref_ = 0.0f;
    static float ddot_q_ref_ = 0.0f;
    static uint64_t t0 = GetSysTimeUs();
    static float M0_ = 1.0f;
    static float Md_ = 1.0f;
    static float Bd_ = 1.0f;
    static float Kd_ = 1.0f;

    if (ctrl_mode_ != CtrlMode::kImpedance)
    {

    }

    if (!pj_.is_used_)
    {
        return;
    }
    float omega = _2PI * 1.0f;
    float amplitude = 2.0f;
    float tnow_sys_us = GetSysTimeUs();
    float sin_t, cos_t;
    arm_sin_cos_f32(RAD_TO_DEG * omega * (tnow_sys_us - t0) / 1000000.0f, &sin_t, &cos_t);
    q_ref_ = amplitude * sin_t;
    dot_q_ref_ = amplitude * omega * cos_t;
    ddot_q_ref_ = - amplitude * omega * omega * sin_t;
    disturbance_observer_.UpdateObserver(pj_.link_rad_, pj_.tor_output_Nm_ / M0_);

    float hat_disturbance = - M0_ * disturbance_observer_.hat_x2_;
    float dot_tilde_q = dot_q_ref_ - pj_.link_vel_radps_;
    float tilde_q = q_ref_ - pj_.link_rad_;

    motor_.torque_forward_ = M0_ * ddot_q_ref_ + M0_ / Md_ * (Bd_ * dot_tilde_q + Kd_ * tilde_q - hat_disturbance) + hat_disturbance;
    motor_.position_ref_ = 0;
    motor_.speed_ref_ = 0;
    motor_.motion_mode_kp_ = 0;
    motor_.motion_mode_kd_ = 0;
    motor_.MotionControl();
}

void KneeJoint::ClosedLoopTorqueControl()
{
    if (!pj_.is_used_) return;

    if (ctrl_mode_ != CtrlMode::kClosedLoopTorque)
    {
        joint_tor_pid_.ResetError();
        ctrl_mode_ = CtrlMode::kClosedLoopTorque;
    }
    
    /* 运动控制, tref = kp * (theta_set - theta_fb) + kd*(vset-vfb) + tff*/
    
    float tor_err_Nm = pj_.tor_output_ref_Nm_- pj_.tor_output_Nm_;
    motor_.torque_forward_ = pj_.tor_output_ref_Nm_;
    motor_.position_ref_ = 0.0f;
    motor_.speed_ref_ =  joint_tor_pid_.kp_ * tor_err_Nm;
    motor_.motion_mode_kp_ = 0.0f;
    motor_.motion_mode_kd_ = 1.0;
    motor_.MotionControl();
}

void KneeJoint::OpenLoopTorqueControl()
{
    if (!pj_.is_used_) return;

    if (ctrl_mode_ != CtrlMode::kOpenLoopTorque)
    {
        ctrl_mode_ = CtrlMode::kOpenLoopTorque;
    }

    motor_.torque_forward_ = pj_.tor_output_ref_Nm_;
    motor_.position_ref_ = 0.0f;
    motor_.speed_ref_ = 0.0f;
    motor_.motion_mode_kp_ = 0.0f;
    motor_.motion_mode_kd_ = 0.0f;
    motor_.MotionControl();
}


void KneeSeaJoint::Calibrate()
{
    if (!pj_.is_used_ || pj_.is_calibrated_) return;

    if (pj_.pos_linear_encoder_mm_ < 0) return;

    /* 假设此时弹簧力为0, 滑块处于中间位置, 并且框处于静止状态, 那么可以由框的当前位置计算得到滑块的偏置 */
    pj_.pos_slider_offset_mm_ = pj_.pos_slider_mm_ - pj_.pos_linear_encoder_mm_;
    pj_.is_calibrated_ = true;
    return;
}

void KneeSeaJoint::Read()
{
    if (!pj_.is_used_) return;

    /* 滑块位移 = 电机输出转动角度 * 螺母导程 - 偏置 */
    pj_.pos_slider_mm_ = pj_.screw_lead_rad2mm_ * motor_.shaft_pos_feedback_rad_ - pj_.pos_slider_offset_mm_;
    pj_.vel_slider_mmps_ = pj_.screw_lead_rad2mm_ * motor_.shaft_speed_feedback_radps_;

    /* sea输出位移 = 磁栅尺反馈位置 - 偏置 */
    pj_.pos_linear_encoder_mm_ = mag_encoder_.absolute_position_mm_ - pj_.pos_linear_encoder_offset_mm_;
    pj_.vel_linear_encoder_mmps_ = 0.0f;  /** #HACK 暂时不需要  */

    /* 弹簧力 = 2 * 刚度 * 压缩长度 */
    pj_.pos_bias_mm_ = pj_.pos_slider_mm_ - pj_.pos_linear_encoder_mm_;
    pj_.force_spring_N_ = pj_.spring_stiffness_Npmm_ * pj_.pos_bias_mm_;

    /* 连杆/机构角由磁栅尺换算; 人体膝角在 Exo::UpdateHumanJointKinematics() 中由 IMU 更新 */
    pj_.link_rad_ = (pj_.pos_linear_encoder_mm_) / (pj_.max_pos_linear_encoder_mm_ - pj_.pos_linear_encoder_offset_mm_) * _PI_2;
    pj_.link_vel_radps_ = 0.0f;

    mag_encoder_.SendRequest();
}

bool KneeSeaJoint::IsMotorConnect()
{
    if (!pj_.is_used_) return true;

    return true; /** 无需主动连接通信 */
}

void KneeSeaJoint::Shutdown()
{
    motor_.DisableMotor();
}

void KneeSeaJoint::Standby()
{
    if (!pj_.is_used_ || !pj_.is_calibrated_) return;

    float force_profile = 0.0f;
    /* SEA驱动左右同向 */
    // force_profile = pj_.is_left_ ? force_profile : -force_profile;

    /* 弹簧零力控制 */
    pj_.force_spring_ref_N_ = force_profile * pe_.user_weight_kg_;
    SpringForceControl(); 
}

void KneeSeaJoint::Assist()
{
    if (!pj_.is_used_) return;

    float force_profile = 0.0f;

    const FsrGaitData &fsr = pj_.is_left_ ? pe_.left_side_.fsr_gait_data_ : pe_.right_side_.fsr_gait_data_;

    /* #HACK 先走几步再助力? */
    if (fsr.is_valid_)
    {
        force_profile = force_profile_generator_.GetForceProfile(fsr.percent_gait_,
                                                                 pj_.sagittal_pos_rad_,
                                                                 pj_.sagittal_vel_radps_);
    }

    pj_.force_spring_ref_N_ = force_profile * pe_.user_weight_kg_;

    SpringForceControl();
    // Standby();
}

void KneeSeaJoint::JointPosControl()
{
    if (ctrl_mode_ != CtrlMode::kPosition)
    {
        joint_pos_pid_.ResetError();
        spring_force_pid_.ResetError();
        ctrl_mode_ = CtrlMode::kPosition;
    }
    
    float pos_err_rad = pj_.link_ref_rad_ - pj_.link_rad_;
    motor_.rotor_iq_reference_amp_ = joint_pos_pid_(pos_err_rad);

    motor_.CurrentControl();
}

void KneeSeaJoint::SpringForceControl()
{
    if (ctrl_mode_ != CtrlMode::kSpringForce)
    {
        joint_pos_pid_.ResetError();
        spring_force_pid_.ResetError();
        ctrl_mode_ = CtrlMode::kSpringForce;
    }

    float force_err_N = pj_.force_spring_ref_N_ - pj_.force_spring_N_;
    motor_.rotor_iq_reference_amp_ = spring_force_pid_(force_err_N);

    motor_.CurrentControl();
}


void HipJoint::Calibrate()
{
    if (!pj_.is_used_ || pj_.is_calibrated_) return;
}

void HipJoint::Read()
{
    if (!pj_.is_used_) return;

    if (pj_.is_left_)
    {
        pj_.link_rad_ = motor_.feedback_.pos_rad_;
        pj_.link_vel_radps_ = motor_.feedback_.vel_radps_;
        pj_.tor_output_Nm_ = motor_.feedback_.tor_output_Nm_;
    }
    else
    {
        pj_.link_rad_ = -motor_.feedback_.pos_rad_;
        pj_.link_vel_radps_ = -motor_.feedback_.vel_radps_;
        pj_.tor_output_Nm_ = -motor_.feedback_.tor_output_Nm_;
    }
}

bool HipJoint::IsMotorConnect()
{
    if (!pj_.is_used_) return true;

    // motor_.ctrl_param_.mode_id_ = DMMotorModeID::kMIT;
    // motor_.SetMotorMode();
    motor_.EnableMotor();
    if (motor_.feedback_.flag_ > 0)
    {
        return true;
    }

    return false;
}

void HipJoint::Shutdown()
{
    motor_.DisableMotor();
}

void HipJoint::Standby()
{
    if (!pj_.is_used_) return;

    motor_.ctrl_param_.mode_id_ = DMMotorModeID::kMIT;
    motor_.ctrl_param_.kp_set_ = 0.0f;
    motor_.ctrl_param_.kd_set_ = 0.0f;
    motor_.ctrl_param_.pos_set_rad_ = 0.0f;
    motor_.ctrl_param_.vel_set_radps_ = 0.0f;
    motor_.ctrl_param_.tor_set_Nm_ = 0.0f;
    motor_.MitControl();
}

namespace {
struct HipFFShared
{
    static constexpr int kHistSize = 128;
    static constexpr int kDelaySamples = 35;     // delay 35*5ms
    static constexpr float kLambda = 0.01f;
    static constexpr float kK = 7.0f;

    float posL[kHistSize] = {0.0f};
    float posR[kHistSize] = {0.0f};
    float lastL = 0.0f;
    float lastR = 0.0f;
    int idx = 0;
    bool inited = false;

    uint32_t last_tick_ms = 0;

    float tau_left = 0.0f;
    float tau_right = 0.0f;

    void UpdateAndCompute(uint32_t now_ms, float posL_now, float posR_now)
    {
        if (last_tick_ms == now_ms) return;
        last_tick_ms = now_ms;

        if (!inited)
        {
            lastL = posL_now;
            lastR = posR_now;
            for (int i = 0; i < kHistSize; ++i)
            {
                posL[i] = posL_now;
                posR[i] = posR_now;
            }
            inited = true;
        }

        posL[idx] = kLambda * posL_now + (1.0f - kLambda) * lastL;
        posR[idx] = kLambda * posR_now + (1.0f - kLambda) * lastR;
        lastL = posL[idx];
        lastR = posR[idx];

        int past = idx - kDelaySamples;
        if (past < 0) past += kHistSize;

        const float posL_d = posL[past];
        const float posR_d = posR[past];

        const float tau = kK * (arm_sin_f32(posR_d) - arm_sin_f32(posL_d));
        tau_left = tau;
        // tau_right = -tau;
        tau_right = tau;

        idx = (idx + 1) % kHistSize;
    }
};
static HipFFShared g_hip_ff;
}

void HipJoint::Assist()
{
    if (!pj_.is_used_) return;

    const float posL_now = pe_.left_side_.hip_joint_.sagittal_pos_rad_;
    const float posR_now = pe_.right_side_.hip_joint_.sagittal_pos_rad_;
    const uint32_t now_ms = GetSysTimeMs();

    if (pj_.is_left_)
    {
        g_hip_ff.UpdateAndCompute(now_ms, posL_now, posR_now);
    }
    const float tau_me = pj_.is_left_ ? g_hip_ff.tau_left : g_hip_ff.tau_right;

    motor_.ctrl_param_.mode_id_ = DMMotorModeID::kMIT;
    motor_.ctrl_param_.kp_set_ = 0.0f;
    motor_.ctrl_param_.kd_set_ = 0.0f;
    motor_.ctrl_param_.pos_set_rad_ = 0.0f;
    motor_.ctrl_param_.vel_set_radps_ = 0.0f;
    motor_.ctrl_param_.tor_set_Nm_ = tau_me;
    motor_.MitControl();
    // motor_.EnableMotor();
}

void FsrGaitEstimator::Calibrate()
{
    if (!gait_data_.is_used_ || gait_data_.is_calibrated_) return;

    uint32_t now_ms = GetSysTimeMs();

    bool has_fsr_data = (gait_data_.heel_.last_update_ms_ > 0) &&
                        (gait_data_.toe_.last_update_ms_ > 0);
    bool fsr_data_fresh = has_fsr_data &&
                          (now_ms - gait_data_.heel_.last_update_ms_ <= FsrGaitData::kFsrDataTimeoutMs) &&
                          (now_ms - gait_data_.toe_.last_update_ms_ <= FsrGaitData::kFsrDataTimeoutMs);
    if (!fsr_data_fresh)
    {
        gait_data_.is_calibrated_ = false;
        return;
    }

    ProcessCalibration(gait_data_.heel_, gait_data_.do_calibration_heel_fsr_, gait_data_.do_calibration_refinement_heel_fsr_);
    ProcessCalibration(gait_data_.toe_, gait_data_.do_calibration_toe_fsr_, gait_data_.do_calibration_refinement_toe_fsr_);

    bool heel_range_valid = (gait_data_.heel_.calibration_refinement_max >
                             gait_data_.heel_.calibration_refinement_min + gait_data_.heel_.adaptive_min_range);
    bool toe_range_valid = (gait_data_.toe_.calibration_refinement_max >
                            gait_data_.toe_.calibration_refinement_min + gait_data_.toe_.adaptive_min_range);
    bool heel_steps_done = (gait_data_.heel_.refinement_step_count >= FsrSensorData::kNumRefinementSteps);
    bool toe_steps_done = (gait_data_.toe_.refinement_step_count >= FsrSensorData::kNumRefinementSteps);
    bool calibration_flags_done = !(gait_data_.do_calibration_heel_fsr_ ||
                                    gait_data_.do_calibration_refinement_heel_fsr_ ||
                                    gait_data_.do_calibration_toe_fsr_ ||
                                    gait_data_.do_calibration_refinement_toe_fsr_);

    gait_data_.is_calibrated_ = calibration_flags_done &&
                                heel_steps_done &&
                                toe_steps_done &&
                                heel_range_valid &&
                                toe_range_valid;
}

void FsrGaitEstimator::PrepareUpdate(uint32_t now_ms)
{
    ClearCycleEvents();
    if (!gait_data_.is_used_) return;

    gait_data_.is_data_fresh_ = CheckDataFreshness(now_ms);
    if (!gait_data_.is_data_fresh_) return;

    ProcessSensorUpdate(gait_data_.heel_);
    ProcessSensorUpdate(gait_data_.toe_);

    gait_data_.heel_contact_state_ = gait_data_.heel_.ground_contact;
    gait_data_.toe_contact_state_ = gait_data_.toe_.ground_contact;

    DetectOwnFsrEvents();
}

void FsrGaitEstimator::FinalizeUpdate(uint32_t now_ms)
{
    if (!gait_data_.is_used_ || !gait_data_.is_data_fresh_) return;

    DetectOppositeFsrEvents();
    if (IsShankImuUsable())
    {
        DetectImuEvents();
    }
    UpdateEventTimings(now_ms);
    ResolvePhase();
    UpdatePercentages(now_ms);
    UpdateValidity();
}

void FsrGaitEstimator::CommitUpdate()
{
    if (!gait_data_.is_used_) return;

    gait_data_.prev_heel_contact_state_ = gait_data_.heel_contact_state_;
    gait_data_.prev_toe_contact_state_  = gait_data_.toe_contact_state_;
}

void FsrGaitEstimator::ClearCycleEvents()
{
    gait_data_.event_ic_ = false;
    gait_data_.event_oto_ = false;
    gait_data_.event_hr_ = false;
    gait_data_.event_oic_ = false;
    gait_data_.event_to_ = false;
    gait_data_.event_fa_ = false;
    gait_data_.event_tv_ = false;
    gait_data_.phase_changed_ = false;
    gait_data_.is_data_fresh_ = false;
    gait_data_.is_valid_ = false;
}

bool FsrGaitData::IsPacketFresh(uint32_t now_ms) const
{
    if (heel_.last_update_ms_ == 0u || toe_.last_update_ms_ == 0u) return false;
    if (now_ms < heel_.last_update_ms_ || now_ms < toe_.last_update_ms_) return false;

    return (now_ms - heel_.last_update_ms_) <= kFsrDataTimeoutMs && (now_ms - toe_.last_update_ms_) <= kFsrDataTimeoutMs;
}

void FsrGaitEstimator::DetectOwnFsrEvents()
{
    gait_data_.event_ic_ = (gait_data_.heel_contact_state_ && !gait_data_.prev_heel_contact_state_);
    gait_data_.event_hr_ = (!gait_data_.heel_contact_state_ && gait_data_.prev_heel_contact_state_);
    gait_data_.event_to_ = (!gait_data_.toe_contact_state_ && gait_data_.prev_toe_contact_state_);
}

void FsrGaitEstimator::DetectOppositeFsrEvents()
{
    FsrGaitData &opposite_fsr = gait_data_.is_left_ ? pe_.right_side_.fsr_gait_data_ : pe_.left_side_.fsr_gait_data_;
    if (!IsOppositeFsrUsable()) return;

    gait_data_.event_oto_ = opposite_fsr.event_to_;
    gait_data_.event_oic_ = opposite_fsr.event_ic_;
}

bool FsrGaitEstimator::IsOppositeFsrUsable()
{
    FsrGaitData &opposite_fsr = gait_data_.is_left_ ? pe_.right_side_.fsr_gait_data_ : pe_.left_side_.fsr_gait_data_;
    return opposite_fsr.is_used_ && opposite_fsr.is_calibrated_ && opposite_fsr.is_data_fresh_;
}

bool FsrGaitEstimator::IsShankImuUsable()
{
    ImuData &shank_imu = gait_data_.is_left_ ? pe_.left_side_.shank_imu_ : pe_.right_side_.shank_imu_;
    return shank_imu.is_used_ && shank_imu.is_calibrated_;
}

/**
 * @brief IMU 事件检测 —— FA (小腿角速度过零) + TV (小腿俯仰接近参考)
 * @note  使用大小腿 IMU 推算人体运动学, 而非外骨骼关节编码器:
 *        FA: 摆动相中小腿矢状面角速度 (gyro pitch rate) 从负(后摆)变正(前摆)的过零点
 *             → 此时膝关节屈曲最大, 双足贴近
 *        TV: 摆动末期小腿俯仰角进入站立标定参考值 ±4° 范围
 *             → 胫骨竖直, 即将足跟着地
 */
void FsrGaitEstimator::DetectImuEvents()
{
    gait_data_.event_fa_ = false;
    gait_data_.event_tv_ = false;

    ImuData &shank_imu = gait_data_.is_left_ ? pe_.left_side_.shank_imu_ : pe_.right_side_.shank_imu_;
    if (!shank_imu.is_used_ || !shank_imu.is_calibrated_) return;

    bool in_swing = (!gait_data_.heel_contact_state_ && !gait_data_.toe_contact_state_);

    /* FA 检测: 小腿矢状面角速度过零 (后摆→前摆) */
    {
        float shank_gyro_now = shank_imu.sagittal_gyro_degps_;
        if (in_swing && gait_data_.shank_gyro_prev_degps_ < 0.0f && shank_gyro_now >= 0.0f)
        {
            gait_data_.event_fa_ = true;
        }
        gait_data_.shank_gyro_prev_degps_ = shank_gyro_now;
    }

    /* TV 检测: 小腿矢状面角接近站立参考值 (胫骨竖直) */
    {
        float ref_deg = shank_imu.sagittal_ref_valid_ ? shank_imu.sagittal_ref_deg_ : gait_data_.shank_standing_ref_deg_;
        float pitch_error = fabsf(shank_imu.sagittal_angle_deg_ - ref_deg);
        if (in_swing && pitch_error < 4.0f)
        {
            gait_data_.event_tv_ = true;
        }
    }
}

/**
 * @brief 检测传感器数据新鲜度 — 超时自动降级 capability
 * @note  FSR 数据超时 → 步态估计完全不可用, percent_gait_ 置无效
 *        IMU 数据超时 → 从 kDualFsrImu 降级到 kDualFsr
 */
bool FsrGaitEstimator::CheckDataFreshness(uint32_t now_ms)
{
    bool has_fsr_data = (gait_data_.heel_.last_update_ms_ > 0u) &&
                        (gait_data_.toe_.last_update_ms_ > 0u);
    if (!has_fsr_data)
    {
        gait_data_.percent_gait_ = -1.0f;
        gait_data_.percent_stance_ = -1.0f;
        gait_data_.percent_swing_ = -1.0f;
        gait_data_.percent_subphase_ = -1.0f;
        return false;
    }

    if (!gait_data_.IsPacketFresh(now_ms))
    {
        gait_data_.percent_gait_ = -1.0f;
        gait_data_.percent_stance_ = -1.0f;
        gait_data_.percent_swing_ = -1.0f;
        gait_data_.percent_subphase_ = -1.0f;
        Reset();
        return false;
    }
    return true;
}

/**
 * @brief 根据传感器能力级别和最新事件边界解析当前 RLA 相位
 */
void FsrGaitEstimator::ResolvePhase()
{
    gait_data_.prev_phase_ = gait_data_.current_phase_;

    uint8_t latest_ev = kIC;
    uint32_t latest_ts = gait_data_.event_ts_ms_[kIC];
    bool has_opposite_fsr = IsOppositeFsrUsable();
    bool has_shank_imu = IsShankImuUsable();

    auto consider_event = [&](uint8_t ev) {
        if (gait_data_.event_ts_ms_[ev] > latest_ts ||
            (gait_data_.event_ts_ms_[ev] > 0 && gait_data_.event_ts_ms_[ev] == latest_ts))
        {
            latest_ts = gait_data_.event_ts_ms_[ev];
            latest_ev = ev;
        }
    };

    if (has_opposite_fsr)
    {
        consider_event(kOTO);
        consider_event(kOIC);
    }
    consider_event(kHR);
    consider_event(kTO);
    if (has_shank_imu)
    {
        consider_event(kFA);
        consider_event(kTV);
    }

    switch (latest_ev)
    {
    case kOTO: gait_data_.current_phase_ = GaitPhase::kMidStance; break;
    case kHR:  gait_data_.current_phase_ = GaitPhase::kTerminalStance; break;
    case kOIC: gait_data_.current_phase_ = GaitPhase::kPreSwing; break;
    case kTO:  gait_data_.current_phase_ = GaitPhase::kInitialSwing; break;
    case kFA:  gait_data_.current_phase_ = GaitPhase::kMidSwing; break;
    case kTV:  gait_data_.current_phase_ = GaitPhase::kTerminalSwing; break;
    case kIC:
    default:   gait_data_.current_phase_ = GaitPhase::kLoadingResponse; break;
    }

    gait_data_.phase_changed_ = (gait_data_.current_phase_ != gait_data_.prev_phase_);
}

/**
 * @brief 更新所有步态百分比
 * @note  percent_gait_: IC → next IC
 *        percent_stance_: IC → TO
 *        percent_swing_: TO → next IC
 *        percent_subphase_: 当前子相内百分比
 */
void FsrGaitEstimator::UpdatePercentages(uint32_t now_ms)
{
    bool has_opposite_fsr = IsOppositeFsrUsable();
    bool has_shank_imu = IsShankImuUsable();
    bool in_stance = (gait_data_.heel_contact_state_ || gait_data_.toe_contact_state_);
    bool in_swing = !in_stance;

    /* percent_gait_: 步态全周期 (IC → next IC) */
    float expected_step = gait_data_.expected_step_duration_ms_;
    if (expected_step > 0.0f)
    {
        gait_data_.percent_gait_ = 100.0f * ((float)(now_ms - gait_data_.event_ts_ms_[kIC]) / expected_step);
        if (gait_data_.percent_gait_ > 100.0f) gait_data_.percent_gait_ = 100.0f;
        if (gait_data_.percent_gait_ < 0.0f)   gait_data_.percent_gait_ = 0.0f;
    }

    /* percent_stance_: 支撑相 (IC → TO) = LR + MS + TS + PS */
    {
        float expected_stance = expected_step > 0.0f ? expected_step * 0.6f : -1.0f;
        float measured_stance = 0.0f;
        bool has_measured_stance = false;

        if (has_opposite_fsr &&
            gait_data_.expected_duration_ms_[kIC] > 0.0f &&
            gait_data_.expected_duration_ms_[kOTO] > 0.0f &&
            gait_data_.expected_duration_ms_[kHR] > 0.0f &&
            gait_data_.expected_duration_ms_[kOIC] > 0.0f)
        {
            measured_stance = gait_data_.expected_duration_ms_[kIC] + gait_data_.expected_duration_ms_[kOTO] +
                              gait_data_.expected_duration_ms_[kHR] + gait_data_.expected_duration_ms_[kOIC];
            has_measured_stance = true;
        }
        else if (gait_data_.expected_duration_ms_[kIC] > 0.0f && gait_data_.expected_duration_ms_[kHR] > 0.0f)
        {
            measured_stance = gait_data_.expected_duration_ms_[kIC] + gait_data_.expected_duration_ms_[kHR];
            has_measured_stance = true;
        }

        if (has_measured_stance) expected_stance = measured_stance;
        if (in_stance && expected_stance > 0.0f)
        {
            float percent = 100.0f * ((float)(now_ms - gait_data_.event_ts_ms_[kIC]) / expected_stance);
            if (percent > 100.0f) percent = 100.0f;
            if (percent < 0.0f) percent = 0.0f;
            if (gait_data_.percent_stance_ >= 0.0f && percent < gait_data_.percent_stance_)
                percent = gait_data_.percent_stance_;
            gait_data_.percent_stance_ = percent;
        }
    }
    if (in_swing)
        gait_data_.percent_stance_ = 0.0f; /* 摆动相中强制归零 */

    /* percent_swing_: 摆动相 (TO → next IC) = ISw + MSw + TSw */
    if (gait_data_.event_ts_ms_[kTO] > 0)
    {
        float expected_swing = expected_step > 0.0f ? expected_step * 0.4f : -1.0f;

        if (has_shank_imu &&
            gait_data_.expected_duration_ms_[kTO] > 0.0f &&
            gait_data_.expected_duration_ms_[kFA] > 0.0f &&
            gait_data_.expected_duration_ms_[kTV] > 0.0f)
        {
            expected_swing = gait_data_.expected_duration_ms_[kTO] + gait_data_.expected_duration_ms_[kFA] + gait_data_.expected_duration_ms_[kTV];
        }
        else if (gait_data_.expected_duration_ms_[kTO] > 0.0f)
        {
            expected_swing = gait_data_.expected_duration_ms_[kTO];
        }

        if (in_swing && expected_swing > 0.0f)
        {
            float percent = 100.0f * ((float)(now_ms - gait_data_.event_ts_ms_[kTO]) / expected_swing);
            if (percent > 100.0f) percent = 100.0f;
            if (percent < 0.0f) percent = 0.0f;
            if (gait_data_.percent_swing_ >= 0.0f && percent < gait_data_.percent_swing_)
                percent = gait_data_.percent_swing_;
            gait_data_.percent_swing_ = percent;
        }
    }
    if (in_stance)
        gait_data_.percent_swing_ = 0.0f;

    /* percent_subphase_: 当前 RLA 子相内百分比 (起始事件 → 下一事件) */
    uint8_t start_ev = kIC;
    switch (gait_data_.current_phase_)
    {
    case GaitPhase::kLoadingResponse: start_ev = kIC;  break;
    case GaitPhase::kMidStance:       start_ev = kOTO; break;
    case GaitPhase::kTerminalStance:  start_ev = kHR;  break;
    case GaitPhase::kPreSwing:        start_ev = kOIC; break;
    case GaitPhase::kInitialSwing:    start_ev = kTO;  break;
    case GaitPhase::kMidSwing:        start_ev = kFA;  break;
    case GaitPhase::kTerminalSwing:   start_ev = kTV;  break;
    }
    float expected_sub = gait_data_.expected_duration_ms_[start_ev];
    if (expected_sub > 0.0f)
    {
        gait_data_.percent_subphase_ = 100.0f * ((float)(now_ms - gait_data_.event_ts_ms_[start_ev]) / expected_sub);
        if (gait_data_.percent_subphase_ > 100.0f) gait_data_.percent_subphase_ = 100.0f;
        if (gait_data_.percent_subphase_ < 0.0f)   gait_data_.percent_subphase_ = 0.0f;
    }
}

void FsrGaitEstimator::UpdateValidity()
{
    gait_data_.is_valid_ =
        gait_data_.is_used_ &&
        gait_data_.is_calibrated_ &&
        gait_data_.is_data_fresh_ &&
        gait_data_.event_ts_ms_[kIC] > 0 &&
        gait_data_.expected_step_duration_ms_ > 0.0f &&
        gait_data_.percent_gait_ >= 0.0f &&
        gait_data_.percent_gait_ <= 100.0f;
}

void FsrGaitEstimator::UpdateEventTimings(uint32_t now_ms)
{
    bool has_opposite_fsr = IsOppositeFsrUsable();
    bool has_shank_imu = IsShankImuUsable();

    if (gait_data_.event_ic_)
    {
        if (gait_data_.event_ts_ms_[kIC] > 0)
        {
            uint32_t step_time = now_ms - gait_data_.event_ts_ms_[kIC];
            const float alpha = 0.25f;
            if (gait_data_.expected_step_duration_ms_ < 0.0f)
                gait_data_.expected_step_duration_ms_ = (float)step_time;
            else
                gait_data_.expected_step_duration_ms_ += alpha * ((float)step_time - gait_data_.expected_step_duration_ms_);
        }

        if (has_shank_imu && gait_data_.event_ts_ms_[kTV] > 0)
            UpdateDurationFromStartEvent(kTV, now_ms);
        else if (gait_data_.event_ts_ms_[kTO] > 0)
            UpdateDurationFromStartEvent(kTO, now_ms);
        RecordEventTimestamp(kIC, now_ms);
    }
    if (gait_data_.event_oto_)
    {
        if (gait_data_.event_ts_ms_[kIC] > 0) UpdateDurationFromStartEvent(kIC, now_ms);
        RecordEventTimestamp(kOTO, now_ms);
    }
    if (gait_data_.event_hr_)
    {
        if (has_opposite_fsr && gait_data_.event_ts_ms_[kOTO] > 0)
            UpdateDurationFromStartEvent(kOTO, now_ms);
        else if (gait_data_.event_ts_ms_[kIC] > 0)
            UpdateDurationFromStartEvent(kIC, now_ms);
        RecordEventTimestamp(kHR, now_ms);
    }
    if (gait_data_.event_oic_)
    {
        if (gait_data_.event_ts_ms_[kHR] > 0) UpdateDurationFromStartEvent(kHR, now_ms);
        RecordEventTimestamp(kOIC, now_ms);
    }
    if (gait_data_.event_to_)
    {
        if (has_opposite_fsr && gait_data_.event_ts_ms_[kOIC] > 0)
            UpdateDurationFromStartEvent(kOIC, now_ms);
        else if (gait_data_.event_ts_ms_[kHR] > 0)
            UpdateDurationFromStartEvent(kHR, now_ms);
        RecordEventTimestamp(kTO, now_ms);
    }
    if (gait_data_.event_fa_)
    {
        if (gait_data_.event_ts_ms_[kTO] > 0) UpdateDurationFromStartEvent(kTO, now_ms);
        RecordEventTimestamp(kFA, now_ms);
    }
    if (gait_data_.event_tv_)
    {
        if (gait_data_.event_ts_ms_[kFA] > 0) UpdateDurationFromStartEvent(kFA, now_ms);
        RecordEventTimestamp(kTV, now_ms);
    }
}

void FsrGaitEstimator::UpdateDurationFromStartEvent(uint8_t start_ev_idx, uint32_t now_ms)
{
    uint32_t phase_time = now_ms - gait_data_.event_ts_ms_[start_ev_idx];
    float expected = gait_data_.expected_duration_ms_[start_ev_idx];

    uint8_t num_uninitialized = 0;
    for (int i = 0; i < FsrGaitData::kNumStepsAvg; i++)
    {
        num_uninitialized += (gait_data_.event_times_ms_[start_ev_idx][i] == 0);
    }

    uint32_t *max_val = std::max_element(gait_data_.event_times_ms_[start_ev_idx], gait_data_.event_times_ms_[start_ev_idx] + FsrGaitData::kNumStepsAvg);
    uint32_t *min_val = std::min_element(gait_data_.event_times_ms_[start_ev_idx], gait_data_.event_times_ms_[start_ev_idx] + FsrGaitData::kNumStepsAvg);

    if (num_uninitialized > 0)
    {
        for (int i = (FsrGaitData::kNumStepsAvg - 1); i > 0; i--)
        {
            gait_data_.event_times_ms_[start_ev_idx][i] = gait_data_.event_times_ms_[start_ev_idx][i - 1];
        }
        gait_data_.event_times_ms_[start_ev_idx][0] = phase_time;
        uint32_t sum_times = 0;
        uint8_t valid_count = 0;
        for (int i = 0; i < FsrGaitData::kNumStepsAvg; i++)
        {
            if (gait_data_.event_times_ms_[start_ev_idx][i] > 0)
            {
                sum_times += gait_data_.event_times_ms_[start_ev_idx][i];
                valid_count++;
            }
        }
        if (valid_count > 0) expected = (float)sum_times / valid_count;
    }
    else if ((phase_time <= (gait_data_.expected_duration_window_upper_coeff_ * *max_val)) &&
             (phase_time >= (gait_data_.expected_duration_window_lower_coeff_ * *min_val)))
    {
        uint32_t sum_times = phase_time;
        for (int i = (FsrGaitData::kNumStepsAvg - 1); i > 0; i--)
        {
            sum_times += gait_data_.event_times_ms_[start_ev_idx][i - 1];
            gait_data_.event_times_ms_[start_ev_idx][i] = gait_data_.event_times_ms_[start_ev_idx][i - 1];
        }
        gait_data_.event_times_ms_[start_ev_idx][0] = phase_time;
        expected = (float)sum_times / FsrGaitData::kNumStepsAvg;
    }
    gait_data_.expected_duration_ms_[start_ev_idx] = expected;
}

void FsrGaitEstimator::RecordEventTimestamp(uint8_t ev_idx, uint32_t now_ms)
{
    gait_data_.prev_event_ts_ms_[ev_idx] = gait_data_.event_ts_ms_[ev_idx];
    gait_data_.event_ts_ms_[ev_idx] = now_ms;
    gait_data_.event_count_[ev_idx]++;
}

/**
 * @brief 重置步态估计器状态
 */
void FsrGaitEstimator::Reset()
{
    for (uint8_t ev = 0; ev < kNumGaitEvents; ev++)
    {
        gait_data_.event_ts_ms_[ev] = 0;
        gait_data_.prev_event_ts_ms_[ev] = 0;
        gait_data_.event_count_[ev] = 0;
        for (int i = 0; i < FsrGaitData::kNumStepsAvg; i++)
            gait_data_.event_times_ms_[ev][i] = 0;
    }
    arm_fill_f32(-1.0f, gait_data_.expected_duration_ms_, kNumGaitEvents); /* CMSIS-DSP 向量填充 */
    gait_data_.expected_step_duration_ms_ = -1.0f;

    gait_data_.current_phase_ = GaitPhase::kLoadingResponse;
    gait_data_.prev_phase_ = GaitPhase::kLoadingResponse;
    gait_data_.phase_changed_ = false;
    gait_data_.is_valid_ = false;
    gait_data_.percent_gait_ = -1.0f;
    gait_data_.percent_stance_ = -1.0f;
    gait_data_.percent_swing_ = -1.0f;
    gait_data_.percent_subphase_ = -1.0f;
    gait_data_.shank_gyro_prev_degps_ = 0.0f;
    gait_data_.heel_contact_state_ = false;
    gait_data_.toe_contact_state_ = false;
    gait_data_.prev_heel_contact_state_ = false;
    gait_data_.prev_toe_contact_state_ = false;
    ClearCycleEvents();
}

void FsrGaitEstimator::ResetCalibration()
{
    gait_data_.is_calibrated_ = false;
    gait_data_.do_calibration_heel_fsr_ = true;
    gait_data_.do_calibration_toe_fsr_ = true;
    gait_data_.do_calibration_refinement_heel_fsr_ = true;
    gait_data_.do_calibration_refinement_toe_fsr_ = true;

    ResetSensorCalibration(gait_data_.heel_);
    ResetSensorCalibration(gait_data_.toe_);
    Reset();
}

void FsrGaitEstimator::ResetSensorCalibration(FsrSensorData &sensor)
{
    sensor.calibrated_reading = 0.0f;
    sensor.calibration_min = -1.0f;
    sensor.calibration_max = -1.0f;
    sensor.calibration_refinement_min = -1.0f;
    sensor.calibration_refinement_max = -1.0f;
    sensor.step_max_sum = 0.0f;
    sensor.step_max = 0.0f;
    sensor.step_min_sum = 0.0f;
    sensor.step_min = 0.0f;
    sensor.calibration_start_sys_ms = 0;
    sensor.refinement_step_count = 0;
    sensor.last_do_calibrate = false;
    sensor.last_do_refinement = false;
    sensor.ground_contact_during_refinement = false;
    sensor.ground_contact = false;
    sensor.adaptive_max = 0.0f;
    sensor.adaptive_min = 0.0f;
    sensor.adaptive_inited = false;
    sensor.last_update_ms_ = 0;
}

void FsrGaitEstimator::ProcessCalibration(FsrSensorData &sensor, bool &do_calibrate, bool &do_refinement)
{
    uint32_t now_ms = GetSysTimeMs();

    /* 1. 基础校准阶段 */
    if (do_calibrate && !sensor.last_do_calibrate)
    {
        sensor.calibration_start_sys_ms = now_ms;
        sensor.calibration_max = sensor.raw_reading;
        sensor.calibration_min = sensor.calibration_max;
    }

    if (do_calibrate && (now_ms - sensor.calibration_start_sys_ms <= FsrSensorData::kCalibrationDurationMs))
    {
        sensor.calibration_max = _max(sensor.calibration_max, sensor.raw_reading);
        sensor.calibration_min = _min(sensor.calibration_min, sensor.raw_reading);
    }
    else if (do_calibrate)
    {
        do_calibrate = false;
    }
    sensor.last_do_calibrate = do_calibrate;

     /* 2. 精细校准阶段 (Refinement) — 等基础标定完成后再启动 */
    if (!do_calibrate && do_refinement && !sensor.last_do_refinement)
    {
        sensor.calibration_start_sys_ms = now_ms; /* 精标计时起点 */
        sensor.refinement_step_count = 0;
        sensor.step_max = (sensor.calibration_max + sensor.calibration_min) / 2.0f;
        sensor.step_min = sensor.step_max;
        sensor.step_max_sum = 0.0f;
        sensor.step_min_sum = 0.0f;

        /* 根据当前 raw 和基标稳定范围, 推真实着地初始状态 */
        float upper = FsrSensorData::kSchmittUpperThresholdRefinement * (sensor.calibration_max - sensor.calibration_min) + sensor.calibration_min;
        sensor.ground_contact_during_refinement = (sensor.raw_reading > upper);
    }

    if (do_refinement && !do_calibrate)
    {
        bool step_count_reached = (sensor.refinement_step_count >= FsrSensorData::kNumRefinementSteps);
        bool timed_out = (now_ms - sensor.calibration_start_sys_ms > FsrSensorData::kCalibrationDurationMs * 2); /* ~10s */

        if (!step_count_reached && !timed_out)
        {
            sensor.step_max = _max(sensor.step_max, sensor.raw_reading);
            sensor.step_min = _min(sensor.step_min, sensor.raw_reading);

            bool last_state = sensor.ground_contact_during_refinement;
            float lower = FsrSensorData::kSchmittLowerThresholdRefinement * (sensor.calibration_max - sensor.calibration_min) + sensor.calibration_min;
            float upper = FsrSensorData::kSchmittUpperThresholdRefinement * (sensor.calibration_max - sensor.calibration_min) + sensor.calibration_min;

            sensor.ground_contact_during_refinement = SchmittTrigger(sensor.raw_reading, last_state, lower, upper);

            if (sensor.ground_contact_during_refinement && !last_state)
            {
                sensor.step_max_sum += sensor.step_max;
                sensor.step_min_sum += sensor.step_min;
                float mid = (sensor.calibration_max + sensor.calibration_min) / 2.0f;
                sensor.step_max = mid;
                sensor.step_min = mid;
                sensor.refinement_step_count++;
            }
        }
        else if (step_count_reached)
        {
            sensor.calibration_refinement_max = sensor.step_max_sum / (float)sensor.refinement_step_count;
            sensor.calibration_refinement_min = sensor.step_min_sum / (float)sensor.refinement_step_count;
            do_refinement = false;
            sensor.adaptive_inited = false;
        }
        else if (timed_out)
        {
            float mid = (sensor.calibration_max + sensor.calibration_min) / 2.0f;
            sensor.calibration_start_sys_ms = now_ms;
            sensor.step_max = mid;
            sensor.step_min = mid;
        }
    }
    sensor.last_do_refinement = do_refinement;
}

void FsrGaitEstimator::UpdateAdaptiveRange(FsrSensorData& sensor)
{
    /* 首次调用或标定刚完成时, 用标定值初始化自适应范围 */
    if (!sensor.adaptive_inited)
    {
        if (sensor.calibration_refinement_max > sensor.calibration_refinement_min + 1e-6f)
        {
            sensor.adaptive_max = sensor.calibration_refinement_max;
            sensor.adaptive_min = sensor.calibration_refinement_min;
        }
        else if (sensor.calibration_max > sensor.calibration_min + 1e-6f)
        {
            sensor.adaptive_max = sensor.calibration_max;
            sensor.adaptive_min = sensor.calibration_min;
        }
        else
        {
            /* 无有效标定值, 以当前读数为基准初始化 */
            sensor.adaptive_max = sensor.raw_reading + sensor.adaptive_min_range;
            sensor.adaptive_min = sensor.raw_reading;
        }
        sensor.adaptive_inited = true;
    }

    /** 非对称包络跟踪:
     *  — 新峰值/谷值: 即时捕获 (快攻)
     *  — 无新峰值/谷值: 向当前读数缓慢衰减/上升 (慢退)
     */
    if (sensor.raw_reading > sensor.adaptive_max)
    {
        sensor.adaptive_max = sensor.raw_reading;
    }
    else
    {
        sensor.adaptive_max += sensor.adaptive_decay_rate * (sensor.raw_reading - sensor.adaptive_max);
    }

    if (sensor.raw_reading < sensor.adaptive_min)
    {
        sensor.adaptive_min = sensor.raw_reading;
    }
    else
    {
        sensor.adaptive_min += sensor.adaptive_decay_rate * (sensor.raw_reading - sensor.adaptive_min);
    }

    /* 防止长时间站立不动导致范围塌缩 */
    float range = sensor.adaptive_max - sensor.adaptive_min;
    if (range < sensor.adaptive_min_range)
    {
        sensor.adaptive_max = sensor.adaptive_min + sensor.adaptive_min_range;
    }
}

void FsrGaitEstimator::ProcessSensorUpdate(FsrSensorData& sensor)
{
    bool has_valid_range = false;
    float range_min = 0.0f;
    float range_max = 1.0f;

    /* 1. 自适应包络跟踪 (在归一化之前更新, 确保使用最新的 raw_reading) */
    if (sensor.enable_adaptive_range)
    {
        UpdateAdaptiveRange(sensor);
        if (sensor.adaptive_inited)
        {
            range_min = sensor.adaptive_min;
            range_max = sensor.adaptive_max;
            has_valid_range = (range_max > range_min + 1e-6f);
        }
    }

    /* 2. 回退: 使用精细标定值 */
    if (!has_valid_range && sensor.calibration_refinement_max > sensor.calibration_refinement_min + 1e-6f)
    {
        range_min = sensor.calibration_refinement_min;
        range_max = sensor.calibration_refinement_max;
        has_valid_range = true;
    }

    /* 3. 回退: 使用基础标定值 */
    if (!has_valid_range && sensor.calibration_max > sensor.calibration_min + 1e-6f)
    {
        range_min = sensor.calibration_min;
        range_max = sensor.calibration_max;
        has_valid_range = true;
    }

    /* 4. 归一化到 [0, 1] */
    if (has_valid_range)
    {
        sensor.calibrated_reading = (sensor.raw_reading - range_min) / (range_max - range_min);
        if (sensor.calibrated_reading < 0.0f) sensor.calibrated_reading = 0.0f;
        if (sensor.calibrated_reading > 1.0f) sensor.calibrated_reading = 1.0f;
    }
    else
    {
        sensor.calibrated_reading = sensor.raw_reading;
    }

    /* 5. 施密特触发器更新着地状态 */
    if (has_valid_range)
    {
        sensor.ground_contact = SchmittTrigger(sensor.calibrated_reading, sensor.ground_contact, sensor.schmitt_lower_threshold_calc_contact, sensor.schmitt_upper_threshold_calc_contact);
    }
}



void Side::Calibrate()
{
    /* #TODO 实现关节的标定 */
    hip_joint_.Calibrate();
    knee_joint_.Calibrate();
    ankle_joint_.Calibrate();
    knee_sea_joint_.Calibrate();

    
    /* 已在knee_sea_joint_.Standby做了零力控制; 膝关节标定完成、进入零力控制后膝关节才走得动, 才能标定fsr */
    if (ps_.knee_sea_joint_.is_calibrated_ || !ps_.knee_sea_joint_.is_used_)
    {
        fsr_gait_estimator_.Calibrate();
    }

    ps_.is_calibrated_ =
        (ps_.fsr_gait_data_.is_calibrated_ || !ps_.fsr_gait_data_.is_used_) &&
        (ps_.hip_joint_.is_calibrated_ || !ps_.hip_joint_.is_used_) &&
        (ps_.knee_joint_.is_calibrated_ || !ps_.knee_joint_.is_used_) &&
        (ps_.ankle_joint_.is_calibrated_ || !ps_.ankle_joint_.is_used_) &&
        (ps_.knee_sea_joint_.is_calibrated_ || !ps_.knee_sea_joint_.is_used_);
}

void Side::Read()
{
    if (!ps_.is_used_) return;

    hip_joint_.Read();
    knee_joint_.Read();
    ankle_joint_.Read();
    knee_sea_joint_.Read();
}

bool Side::IsMotorConnect()
{
    if (!ps_.is_used_) return true;

    bool hip_ok = hip_joint_.IsMotorConnect();
    bool knee_ok = knee_joint_.IsMotorConnect();
    bool ankle_ok = ankle_joint_.IsMotorConnect();

    bool knee_sea_ok = knee_sea_joint_.IsMotorConnect();

    return hip_ok && knee_ok && ankle_ok && knee_sea_ok;
}

void Side::Standby()
{
    if (!ps_.is_used_) return;

    /* #TODO Standby的策略需要优化 */
    hip_joint_.Standby();
    knee_joint_.Standby();
    ankle_joint_.Standby();

    knee_sea_joint_.Standby();
}

void Side::Assist()
{
    if (!ps_.is_used_) return;

    hip_joint_.Assist();
    knee_joint_.Assist();
    ankle_joint_.Assist();

    knee_sea_joint_.Assist();
}

void Side::Shutdown()
{
    hip_joint_.Shutdown();
    knee_joint_.Shutdown();
    ankle_joint_.Shutdown();
    knee_sea_joint_.Shutdown();

    fsr_gait_estimator_.Reset();
    stair_phase_estimator_.Reset();
}

void StairPhaseEstimator::Update()
{
    StairPhaseData &stair = ps_.stair_phase_data_;
    stair.prev_phase_ = stair.current_phase_;
    stair.phase_changed_ = false;
    stair.is_valid_ = false;

    /* HACK: 暂时删除 */
    // bool fsr_ok = ps_.fsr_gait_data_.is_used_ && ps_.fsr_gait_data_.is_calibrated_ && ps_.fsr_gait_data_.is_data_fresh_;
    // bool imu_ok = ps_.thigh_imu_.is_used_ && ps_.thigh_imu_.is_calibrated_ && ps_.shank_imu_.is_used_ && ps_.shank_imu_.is_calibrated_;
    // if (!fsr_ok || !imu_ok)
    // {
    //     TransitionTo(StairPhase::kSwing, stair.knee_angle_deg_);
    //     stair.percent_subphase_ = -1.0f;
    //     stair.phase_changed_ = (stair.current_phase_ != stair.prev_phase_);
    //     return;
    // }

    // 直接从绑定的 side_ 引用中提取所需物理数据
    bool is_contact = (ps_.fsr_gait_data_.heel_contact_state_ || ps_.fsr_gait_data_.toe_contact_state_);
    float knee_angle = ps_.knee_joint_.sagittal_valid_ ?
        ps_.knee_joint_.sagittal_pos_rad_ * RAD_TO_DEG :
        ps_.thigh_imu_.sagittal_angle_deg_ - ps_.shank_imu_.sagittal_angle_deg_;
    float knee_vel = ps_.knee_joint_.sagittal_valid_ ?
        ps_.knee_joint_.sagittal_vel_radps_ * RAD_TO_DEG :
        ps_.thigh_imu_.sagittal_gyro_degps_ - ps_.shank_imu_.sagittal_gyro_degps_;

    bool prev_contact = stair.is_contact_;
    bool contact_rising = is_contact && !prev_contact;
    bool contact_falling = !is_contact && prev_contact;

    stair.is_valid_ = true;
    stair.prev_is_contact_ = prev_contact;
    stair.is_contact_ = is_contact;
    stair.knee_angle_deg_ = knee_angle;
    stair.knee_vel_degps_ = knee_vel;

    if (contact_rising)
    {
        stair.contact_start_knee_angle_deg_ = knee_angle;
        stair.min_knee_angle_since_contact_deg_ = knee_angle;
    }
    else if (is_contact)
    {
        if (knee_angle < stair.min_knee_angle_since_contact_deg_)
        {
            stair.min_knee_angle_since_contact_deg_ = knee_angle;
        }
    }

    if (pe_.intention_data_.current_mode_ == ExoData::LocoMode::kStairAscent)
    {
        switch (stair.current_phase_) {
            case StairPhase::kSwing:
                if (contact_rising && knee_angle >= kStairStepContactAngleThresh)
                {
                    TransitionTo(StairPhase::kWeightAcceptance, knee_angle);
                }
                break;
            case StairPhase::kWeightAcceptance:
                if (contact_falling)
                {
                    TransitionTo(StairPhase::kSwing, knee_angle);
                    break;
                }
                if ((stair.contact_start_knee_angle_deg_ - knee_angle) >= kPullUpStartDeltaDeg)
                {
                    TransitionTo(StairPhase::kPullUp, knee_angle);
                }
                break;
            case StairPhase::kPullUp:
                if (contact_falling)
                {
                    TransitionTo(StairPhase::kSwing, knee_angle);
                    break;
                }
                if (knee_angle < kKneeStraightAngle)
                {
                    TransitionTo(StairPhase::kForwardContinuance, knee_angle);
                }
                break;
            case StairPhase::kForwardContinuance:
                if (contact_falling)
                {
                    TransitionTo(StairPhase::kSwing, knee_angle);
                    break;
                }
                if ((knee_angle - stair.min_knee_angle_since_contact_deg_) >= kForwardEndFlexionDeltaDeg)
                {
                    TransitionTo(StairPhase::kSwing, knee_angle);
                }
                break;
            default:
                TransitionTo(StairPhase::kSwing, knee_angle);
                break;
        }
    }
    else if (pe_.intention_data_.current_mode_ == ExoData::LocoMode::kStairDescent)
    {
        switch (stair.current_phase_)
        {
        case StairPhase::kSwing:
            if (contact_rising)
            {
                TransitionTo(StairPhase::kWeightAcceptance, knee_angle);
            }
            break;
        case StairPhase::kWeightAcceptance:
            if (contact_falling)
            {
                TransitionTo(StairPhase::kSwing, knee_angle);
                break;
            }
            if ((knee_angle - stair.contact_start_knee_angle_deg_) >= kControlledLoweringStartDeltaDeg ||
                knee_vel > kKneeFlexingVelThresh)
            {
                TransitionTo(StairPhase::kControlledLowering, knee_angle);
            }
            break;
        case StairPhase::kControlledLowering:
            if (contact_falling)
            {
                TransitionTo(StairPhase::kSwing, knee_angle);
            }
            break;
        default:
            TransitionTo(StairPhase::kSwing, knee_angle);
            break;
        }
    }

    UpdatePercentSubphase(knee_angle);
    stair.phase_changed_ = (stair.current_phase_ != stair.prev_phase_);
}

void StairPhaseEstimator::Reset()
{
    StairPhaseData &stair = ps_.stair_phase_data_;
    stair.current_phase_ = StairPhase::kSwing;
    stair.prev_phase_ = StairPhase::kSwing;
    stair.phase_changed_ = false;
    stair.is_valid_ = false;
    stair.is_contact_ = false;
    stair.prev_is_contact_ = false;
    stair.knee_angle_deg_ = 0.0f;
    stair.knee_vel_degps_ = 0.0f;
    stair.contact_start_knee_angle_deg_ = 0.0f;
    stair.min_knee_angle_since_contact_deg_ = 0.0f;
    stair.phase_start_knee_angle_deg_ = 0.0f;
    stair.phase_target_knee_angle_deg_ = 0.0f;
    stair.percent_subphase_ = -1.0f;
}

void StairPhaseEstimator::TransitionTo(StairPhase next_phase, float knee_angle_deg)
{
    StairPhaseData &stair = ps_.stair_phase_data_;
    if (stair.current_phase_ == next_phase) return;

    stair.current_phase_ = next_phase;
    stair.phase_start_knee_angle_deg_ = knee_angle_deg;
    stair.phase_target_knee_angle_deg_ = knee_angle_deg;

    switch (next_phase)
    {
    case StairPhase::kPullUp:
        stair.phase_target_knee_angle_deg_ = kKneeStraightAngle;
        break;
    case StairPhase::kControlledLowering:
        stair.phase_target_knee_angle_deg_ = knee_angle_deg + kControlledLoweringRangeDeg;
        break;
    default:
        break;
    }
}

void StairPhaseEstimator::UpdatePercentSubphase(float knee_angle_deg)
{
    StairPhaseData &stair = ps_.stair_phase_data_;
    float percent = -1.0f;

    switch (stair.current_phase_)
    {
    case StairPhase::kPullUp:
    {
        float denom = stair.phase_start_knee_angle_deg_ - stair.phase_target_knee_angle_deg_;
        if (denom < kPullUpMinRangeDeg) denom = kPullUpMinRangeDeg;
        percent = 100.0f * StairPhaseEstimator::Clamp01((stair.phase_start_knee_angle_deg_ - knee_angle_deg) / denom);
        break;
    }
    case StairPhase::kControlledLowering:
        percent = 100.0f * StairPhaseEstimator::Clamp01((knee_angle_deg - stair.phase_start_knee_angle_deg_) / kControlledLoweringRangeDeg);
        break;
    case StairPhase::kForwardContinuance:
        percent = 100.0f;
        break;
    case StairPhase::kWeightAcceptance:
    case StairPhase::kSwing:
    default:
        percent = 0.0f;
        break;
    }

    stair.percent_subphase_ = percent;
}

float StairPhaseEstimator::Clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

void StsPhaseEstimator::Update()
{
    StsPhaseData &sts = pe_.sts_phase_data_;
    sts.prev_phase_ = sts.current_phase_;
    sts.phase_changed_ = false;
    sts.is_valid_ = false;
    sts.valid_side_count_ = 0;

    bool is_sts_mode = (pe_.intention_data_.current_mode_ == ExoData::LocoMode::kSitToStand ||
                        pe_.intention_data_.current_mode_ == ExoData::LocoMode::kStandToSit);
    if (!is_sts_mode || !pe_.intention_data_.is_sts_ref_valid_)
    {
        sts.percent_transition_ = -1.0f;
        return;
    }

    float thigh_pitch_sum = 0.0f;
    float thigh_vel_sum = 0.0f;
    float knee_angle_sum = 0.0f;
    float knee_vel_sum = 0.0f;

    auto accumulate_side = [&](const SideData &side) {
        const ImuData &thigh = side.thigh_imu_;
        const ImuData &shank = side.shank_imu_;
        if (!thigh.is_used_ || !thigh.is_calibrated_ ||
            !shank.is_used_ || !shank.is_calibrated_)
        {
            return;
        }

        thigh_pitch_sum += thigh.sagittal_angle_deg_;
        thigh_vel_sum += thigh.sagittal_gyro_degps_;
        knee_angle_sum += GetStandingReferencedKneePitchDeg(pe_, side);
        knee_vel_sum += side.knee_joint_.sagittal_valid_ ?
            side.knee_joint_.sagittal_vel_radps_ * RAD_TO_DEG :
            thigh.sagittal_gyro_degps_ - shank.sagittal_gyro_degps_;
        sts.valid_side_count_++;
    };

    accumulate_side(pe_.left_side_);
    accumulate_side(pe_.right_side_);

    if (sts.valid_side_count_ == 0)
    {
        sts.percent_transition_ = -1.0f;
        return;
    }

    float inv_count = 1.0f / (float)sts.valid_side_count_;
    sts.avg_thigh_pitch_deg_ = thigh_pitch_sum * inv_count;
    sts.avg_thigh_vel_degps_ = thigh_vel_sum * inv_count;
    sts.avg_knee_angle_deg_ = knee_angle_sum * inv_count;
    sts.avg_knee_vel_degps_ = knee_vel_sum * inv_count;
    sts.is_feet_loaded_ =
        (pe_.left_side_.fsr_gait_data_.heel_contact_state_ || pe_.left_side_.fsr_gait_data_.toe_contact_state_) &&
        (pe_.right_side_.fsr_gait_data_.heel_contact_state_ || pe_.right_side_.fsr_gait_data_.toe_contact_state_);
    sts.is_valid_ = true;

    if (pe_.intention_data_.current_mode_ == ExoData::LocoMode::kSitToStand)
    {
        if (sts.current_phase_ == StsPhase::kYielding || sts.current_phase_ == StsPhase::kSeated)
        {
            TransitionTo(StsPhase::kSeatOff);
        }

        switch (sts.current_phase_)
        {
        case StsPhase::kSeatOff:
            if (sts.avg_knee_vel_degps_ < kKneeExtendingVelThresh)
            {
                TransitionTo(StsPhase::kExtension);
            }
            break;
        case StsPhase::kExtension:
            if (fabsf(sts.avg_thigh_pitch_deg_ - pe_.intention_data_.thigh_pitch_standing_ref_deg_) < kCompleteThreshDeg &&
                sts.avg_knee_angle_deg_ < kKneeStraightAngle)
            {
                TransitionTo(StsPhase::kStabilization);
            }
            break;
        case StsPhase::kStabilization:
            break;
        default:
            TransitionTo(StsPhase::kSeatOff);
            break;
        }
    }
    else if (pe_.intention_data_.current_mode_ == ExoData::LocoMode::kStandToSit)
    {
        if (sts.current_phase_ != StsPhase::kYielding && sts.current_phase_ != StsPhase::kSeated)
        {
            TransitionTo(StsPhase::kYielding);
        }

        switch (sts.current_phase_)
        {
        case StsPhase::kYielding:
            if (pe_.intention_data_.sitting_posture_ref_valid_)
            {
                if (fabsf(sts.avg_thigh_pitch_deg_ - pe_.intention_data_.thigh_pitch_sitting_ref_deg_) < kCompleteThreshDeg)
                {
                    TransitionTo(StsPhase::kSeated);
                }
            }
            else if (sts.avg_knee_angle_deg_ > kSeatedKneeAngle &&
                     fabsf(sts.avg_knee_vel_degps_) < kKneeStillVelThresh)
            {
                TransitionTo(StsPhase::kSeated);
            }
            break;
        case StsPhase::kSeated:
            break;
        default:
            TransitionTo(StsPhase::kYielding);
            break;
        }
    }

    UpdatePercentTransition();
    sts.phase_changed_ = (sts.current_phase_ != sts.prev_phase_);
}

void StsPhaseEstimator::Reset()
{
    StsPhaseData &sts = pe_.sts_phase_data_;
    sts.current_phase_ = StsPhase::kSeatOff;
    sts.prev_phase_ = StsPhase::kSeatOff;
    sts.phase_changed_ = false;
    sts.is_valid_ = false;
    sts.is_feet_loaded_ = false;
    sts.valid_side_count_ = 0;
    sts.avg_thigh_pitch_deg_ = 0.0f;
    sts.avg_thigh_vel_degps_ = 0.0f;
    sts.avg_knee_angle_deg_ = 0.0f;
    sts.avg_knee_vel_degps_ = 0.0f;
    sts.percent_transition_ = -1.0f;
}

void StsPhaseEstimator::TransitionTo(StsPhase next_phase)
{
    StsPhaseData &sts = pe_.sts_phase_data_;
    if (sts.current_phase_ == next_phase) return;
    sts.current_phase_ = next_phase;
}

void StsPhaseEstimator::UpdatePercentTransition()
{
    StsPhaseData &sts = pe_.sts_phase_data_;
    float progress = 0.0f;
    if (pe_.intention_data_.sitting_posture_ref_valid_)
    {
        float range = pe_.intention_data_.thigh_pitch_sitting_ref_deg_ - pe_.intention_data_.thigh_pitch_standing_ref_deg_;
        if (fabsf(range) < kMinTransitionRangeDeg)
        {
            sts.percent_transition_ = -1.0f;
            sts.is_valid_ = false;
            return;
        }

        if (pe_.intention_data_.current_mode_ == ExoData::LocoMode::kSitToStand)
        {
            progress = (pe_.intention_data_.thigh_pitch_sitting_ref_deg_ - sts.avg_thigh_pitch_deg_) / range;
        }
        else if (pe_.intention_data_.current_mode_ == ExoData::LocoMode::kStandToSit)
        {
            progress = (sts.avg_thigh_pitch_deg_ - pe_.intention_data_.thigh_pitch_standing_ref_deg_) / range;
        }
        else
        {
            sts.percent_transition_ = -1.0f;
            sts.is_valid_ = false;
            return;
        }
    }
    else if (pe_.intention_data_.current_mode_ == ExoData::LocoMode::kSitToStand)
    {
        progress = 1.0f - (sts.avg_knee_angle_deg_ / kSeatedKneeAngle);
    }
    else if (pe_.intention_data_.current_mode_ == ExoData::LocoMode::kStandToSit)
    {
        progress = sts.avg_knee_angle_deg_ / kSeatedKneeAngle;
    }
    else
    {
        sts.percent_transition_ = -1.0f;
        sts.is_valid_ = false;
        return;
    }

    sts.percent_transition_ = 100.0f * StsPhaseEstimator::Clamp01(progress);
}

float StsPhaseEstimator::Clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}



bool AdaptiveOscillator::IsAoImuUsable(const ImuData &imu)
{
    return imu.is_used_ && imu.is_calibrated_;
}

bool AdaptiveOscillator::IsAoFsrUsable(const FsrGaitData &fsr)
{
    return fsr.is_used_ && fsr.is_calibrated_ && fsr.is_data_fresh_;
}

float AdaptiveOscillator::ClampValue(float value, float lower, float upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

float AdaptiveOscillator::WrapTo2Pi(float angle_rad)
{
    float wrapped = fmodf(angle_rad, _2PI);
    if (wrapped < 0.0f) wrapped += _2PI;
    return wrapped;
}

float AdaptiveOscillator::WrapToPi(float angle_rad)
{
    float wrapped = WrapTo2Pi(angle_rad);
    if (wrapped > _PI) wrapped -= _2PI;
    return wrapped;
}

void AdaptiveOscillator::ResetCoreState()
{
    left_tk_sys_us_ = 0;
    right_tk_sys_us_ = 0;
    both_foot_contact_duration_us_ = 0;

    pe_.ao_data_.left_event_cnt_ = 0;
    pe_.ao_data_.right_event_cnt_ = 0;
    pe_.ao_data_.left_phi_comp_rad_ = 0.0f;
    pe_.ao_data_.right_phi_comp_rad_ = 0.0f;
    pe_.ao_data_.fitted_signal_rad_ = 0.0f;
    pe_.ao_data_.frequency_hz_ = kDefaultFrequencyHz;
    pe_.ao_data_.is_valid_ = false;

    hat_x_ = 0.0f;
    omega_ = _2PI * kDefaultFrequencyHz;
    arm_fill_f32(0.0f, phi_, kNumAOs);
    arm_fill_f32(0.0f, alpha_, kNumAOs);
    alpha_[0] = kInitialHarmonicAmplitudeRad;
    alpha0_ = 0.0f;
    vel_energy_ema_ = 0.0f;

    left_epsilon_phi_tk_ = 0.0f;
    right_epsilon_phi_tk_ = 0.0f;
    left_phi_e_ = 0.0f;
    right_phi_e_ = 0.0f;
}

/**
 * @brief 自适应频率振荡器更新 —— 用左右大腿 IMU roll 差估计步态相位
 *
 * AO 的连续相位由 left_thigh_roll - right_thigh_roll 驱动; 左/右足底首次接触
 * 分别作为该侧 0 相位锚点。这样 FSR 只负责相位复位, IMU 周期信号负责频率自适应。
 */
void AdaptiveOscillator::Update()
{
    if (!pe_.ao_data_.is_used_)
    {
        if (pe_.ao_data_.is_valid_ ||
            pe_.ao_data_.left_event_cnt_ != 0u ||
            pe_.ao_data_.right_event_cnt_ != 0u)
        {
            Reset();
        }
        return;
    }

    const uint64_t tnow_sys_us = GetSysTimeUs();
    uint64_t delta_ts_us = 1000u;
    if (tprev_sys_us_ != 0u && tnow_sys_us >= tprev_sys_us_)
    {
        delta_ts_us = tnow_sys_us - tprev_sys_us_;
        if (delta_ts_us > 10000u) delta_ts_us = 10000u;
        if (delta_ts_us == 0u) delta_ts_us = 1000u;
    }
    const float delta_ts_s = (float)delta_ts_us * 1e-6f;

    const SideData &left_side = pe_.left_side_;
    const SideData &right_side = pe_.right_side_;
    const ImuData &left_thigh = left_side.thigh_imu_;
    const ImuData &right_thigh = right_side.thigh_imu_;

    if (!IsAoImuUsable(left_thigh) || !IsAoImuUsable(right_thigh))
    {
        ResetCoreState();
        contact_state_initialized_ = false;
        tprev_sys_us_ = tnow_sys_us;
        return;
    }

    const float left_roll_rad = left_thigh.sagittal_angle_rad_;
    const float right_roll_rad = right_thigh.sagittal_angle_rad_;
    const float x_teach = WrapToPi(left_roll_rad - right_roll_rad);
    pe_.ao_data_.teach_signal_rad_ = x_teach;

    const FsrGaitData &left_fsr = left_side.fsr_gait_data_;
    const FsrGaitData &right_fsr = right_side.fsr_gait_data_;
    const bool left_fsr_ok = IsAoFsrUsable(left_fsr);
    const bool right_fsr_ok = IsAoFsrUsable(right_fsr);
    const bool left_contact = left_fsr_ok && (left_fsr.heel_contact_state_ || left_fsr.toe_contact_state_);
    const bool right_contact = right_fsr_ok && (right_fsr.heel_contact_state_ || right_fsr.toe_contact_state_);

    bool left_contact_edge = false;
    bool right_contact_edge = false;
    if (!contact_state_initialized_)
    {
        left_contact_prev_ = left_contact;
        right_contact_prev_ = right_contact;
        contact_state_initialized_ = true;
    }
    else
    {
        left_contact_edge = left_contact && !left_contact_prev_;
        right_contact_edge = right_contact && !right_contact_prev_;
        left_contact_prev_ = left_contact;
        right_contact_prev_ = right_contact;
    }

    bool is_left_event = (left_fsr_ok && left_fsr.event_ic_) || left_contact_edge;
    bool is_right_event = (right_fsr_ok && right_fsr.event_ic_) || right_contact_edge;

    const float left_roll_vel_radps = left_thigh.gyro_degps_[0] * DEG_TO_RAD;
    const float right_roll_vel_radps = right_thigh.gyro_degps_[0] * DEG_TO_RAD;
    float vel_energy = 0.0f;
    (void)arm_sqrt_f32(left_roll_vel_radps * left_roll_vel_radps +
                       right_roll_vel_radps * right_roll_vel_radps,
                       &vel_energy);
    const float energy_alpha = 1.0f - expf(-delta_ts_s / kEmaTauS);
    vel_energy_ema_ = energy_alpha * vel_energy + (1.0f - energy_alpha) * vel_energy_ema_;

    if (left_contact && right_contact)
    {
        both_foot_contact_duration_us_ += delta_ts_us;
    }
    else
    {
        both_foot_contact_duration_us_ = 0;
    }

    const bool is_stopping = (both_foot_contact_duration_us_ > kMaxStoppingDurationUs);
    const bool is_long_time_no_event =
        ((pe_.ao_data_.left_event_cnt_ > 0u) && (tnow_sys_us - left_tk_sys_us_ > kMaxTstrideUs)) ||
        ((pe_.ao_data_.right_event_cnt_ > 0u) && (tnow_sys_us - right_tk_sys_us_ > kMaxTstrideUs));
    const bool is_two_side_event_cnt_abnormal =
        (pe_.ao_data_.left_event_cnt_ > pe_.ao_data_.right_event_cnt_ + 1u) ||
        (pe_.ao_data_.right_event_cnt_ > pe_.ao_data_.left_event_cnt_ + 1u);

    if (is_stopping || is_long_time_no_event || is_two_side_event_cnt_abnormal)
    {
        ResetCoreState();
        contact_state_initialized_ = false;
        pe_.ao_data_.teach_signal_rad_ = x_teach;
        tprev_sys_us_ = tnow_sys_us;
        return;
    }

    auto accept_event = [&](bool event, uint32_t &event_cnt, uint64_t &event_time_us) -> bool {
        if (!event) return false;
        if (event_cnt > 0u && (tnow_sys_us - event_time_us) <= kMinTstrideUs) return false;
        event_cnt++;
        event_time_us = tnow_sys_us;
        return true;
    };

    is_left_event = accept_event(is_left_event, pe_.ao_data_.left_event_cnt_, left_tk_sys_us_);
    is_right_event = accept_event(is_right_event, pe_.ao_data_.right_event_cnt_, right_tk_sys_us_);

    const bool has_event_anchor =
        pe_.ao_data_.left_event_cnt_ > 0u || pe_.ao_data_.right_event_cnt_ > 0u;
    if (!has_event_anchor)
    {
        pe_.ao_data_.fitted_signal_rad_ = hat_x_;
        pe_.ao_data_.frequency_hz_ = omega_ / _2PI;
        pe_.ao_data_.is_valid_ = false;
        tprev_sys_us_ = tnow_sys_us;
        return;
    }

    hat_x_ = alpha0_;
    float amplitude = fabsf(alpha0_);
    for (uint8_t i = 0; i < kNumAOs; i++)
    {
        hat_x_ += alpha_[i] * arm_sin_f32(phi_[i]);
        amplitude += fabsf(alpha_[i]);
    }
    if (amplitude < kMinOscAmplitudeRad) amplitude = kMinOscAmplitudeRad;

    const float error = x_teach - hat_x_;
    const float error_norm = ClampValue(error / amplitude, -kMaxNormalizedError, kMaxNormalizedError);

    for (uint8_t i = 0; i < kNumAOs; i++)
    {
        float sin_val = 0.0f;
        float cos_val = 0.0f;
        arm_sin_cos_f32(RAD_TO_DEG * phi_[i], &sin_val, &cos_val);

        const float harmonic = (float)(i + 1u);
        const float dot_phi = harmonic * omega_ + v_phi_ * error_norm * cos_val;
        const float dot_alpha = eta_ * error * sin_val;
        phi_[i] = WrapTo2Pi(phi_[i] + dot_phi * delta_ts_s);
        alpha_[i] = ClampValue(alpha_[i] + dot_alpha * delta_ts_s,
                               -kMaxHarmonicAmplitudeRad,
                               kMaxHarmonicAmplitudeRad);
    }

    const float dot_omega = v_omega_ * error_norm * arm_cos_f32(phi_[0]);
    omega_ = ClampValue(omega_ + dot_omega * delta_ts_s,
                        _2PI * kMinFrequencyHz,
                        _2PI * kMaxFrequencyHz);
    alpha0_ = ClampValue(alpha0_ + eta_ * error * delta_ts_s, -kMaxBiasRad, kMaxBiasRad);

    const float phi_n = WrapTo2Pi(phi_[0]);
    auto update_phase_compensation = [&](bool event,
                                         uint64_t event_time_us,
                                         float &epsilon_phi_tk,
                                         float &phi_e) -> float {
        if (event)
        {
            const float compensated_phase = WrapTo2Pi(phi_n + phi_e);
            const float phase_error = WrapToPi(-compensated_phase);
            epsilon_phi_tk = kp_ * phase_error;
        }

        if (event_time_us != 0u && tnow_sys_us >= event_time_us)
        {
            const float elapsed_s = (float)(tnow_sys_us - event_time_us) * 1e-6f;
            const float decay = expf(-omega_ * elapsed_s);
            phi_e = WrapToPi(phi_e + epsilon_phi_tk * omega_ * decay * delta_ts_s);
        }
        return WrapTo2Pi(phi_n + phi_e);
    };

    pe_.ao_data_.left_phi_comp_rad_ =
        update_phase_compensation(is_left_event, left_tk_sys_us_, left_epsilon_phi_tk_, left_phi_e_);
    pe_.ao_data_.right_phi_comp_rad_ =
        update_phase_compensation(is_right_event, right_tk_sys_us_, right_epsilon_phi_tk_, right_phi_e_);

    pe_.ao_data_.fitted_signal_rad_ = hat_x_;
    pe_.ao_data_.frequency_hz_ = omega_ / _2PI;
    pe_.ao_data_.is_valid_ = (pe_.ao_data_.left_event_cnt_ > 0u &&
                              pe_.ao_data_.right_event_cnt_ > 0u &&
                              left_fsr_ok &&
                              right_fsr_ok);

    tprev_sys_us_ = tnow_sys_us;
}

void AdaptiveOscillator::Reset()
{
    tprev_sys_us_ = 0;
    contact_state_initialized_ = false;
    left_contact_prev_ = false;
    right_contact_prev_ = false;
    ResetCoreState();
    pe_.ao_data_.teach_signal_rad_ = 0.0f;
}

ExoShell::ExoShell(UART_HandleTypeDef &huart, Exo &exo) : Shell(huart), exo_(exo)
{
    RegisterCommand("setlm", CmdWrapper<ExoShell, &ExoShell::OnCmdSetLocoMode>, this);
    RegisterCommand("setslope", CmdWrapper<ExoShell, &ExoShell::OnCmdSetSlope>, this);

    RegisterCommand("lbvalidsw", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.intention_data_.label_is_valid_ = ! shell.exo_.pe_.intention_data_.label_is_valid_;
        if (shell.exo_.pe_.intention_data_.label_is_valid_) shell.SendString("Data label valid: ON\r\n");
        else shell.SendString("Data label valid: OFF\r\n");
    }, this);

    RegisterCommand("vofasw", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.telemetry_config_.enable = ! shell.exo_.pe_.telemetry_config_.enable;
        if (shell.exo_.pe_.telemetry_config_.enable) shell.SendString("VOFA telemetry: ON\r\n");
        else shell.SendString("VOFA telemetry: OFF\r\n");
    }, this);

    RegisterCommand("estop", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kEmergencyStop;
        shell.SendString("!!! EMERGENCY STOP: power cycle required !!!\r\n");
    }, this);

    RegisterCommand("wakeup", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kWakeup;
        shell.SendString("-> Wakeup Requested\r\n");
    }, this);

    RegisterCommand("calib", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kStartCalibrate;
        shell.SendString("-> Calibration Requested\r\n");
        shell.SendString("   Keep shoes flat on level ground and stand still.\r\n");
    }, this);

    RegisterCommand("calibseat", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        if (shell.exo_.CalibrateSittingReference())
        {
            shell.SendString("-> Sitting reference calibrated\r\n");
        }
        else
        {
            shell.SendString("Error: usable thigh+shank IMU pair required for sitting reference\r\n");
        }
    }, this);

    RegisterCommand("start", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kStartAssist;
        shell.SendString("-> Start Assist Requested\r\n");
    }, this);

    RegisterCommand("stop", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kStopAssist;
        shell.SendString("-> Stop Assist Requested\r\n");
    }, this);

    RegisterCommand("sleep", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kEnterSleep;
        shell.SendString("-> Sleep Requested\r\n");
    }, this);

    RegisterCommand("clearfaults", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kClearFaults;
        shell.SendString("-> Clear Faults Requested\r\n");
    }, this);

    RegisterCommand("testsw", [](void *ctx, int, char **) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.do_test = ! shell.exo_.pe_.do_test;
        if (shell.exo_.pe_.do_test) shell.SendString("Test: ON\r\n");
        else shell.SendString("Test: OFF\r\n");
    }, this);

    
    /* 注册需要实时调节的参数 */
    RegisterRwParam("weight", &exo_.pe_.user_weight_kg_);
    RegisterRwParam("assiston",  &exo_.left_side_.ankle_joint_.assistance_start_phase_percent_);
    RegisterRwParam("assistoff", &exo_.left_side_.ankle_joint_.assistance_end_phase_percent_);
    RegisterRwParam("cablepre", &exo_.left_side_.ankle_joint_.cable_pre_tensioned_position_);
    RegisterRwParam("cableten", &exo_.left_side_.ankle_joint_.cable_tensioned_position_);
    RegisterRwParam("torkp", &exo_.right_side_.knee_joint_.joint_tor_pid_.kp_);
    RegisterRwParam("torki", &exo_.right_side_.knee_joint_.joint_tor_pid_.ki_);
    RegisterRwParam("lslopea", &exo_.pe_.intention_data_.left_slope_fit_a_);
    RegisterRwParam("lslopeb", &exo_.pe_.intention_data_.left_slope_fit_b_);
    RegisterRwParam("rslopea", &exo_.pe_.intention_data_.right_slope_fit_a_);
    RegisterRwParam("rslopeb", &exo_.pe_.intention_data_.right_slope_fit_b_);

    RegisterCommand("fsrdecay", [](void *ctx, int argc, char **argv) {
        auto& shell = *static_cast<ExoShell *>(ctx);
        auto& exo = shell.exo_;
        if (argc < 2 || argv == nullptr || argv[1] == nullptr)
        {
            shell.SendString("Usage: fsrdecay <0.0~1.0>\r\n");
            return;
        }

        float decay = Shell::GetFloat(argc, argv, 1, 0.0f);
        if (decay < 0.0f || decay > 1.0f)
        {
            shell.SendString("Error: fsrdecay range is 0.0~1.0\r\n");
            return;
        }

        exo.pe_.left_side_.fsr_gait_data_.heel_.adaptive_decay_rate = decay;
        exo.pe_.left_side_.fsr_gait_data_.toe_.adaptive_decay_rate = decay;
        exo.pe_.right_side_.fsr_gait_data_.heel_.adaptive_decay_rate = decay;
        exo.pe_.right_side_.fsr_gait_data_.toe_.adaptive_decay_rate = decay;
        shell.Printf("FSR adaptive decay: %.6f\r\n", decay);
    }, this);

    RegisterCommand("fsradaptsw", [](void *ctx, int, char **) {
        auto& exo = static_cast<ExoShell *>(ctx)->exo_;
        /* 一次性切换全部 4 个 FSR 的自适应范围开关 */
        bool new_state = !exo.pe_.left_side_.fsr_gait_data_.heel_.enable_adaptive_range;
        exo.pe_.left_side_.fsr_gait_data_.heel_.enable_adaptive_range  = new_state;
        exo.pe_.left_side_.fsr_gait_data_.toe_.enable_adaptive_range   = new_state;
        exo.pe_.right_side_.fsr_gait_data_.heel_.enable_adaptive_range = new_state;
        exo.pe_.right_side_.fsr_gait_data_.toe_.enable_adaptive_range  = new_state;
        /* 重置自适应跟踪器, 下次周期用当前标定值重新初始化 */
        exo.pe_.left_side_.fsr_gait_data_.heel_.adaptive_inited  = false;
        exo.pe_.left_side_.fsr_gait_data_.toe_.adaptive_inited   = false;
        exo.pe_.right_side_.fsr_gait_data_.heel_.adaptive_inited = false;
        exo.pe_.right_side_.fsr_gait_data_.toe_.adaptive_inited  = false;
        auto& shell = *static_cast<ExoShell *>(ctx);
        if (new_state) shell.SendString("FSR adaptive range: ON\r\n");
        else           shell.SendString("FSR adaptive range: OFF\r\n");
    }, this);
}

void ExoShell::OnCmdSetLocoMode(int argc, char **argv)
{
    if (argc < 2 || argv == nullptr || argv[1] == nullptr)
    {
        SendString("Usage: setlm <auto|sit|sts|stand|stand2sit|walk|ru|rd|su|sd>\r\n");
        return;
    }

    const char *mode_str = GetString(argc, argv, 1, "");
    if (StringEquals(mode_str, "auto"))
    {
        exo_.pe_.intention_data_.override_usr_.enable_locomode_override = false;
        Printf("Mode override: auto, label=%s\r\n", LocoModeToString(exo_.pe_.intention_data_.label_mode_));
        return;
    }

    ExoData::LocoMode mode = ExoData::LocoMode::kWalking;
    if (!ParseLocoMode(mode_str, mode))
    {
        Printf("Unknown mode: %s\r\n", mode_str);
        SendString("Modes: sit, sts, stand, stand2sit, walk, ru, rd, su, sd\r\n");
        return;
    }

    exo_.pe_.intention_data_.label_mode_ = mode;
    exo_.pe_.intention_data_.override_usr_.enable_locomode_override = true;
    exo_.pe_.intention_data_.override_usr_.forced_locomode = mode;
    Printf("Mode override+label: %s\r\n", LocoModeToString(mode));
}

bool ExoShell::StringEquals(const char *lhs, const char *rhs)
{
    return lhs != nullptr && rhs != nullptr && std::strcmp(lhs, rhs) == 0;
}

const char *ExoShell::LocoModeToString(ExoData::LocoMode mode)
{
    switch (mode)
    {
    case ExoData::LocoMode::kSitting:      return "sitting";
    case ExoData::LocoMode::kSitToStand:   return "sit2stand";
    case ExoData::LocoMode::kStanding:     return "standing";
    case ExoData::LocoMode::kStandToSit:   return "stand2sit";
    case ExoData::LocoMode::kWalking:      return "walking";
    case ExoData::LocoMode::kRampAscent:   return "rampup";
    case ExoData::LocoMode::kRampDescent:  return "rampdown";
    case ExoData::LocoMode::kStairAscent:  return "stairup";
    case ExoData::LocoMode::kStairDescent: return "stairdown";
    default:                               return "unknown";
    }
}

bool ExoShell::ParseLocoMode(const char *text, ExoData::LocoMode &mode)
{
    if (StringEquals(text, "sitting") || StringEquals(text, "sit"))
    {
        mode = ExoData::LocoMode::kSitting;
        return true;
    }
    if (StringEquals(text, "sit2stand") || StringEquals(text, "sit2s"))
    {
        mode = ExoData::LocoMode::kSitToStand;
        return true;
    }
    if (StringEquals(text, "standing") || StringEquals(text, "stand"))
    {
        mode = ExoData::LocoMode::kStanding;
        return true;
    }
    if (StringEquals(text, "stand2sit") || StringEquals(text, "stand2s"))
    {
        mode = ExoData::LocoMode::kStandToSit;
        return true;
    }
    if (StringEquals(text, "walking") || StringEquals(text, "walk") || StringEquals(text, "w"))
    {
        mode = ExoData::LocoMode::kWalking;
        return true;
    }
    if (StringEquals(text, "rampascent") || StringEquals(text, "ra"))
    {
        mode = ExoData::LocoMode::kRampAscent;
        return true;
    }
    if (StringEquals(text, "rampdescent") || StringEquals(text, "rd"))
    {
        mode = ExoData::LocoMode::kRampDescent;
        return true;
    }
    if (StringEquals(text, "stairascent") || StringEquals(text, "sa"))
    {
        mode = ExoData::LocoMode::kStairAscent;
        return true;
    }
    if (StringEquals(text, "stairdescent") || StringEquals(text, "sd"))
    {
        mode = ExoData::LocoMode::kStairDescent;
        return true;
    }
    return false;
}

void ExoShell::OnCmdSetSlope(int argc, char **argv)
{
    if (argc < 2 || argv == nullptr || argv[1] == nullptr)
    {
        SendString("Usage: setslope <deg>\r\n");
        return;
    }

    float slope_deg = GetFloat(argc, argv, 1, 0.0f);
    exo_.pe_.intention_data_.label_slope_deg_ = slope_deg;
    Printf("Slope label: %.3f deg\r\n", slope_deg);
}

void BodyImu::Read()
{
    if (!body_imu_.is_used_) return;

    BMI088_read(body_imu_.gyro_degps_, body_imu_.accel_mps2_, &body_imu_.chip_temp_c_);
    mahony_filter_.Update6Axis(body_imu_.gyro_degps_, body_imu_.accel_mps2_);
    mahony_filter_.GetQuaternion(body_imu_.q_);
    body_imu_.quaternion_valid_ = true;
    mahony_filter_.GetEulerAnglesRad(body_imu_.roll_rad_, body_imu_.pitch_rad_, body_imu_.yaw_rad_);
    mahony_filter_.GetEulerAnglesDeg(body_imu_.roll_deg_, body_imu_.pitch_deg_, body_imu_.yaw_deg_);
}


void DaMiaoImuHub::CanRxCallback(uint32_t can_id, const uint8_t *data)
{
    switch (can_id)
    {
    case MstID::kLeftShankMst:
        UpdateImuData(pe_.left_side_.shank_imu_, data);
        break;
    case MstID::kRightShankMst:
        UpdateImuData(pe_.right_side_.shank_imu_, data);
        break;
    case MstID::kLeftThighMst:
        UpdateImuData(pe_.left_side_.thigh_imu_, data);
        break;
    case MstID::kRightThighMst:
        UpdateImuData(pe_.right_side_.thigh_imu_, data);
        break;
    default:
        break;
    }
}

void DaMiaoImuHub::UpdateImuData(ImuData &imu_data, const uint8_t *data)
{
    uint16_t temp[3] = {0};
    int w, x, y, z;
    switch (data[0])
    {
    case 0x01:
        temp[0] = data[3] << 8 | data[2];
        temp[1] = data[5] << 8 | data[4];
        temp[2] = data[7] << 8 | data[6];
        imu_data.accel_mps2_[0] = UintToFloat(temp[0], kAccelCanMin, kAccelCanMax, 16);
        imu_data.accel_mps2_[1] = UintToFloat(temp[1], kAccelCanMin, kAccelCanMax, 16);
        imu_data.accel_mps2_[2] = UintToFloat(temp[2], kAccelCanMin, kAccelCanMax, 16);
        break;

    case 0x02:
        temp[0] = data[3] << 8 | data[2];
        temp[1] = data[5] << 8 | data[4];
        temp[2] = data[7] << 8 | data[6];
        imu_data.gyro_degps_[0] = UintToFloat(temp[0], kGyroCanMin, kGyroCanMax, 16);
        imu_data.gyro_degps_[1] = UintToFloat(temp[1], kGyroCanMin, kGyroCanMax, 16);
        imu_data.gyro_degps_[2] = UintToFloat(temp[2], kGyroCanMin, kGyroCanMax, 16);
        break;

    case 0x03:
        temp[0] = data[3] << 8 | data[2];
        temp[1] = data[5] << 8 | data[4];
        temp[2] = data[7] << 8 | data[6];
        imu_data.roll_deg_ = UintToFloat(temp[2], kRollCanMin, kRollCanMax, 16);
        imu_data.pitch_deg_ = UintToFloat(temp[0], kPitchCanMin, kPitchCanMax, 16);
        imu_data.yaw_deg_ = UintToFloat(temp[1], kYawCanMin, kYawCanMax, 16);
        imu_data.roll_rad_ = imu_data.roll_deg_ * DEG_TO_RAD;
        imu_data.pitch_rad_ = imu_data.pitch_deg_ * DEG_TO_RAD;
        imu_data.yaw_rad_ = imu_data.yaw_deg_ * DEG_TO_RAD;
        EulerRad2Quaternion(imu_data.roll_rad_, imu_data.pitch_rad_, imu_data.yaw_rad_, imu_data.q_);
        imu_data.quaternion_valid_ = false;
        break;

    case 0x04:
        w = data[1] << 6 | ((data[2] & 0xF8) >> 2);
        x = (data[2] & 0x03) << 12 | (data[3] << 4) | ((data[4] & 0xF0) >> 4);
        y = (data[4] & 0x0F) << 10 | (data[5] << 2) | (data[6] & 0xC0) >> 6;
        z = (data[6] & 0x3F) << 8 | data[7];
        imu_data.q_[0] = UintToFloat(w, kQuaternionMin, kQuaternionMax, 14);
        imu_data.q_[1] = UintToFloat(x, kQuaternionMin, kQuaternionMax, 14);
        imu_data.q_[2] = UintToFloat(y, kQuaternionMin, kQuaternionMax, 14);
        imu_data.q_[3] = UintToFloat(z, kQuaternionMin, kQuaternionMax, 14);
        imu_data.quaternion_valid_ = true;
        Quaternion2EulerRad(imu_data.q_, &imu_data.roll_rad_, &imu_data.pitch_rad_, &imu_data.yaw_rad_);
        imu_data.roll_deg_ = imu_data.roll_rad_ * RAD_TO_DEG;
        imu_data.pitch_deg_ = imu_data.pitch_rad_ * RAD_TO_DEG;
        imu_data.yaw_deg_ = imu_data.yaw_rad_ * RAD_TO_DEG;
        break;

    default:
        break;
    }
}

bool IntentionRecognizer::IsGaitMode(ExoData::LocoMode mode)
{
    return mode == ExoData::LocoMode::kWalking ||
           mode == ExoData::LocoMode::kRampAscent ||
           mode == ExoData::LocoMode::kRampDescent ||
           mode == ExoData::LocoMode::kStairAscent ||
           mode == ExoData::LocoMode::kStairDescent;
}

bool IntentionRecognizer::IsSvmGaitMode(ExoData::LocoMode mode)
{
    return IsGaitMode(mode);
}

bool IntentionRecognizer::IsSlopeEstimationMode(ExoData::LocoMode mode)
{
    return mode == ExoData::LocoMode::kStanding ||
           mode == ExoData::LocoMode::kWalking ||
           mode == ExoData::LocoMode::kRampAscent ||
           mode == ExoData::LocoMode::kRampDescent;
}

SvmPrediction IntentionRecognizer::SvmClassifyIntention(const SvmFeatureVector &fv)
{
    SvmPrediction prediction;

    static constexpr bool kEnableLinearSvm = false; /* 模型移植完成后改为 true */
    static constexpr uint8_t kNumClasses = 5;
    static constexpr uint8_t kNumFeatures = 6;
    static constexpr ExoData::LocoMode kClasses[kNumClasses] = {
        ExoData::LocoMode::kWalking,
        ExoData::LocoMode::kRampAscent,
        ExoData::LocoMode::kRampDescent,
        ExoData::LocoMode::kStairAscent,
        ExoData::LocoMode::kStairDescent,
    };

    /* TODO: 用离线训练得到的 mean/std、W、b 替换这些占位参数 */
    static constexpr float kFeatureMean[kNumFeatures] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    static constexpr float kFeatureInvStd[kNumFeatures] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    static constexpr float kWeights[kNumClasses][kNumFeatures] = {{0.0f}};
    static constexpr float kBias[kNumClasses] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    if (!kEnableLinearSvm)
    {
        return prediction; /* SVM 壳未启用: abstain, 不参与模式切换 */
    }

    float x[kNumFeatures] = {
        fv.max_thigh_pitch_rad,
        fv.max_knee_flexion_rad,
        fv.mean_thigh_omega_radps,
        fv.var_accel_z_mps2,
        (float)fv.zero_crossings,
        fv.final_calf_pitch_rad,
    };
    for (uint8_t i = 0; i < kNumFeatures; ++i)
    {
        x[i] = (x[i] - kFeatureMean[i]) * kFeatureInvStd[i];
    }

    uint8_t best_idx = 0;
    float best_score = -3.4e38f;
    float second_score = -3.4e38f;
    for (uint8_t cls = 0; cls < kNumClasses; ++cls)
    {
        float score = kBias[cls];
        for (uint8_t i = 0; i < kNumFeatures; ++i)
        {
            score += kWeights[cls][i] * x[i];
        }

        if (score > best_score)
        {
            second_score = best_score;
            best_score = score;
            best_idx = cls;
        }
        else if (score > second_score)
        {
            second_score = score;
        }
    }

    prediction.mode = kClasses[best_idx];
    prediction.confidence = best_score - second_score;
    prediction.is_valid = prediction.confidence >= kSvmConfidenceThreshold;
    return prediction;
}

void IntentionRecognizer::PushVote(const SvmPrediction &prediction)
{
    if (!prediction.is_valid ||
        !IsSvmGaitMode(prediction.mode) ||
        prediction.confidence < kSvmConfidenceThreshold)
    {
        return;
    }

    vote_buffer_[vote_buffer_idx_] = prediction.mode;
    vote_confidence_[vote_buffer_idx_] = prediction.confidence;
    vote_buffer_idx_ = (vote_buffer_idx_ + 1) % kVoteBufferSize;
    if (vote_count_ < kVoteBufferSize) vote_count_++;
}

SvmPrediction IntentionRecognizer::GetMajorityVote()
{
    SvmPrediction result;
    uint8_t counts[9] = {0};
    float confidence_sum[9] = {0.0f};
    uint8_t total = (vote_count_ < kVoteBufferSize) ? vote_count_ : kVoteBufferSize;

    for (uint8_t i = 0; i < total; i++)
    {
        uint8_t mode_val = static_cast<uint8_t>(vote_buffer_[i]);
        if (mode_val < 9 && IsSvmGaitMode(vote_buffer_[i]))
        {
            counts[mode_val]++;
            confidence_sum[mode_val] += vote_confidence_[i];
        }
    }

    uint8_t best_mode_val = static_cast<uint8_t>(pe_.intention_data_.detected_mode_);
    uint8_t best_count = 0;
    float best_confidence_sum = 0.0f;
    for (uint8_t i = 0; i < 9; i++)
    {
        if (counts[i] > best_count ||
            (counts[i] == best_count && confidence_sum[i] > best_confidence_sum))
        {
            best_count = counts[i];
            best_confidence_sum = confidence_sum[i];
            best_mode_val = i;
        }
    }

    if (best_count < 2)
    {
        return result; /* 3 票窗口中至少 2 票一致才接受 */
    }

    result.mode = static_cast<ExoData::LocoMode>(best_mode_val);
    result.confidence = best_confidence_sum / (float)best_count;
    result.is_valid = result.confidence >= kSvmConfidenceThreshold;
    return result;
}

bool IntentionRecognizer::IsLegalTransition(ExoData::LocoMode from, ExoData::LocoMode to) const
{
    if (from == to) return true;

    if (IsGaitMode(from))
    {
        return IsGaitMode(to) || to == ExoData::LocoMode::kStanding;
    }

    switch (from)
    {
    case ExoData::LocoMode::kSitting:
        return to == ExoData::LocoMode::kSitToStand;
    case ExoData::LocoMode::kSitToStand:
        return to == ExoData::LocoMode::kStanding || to == ExoData::LocoMode::kSitting;
    case ExoData::LocoMode::kStanding:
        return to == ExoData::LocoMode::kWalking || to == ExoData::LocoMode::kStandToSit;
    case ExoData::LocoMode::kStandToSit:
        return to == ExoData::LocoMode::kSitting || to == ExoData::LocoMode::kStanding;
    default:
        return false;
    }
}

bool IntentionRecognizer::TryAcceptMode(ExoData::LocoMode next_mode, uint32_t now_ms, bool force)
{
    ExoData::LocoMode current = pe_.intention_data_.detected_mode_;
    if (next_mode == current) return true;

    if (!IsLegalTransition(current, next_mode))
    {
        return false;
    }

    if (!force && last_mode_change_ms_ > 0 && (now_ms - last_mode_change_ms_) < kMinModeHoldMs)
    {
        return false;
    }

    pe_.intention_data_.detected_mode_ = next_mode;
    last_mode_change_ms_ = now_ms;
    vote_count_ = 0;
    vote_buffer_idx_ = 0;
    sit_to_stand_intent_detected_ = false;
    stand_to_sit_intent_detected_ = false;
    return true;
}

void IntentionRecognizer::UpdateSlopeEstimate(uint32_t now_ms, uint32_t delta_ms)
{
    if (delta_ms == 0u) delta_ms = 1u;

    FsrGaitData &left_fsr = pe_.left_side_.fsr_gait_data_;
    FsrGaitData &right_fsr = pe_.right_side_.fsr_gait_data_;
    ImuData &left_foot = pe_.left_side_.foot_imu_;
    ImuData &right_foot = pe_.right_side_.foot_imu_;

    auto reset_side_contact = [](SlopeSideState &state) {
        state.was_full_contact = false;
        state.contact_duration_ms = 0;
        state.sample_count = 0;
        state.foot_pitch_sum_deg = 0.0f;
    };

    auto decay_confidence = [&]() {
        float alpha = TimeAlpha(delta_ms, kSlopeConfidenceFallTauMs);
        pe_.intention_data_.terrain_slope_confidence_ += (0.0f - pe_.intention_data_.terrain_slope_confidence_) * alpha;
        pe_.intention_data_.terrain_slope_confidence_ = Clamp01Value(pe_.intention_data_.terrain_slope_confidence_);
        pe_.intention_data_.terrain_slope_valid_ = pe_.intention_data_.terrain_slope_confidence_ >= kSlopeConfidenceValidThresh;
    };

    if (!IsSlopeEstimationMode(pe_.intention_data_.detected_mode_) ||
        (!pe_.intention_data_.left_foot_pitch_ref_valid_ && !pe_.intention_data_.right_foot_pitch_ref_valid_))
    {
        left_slope_state_ = SlopeSideState{};
        right_slope_state_ = SlopeSideState{};
        pe_.intention_data_.left_terrain_slope_valid_ = false;
        pe_.intention_data_.right_terrain_slope_valid_ = false;
        decay_confidence();
        return;
    }

    auto update_side = [&](SlopeSideState &state,
                           FsrGaitData &fsr,
                           ImuData &foot,
                           bool is_left,
                           bool ref_valid,
                           float pitch_ref_deg,
                           float fit_a,
                           float fit_b) -> bool {
        bool fsr_ok = fsr.is_calibrated_ && (fsr.is_data_fresh_ || fsr.IsPacketFresh(now_ms));
        bool full_contact = fsr_ok &&
                            ref_valid &&
                            IsImuUsable(foot) &&
                            fsr.heel_contact_state_ &&
                            fsr.toe_contact_state_;

        if (full_contact)
        {
            state.was_full_contact = true;
            state.contact_duration_ms += delta_ms;
            state.sample_count++;
            state.foot_pitch_sum_deg += foot.sagittal_angle_deg_;
            return false;
        }

        bool produced_sample = false;
        if (state.was_full_contact &&
            state.contact_duration_ms >= kSlopeMinFullContactMs &&
            state.sample_count > 0u &&
            fit_a > 0.001f)
        {
            float mean_pitch_deg = state.foot_pitch_sum_deg / (float)state.sample_count;
            float pitch_delta_aligned_deg = mean_pitch_deg - pitch_ref_deg;
            if (!is_left)
            {
                pitch_delta_aligned_deg = -pitch_delta_aligned_deg;
            }

            float step_slope_deg = kSlopeSign * ((pitch_delta_aligned_deg - fit_b) / fit_a);
            state.last_step_slope_deg = step_slope_deg;
            state.has_step_slope = true;
            state.last_sample_ms = now_ms;

            state.smooth_window_deg[state.smooth_window_idx] = step_slope_deg;
            state.smooth_window_idx = (uint8_t)((state.smooth_window_idx + 1u) % kSlopeSmoothWindowSteps);
            if (state.smooth_window_count < kSlopeSmoothWindowSteps)
            {
                state.smooth_window_count++;
            }
            produced_sample = true;
        }

        reset_side_contact(state);
        return produced_sample;
    };

    bool left_new_sample = update_side(left_slope_state_,
                                       left_fsr,
                                       left_foot,
                                       true,
                                       pe_.intention_data_.left_foot_pitch_ref_valid_,
                                       pe_.intention_data_.left_foot_pitch_level_ref_deg_,
                                       pe_.intention_data_.left_slope_fit_a_,
                                       pe_.intention_data_.left_slope_fit_b_);
    bool right_new_sample = update_side(right_slope_state_,
                                        right_fsr,
                                        right_foot,
                                        false,
                                        pe_.intention_data_.right_foot_pitch_ref_valid_,
                                        pe_.intention_data_.right_foot_pitch_level_ref_deg_,
                                        pe_.intention_data_.right_slope_fit_a_,
                                        pe_.intention_data_.right_slope_fit_b_);

    auto smooth_side = [](const SlopeSideState &state, float &out_deg) -> bool {
        if (!state.has_step_slope || state.smooth_window_count == 0u)
        {
            return false;
        }

        float sum = 0.0f;
        for (uint8_t i = 0; i < state.smooth_window_count; ++i)
        {
            sum += state.smooth_window_deg[i];
        }
        out_deg = sum / (float)state.smooth_window_count;
        return true;
    };

    float left_slope_deg = 0.0f;
    float right_slope_deg = 0.0f;
    bool left_valid = smooth_side(left_slope_state_, left_slope_deg);
    bool right_valid = smooth_side(right_slope_state_, right_slope_deg);

    pe_.intention_data_.left_terrain_slope_deg_ = left_slope_state_.last_step_slope_deg;
    pe_.intention_data_.right_terrain_slope_deg_ = right_slope_state_.last_step_slope_deg;
    pe_.intention_data_.left_terrain_slope_valid_ = left_valid;
    pe_.intention_data_.right_terrain_slope_valid_ = right_valid;

    const bool has_new_sample = left_new_sample || right_new_sample;
    if (!has_new_sample)
    {
        uint32_t last_left_ms = left_slope_state_.last_sample_ms;
        uint32_t last_right_ms = right_slope_state_.last_sample_ms;
        uint32_t last_sample_ms = (last_left_ms > last_right_ms) ? last_left_ms : last_right_ms;
        if (last_sample_ms == 0u || now_ms < last_sample_ms || (now_ms - last_sample_ms) > kSlopeSampleTimeoutMs)
        {
            decay_confidence();
        }
        return;
    }

    uint8_t slope_count = 0;
    float slope_sum_deg = 0.0f;
    if (left_valid) { slope_sum_deg += left_slope_deg; slope_count++; }
    if (right_valid) { slope_sum_deg += right_slope_deg; slope_count++; }

    if (slope_count == 0u)
    {
        decay_confidence();
        return;
    }

    if (left_valid && right_valid && fabsf(left_slope_deg - right_slope_deg) > kSlopeSideDisagreeDeg)
    {
        decay_confidence();
        return;
    }

    float slope_sample_deg = slope_sum_deg / (float)slope_count;
    pe_.intention_data_.terrain_slope_deg_ = slope_sample_deg;

    float confidence_target = (slope_count >= 2u) ? 1.0f : 0.75f;
    float confidence_alpha = TimeAlpha(delta_ms, kSlopeConfidenceRiseTauMs);
    pe_.intention_data_.terrain_slope_confidence_ += (confidence_target - pe_.intention_data_.terrain_slope_confidence_) * confidence_alpha;
    pe_.intention_data_.terrain_slope_confidence_ = Clamp01Value(pe_.intention_data_.terrain_slope_confidence_);
    pe_.intention_data_.terrain_slope_valid_ = pe_.intention_data_.terrain_slope_confidence_ >= kSlopeConfidenceValidThresh;
}

void IntentionRecognizer::UpdateRampModeBySlope(uint32_t now_ms)
{
    if (!pe_.intention_data_.terrain_slope_valid_) return;

    ExoData::LocoMode current = pe_.intention_data_.detected_mode_;
    if (current != ExoData::LocoMode::kWalking &&
        current != ExoData::LocoMode::kRampAscent &&
        current != ExoData::LocoMode::kRampDescent)
    {
        return;
    }

    float slope_deg = pe_.intention_data_.terrain_slope_deg_;
    ExoData::LocoMode next = current;
    if (current == ExoData::LocoMode::kWalking)
    {
        if (slope_deg > kSlopeEnterDeg)
        {
            next = ExoData::LocoMode::kRampAscent;
        }
        else if (slope_deg < -kSlopeEnterDeg)
        {
            next = ExoData::LocoMode::kRampDescent;
        }
    }
    else if (current == ExoData::LocoMode::kRampAscent)
    {
        if (slope_deg < -kSlopeEnterDeg)
        {
            next = ExoData::LocoMode::kRampDescent;
        }
        else if (fabsf(slope_deg) < kSlopeExitDeg)
        {
            next = ExoData::LocoMode::kWalking;
        }
    }
    else if (current == ExoData::LocoMode::kRampDescent)
    {
        if (slope_deg > kSlopeEnterDeg)
        {
            next = ExoData::LocoMode::kRampAscent;
        }
        else if (fabsf(slope_deg) < kSlopeExitDeg)
        {
            next = ExoData::LocoMode::kWalking;
        }
    }

    if (next != current)
    {
        TryAcceptMode(next, now_ms);
    }
}

void IntentionRecognizer::UpdateStaticRuleDetector(uint32_t now_ms, uint32_t delta_ms)
{
    FsrGaitData *left_fsr = &pe_.left_side_.fsr_gait_data_;
    FsrGaitData *right_fsr = &pe_.right_side_.fsr_gait_data_;
    ImuData *left_thigh = &pe_.left_side_.thigh_imu_;
    ImuData *left_shank = &pe_.left_side_.shank_imu_;
    ImuData *right_thigh = &pe_.right_side_.thigh_imu_;
    ImuData *right_shank = &pe_.right_side_.shank_imu_;

    bool is_double_support = (left_fsr->heel_contact_state_ || left_fsr->toe_contact_state_) &&
                             (right_fsr->heel_contact_state_ || right_fsr->toe_contact_state_);
    if (is_double_support)
    {
        double_support_timer_ms_ += delta_ms;
        float kinetic_energy = fabsf(left_thigh->sagittal_gyro_degps_) + fabsf(right_thigh->sagittal_gyro_degps_);
        if ((IsGaitMode(pe_.intention_data_.detected_mode_) || pe_.intention_data_.detected_mode_ == ExoData::LocoMode::kStanding) &&
            double_support_timer_ms_ > kDoubleSupportTimeoutMs &&
            kinetic_energy < kStillEnergyThreshold)
        {
            TryAcceptMode(ExoData::LocoMode::kStanding, now_ms);
        }
    }
    else
    {
        double_support_timer_ms_ = 0;
    }

    if (pe_.intention_data_.detected_mode_ == ExoData::LocoMode::kSitting)
    {
        if (!pe_.intention_data_.standing_posture_ref_valid_) return;

        bool lean_forward = left_thigh->sagittal_gyro_degps_ < kSitLeanVelThresh &&
                            right_thigh->sagittal_gyro_degps_ < kSitLeanVelThresh;
        if (lean_forward)
        {
            sit_to_stand_intent_detected_ = true;
        }

        bool both_feet_loaded = (left_fsr->heel_contact_state_ || left_fsr->toe_contact_state_) &&
                                (right_fsr->heel_contact_state_ || right_fsr->toe_contact_state_);
        if (sit_to_stand_intent_detected_ && both_feet_loaded)
        {
            TryAcceptMode(ExoData::LocoMode::kSitToStand, now_ms, true);
        }
    }
    else if (pe_.intention_data_.detected_mode_ == ExoData::LocoMode::kStanding)
    {
        bool sts_ref_ready = pe_.intention_data_.standing_posture_ref_valid_;
        bool both_feet_flat = (left_fsr->heel_contact_state_ && left_fsr->toe_contact_state_) &&
                              (right_fsr->heel_contact_state_ && right_fsr->toe_contact_state_);
        float l_knee_vel = pe_.left_side_.knee_joint_.sagittal_valid_ ?
            pe_.left_side_.knee_joint_.sagittal_vel_radps_ * RAD_TO_DEG :
            left_thigh->sagittal_gyro_degps_ - left_shank->sagittal_gyro_degps_;
        float r_knee_vel = pe_.right_side_.knee_joint_.sagittal_valid_ ?
            pe_.right_side_.knee_joint_.sagittal_vel_radps_ * RAD_TO_DEG :
            right_thigh->sagittal_gyro_degps_ - right_shank->sagittal_gyro_degps_;
        float l_knee_ang = GetStandingReferencedKneePitchDeg(pe_, pe_.left_side_);
        float r_knee_ang = GetStandingReferencedKneePitchDeg(pe_, pe_.right_side_);

        if (sts_ref_ready &&
            both_feet_flat &&
            l_knee_vel > kKneeBendVelThresh &&
            r_knee_vel > kKneeBendVelThresh)
        {
            stand_to_sit_intent_detected_ = true;
        }
        if (sts_ref_ready &&
            stand_to_sit_intent_detected_ &&
            (l_knee_ang > kKneeBendAngleThresh || r_knee_ang > kKneeBendAngleThresh))
        {
            TryAcceptMode(ExoData::LocoMode::kStandToSit, now_ms, true);
        }

        bool left_heel_off = (!left_fsr->heel_contact_state_ && left_fsr->toe_contact_state_);
        bool right_heel_off = (!right_fsr->heel_contact_state_ && right_fsr->toe_contact_state_);
        if ((left_heel_off || right_heel_off) && (left_fsr->event_ic_ || right_fsr->event_to_))
        {
            if (TryAcceptMode(ExoData::LocoMode::kWalking, now_ms, true))
            {
                double_support_timer_ms_ = 0;
            }
        }
    }
}

void IntentionRecognizer::UpdateGaitSvmRouter()
{
    if (!IsSvmGaitMode(pe_.intention_data_.detected_mode_)) return;

    auto update_side = [&](FsrGaitData &fsr,
                           ImuData &thigh,
                           ImuData &shank,
                           StreamingFeatureExtractor &extractor,
                           bool &svm_done_this_step) {
        if (fsr.event_to_)
        {
            extractor.Reset();
            svm_done_this_step = false;
        }

        bool sensors_ok = fsr.is_data_fresh_ &&
                          thigh.is_used_ && thigh.is_calibrated_ &&
                          shank.is_used_ && shank.is_calibrated_;
        bool in_swing = (!fsr.heel_contact_state_ && !fsr.toe_contact_state_);
        if (!sensors_ok || !in_swing)
        {
            return;
        }

        float knee_w_degps = thigh.sagittal_gyro_degps_ - shank.sagittal_gyro_degps_;
        extractor.Update(thigh.sagittal_angle_rad_,
                         shank.sagittal_angle_rad_,
                         thigh.sagittal_gyro_radps_,
                         knee_w_degps * DEG_TO_RAD,
                         thigh.accel_mps2_[2]);

        if (fsr.percent_swing_ > 75.0f && fsr.percent_swing_ < 95.0f && !svm_done_this_step)
        {
            SvmFeatureVector fv = extractor.GetFeatures(shank.sagittal_angle_rad_);
            PushVote(SvmClassifyIntention(fv));
            svm_done_this_step = true;
        }
    };

    update_side(pe_.left_side_.fsr_gait_data_,
                pe_.left_side_.thigh_imu_,
                pe_.left_side_.shank_imu_,
                left_feature_extractor_,
                left_svm_done_this_step_);
    update_side(pe_.right_side_.fsr_gait_data_,
                pe_.right_side_.thigh_imu_,
                pe_.right_side_.shank_imu_,
                right_feature_extractor_,
                right_svm_done_this_step_);
}

void IntentionRecognizer::UpdateTransitionCompletion(uint32_t now_ms)
{
    if (!pe_.intention_data_.standing_posture_ref_valid_) return;

    float thigh_pitch_sum = 0.0f;
    float knee_angle_sum = 0.0f;
    float knee_vel_sum = 0.0f;
    uint8_t valid_count = 0;
    auto accumulate_side = [&](const SideData &side) {
        const ImuData &thigh = side.thigh_imu_;
        const ImuData &shank = side.shank_imu_;
        if (!thigh.is_used_ || !shank.is_used_) return;
        thigh_pitch_sum += thigh.sagittal_angle_deg_;
        knee_angle_sum += GetStandingReferencedKneePitchDeg(pe_, side);
        knee_vel_sum += side.knee_joint_.sagittal_valid_ ?
            side.knee_joint_.sagittal_vel_radps_ * RAD_TO_DEG :
            thigh.sagittal_gyro_degps_ - shank.sagittal_gyro_degps_;
        valid_count++;
    };
    accumulate_side(pe_.left_side_);
    accumulate_side(pe_.right_side_);
    if (valid_count == 0) return;

    float avg_thigh_pitch = thigh_pitch_sum / (float)valid_count;
    float avg_knee_angle = knee_angle_sum / (float)valid_count;
    float avg_knee_vel = knee_vel_sum / (float)valid_count;
    if (pe_.intention_data_.detected_mode_ == ExoData::LocoMode::kSitToStand)
    {
        if (fabsf(avg_thigh_pitch - pe_.intention_data_.thigh_pitch_standing_ref_deg_) < kSitStandCompleteThreshDeg)
        {
            TryAcceptMode(ExoData::LocoMode::kStanding, now_ms, true);
        }
    }
    else if (pe_.intention_data_.detected_mode_ == ExoData::LocoMode::kStandToSit)
    {
        bool sit_complete = false;
        if (pe_.intention_data_.sitting_posture_ref_valid_)
        {
            sit_complete = fabsf(avg_thigh_pitch - pe_.intention_data_.thigh_pitch_sitting_ref_deg_) < kSitStandCompleteThreshDeg;
        }
        else
        {
            static constexpr float kFallbackSeatedKneeAngleDeg = 65.0f;
            static constexpr float kFallbackKneeStillVelDegps = 8.0f;
            sit_complete = avg_knee_angle > kFallbackSeatedKneeAngleDeg &&
                           fabsf(avg_knee_vel) < kFallbackKneeStillVelDegps;
        }

        if (sit_complete)
        {
            TryAcceptMode(ExoData::LocoMode::kSitting, now_ms, true);
        }
    }
}

void IntentionRecognizer::Update()
{
    uint32_t now_ms = GetSysTimeMs();
    uint32_t delta_ms = (last_update_ms_ == 0 || now_ms < last_update_ms_) ? 1u : (now_ms - last_update_ms_);
    if (delta_ms == 0u) delta_ms = 1u;
    last_update_ms_ = now_ms;

    UpdateSlopeEstimate(now_ms, delta_ms);
    UpdateRampModeBySlope(now_ms);
    UpdateStaticRuleDetector(now_ms, delta_ms);
    UpdateGaitSvmRouter();

    if ((pe_.left_side_.fsr_gait_data_.event_ic_ || pe_.right_side_.fsr_gait_data_.event_ic_) &&
        vote_count_ >= kVoteBufferSize)
    {
        SvmPrediction next = GetMajorityVote();
        if (next.is_valid)
        {
            TryAcceptMode(next.mode, now_ms);
        }
    }

    UpdateTransitionCompletion(now_ms);
}


/**
 * @brief 外骨骼系统初始化 —— 注册回调、启动通信、配置默认参数
 * @note  在 alt_main.cpp 中电机上电 1s 后调用
 */
void Exo::Initialize()
{
    /* 注册回调，以便 BSP 回调能转发到当前 Exo 实例 */
    BspCanRegisterRxCallback(this, ExoCanRxBridge);
    BspUsartRegisterRxCallback(this, ExoUartRxBridge);
    BspUsartRegisterErrorCallback(this, ExoUartErrorBridge);
    BspSpiRegisterErrorCallback(this, ExoSpiErrorBridge);
    BspGpioRegisterExtiCallback(this, ExoGpioExtiBridge);

    /* 启动串口接收 */
    // SensorUartReceiveDma();
    shell_.UartReceiveDma();

    /* 调试: 重置标定标志. */
    ResetCalibrationFlags();

    /* 调试: 选择助力的关节 */
    pe_.left_side_.hip_joint_.is_used_ = false;
    pe_.right_side_.hip_joint_.is_used_ = false;
    pe_.left_side_.knee_joint_.is_used_ = false;
    pe_.right_side_.knee_joint_.is_used_ = false;
    pe_.left_side_.ankle_joint_.is_used_ = false;
    pe_.right_side_.ankle_joint_.is_used_ = false;
    pe_.left_side_.knee_sea_joint_.is_used_ = false;
    pe_.right_side_.knee_sea_joint_.is_used_ = false;

    pe_.body_imu_.is_used_ = true;
    pe_.left_side_.shank_imu_.is_used_ = true;
    pe_.left_side_.thigh_imu_.is_used_ = true;
    pe_.right_side_.shank_imu_.is_used_ = true;
    pe_.right_side_.thigh_imu_.is_used_ = true;
    pe_.left_side_.foot_imu_.is_used_ = true;
    pe_.right_side_.foot_imu_.is_used_ = true;
    pe_.ao_data_.is_used_ = false;

    /* 调试: 髋关节参数. */

    /* 调试: 膝关节参数. */
    pe_.user_weight_kg_ = 60.0f;
    left_side_.knee_joint_.force_profile_generator_.damping_ = 0.0f;
    left_side_.knee_joint_.force_profile_generator_.stiffness_ = 0.0f;

    /* 调试: 踝关节参数 */
    left_side_.ankle_joint_.cable_released_position_ = 0.2f;
    left_side_.ankle_joint_.cable_pre_tensioned_position_ = 0.6f;
    left_side_.ankle_joint_.cable_tensioned_position_ = 1.4f;
    left_side_.ankle_joint_.assistance_start_phase_percent_ = 35.0f;
    left_side_.ankle_joint_.assistance_end_phase_percent_ = 65.0f;

    right_side_.ankle_joint_.cable_released_position_ = 0.2f;
    right_side_.ankle_joint_.cable_pre_tensioned_position_ = 0.4f;
    right_side_.ankle_joint_.cable_tensioned_position_ = 1.0f;
    right_side_.ankle_joint_.assistance_start_phase_percent_ = 35.0f;
    right_side_.ankle_joint_.assistance_end_phase_percent_ = 65.0f;
}

/**
 * @brief  外骨骼 1kHz 主控制循环
 *
 * 每周期执行顺序:
 *  1. Read()         — 电池电压、NRF54 足部传感器 (SPI/UART)、BMI088 躯干 IMU、各关节反馈
 *  2. 事件过滤        — 根据当前状态掩码非法事件 (AllowedEventsForState)
 *  3. E-Stop 检查     — 最高优先级, 直接关机电机关断并 return
 *  4. CheckSystemHealth() — 电池电压 + 电机故障检测, 仅在非 kSleep 状态执行
 *  5. 故障转换        — 欠压 → kFaultLowBattery, 其他故障 → kFaultSystem, sleep 命令 → kSleep
 *  6. switch(state)   — 顶层状态机 (详见 ExoData::State 文档)
 *  7. 后处理          — DJI ESC CAN 批发送 → Shell 命令处理 → VOFA 遥测 → LED 状态指示
 *
 * @note
 */
void Exo::Run()
{
#if 0
    pe_.intention_data_.current_mode_ = ExoData::LocoMode::kStairAscent;
    left_side_.stair_phase_estimator_.Update();
    right_side_.stair_phase_estimator_.Update();
    VofaSendTelemetry();
    return;

#elif 0
    /* 调试坡度估计 */
    pe_.intention_data_.override_usr_.enable_locomode_override = false;
    UpdateHumanJointKinematics();
    intention_recognizer_.Update();
    pe_.intention_data_.current_mode_ = pe_.intention_data_.detected_mode_;

    DmaUnionBuffer buf = {0};
    uint16_t idx = 0;
    buf.f_data[idx++] = pe_.intention_data_.label_is_valid_ ? 1.0f : 0.0f;
    buf.f_data[idx++] = pe_.intention_data_.label_slope_deg_;
    buf.f_data[idx++] = pe_.intention_data_.terrain_slope_deg_;
    buf.f_data[idx++] = pe_.intention_data_.terrain_slope_confidence_;
    buf.f_data[idx++] = pe_.intention_data_.terrain_slope_valid_ ? 1.0f : 0.0f;
    buf.f_data[idx++] = pe_.intention_data_.left_terrain_slope_deg_;
    buf.f_data[idx++] = pe_.intention_data_.left_terrain_slope_valid_ ? 1.0f : 0.0f;
    buf.f_data[idx++] = pe_.intention_data_.right_terrain_slope_deg_;
    buf.f_data[idx++] = pe_.intention_data_.right_terrain_slope_valid_ ? 1.0f : 0.0f;
    buf.f_data[idx++] = static_cast<float>(pe_.intention_data_.detected_mode_);
    buf.f_data[idx++] = pe_.left_side_.foot_imu_.sagittal_angle_deg_;
    buf.f_data[idx++] = pe_.right_side_.foot_imu_.sagittal_angle_deg_;
    buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.heel_contact_state_ ? 1.0f : 0.0f;
    buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.toe_contact_state_ ? 1.0f : 0.0f;
    buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.heel_contact_state_ ? 1.0f : 0.0f;
    buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.toe_contact_state_ ? 1.0f : 0.0f;
    VofaTransmitJustFloat(buf, idx);
    return;
#else
    uint32_t start_ticks = DWT_CYCCNT;

    /* 1. 读取/转换(有些传感器在中断回调中读取)传感器数据 */
    Read();

    /* 2. 根据系统当前状态过滤无效事件 */
    pe_.pending_events_ &= AllowedEventsForState(pe_.state_);

    /* 3. 处理用户发起的estop急停命令 */
    const bool is_estop_triggered = ((pe_.pending_events_ & ExoData::SysEvent::kEmergencyStop) != ExoData::SysEvent::kNone);
    if (is_estop_triggered)
    {
        ClearNonCriticalEvents(pe_);
        Shutdown();
        dji_esc_hub_.SendAllCanTxData();
        pe_.state_ = ExoData::State::kEstopped;
        state_led_.UpdateEmergencyStopColorBDMA();
        /* 没有命令可以撤销E-stop, 需要断电重启. */
        return;
    }

    /* 4. 检查是否出现欠压 或 其他故障, 计算error_code_ */
    CheckSystemHealth();

    const bool battery_low = ((pe_.error_code_ & ExoData::Error::kBatteryLow) != ExoData::Error::kNone);
    const bool has_any_fault = (pe_.error_code_ != ExoData::Error::kNone);
    if (battery_low) /* 电池欠压则将状态机转入kFaultLowBattery */
    {
        if (pe_.state_ != ExoData::State::kFaultLowBattery)
        {
            ClearNonCriticalEvents(pe_);
            pe_.state_ = ExoData::State::kFaultLowBattery;
        }
    }
    else if (has_any_fault) /* 出现故障则将状态机转入kFaultSystem */
    {
        if (pe_.state_ != ExoData::State::kFaultSystem)
        {
            ClearNonCriticalEvents(pe_);
            pe_.state_ = ExoData::State::kFaultSystem;
        }
    }
    else if ((pe_.pending_events_ & ExoData::SysEvent::kEnterSleep) != ExoData::SysEvent::kNone) /* 用户的休眠命令可令状态机直接转入kSleep */
    {
        pe_.pending_events_ &= ~ExoData::SysEvent::kEnterSleep;
        pe_.state_ = ExoData::State::kSleep;
    }

    /* 5. 运行外骨骼顶层状态机*/
    switch (pe_.state_)
    {
    case ExoData::State::kSleep:
        Shutdown();
        if (((pe_.pending_events_ & ExoData::SysEvent::kWakeup) != ExoData::SysEvent::kNone) && pe_.battery_voltage_ >= 19.5f)
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kWakeup;
            pe_.state_ = ExoData::State::kWaitMotorComm;
        }
        break;

    case ExoData::State::kWaitMotorComm: /* 接收到calib命令并且电机通信检查完毕则转入kCalibrating */
        if (IsMotorConnect() && ((pe_.pending_events_ & ExoData::SysEvent::kStartCalibrate) != ExoData::SysEvent::kNone))
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kStartCalibrate;
            ResetCalibrationFlags();
            pe_.state_ = ExoData::State::kCalibrating;
        }
        break;

    case ExoData::State::kCalibrating: /* 标定完毕则自动转入kReady */
        Calibrate();
        Standby(); /* 为了获取电机/关节状态, 以及保持通信 */
        if (IsCalibrateDone())
        {
            pe_.state_ = ExoData::State::kReady;
        }
        break;

    case ExoData::State::kReady: /* 用户发起start命令则转入kAssisting */
        Estimate();
        Standby(); /* 为了获取电机/关节状态, 以及保持通信 */
        if ((pe_.pending_events_ & ExoData::SysEvent::kStartCalibrate) != ExoData::SysEvent::kNone) /* 此时如果对标定结果不满意可发起calib命令重新标定 */
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kStartCalibrate;
            ResetCalibrationFlags();
            pe_.state_ = ExoData::State::kCalibrating;
        }
        else if ((pe_.pending_events_ & ExoData::SysEvent::kStartAssist) != ExoData::SysEvent::kNone)
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kStartAssist;
            pe_.state_ = ExoData::State::kAssisting;
        }
        break;

    case ExoData::State::kAssisting:
        Estimate(); /* 估计运动模式及该模式下的参数, 如walking及步态相位 */

        /* 如果用户发起了stop命令则回到kReady */
        if ((pe_.pending_events_ & ExoData::SysEvent::kStopAssist) != ExoData::SysEvent::kNone)
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kStopAssist;
            Standby();
            pe_.state_ = ExoData::State::kReady;
        }
        else
        {
            Assist();
        }
        break;

    case ExoData::State::kFaultLowBattery:
        Shutdown();
        /* 充电到大于19.5V则重转入kSleep */
        if (pe_.battery_voltage_ >= 19.5f)
        {
            pe_.state_ = ExoData::State::kSleep;
        }
        break;

    case ExoData::State::kFaultSystem:
        Shutdown();
        /* 最好不用ClearFaults命令, 有问题就关机排查 */
        if (((pe_.pending_events_ & ExoData::SysEvent::kClearFaults) != ExoData::SysEvent::kNone))
        {
            pe_.pending_events_ &= ~ExoData::SysEvent::kClearFaults;
            pe_.error_code_ = ExoData::Error::kNone;
            pe_.state_ = ExoData::State::kSleep;
        }
        break;

    default:
        break;
    }
    dji_esc_hub_.SendAllCanTxData(); /** important */


    /* 处理上位机蓝牙发下来的命令, 所有命令见ExoShell的构造函数 */  
    if (shell_.ProcessPendingCommand())
    {
        pe_.telemetry_config_.pause_until_ms = GetSysTimeMs() + 3000U;
    }

    /* 由于处理命令后需要反馈给上位机, 为了反馈不被数据掩盖, 延时一段时间再发数据 */  
    const uint32_t now_ms = GetSysTimeMs();
    if (pe_.telemetry_config_.enable && (now_ms >= pe_.telemetry_config_.pause_until_ms))
    {
        VofaSendTelemetry();
    }
    
    /* 指示系统状态机当前是什么状态 */
    state_led_.UpdateColorBDMA(static_cast<uint8_t>(pe_.state_));
    exo_run_time_us = DWTGetDeltaUs(start_ticks);
#endif
}

bool Exo::DebugInjectStairTelemetryFrame(uint32_t seq, const float *values, uint8_t count)
{
    (void)seq;
    if (values == nullptr || count < 49u)
    {
        return false;
    }
    
    InjectReplayStairSide(pe_.left_side_, values, 5u);
    InjectReplayStairSide(pe_.right_side_, values, 27u);
    return true;
}

bool Exo::DebugInjectSlopeTelemetryFrame(uint32_t seq, const float *values, uint8_t count)
{
    (void)seq;
    if (values == nullptr || count < 60u)
    {
        return false;
    }

    pe_.intention_data_.label_is_valid_ = values[2] > 0.5f;
    ExoData::LocoMode label_mode = ParseReplayLocoMode(values[3]);
    pe_.intention_data_.label_mode_ = label_mode;
    pe_.intention_data_.label_slope_deg_ = values[4];

    pe_.body_imu_.is_used_ = true;
    pe_.body_imu_.is_calibrated_ = true;
    pe_.body_imu_.roll_deg_ = values[5];
    pe_.body_imu_.pitch_deg_ = values[6];
    pe_.body_imu_.yaw_deg_ = values[7];
    pe_.body_imu_.gyro_degps_[0] = values[8];
    pe_.body_imu_.gyro_degps_[1] = values[9];
    pe_.body_imu_.gyro_degps_[2] = values[10];
    pe_.body_imu_.accel_mps2_[0] = values[11];
    pe_.body_imu_.accel_mps2_[1] = values[12];
    pe_.body_imu_.accel_mps2_[2] = values[13];
    pe_.body_imu_.quaternion_valid_ = false;

    uint32_t now_ms = GetSysTimeMs();
    InjectReplaySlopeSide(pe_.left_side_, values, 14u, now_ms);
    InjectReplaySlopeSide(pe_.right_side_, values, 37u, now_ms);
    UpdateReplaySlopeReferences(pe_, values, count);

    pe_.intention_data_.override_usr_.enable_locomode_override = false;
    pe_.intention_data_.detected_mode_ = IsReplaySlopeMode(label_mode) ? label_mode : ExoData::LocoMode::kWalking;
    pe_.intention_data_.current_mode_ = pe_.intention_data_.detected_mode_;
    pe_.intention_data_.prev_mode_ = pe_.intention_data_.detected_mode_;
    return true;
}

/**
 * @brief 标定双侧外骨骼 —— 关节零位 + FSR 踩踏力范围
 */
void Exo::Calibrate()
{
    uint32_t now_ms = GetSysTimeMs();
    if (!pe_.intention_data_.standing_posture_ref_valid_)
    {
        CapturePostureReference(pe_, false, now_ms);
    }
    CaptureFootPitchLevelReference(pe_, now_ms);

    left_side_.Calibrate();
    right_side_.Calibrate();
}

bool Exo::CalibrateSittingReference()
{
    return CapturePostureReference(pe_, true, GetSysTimeMs());
}

/**
 * @brief 重置所有标定标志位, 准备重新标定
 * @note  外骨骼顶层只重置各模块标定状态; FSR 传感器内部标定细节由 FsrGaitEstimator 负责。
 */
void Exo::ResetCalibrationFlags()
{
    /* 关节部分 */
    pe_.left_side_.is_calibrated_ = false;
    pe_.left_side_.hip_joint_.is_calibrated_ = false;
    pe_.left_side_.knee_joint_.is_calibrated_ = false;
    pe_.left_side_.ankle_joint_.is_calibrated_ = false;
    pe_.left_side_.knee_sea_joint_.is_calibrated_ = false;
    pe_.right_side_.is_calibrated_ = false;
    pe_.right_side_.hip_joint_.is_calibrated_ = false;
    pe_.right_side_.knee_joint_.is_calibrated_ = false;
    pe_.right_side_.ankle_joint_.is_calibrated_ = false;
    pe_.right_side_.knee_sea_joint_.is_calibrated_ = false;

    auto reset_joint_kinematics = [](JointData &joint) {
        joint.sagittal_pos_offset_rad_ = 0.0f;
        joint.sagittal_pos_rad_ = 0.0f;
        joint.sagittal_vel_radps_ = 0.0f;
        joint.sagittal_valid_ = false;
        joint.frontal_valid_ = false;
        joint.link_offset_rad_ = 0.0f;
        joint.link_rad_ = 0.0f;
        joint.link_vel_radps_ = 0.0f;
    };
    reset_joint_kinematics(pe_.left_side_.hip_joint_);
    reset_joint_kinematics(pe_.left_side_.knee_joint_);
    reset_joint_kinematics(pe_.left_side_.ankle_joint_);
    reset_joint_kinematics(pe_.left_side_.knee_sea_joint_);
    reset_joint_kinematics(pe_.right_side_.hip_joint_);
    reset_joint_kinematics(pe_.right_side_.knee_joint_);
    reset_joint_kinematics(pe_.right_side_.ankle_joint_);
    reset_joint_kinematics(pe_.right_side_.knee_sea_joint_);

    left_side_.fsr_gait_estimator_.ResetCalibration();
    right_side_.fsr_gait_estimator_.ResetCalibration();

    /* IMU读数暂不需要标定 */
    pe_.body_imu_.is_calibrated_ = true;
    pe_.left_side_.foot_imu_.is_calibrated_ = true;
    pe_.left_side_.shank_imu_.is_calibrated_ = true;
    pe_.left_side_.thigh_imu_.is_calibrated_ = true;
    pe_.right_side_.foot_imu_.is_calibrated_ = true;
    pe_.right_side_.shank_imu_.is_calibrated_ = true;
    pe_.right_side_.thigh_imu_.is_calibrated_ = true;

    auto reset_imu_sagittal_ref = [](ImuData &imu) {
        imu.sagittal_ref_rad_ = 0.0f;
        imu.sagittal_ref_deg_ = 0.0f;
        imu.sagittal_ref_valid_ = false;
    };
    reset_imu_sagittal_ref(pe_.body_imu_);
    reset_imu_sagittal_ref(pe_.left_side_.foot_imu_);
    reset_imu_sagittal_ref(pe_.left_side_.shank_imu_);
    reset_imu_sagittal_ref(pe_.left_side_.thigh_imu_);
    reset_imu_sagittal_ref(pe_.right_side_.foot_imu_);
    reset_imu_sagittal_ref(pe_.right_side_.shank_imu_);
    reset_imu_sagittal_ref(pe_.right_side_.thigh_imu_);

    pe_.intention_data_.left_foot_pitch_level_ref_deg_ = 0.0f;
    pe_.intention_data_.right_foot_pitch_level_ref_deg_ = 0.0f;
    pe_.intention_data_.left_foot_pitch_ref_valid_ = false;
    pe_.intention_data_.right_foot_pitch_ref_valid_ = false;
    pe_.intention_data_.thigh_pitch_standing_ref_deg_ = 0.0f;
    pe_.intention_data_.thigh_pitch_sitting_ref_deg_ = 0.0f;
    pe_.intention_data_.shank_pitch_standing_ref_deg_ = 0.0f;
    pe_.intention_data_.shank_pitch_sitting_ref_deg_ = 0.0f;
    pe_.intention_data_.left_thigh_pitch_standing_ref_deg_ = 0.0f;
    pe_.intention_data_.right_thigh_pitch_standing_ref_deg_ = 0.0f;
    pe_.intention_data_.left_shank_pitch_standing_ref_deg_ = 0.0f;
    pe_.intention_data_.right_shank_pitch_standing_ref_deg_ = 0.0f;
    pe_.intention_data_.left_thigh_pitch_sitting_ref_deg_ = 0.0f;
    pe_.intention_data_.right_thigh_pitch_sitting_ref_deg_ = 0.0f;
    pe_.intention_data_.left_shank_pitch_sitting_ref_deg_ = 0.0f;
    pe_.intention_data_.right_shank_pitch_sitting_ref_deg_ = 0.0f;
    pe_.intention_data_.standing_posture_ref_valid_ = false;
    pe_.intention_data_.sitting_posture_ref_valid_ = false;
    pe_.intention_data_.is_sts_ref_valid_ = false;
    pe_.intention_data_.terrain_slope_deg_ = 0.0f;
    pe_.intention_data_.terrain_slope_confidence_ = 0.0f;
    pe_.intention_data_.terrain_slope_valid_ = false;
    pe_.intention_data_.left_terrain_slope_deg_ = 0.0f;
    pe_.intention_data_.right_terrain_slope_deg_ = 0.0f;
    pe_.intention_data_.left_terrain_slope_valid_ = false;
    pe_.intention_data_.right_terrain_slope_valid_ = false;
}


/**
 * @brief 读取所有传感器数据
 * @note  电池电压由 ADC DMA 持续采集 (g_adc_data[0]),
 *        NRF54 足部传感器通过 SPI 双缓冲 DMA 或 UART DMA 接收,
 *        躯干 IMU (BMI088) 和四肢 IMU (达妙 CAN) 也在此更新
 */
void Exo::Read()
{
    /* 电池电压转换: ADC 16-bit, 3.3V 参考电压, 分压系数 11 (100k & 10k) */
    pe_.battery_voltage_ = (g_adc_data[0] * 3.3f / 65535) * 11;

    /* SPI 数据处理 */
    if (spi_data_ready_)
    {
        ProcessSpiData();
        spi_data_ready_ = false;
    }

    body_imu_.Read();
    left_side_.Read();
    right_side_.Read();
    UpdateHumanJointKinematics();
}

void Exo::UpdateHumanJointKinematics()
{
    UpdateSegmentSagittalAxes(pe_);

    auto update_side = [&](SideData &side) {
        float hip_raw = 0.0f;
        float hip_vel = 0.0f;
        float knee_raw = 0.0f;
        float knee_vel = 0.0f;
        float ankle_raw = 0.0f;
        float ankle_vel = 0.0f;
        ComputeRawJointSagittal(pe_, side, hip_raw, hip_vel, knee_raw, knee_vel, ankle_raw, ankle_vel);

        SetJointSagittalState(side.hip_joint_,
                              hip_raw,
                              hip_vel,
                              IsImuUsable(pe_.body_imu_) && IsImuUsable(side.thigh_imu_));
        SetJointSagittalState(side.knee_joint_,
                              knee_raw,
                              knee_vel,
                              IsImuUsable(side.thigh_imu_) && IsImuUsable(side.shank_imu_));
        SetJointSagittalState(side.ankle_joint_,
                              ankle_raw,
                              ankle_vel,
                              IsImuUsable(side.shank_imu_) && IsImuUsable(side.foot_imu_));
    };

    update_side(pe_.left_side_);
    update_side(pe_.right_side_);
}

/**
 * @brief 运动意图与步态参数估计 (中层控制)
 *
 * 两层架构:
 *  High level: IntentionRecognizer 算法始终运行 → pe_.intention_data_.detected_mode_
 *              pe_.intention_data_.current_mode_ = override ? forced : detected
 *  Mid level:  根据 pe_.intention_data_.current_mode_ 估计相位 (FSR 步态 / 坐立角度 / AO)
 *
 * @note  用户可通过 Shell setlocomode 启用手动覆盖, 覆盖时算法仍在后台运行
 */
void Exo::Estimate()
{
    /*----------------------------------------------------------------------
     * 步骤 1: 高层控制
     * 结果写入 intention_data_.detected_mode_
     *----------------------------------------------------------------------*/
    intention_recognizer_.Update();

    /* 最终有效模式: 用户覆盖优先, 否则使用算法结果 */
    pe_.intention_data_.current_mode_ = pe_.intention_data_.override_usr_.enable_locomode_override ? pe_.intention_data_.override_usr_.forced_locomode : pe_.intention_data_.detected_mode_;

    /*----------------------------------------------------------------------
     * 步骤 2: 检测是否转换了模式, 如果是则重置相关的步态估计器状态以适应新模式
     *----------------------------------------------------------------------*/

    /* 模式转换检测: 按中层估计链路分类 */
    auto is_fsr_gait_phase_mode = [](ExoData::LocoMode m) -> bool {
        return m == ExoData::LocoMode::kWalking
            || m == ExoData::LocoMode::kRampAscent
            || m == ExoData::LocoMode::kRampDescent;
    };
    auto is_stair_mode = [](ExoData::LocoMode m) -> bool {
        return m == ExoData::LocoMode::kStairAscent
            || m == ExoData::LocoMode::kStairDescent;
    };
    auto is_sts_mode = [](ExoData::LocoMode m) -> bool {
        return m == ExoData::LocoMode::kSitToStand
            || m == ExoData::LocoMode::kStandToSit;
    };

    bool mode_changed = (pe_.intention_data_.prev_mode_ != pe_.intention_data_.current_mode_);
    bool was_fsr_gait_phase = is_fsr_gait_phase_mode(pe_.intention_data_.prev_mode_);
    bool now_fsr_gait_phase = is_fsr_gait_phase_mode(pe_.intention_data_.current_mode_);
    bool was_stair = is_stair_mode(pe_.intention_data_.prev_mode_);
    bool now_stair = is_stair_mode(pe_.intention_data_.current_mode_);
    bool was_sts = is_sts_mode(pe_.intention_data_.prev_mode_);
    bool now_sts = is_sts_mode(pe_.intention_data_.current_mode_);
    bool was_fsr_contact_mode = was_fsr_gait_phase || was_stair;
    bool now_fsr_contact_mode = now_fsr_gait_phase || now_stair;

    /* FSR 步态估计器承担两类职责:
     * 1) walking/ramp: 完整 RLA 相位与 percent_gait_
     * 2) stair: 仅 FSR 接触预处理
     * 只有在跨估计链路切换时重置; walking/ramp 内部切换不重置相位。
     */
    if (mode_changed &&
        (was_fsr_contact_mode != now_fsr_contact_mode ||
         was_fsr_gait_phase != now_fsr_gait_phase ||
         was_stair != now_stair))
    {
        left_side_.fsr_gait_estimator_.Reset();
        right_side_.fsr_gait_estimator_.Reset();
    }

    /* AO 只在 walking/ramp 链路中更新, 进入/退出该链路时清理旧振荡器状态。 */
    if (mode_changed && (was_fsr_gait_phase != now_fsr_gait_phase))
    {
        ao_.Reset();
    }
    if (mode_changed && (was_stair || now_stair))
    {
        left_side_.stair_phase_estimator_.Reset();
        right_side_.stair_phase_estimator_.Reset();
    }
    if (mode_changed && (was_sts || now_sts))
    {
        sts_phase_estimator_.Reset();
    }

    pe_.intention_data_.prev_mode_ = pe_.intention_data_.current_mode_;

    /*----------------------------------------------------------------------
     * 步骤 3: 中层控制: 根据最终有效模式估计相位
     *----------------------------------------------------------------------*/
    uint32_t now_ms = GetSysTimeMs();
    switch (pe_.intention_data_.current_mode_)
    {
        /* 步态类: FSR 相位估计 + 自适应振荡器 */
    case ExoData::LocoMode::kWalking:
    case ExoData::LocoMode::kRampAscent:
    case ExoData::LocoMode::kRampDescent:
        left_side_.fsr_gait_estimator_.PrepareUpdate(now_ms);
        right_side_.fsr_gait_estimator_.PrepareUpdate(now_ms);
        left_side_.fsr_gait_estimator_.FinalizeUpdate(now_ms);
        right_side_.fsr_gait_estimator_.FinalizeUpdate(now_ms);
        left_side_.fsr_gait_estimator_.CommitUpdate();
        right_side_.fsr_gait_estimator_.CommitUpdate();
        ao_.Update();
        break;
    case ExoData::LocoMode::kStairAscent:
    case ExoData::LocoMode::kStairDescent:
        left_side_.fsr_gait_estimator_.PrepareUpdate(now_ms);
        right_side_.fsr_gait_estimator_.PrepareUpdate(now_ms);
        left_side_.stair_phase_estimator_.Update();
        right_side_.stair_phase_estimator_.Update();
        left_side_.fsr_gait_estimator_.CommitUpdate();
        right_side_.fsr_gait_estimator_.CommitUpdate();
        break;
    case ExoData::LocoMode::kSitToStand:
    case ExoData::LocoMode::kStandToSit:
        sts_phase_estimator_.Update();
        break;
    case ExoData::LocoMode::kSitting:
        break;
    case ExoData::LocoMode::kStanding:
        break;
    default:
        /* 静态模式: 无相位估计 */
        break;
    }
}

/**
 * @brief 待机控制 —— 各关节进入零力/零力矩模式, 保持电机通信
 * @note  在 kCalibrating 和 kReady 状态下每周期调用
 */
void Exo::Standby()
{
    left_side_.Standby();
    right_side_.Standby();
}

/**
 * @brief 助力控制 —— 各关节根据步态相位输出助力力矩/力
 * @note  仅在 kAssisting 状态下调用, 内部通过步态百分比驱动各关节的力曲线
 */
void Exo::Assist()
{
    left_side_.Assist();
    right_side_.Assist();
}

/**
 * @brief 紧急关断 —— 所有电机关断 (DisableMotor), 步态估计器重置
 * @note  在 kSleep / kFaultLowBattery / kFaultSystem / E-Stop 路径中调用
 */
void Exo::Shutdown()
{
    left_side_.Shutdown();
    right_side_.Shutdown();
    sts_phase_estimator_.Reset();
}

/**
 * @brief 检查系统健康状态 —— 电池电压 + 各关节电机故障
 * @note  更新 pe_.error_code_ 位掩码, 但不执行状态转换 (状态转换在 Run() 中)
 */
void Exo::CheckSystemHealth()
{
    pe_.error_code_ = ExoData::Error::kNone;

    /* 检查电池电压 */
    if (pe_.battery_voltage_ < 19.0f)
    {
        pe_.error_code_ |= ExoData::Error::kBatteryLow;
    }

    /* 检查电机故障 */
    if (pe_.left_side_.knee_joint_.is_used_ &&
        (left_side_.knee_joint_.motor_.error_code_ != 0 || left_side_.knee_joint_.motor_.fault_code_ != 0))
        pe_.error_code_ |= ExoData::Error::kLeftKneeFault;
    if (pe_.right_side_.knee_joint_.is_used_ &&
        (right_side_.knee_joint_.motor_.error_code_ != 0 || right_side_.knee_joint_.motor_.fault_code_ != 0))
        pe_.error_code_ |= ExoData::Error::kRightKneeFault;
    if (pe_.left_side_.ankle_joint_.is_used_ &&
        (left_side_.ankle_joint_.motor_.error_code_ != 0 || left_side_.ankle_joint_.motor_.fault_code_ != 0))
        pe_.error_code_ |= ExoData::Error::kLeftAnkleFault;
    if (pe_.right_side_.ankle_joint_.is_used_ &&
        (right_side_.ankle_joint_.motor_.error_code_ != 0 || right_side_.ankle_joint_.motor_.fault_code_ != 0))
        pe_.error_code_ |= ExoData::Error::kRightAnkleFault;
}


extern float chirp_output;

extern float semg_pa0_raw;
extern float semg_pa2_raw;
extern float semg_pa0_filtered;
extern float semg_pa2_filtered;
extern float semg_pa0_envelope;
extern float semg_pa2_envelope;


uint8_t BuildFsrEventMask(const FsrGaitData &fsr)
{
    uint8_t mask = 0;
    if (fsr.event_ic_)  mask |= 1u << kIC;
    if (fsr.event_oto_) mask |= 1u << kOTO;
    if (fsr.event_hr_)  mask |= 1u << kHR;
    if (fsr.event_oic_) mask |= 1u << kOIC;
    if (fsr.event_to_)  mask |= 1u << kTO;
    if (fsr.event_fa_)  mask |= 1u << kFA;
    if (fsr.event_tv_)  mask |= 1u << kTV;
    return mask;
}

/**
 * @brief 通过 USB CDC 发送 VOFA + JustFloat 格式遥测数据
 * @note  通过 kVofaDownsample 控制下采样率; kVofaDownsample = 1 时 1kHz 全量发送.
 *        通过 kVofaTelemetryMode 选择平地/坡道、楼梯完整调试或楼梯输入采集帧.
 */
void Exo::VofaSendTelemetry()
{
    static constexpr uint8_t kVofaDownsample = 2;
    static uint8_t downsample_cnt = 1;

    if (downsample_cnt++ < kVofaDownsample) return;
    downsample_cnt = 1;

    static uint32_t loop_cnt = 0;
    uint32_t now_ms = GetSysTimeMs();
    uint32_t frame_cnt = loop_cnt++;

#if 1 
    VofaSendSensorTelemetry(now_ms, frame_cnt);

#else
    /* replay时, 用shell通过蓝牙发送数据 */
    const SideData &left_side = pe_.left_side_;
    const FsrGaitData &left_fsr = left_side.fsr_gait_data_;
    const StairPhaseData &left_stair = left_side.stair_phase_data_;

    const SideData &right_side = pe_.right_side_;
    const FsrGaitData &right_fsr = right_side.fsr_gait_data_;
    const StairPhaseData &right_stair = right_side.stair_phase_data_;

    uint8_t idx = 0;
    shell_.SetVofaJustFloatData(idx++, left_fsr.heel_contact_state_ ? 1.0f : 0.0f); //0
    shell_.SetVofaJustFloatData(idx++, left_fsr.toe_contact_state_ ? 1.0f : 0.0f);
    shell_.SetVofaJustFloatData(idx++, left_stair.is_contact_ ? 1.0f : 0.0f);
    shell_.SetVofaJustFloatData(idx++, left_stair.knee_angle_deg_);
    shell_.SetVofaJustFloatData(idx++, left_stair.knee_vel_degps_);
    shell_.SetVofaJustFloatData(idx++, (float)left_stair.current_phase_);

    shell_.SetVofaJustFloatData(idx++, right_fsr.heel_contact_state_ ? 1.0f : 0.0f); //6
    shell_.SetVofaJustFloatData(idx++, right_fsr.toe_contact_state_ ? 1.0f : 0.0f);
    shell_.SetVofaJustFloatData(idx++, right_stair.is_contact_ ? 1.0f : 0.0f);
    shell_.SetVofaJustFloatData(idx++, right_stair.knee_angle_deg_);
    shell_.SetVofaJustFloatData(idx++, right_stair.knee_vel_degps_);
    shell_.SetVofaJustFloatData(idx++, (float)right_stair.current_phase_);

    shell_.SetVofaJustFloatData(idx++, pe_.intention_data_.label_is_valid_ ? 1.0f : 0.0f);
    shell_.SetVofaJustFloatData(idx++, (float)static_cast<uint8_t>(pe_.intention_data_.label_mode_));
    shell_.SetVofaJustFloatData(idx++, pe_.intention_data_.label_slope_deg_);

    shell_.SendVofaJustFloatFrame(idx);
#endif
}

/**
 * @brief 原始传感器 + 数据集标签 + 意图识别调试帧
 * @note  前 60 个 float 保持原始传感器采集格式:
 *        0..4 通用标签; 5..13 body IMU; 14..36 左侧; 37..59 右侧.
 *        60..68 为意图/坡度估计调试输出.
 */
void Exo::VofaSendSensorTelemetry(uint32_t now_ms, uint32_t loop_cnt)
{
    DmaUnionBuffer buf = {0};
    uint16_t idx = 0;

    buf.f_data[idx++] = (float)now_ms; //0
    buf.f_data[idx++] = (float)loop_cnt; //1

    buf.f_data[idx++] = pe_.intention_data_.label_is_valid_; //2
    buf.f_data[idx++] = static_cast<float>(pe_.intention_data_.label_mode_); //3
    buf.f_data[idx++] = pe_.intention_data_.label_slope_deg_; //4

    buf.f_data[idx++] = pe_.body_imu_.roll_deg_; //5
    buf.f_data[idx++] = pe_.body_imu_.pitch_deg_; //6
    buf.f_data[idx++] = pe_.body_imu_.yaw_deg_; //7
    buf.f_data[idx++] = pe_.body_imu_.gyro_degps_[0]; //8
    buf.f_data[idx++] = pe_.body_imu_.gyro_degps_[1]; //9
    buf.f_data[idx++] = pe_.body_imu_.gyro_degps_[2]; //10
    buf.f_data[idx++] = pe_.body_imu_.accel_mps2_[0]; //11
    buf.f_data[idx++] = pe_.body_imu_.accel_mps2_[1]; //12
    buf.f_data[idx++] = pe_.body_imu_.accel_mps2_[2]; //13

    const SideData *sides[2] = {&pe_.left_side_, &pe_.right_side_};
    for (uint8_t i = 0; i < 2; ++i)
    {
        const SideData &side = *sides[i];
        const FsrGaitData &fsr = side.fsr_gait_data_;
        const ImuData &foot_imu = side.foot_imu_;
        const ImuData &shank_imu = side.shank_imu_;
        const ImuData &thigh_imu = side.thigh_imu_;

        buf.f_data[idx++] = fsr.heel_.raw_reading; //14, 37
        buf.f_data[idx++] = fsr.toe_.raw_reading; //15, 38
        buf.f_data[idx++] = fsr.heel_.calibrated_reading; //16, 39
        buf.f_data[idx++] = fsr.toe_.calibrated_reading; //17, 40
        buf.f_data[idx++] = fsr.heel_contact_state_ ? 1.0f : 0.0f; //18, 41
        buf.f_data[idx++] = fsr.toe_contact_state_ ? 1.0f : 0.0f; //19, 42
        buf.f_data[idx++] = (float)BuildFsrEventMask(fsr); //20, 43
        buf.f_data[idx++] = fsr.is_valid_ ? 1.0f : 0.0f; //21, 44

        buf.f_data[idx++] = foot_imu.roll_deg_; //22, 45
        buf.f_data[idx++] = foot_imu.pitch_deg_; //23, 46
        buf.f_data[idx++] = foot_imu.yaw_deg_; //24, 47

        buf.f_data[idx++] = shank_imu.roll_deg_; //25, 48
        buf.f_data[idx++] = shank_imu.pitch_deg_; //26, 49
        buf.f_data[idx++] = shank_imu.yaw_deg_; //27, 50
        buf.f_data[idx++] = shank_imu.gyro_degps_[0]; //28, 51
        buf.f_data[idx++] = shank_imu.gyro_degps_[1]; //29, 52
        buf.f_data[idx++] = shank_imu.gyro_degps_[2]; //30, 53
        
        buf.f_data[idx++] = thigh_imu.roll_deg_; //31, 54
        buf.f_data[idx++] = thigh_imu.pitch_deg_; //32, 55
        buf.f_data[idx++] = thigh_imu.yaw_deg_; //33, 56
        buf.f_data[idx++] = thigh_imu.gyro_degps_[0]; //34, 57
        buf.f_data[idx++] = thigh_imu.gyro_degps_[1]; //35, 58
        buf.f_data[idx++] = thigh_imu.gyro_degps_[2]; //36, 59
    }
    /* 下面新增的我注释了, 后面实际调试的时候我自己用shell打印 */
    // buf.f_data[idx++] = static_cast<float>(pe_.intention_data_.detected_mode_); //60
    // buf.f_data[idx++] = static_cast<float>(pe_.intention_data_.current_mode_); //61
    // buf.f_data[idx++] = pe_.intention_data_.terrain_slope_deg_; //62
    // buf.f_data[idx++] = pe_.intention_data_.terrain_slope_confidence_; //63
    // buf.f_data[idx++] = pe_.intention_data_.terrain_slope_valid_ ? 1.0f : 0.0f; //64
    // buf.f_data[idx++] = pe_.intention_data_.left_terrain_slope_deg_; //65
    // buf.f_data[idx++] = pe_.intention_data_.left_terrain_slope_valid_ ? 1.0f : 0.0f; //66
    // buf.f_data[idx++] = pe_.intention_data_.right_terrain_slope_deg_; //67
    // buf.f_data[idx++] = pe_.intention_data_.right_terrain_slope_valid_ ? 1.0f : 0.0f; //68

    VofaTransmitJustFloat(buf, idx);
}

/**
 * @brief 上下楼梯相位估计调试帧
 * @note  共 52 个 float:
 *        0..4 通用; 5..26 左侧; 27..48 右侧; 49..51 数据集标签(valid/mode/slope).
 */
void Exo::VofaSendStairTelemetry(uint32_t now_ms, uint32_t loop_cnt)
{
    DmaUnionBuffer buf = {0};
    uint16_t idx = 0;

    buf.f_data[idx++] = (float)now_ms;//0,
    buf.f_data[idx++] = (float)loop_cnt;//1,
    buf.f_data[idx++] = (float)static_cast<uint8_t>(pe_.state_);//2,
    buf.f_data[idx++] = (float)static_cast<uint8_t>(pe_.intention_data_.current_mode_);//3,
    buf.f_data[idx++] = (float)static_cast<uint32_t>(pe_.error_code_);//4,

    const SideData *sides[2] = {&pe_.left_side_, &pe_.right_side_};
    for (uint8_t i = 0; i < 2; ++i)
    {
        const SideData &side = *sides[i];
        const FsrGaitData &fsr = side.fsr_gait_data_;
        const StairPhaseData &stair = side.stair_phase_data_;
        const ImuData &foot_imu = side.foot_imu_;
        const ImuData &thigh_imu = side.thigh_imu_;
        const ImuData &shank_imu = side.shank_imu_;

        uint8_t sensor_mask = 0;
        if (fsr.is_used_ && fsr.is_calibrated_ && fsr.is_data_fresh_) sensor_mask |= 1u << 0;
        if (foot_imu.is_used_ && foot_imu.is_calibrated_) sensor_mask |= 1u << 1;
        if (thigh_imu.is_used_ && thigh_imu.is_calibrated_) sensor_mask |= 1u << 2;
        if (shank_imu.is_used_ && shank_imu.is_calibrated_) sensor_mask |= 1u << 3;

        buf.f_data[idx++] = fsr.heel_.raw_reading;   //5, 27
        buf.f_data[idx++] = fsr.toe_.raw_reading;   //6, 28
        buf.f_data[idx++] = fsr.heel_.calibrated_reading; //7, 29
        buf.f_data[idx++] = fsr.toe_.calibrated_reading; //8, 30

        buf.f_data[idx++] = fsr.heel_contact_state_ ? 1.0f : 0.0f; //9, 31
        buf.f_data[idx++] = fsr.toe_contact_state_ ? 1.0f : 0.0f; //10, 32
        buf.f_data[idx++] = fsr.is_data_fresh_ ? 1.0f : 0.0f; //11, 33
        buf.f_data[idx++] = (float)sensor_mask; //12, 34

        buf.f_data[idx++] = (float)static_cast<uint8_t>(stair.current_phase_); //13, 35
        buf.f_data[idx++] = stair.phase_changed_ ? 1.0f : 0.0f; //14, 36
        buf.f_data[idx++] = stair.is_valid_ ? 1.0f : 0.0f; //15, 37
        buf.f_data[idx++] = stair.is_contact_ ? 1.0f : 0.0f; //16, 38
        buf.f_data[idx++] = stair.percent_subphase_ / 100.0f; //17, 39
        buf.f_data[idx++] = stair.knee_angle_deg_; //18, 40
        buf.f_data[idx++] = stair.knee_vel_degps_; //19, 41

        buf.f_data[idx++] = thigh_imu.roll_deg_; //20, 42
        buf.f_data[idx++] = shank_imu.roll_deg_; //21, 43
        buf.f_data[idx++] = thigh_imu.gyro_degps_[0]; //22, 44
        buf.f_data[idx++] = shank_imu.gyro_degps_[0]; //23, 45

        buf.f_data[idx++] = foot_imu.roll_deg_; //24, 46
        buf.f_data[idx++] = foot_imu.pitch_deg_; //25, 47
        buf.f_data[idx++] = foot_imu.yaw_deg_; //26, 48
    }

    buf.f_data[idx++] = pe_.intention_data_.label_is_valid_ ? 1.0f : 0.0f;
    buf.f_data[idx++] = (float)static_cast<uint8_t>(pe_.intention_data_.label_mode_);
    buf.f_data[idx++] = pe_.intention_data_.label_slope_deg_;

    VofaTransmitJustFloat(buf, idx);
}

bool Exo::IsMotorConnect()
{
    return left_side_.IsMotorConnect() && right_side_.IsMotorConnect();
}

bool Exo::IsCalibrateDone()
{
    bool left_foot_ref_ok = !pe_.left_side_.foot_imu_.is_used_ ||
                            pe_.intention_data_.left_foot_pitch_ref_valid_;
    bool right_foot_ref_ok = !pe_.right_side_.foot_imu_.is_used_ ||
                             pe_.intention_data_.right_foot_pitch_ref_valid_;
    return pe_.left_side_.is_calibrated_ &&
           pe_.right_side_.is_calibrated_ &&
           pe_.intention_data_.standing_posture_ref_valid_ &&
           left_foot_ref_ok &&
           right_foot_ref_ok;
}

bool Exo::IsStopWalking()
{
    return pe_.ao_data_.left_event_cnt_ <= 1 && pe_.ao_data_.right_event_cnt_ <= 1;
}

ExoData::SysEvent Exo::AllowedEventsForState(ExoData::State state)
{
    ExoData::SysEvent allowed_events = ExoData::SysEvent::kEmergencyStop;
    switch (state)
    {
    case ExoData::State::kSleep:
        allowed_events |= ExoData::SysEvent::kWakeup;
        break;

    case ExoData::State::kWaitMotorComm:
        allowed_events |= ExoData::SysEvent::kStartCalibrate;
        allowed_events |= ExoData::SysEvent::kEnterSleep;
        break;
    
    case ExoData::State::kCalibrating:
        allowed_events |= ExoData::SysEvent::kEnterSleep;
        break;
        
    case ExoData::State::kReady:
        allowed_events |= ExoData::SysEvent::kStartCalibrate;
        allowed_events |= ExoData::SysEvent::kStartAssist;
        allowed_events |= ExoData::SysEvent::kEnterSleep;
        break;

    case ExoData::State::kAssisting:
        allowed_events |= ExoData::SysEvent::kStopAssist;
        allowed_events |= ExoData::SysEvent::kEnterSleep;
        break;

    case ExoData::State::kFaultLowBattery:
        break;

    case ExoData::State::kFaultSystem:
        allowed_events |= ExoData::SysEvent::kClearFaults;
        break;

    default:
        break;
    }
    return allowed_events;
}

void Exo::SensorUartRxCallback(const uint8_t *data, uint16_t data_size)
{
}


/**
 * @brief CAN 总线接收回调分发
 * @note  根据 CAN 外设句柄分发到对应总线:
 *        motor_can → 髋/膝/踝电机 + DJI ESC
 *        dm_imu_can → 达妙四肢 IMU
 */
void Exo::CanRxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, const uint8_t *data)
{
    if (hfdcan == &hw_.motor_can)
    {
        left_side_.hip_joint_.motor_.CanRxCallBack(can_id, data);
        right_side_.hip_joint_.motor_.CanRxCallBack(can_id, data);
        left_side_.knee_joint_.motor_.CanRxCallBack(can_id, data);
        right_side_.knee_joint_.motor_.CanRxCallBack(can_id, data);
        left_side_.ankle_joint_.motor_.CanRxCallBack(can_id, data);
        right_side_.ankle_joint_.motor_.CanRxCallBack(can_id, data);
        dji_esc_hub_.CanRxCallBack(can_id, data); 
    }
    else if (hfdcan == &hw_.dm_imu_can)
    {
        dm_imu_hub_.CanRxCallback(can_id, data);
    }
}

void Exo::SensorUartReceiveDma(void)
{
    /* DMA接收双足无线传感数据, 波特率1000000 Bits/s */
    HAL_UARTEx_ReceiveToIdle_DMA(&hw_.sensor_uart, sensor_uart_rx_buffer, SENSOR_UART_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(hw_.sensor_uart.hdmarx, DMA_IT_HT);
}

void Exo::UartRxCallback(UART_HandleTypeDef *huart, uint16_t data_size)
{
    const uint8_t *data = huart->pRxBuffPtr;
    if (huart == &hw_.sensor_uart)
    {
        SensorUartRxCallback(data, data_size);
        SensorUartReceiveDma();
    }
    else if (huart == &hw_.shell_uart)
    {
        shell_.PushPendingCommand(data, data_size);
    }
    else
    {
        left_side_.knee_sea_joint_.mag_encoder_.UartRxCallback(huart, data, data_size);
        right_side_.knee_sea_joint_.mag_encoder_.UartRxCallback(huart, data, data_size);
    }
}

void Exo::UartErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &hw_.sensor_uart)
    {
        HAL_UART_AbortReceive(&hw_.sensor_uart);
        SensorUartReceiveDma();
    }
    else if (huart == &hw_.shell_uart)
    {
        HAL_UART_AbortReceive(&hw_.shell_uart);
        shell_.UartReceiveDma();
    }
}

/**
 * @brief SPI CS 下降沿中断 → 启动 DMA 接收
 * @note  先 Abort 上一次传输 (如果还在进行), 然后启动新一轮 DMA 接收.
 *        双缓冲索引 spi_dma_reading_idx_ 在 CS 上升沿回调中翻转.
 */
/* 启动SPI接收, 在cs下降沿中断时调用 */
void Exo::SpiRxStart(void)
{
    HAL_SPI_Abort(&hw_.sensor_spi);
    HAL_SPI_Receive_DMA(&hw_.sensor_spi, spi_rx_dma_buf[spi_dma_reading_idx_], SENSOR_SPI_RX_BUF_SIZE);
}

/**
 * @brief SPI CS 上升沿中断 → 完成 DMA 接收, 翻转双缓冲索引
 * @note  双缓冲机制: 一个 buffer 正在 DMA 写入 (reading_idx),
 *        另一个 buffer 正在被主循环处理 (handling_idx).
 *        CS 上升沿时: 记录本次传输字节数 → 切换 handling_idx → 翻转 reading_idx
 */
/* 完成SPI接收, 在cs上升沿中断时调用 */
void Exo::SpiRxCallback(void)
{
    uint8_t remaining_bytes = __HAL_DMA_GET_COUNTER(hw_.sensor_spi.hdmarx);
    spi_dma_readed_size_ = SENSOR_SPI_RX_BUF_SIZE - remaining_bytes;

    HAL_SPI_Abort(&hw_.sensor_spi);
    spi_dma_handling_idx_ = spi_dma_reading_idx_;
    spi_dma_reading_idx_ = (spi_dma_reading_idx_ + 1) % 2;
    spi_data_ready_ = true;
}

/* SPI错误回调函数 */
void Exo::SpiErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi != &hw_.sensor_spi) return;
    HAL_SPI_Abort(hspi);
}


void Exo::ProcessSpiData()
{
    const uint8_t *data = spi_rx_dma_buf[spi_dma_handling_idx_];

    if (spi_dma_readed_size_ == sizeof(exo_sensor_packet_t))
    {
        exo_sensor_packet_t *packet = (exo_sensor_packet_t *)data;
        pe_.left_side_.fsr_gait_data_.heel_.raw_reading = 3.4f - packet->left_foot.mV_toe / 1000.0f;
        pe_.left_side_.fsr_gait_data_.toe_.raw_reading = 3.4f - packet->left_foot.mV_heel / 1000.0f;
        float k_left = - 65.04673f;
        float b_left = - 0.58726f;
        pe_.left_side_.ankle_joint_.plantarflexion_force_N_ = k_left * packet->left_foot.force + b_left;
        pe_.left_side_.foot_imu_.q_[1] = packet->left_foot.quatI;
        pe_.left_side_.foot_imu_.q_[2] = packet->left_foot.quatJ;
        pe_.left_side_.foot_imu_.q_[3] = packet->left_foot.quatK;
        pe_.left_side_.foot_imu_.q_[0] = packet->left_foot.quatReal;
        pe_.left_side_.foot_imu_.quaternion_valid_ = true;
        Quaternion2EulerRad(pe_.left_side_.foot_imu_.q_, &pe_.left_side_.foot_imu_.roll_rad_, &pe_.left_side_.foot_imu_.pitch_rad_, &pe_.left_side_.foot_imu_.yaw_rad_);
        pe_.left_side_.foot_imu_.roll_deg_ = pe_.left_side_.foot_imu_.roll_rad_ * RAD_TO_DEG;
        pe_.left_side_.foot_imu_.pitch_deg_ = pe_.left_side_.foot_imu_.pitch_rad_ * RAD_TO_DEG;
        pe_.left_side_.foot_imu_.yaw_deg_ = pe_.left_side_.foot_imu_.yaw_rad_ * RAD_TO_DEG;

        pe_.right_side_.fsr_gait_data_.heel_.raw_reading = 3.4f - packet->right_foot.mV_toe / 1000.0f;
        pe_.right_side_.fsr_gait_data_.toe_.raw_reading = 3.4f - packet->right_foot.mV_heel / 1000.0f;
        float k_right = 68.64289f;
        float b_right = -2.3153f;
        pe_.right_side_.ankle_joint_.plantarflexion_force_N_ = k_right * packet->right_foot.force + b_right;
        pe_.right_side_.foot_imu_.q_[1] = packet->right_foot.quatI;
        pe_.right_side_.foot_imu_.q_[2] = packet->right_foot.quatJ;
        pe_.right_side_.foot_imu_.q_[3] = packet->right_foot.quatK;
        pe_.right_side_.foot_imu_.q_[0] = packet->right_foot.quatReal;
        pe_.right_side_.foot_imu_.quaternion_valid_ = true;
        Quaternion2EulerRad(pe_.right_side_.foot_imu_.q_, &pe_.right_side_.foot_imu_.roll_rad_, &pe_.right_side_.foot_imu_.pitch_rad_, &pe_.right_side_.foot_imu_.yaw_rad_);
        pe_.right_side_.foot_imu_.roll_deg_ = pe_.right_side_.foot_imu_.roll_rad_ * RAD_TO_DEG;
        pe_.right_side_.foot_imu_.pitch_deg_ = pe_.right_side_.foot_imu_.pitch_rad_ * RAD_TO_DEG;
        pe_.right_side_.foot_imu_.yaw_deg_ = pe_.right_side_.foot_imu_.yaw_rad_ * RAD_TO_DEG;

        /* 数据新鲜度时间戳 (FSR + 足部 IMU) */
        uint32_t t_now = GetSysTimeMs();
        pe_.left_side_.fsr_gait_data_.heel_.last_update_ms_  = t_now;
        pe_.left_side_.fsr_gait_data_.toe_.last_update_ms_   = t_now;
        pe_.right_side_.fsr_gait_data_.heel_.last_update_ms_ = t_now;
        pe_.right_side_.fsr_gait_data_.toe_.last_update_ms_  = t_now;
    }
}
