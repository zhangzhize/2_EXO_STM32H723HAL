#include "exo.hpp"
#include <algorithm>
#include <cstring>
extern "C"
{
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

extern uint32_t g_adc_data[3]; /* definition in alt_main.cpp */
static uint32_t exo_run_time_us = 0; /* 用于统计 Exo::Run() 的运行时间, 单位微秒 */

void VofaCdcSendJustFloat(DmaUnionBuffer &buf, uint16_t float_count)
{
  if (float_count >= DMA_UNION_BUF_SIZE_FLOATS) return;

  uint16_t count = 4u * float_count;
  buf.u8_data[count++] = 0x00;
  buf.u8_data[count++] = 0x00;
  buf.u8_data[count++] = 0x80;
  buf.u8_data[count++] = 0x7f;
  CDC_Transmit_HS(buf.u8_data, count);
}

extern "C"
{
static void ExoCanRxBridge(void *ctx, FDCAN_HandleTypeDef *hfdcan, uint32_t can_ext_id, const uint8_t *rx_data, uint32_t rx_dlc)
{
  static_cast<Exo *>(ctx)->CanRxCallback(hfdcan, can_ext_id, rx_data, rx_dlc);
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
#define SENSOR_SPI_RX_BUF_SIZE 256
#define SENSOR_UART_RX_BUF_SIZE 256
__attribute__((section(".dma_buf"), aligned(32))) uint8_t spi_rx_dma_buf[2][SENSOR_SPI_RX_BUF_SIZE];
__attribute__((section(".dma_buf"), aligned(32))) uint8_t sensor_uart_rx_buffer[SENSOR_UART_RX_BUF_SIZE];

void AnkleJoint::Calibrate()
{
  /* 连杆角度标定: 仅电机启用时执行 */
  if (pj_.is_actuator_enabled_ && !pj_.is_link_pos_offset_valid_)
  {
    pj_.link_pos_offset_rad_ = pj_.link_pos_rad_;
    pj_.is_link_pos_offset_valid_ = true;
  }

  /* 运动学偏置: 有 IMU 即可 */
  if (ps_.shank_imu_.IsUsable() && ps_.foot_imu_.IsUsable() && !pj_.is_sagittal_pos_offset_valid_)
  {
    pj_.sagittal_pos_offset_rad_ =
      WrapPi(ps_.foot_imu_.SagittalRawRad() - ps_.shank_imu_.SagittalRawRad());
    pj_.is_sagittal_pos_offset_valid_ = true;
  }
}
bool AnkleJoint::IsMotorConnect()
{
  if (!pj_.is_actuator_enabled_) return true;

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
  if (!pj_.is_actuator_enabled_) return;

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
  if (!pj_.is_actuator_enabled_) return;

  if (pj_.is_left_)
  {
    pj_.link_pos_rad_ = motor_.position_ - pj_.link_pos_offset_rad_;
    pj_.UpdateLinkVelocity(motor_.speed_);
    pj_.tor_output_Nm_ = motor_.torque_;
  }
  else
  {
    pj_.link_pos_rad_ = -motor_.position_ - pj_.link_pos_offset_rad_;
    pj_.UpdateLinkVelocity(-motor_.speed_);
    pj_.tor_output_Nm_ = -motor_.torque_;
  }
}

void AnkleJoint::Assist()
{
  if (!pj_.is_actuator_enabled_) return;

  /* 1. 获取 FSR 估计的步态相位百分比 (0.0f~100.0f) */
  const FsrGaitData &fsr = pj_.is_left_ ? pe_.left_side_.fsr_gait_data_ : pe_.right_side_.fsr_gait_data_;
  float phase_percent = fsr.is_phase_valid_ ? fsr.percent_gait_ : -1.0f;

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

KneeJoint::BiLegContex KneeJoint::bi_leg_ctx_;
KneeJoint::DivekarParams KneeJoint::divekar_params_;

void KneeJoint::ComputeAnkleGeometry()
{
  float s_th, c_th;
  float s_sh, c_sh;
  const float theta_th_deg = divekar_state_.theta_th_rad * RAD_TO_DEG;
  const float theta_sh_deg = divekar_state_.theta_sh_rad * RAD_TO_DEG;
  arm_sin_cos_f32(theta_th_deg, &s_th, &c_th);
  arm_sin_cos_f32(theta_sh_deg, &s_sh, &c_sh);

  const float knee_x_m = pe_.user_info_.thigh_length_m * s_th;
  const float knee_y_m = -pe_.user_info_.thigh_length_m * c_th;
  divekar_state_.ankle_x_m = knee_x_m + pe_.user_info_.shank_length_m * s_sh;
  divekar_state_.ankle_y_m = knee_y_m - pe_.user_info_.shank_length_m * c_sh;

  const float hip_to_ankle_x_m = -divekar_state_.ankle_x_m;
  const float hip_to_ankle_y_m = -divekar_state_.ankle_y_m;
  divekar_state_.theta_la_rad = atan2f(hip_to_ankle_x_m, hip_to_ankle_y_m);
}

void KneeJoint::UpdateDivekarBiLegContext(KneeJoint &left_knee, KneeJoint &right_knee, const ExoData &pe)
{
  uint64_t now_us = GetSysTimeUs();
  bi_leg_ctx_.dt_s = (bi_leg_ctx_.update_prev_us > 0u) ? (float)(now_us - bi_leg_ctx_.update_prev_us) * 1.0e-6f : 0.001f;
  bi_leg_ctx_.update_prev_us = now_us;

  left_knee.UpdateDivekarFeedbackKinematics();
  right_knee.UpdateDivekarFeedbackKinematics();
  left_knee.ComputeAnkleGeometry();
  right_knee.ComputeAnkleGeometry();
  bi_leg_ctx_.delta_ajc_y_l_minus_r_cm = (left_knee.divekar_state_.ankle_y_m - right_knee.divekar_state_.ankle_y_m) * 100.0f;
  const float dx_m = left_knee.divekar_state_.ankle_x_m - right_knee.divekar_state_.ankle_x_m;
  const float dy_m = left_knee.divekar_state_.ankle_y_m - right_knee.divekar_state_.ankle_y_m;
  bi_leg_ctx_.delta_ajc_dist_cm = sqrtf(dx_m * dx_m + dy_m * dy_m) * 100.0f;
  bi_leg_ctx_.gate_delta_ajc_dist = Sigmoid(bi_leg_ctx_.delta_ajc_dist_cm, divekar_params_.m_ajc_dist, divekar_params_.d_ajc_dist);

  bi_leg_ctx_.theta_k_abs_diff_rad = fabs(left_knee.divekar_state_.theta_k_rad - right_knee.divekar_state_.theta_k_rad);
  bi_leg_ctx_.gate_sts_knee_pos_sym = Sigmoid(bi_leg_ctx_.theta_k_abs_diff_rad, divekar_params_.m_sts_knee_pos_sym, divekar_params_.d_sts_knee_pos_sym);
  bi_leg_ctx_.gate_sts_left_knee_raise_vel = Sigmoid(left_knee.divekar_state_.theta_k_dot_radps, divekar_params_.m_sts_knee_raise_vel, divekar_params_.d_sts_knee_raise_vel);
  bi_leg_ctx_.gate_sts_right_knee_raise_vel = Sigmoid(right_knee.divekar_state_.theta_k_dot_radps, divekar_params_.m_sts_knee_raise_vel, divekar_params_.d_sts_knee_raise_vel);
  bi_leg_ctx_.gate_sts_bilateral_knee_raise_vel = sqrtf(bi_leg_ctx_.gate_sts_left_knee_raise_vel * bi_leg_ctx_.gate_sts_right_knee_raise_vel);
  bi_leg_ctx_.gate_sts_left_knee_lower_vel = Sigmoid(left_knee.divekar_state_.theta_k_dot_radps, divekar_params_.m_sts_knee_lower_vel, divekar_params_.d_sts_knee_lower_vel);
  bi_leg_ctx_.gate_sts_right_knee_lower_vel = Sigmoid(right_knee.divekar_state_.theta_k_dot_radps, divekar_params_.m_sts_knee_lower_vel, divekar_params_.d_sts_knee_lower_vel);
  bi_leg_ctx_.gate_sts_bilateral_knee_lower_vel = sqrtf(bi_leg_ctx_.gate_sts_left_knee_lower_vel * bi_leg_ctx_.gate_sts_right_knee_lower_vel);
  const float left_lower_vel_radps = _max(left_knee.divekar_state_.theta_k_dot_radps, 0.0f);
  const float right_lower_vel_radps = _max(right_knee.divekar_state_.theta_k_dot_radps, 0.0f);
  bi_leg_ctx_.bilateral_lower_vel_radps = sqrtf(left_lower_vel_radps * right_lower_vel_radps);
  const float theta_k_mean_rad =
    0.5f * (left_knee.divekar_state_.theta_k_rad + right_knee.divekar_state_.theta_k_rad);
  bi_leg_ctx_.gate_sts_deep_flex_decay =
    Sigmoid(theta_k_mean_rad,
            divekar_params_.m_sts_deep_flex_decay,
            divekar_params_.d_sts_deep_flex_decay);
  bi_leg_ctx_.gate_sts_left_thigh_flex = Sigmoid(left_knee.divekar_state_.theta_th_rad, divekar_params_.m_sts_thigh_flex, divekar_params_.d_sts_thigh_flex);
  bi_leg_ctx_.gate_sts_right_thigh_flex = Sigmoid(right_knee.divekar_state_.theta_th_rad, divekar_params_.m_sts_thigh_flex, divekar_params_.d_sts_thigh_flex);
  bi_leg_ctx_.gate_sts_bilateral_thigh_flex = _min(bi_leg_ctx_.gate_sts_left_thigh_flex, bi_leg_ctx_.gate_sts_right_thigh_flex);

  const float body_weight_N = _max(pe.user_info_.weight_kg_, 1.0f) * 10.0f;
  const FsrGaitData &left_fsr = pe.left_side_.fsr_gait_data_;
  const FsrGaitData &right_fsr = pe.right_side_.fsr_gait_data_;
  if (left_fsr.IsContactReady())
  {
    bi_leg_ctx_.left_grf_BW = _constrain(left_fsr.vgrf_N_ / body_weight_N, 0.0f, 2.5f);
    bi_leg_ctx_.left_heel_BW = _constrain(left_fsr.heel_.force_N / body_weight_N, 0.0f, 2.5f);
  }
  else
  {
    bi_leg_ctx_.left_grf_BW = 0.0f;
    bi_leg_ctx_.left_heel_BW = 0.0f;
  }

  if (right_fsr.IsContactReady())
  {
    bi_leg_ctx_.right_grf_BW = _constrain(right_fsr.vgrf_N_ / body_weight_N, 0.0f, 2.5f);
    bi_leg_ctx_.right_heel_BW = _constrain(right_fsr.heel_.force_N / body_weight_N, 0.0f, 2.5f);
  }
  else
  {
    bi_leg_ctx_.right_grf_BW = 0.0f;
    bi_leg_ctx_.right_heel_BW = 0.0f;
  }
  bi_leg_ctx_.gate_F_heel_left = Sigmoid(bi_leg_ctx_.left_heel_BW, divekar_params_.m_heel, divekar_params_.d_heel);
  bi_leg_ctx_.gate_F_heel_right = Sigmoid(bi_leg_ctx_.right_heel_BW, divekar_params_.m_heel, divekar_params_.d_heel);
  bi_leg_ctx_.gate_F_grf_inv_left = Sigmoid(bi_leg_ctx_.left_grf_BW, -divekar_params_.m_grf_unloaded, divekar_params_.d_grf_unloaded);
  bi_leg_ctx_.gate_F_grf_inv_right = Sigmoid(bi_leg_ctx_.right_grf_BW, -divekar_params_.m_grf_unloaded, divekar_params_.d_grf_unloaded);

  bi_leg_ctx_.gate_sts_left_foot_loaded = Sigmoid(bi_leg_ctx_.left_grf_BW, divekar_params_.m_sts_foot_loaded, divekar_params_.d_sts_foot_loaded);
  bi_leg_ctx_.gate_sts_right_foot_loaded = Sigmoid(bi_leg_ctx_.right_grf_BW, divekar_params_.m_sts_foot_loaded, divekar_params_.d_sts_foot_loaded);
  bi_leg_ctx_.gate_sts_both_foot_loaded = sqrtf(bi_leg_ctx_.gate_sts_left_foot_loaded * bi_leg_ctx_.gate_sts_right_foot_loaded);
}

void KneeJoint::LatchDivekarLeadingLeg(KneeJoint &left_knee, KneeJoint &right_knee, bool left_fs, bool right_fs)
{
  bi_leg_ctx_.left_fs_accepted = false;
  bi_leg_ctx_.right_fs_accepted = false;

  if (left_fs == right_fs)
  {
    return;
  }

  if (left_fs)
  {
    bi_leg_ctx_.left_fs_accepted = true;
    left_knee.divekar_state_.delta_ajc_y_fs_cm = bi_leg_ctx_.delta_ajc_y_l_minus_r_cm;
    left_knee.divekar_state_.gate_delta_ajc_y_a_fs = Sigmoid(left_knee.divekar_state_.delta_ajc_y_fs_cm, divekar_params_.m_ajc_y_a, divekar_params_.d_ajc_y_a);
    left_knee.divekar_state_.gate_delta_ajc_y_na_fs = Sigmoid(left_knee.divekar_state_.delta_ajc_y_fs_cm, divekar_params_.m_ajc_y_na, divekar_params_.d_ajc_y_na);

    bi_leg_ctx_.delta_ajc_dist_fs_cm = bi_leg_ctx_.delta_ajc_dist_cm;
    bi_leg_ctx_.gate_delta_ajc_dist_fs = bi_leg_ctx_.gate_delta_ajc_dist;

    bi_leg_ctx_.leading_leg = LeadingLeg::kLeft;
  }
  else
  {
    bi_leg_ctx_.right_fs_accepted = true;
    right_knee.divekar_state_.delta_ajc_y_fs_cm = -bi_leg_ctx_.delta_ajc_y_l_minus_r_cm;
    right_knee.divekar_state_.gate_delta_ajc_y_a_fs = Sigmoid(right_knee.divekar_state_.delta_ajc_y_fs_cm, divekar_params_.m_ajc_y_a, divekar_params_.d_ajc_y_a);
    right_knee.divekar_state_.gate_delta_ajc_y_na_fs = Sigmoid(right_knee.divekar_state_.delta_ajc_y_fs_cm, divekar_params_.m_ajc_y_na, divekar_params_.d_ajc_y_na);

    bi_leg_ctx_.delta_ajc_dist_fs_cm = bi_leg_ctx_.delta_ajc_dist_cm;
    bi_leg_ctx_.gate_delta_ajc_dist_fs = bi_leg_ctx_.gate_delta_ajc_dist;

    bi_leg_ctx_.leading_leg = LeadingLeg::kRight;
  }
}

void KneeJoint::DivekarReset()
{
  divekar_state_ = DivekarState{};
  divekar_output_ = DivekarOutput{};

  UpdateDivekarFeedbackKinematics();
  divekar_state_.theta_k_hs_rad = divekar_state_.theta_k_rad;
  divekar_state_.theta_k_dot_prev_radps = divekar_state_.theta_k_dot_radps;
  divekar_state_.tau_prev_Nm = 0.0f;
  divekar_state_.torque_history_valid = true;
}

void KneeJoint::ResetDivekarBiLegContext()
{
  bi_leg_ctx_ = BiLegContex{};
}

void KneeJoint::UpdateDivekarFeedbackKinematics()
{
  divekar_state_.theta_th_rad = ps_.thigh_imu_.SagittalFromStandRefRad();

  if (divekar_params_.use_thigh_imu_and_link_feedback)
  {
    divekar_state_.theta_k_rad = pj_.link_pos_rad_;
    divekar_state_.theta_k_dot_radps = pj_.link_vel_lpf_radps_;
    divekar_state_.theta_sh_rad = divekar_state_.theta_th_rad - divekar_state_.theta_k_rad;
  }
  else
  {
    divekar_state_.theta_k_rad = pj_.sagittal_pos_rad_;
    divekar_state_.theta_k_dot_radps = pj_.sagittal_vel_lpf_radps_;
    divekar_state_.theta_sh_rad = ps_.shank_imu_.SagittalFromStandRefRad();
  }

  divekar_state_.theta_trunk_rad = pe_.body_imu_.SagittalFromStandRefRad();
}

void KneeJoint::DivekarUpdate()
{
  bool own_foot_strike_event = ps_.is_left_ ? bi_leg_ctx_.left_fs_accepted : bi_leg_ctx_.right_fs_accepted;
  if (own_foot_strike_event)
  {
    divekar_state_.theta_k_hs_rad = divekar_state_.theta_k_rad;
    divekar_state_.theta_kd_max_rad = 0.0f;
  }
  const float theta_kd = divekar_state_.theta_k_rad - divekar_state_.theta_k_hs_rad;
  if (theta_kd > divekar_state_.theta_kd_max_rad)
  {
    divekar_state_.theta_kd_max_rad = theta_kd;
  }
  divekar_state_.theta_kd_rad = theta_kd;

  const bool is_left = ps_.is_left_;
  const float f_grf_ipsi_BW = is_left ? bi_leg_ctx_.left_grf_BW : bi_leg_ctx_.right_grf_BW;
  const float gate_F_heel_ipsi = is_left ? bi_leg_ctx_.gate_F_heel_left : bi_leg_ctx_.gate_F_heel_right;
  const float gate_F_heel_contra = is_left ? bi_leg_ctx_.gate_F_heel_right : bi_leg_ctx_.gate_F_heel_left;
  const float gate_F_grf_contra_inv = is_left ? bi_leg_ctx_.gate_F_grf_inv_right : bi_leg_ctx_.gate_F_grf_inv_left;
  const float gate_delta_ajc_dist_inv_fs = 1.0f - bi_leg_ctx_.gate_delta_ajc_dist_fs;

  /* Task sensitization variables */
  const bool is_leading = ps_.is_left_ ?
                            bi_leg_ctx_.leading_leg == LeadingLeg::kLeft :
                            bi_leg_ctx_.leading_leg == LeadingLeg::kRight;
  const float xi = is_leading ? 1.0f : 0.0f;

  const float dt_s = _constrain(bi_leg_ctx_.dt_s, 1.0e-5f, 0.1f);
  divekar_state_.theta_k_dot_sample_elapsed_s =
    _constrain(divekar_state_.theta_k_dot_sample_elapsed_s + dt_s, 1.0e-5f, 0.1f);
  uint32_t theta_k_dot_sample_id = 0u;
  float theta_k_ddot_lpf_alpha = divekar_params_.theta_k_ddot_lpf_alpha;

  if (divekar_params_.use_thigh_imu_and_link_feedback)
  {
    theta_k_dot_sample_id = motor_.status_feedback_cnt_;
  }
  else
  {
    /* A human-knee velocity sample is complete only after both IMUs advance. */
    const uint32_t thigh_sample_ms = ps_.thigh_imu_.last_update_ms_;
    const uint32_t shank_sample_ms = ps_.shank_imu_.last_update_ms_;
    if (thigh_sample_ms > 0u && shank_sample_ms > 0u)
    {
      theta_k_dot_sample_id = _min(thigh_sample_ms, shank_sample_ms);
    }

    theta_k_ddot_lpf_alpha = divekar_params_.theta_k_ddot_lpf_alpha_imu;
  }

  const bool has_new_theta_k_dot_sample =
    theta_k_dot_sample_id != divekar_state_.theta_k_dot_sample_id;
  if (has_new_theta_k_dot_sample)
  {
    divekar_state_.theta_k_dot_sample_id = theta_k_dot_sample_id;
  }

  const float theta_k_ddot_lpf_radps2 =
    GetThetaKddotLpf(divekar_state_.theta_k_dot_radps,
                     divekar_state_.theta_k_dot_sample_elapsed_s,
                     has_new_theta_k_dot_sample,
                     theta_k_ddot_lpf_alpha);
  if (has_new_theta_k_dot_sample)
  {
    divekar_state_.theta_k_dot_sample_elapsed_s = 0.0f;
  }

  divekar_state_.gate_theta_la = Sigmoid(divekar_state_.theta_la_rad, divekar_params_.m_theta_la, divekar_params_.d_theta_la);
  divekar_state_.gate_theta_kd_max = Sigmoid(divekar_state_.theta_kd_max_rad, divekar_params_.m_na, divekar_params_.d_na);
  divekar_state_.gate_theta_k_LL_nested = divekar_params_.x1 * Sigmoid(divekar_state_.theta_k_rad, divekar_params_.m_LL2, divekar_params_.d_LL) + divekar_params_.x2;
  divekar_state_.gate_theta_k_dot_LL = Sigmoid(divekar_state_.theta_k_dot_radps, divekar_params_.m_LL1, divekar_state_.gate_theta_k_LL_nested);

  /* ------------------------- Stance basis functions ------------------------- */
  /* ascent spring */
  divekar_output_.tau_a_Nm =
    divekar_params_.k_a *
    divekar_state_.theta_k_rad *
    Step(divekar_state_.theta_k_rad) *
    divekar_state_.gate_theta_la *
    xi;

  /* non-ascent spring damper */
  const float k_na_eff = divekar_params_.k_na * divekar_state_.gate_theta_kd_max;
  divekar_output_.tau_na_Nm =
    (k_na_eff * theta_kd + divekar_params_.c_na * divekar_state_.theta_k_dot_radps * Step(divekar_state_.theta_k_dot_radps)) *
    Step(theta_kd) *
    divekar_state_.gate_theta_la *
    xi;

  /* LL spring */
  divekar_output_.tau_LL_Nm =
    divekar_params_.k_LL *
    divekar_state_.theta_k_rad *
    divekar_state_.gate_theta_k_dot_LL *
    f_grf_ipsi_BW;

  /* partial gravity compensation in stance */
  divekar_output_.tau_grav_st_Nm =
    divekar_params_.g_st *
    sinf(divekar_state_.theta_th_rad) *
    f_grf_ipsi_BW *
    Step(divekar_state_.theta_th_rad);  /* TODO: CHECK */

  /* -------------------------- Task sensitization --------------------------- */
  float gate_F_heel_contra_inv = 1.0f - gate_F_heel_contra;
  float gate_delta_ajc_dist_plus_F_grf = _min(bi_leg_ctx_.gate_delta_ajc_dist_fs + gate_F_grf_contra_inv, 1.0f);
  divekar_state_.gate_theta_k_dot_sw = Sigmoid(divekar_state_.theta_k_dot_radps, divekar_params_.m_grav_sw, divekar_params_.d_grav_sw);
  divekar_state_.gate_F_grf_u = Sigmoid(f_grf_ipsi_BW, divekar_params_.m_grf_u, divekar_params_.d_grf_u);

  /* LL spring */
  divekar_output_.tau_LL_mod_Nm =
    divekar_output_.tau_LL_Nm *
    gate_F_heel_ipsi *
    gate_F_heel_contra *
    gate_delta_ajc_dist_inv_fs;

  /* ascent spring */
  divekar_output_.tau_a_mod_Nm =
    divekar_output_.tau_a_Nm *
    divekar_state_.gate_delta_ajc_y_a_fs *
    bi_leg_ctx_.gate_delta_ajc_dist_fs;

  /* non-ascent spring */
  divekar_output_.tau_na_mod_Nm =
    divekar_output_.tau_na_Nm *
    divekar_state_.gate_delta_ajc_y_na_fs *
    gate_F_heel_contra_inv *
    gate_delta_ajc_dist_plus_F_grf;

  /* partial gravity compensation in stance */
  divekar_output_.tau_grav_st_mod_Nm =
    divekar_output_.tau_grav_st_Nm *
    bi_leg_ctx_.gate_delta_ajc_dist_fs *
    gate_F_heel_contra_inv;

  /* -------------------------- final assistive stance torque  ------------------------- */
  divekar_output_.tau_st_Nm =
    divekar_output_.tau_a_mod_Nm +
    divekar_output_.tau_na_mod_Nm +
    divekar_output_.tau_grav_st_mod_Nm;

  /* -------------------------- Swing basis functions ------------------------- */
  /* gravity compensation in swing */
  divekar_output_.tau_grav_sw_Nm =
    divekar_params_.g_sw *
    sinf(divekar_state_.theta_sh_rad) *
    divekar_state_.gate_theta_k_dot_sw;

  /* inertial forces in swing */
  divekar_output_.tau_inertial_sw_Nm = 0.0f;
  if (divekar_state_.theta_sh_rad < 0.0f && theta_k_ddot_lpf_radps2 < 0.0f)
  {
    const float knee_ext_acc = _min(-theta_k_ddot_lpf_radps2, 30.0f);
    float inertial_gate = 1.0f - expf(-divekar_params_.x3 * knee_ext_acc);
    inertial_gate = _constrain(inertial_gate, 0.0f, 1.0f);
    divekar_output_.tau_inertial_sw_Nm = divekar_params_.a_sw * inertial_gate;
  }

  /* spring-damper in swing */
  const float exp_arg = _constrain(divekar_params_.x4 * (divekar_state_.theta_k_rad - divekar_params_.theta_k_eq), -8.0f, 8.0f);
  divekar_output_.tau_sd_sw_Nm =
    (-divekar_params_.k_sw * expf(exp_arg) +
     divekar_params_.c_sw * divekar_state_.theta_k_dot_radps * Step(-divekar_state_.theta_k_dot_radps)) *
    Step(divekar_params_.theta_k_eq - divekar_state_.theta_k_rad);

  /* final assistive swing torque */
  divekar_output_.tau_sw_Nm = divekar_output_.tau_grav_sw_Nm + divekar_output_.tau_inertial_sw_Nm + divekar_output_.tau_sd_sw_Nm;

  /* ZZZ: STS */
  divekar_state_.gate_sts_stand2sit =
    bi_leg_ctx_.gate_sts_bilateral_knee_lower_vel *
    bi_leg_ctx_.gate_sts_both_foot_loaded;
  divekar_state_.gate_sts_sit2stand =
    bi_leg_ctx_.gate_sts_bilateral_knee_raise_vel *
    bi_leg_ctx_.gate_sts_both_foot_loaded;
  divekar_output_.tau_sit2stand_Nm =
    divekar_state_.gate_sts_sit2stand *
    divekar_params_.k_sts *
    divekar_state_.theta_k_rad *
    Step(divekar_state_.theta_k_rad);
  divekar_output_.tau_stand2sit_Nm =
    divekar_state_.gate_sts_stand2sit *
    (divekar_params_.k_stand2sit *
       bi_leg_ctx_.gate_sts_deep_flex_decay *
       divekar_state_.theta_k_rad *
       Step(divekar_state_.theta_k_rad) +
     divekar_params_.c_sts *
       bi_leg_ctx_.bilateral_lower_vel_radps);
  divekar_output_.tau_sts_Nm = divekar_output_.tau_sit2stand_Nm + divekar_output_.tau_stand2sit_Nm;
  divekar_state_.gate_sts_ctx =
    (1.0f - bi_leg_ctx_.gate_delta_ajc_dist) *
    bi_leg_ctx_.gate_sts_knee_pos_sym *
    bi_leg_ctx_.gate_sts_bilateral_thigh_flex;
  divekar_output_.tau_sts_mod_Nm = divekar_state_.gate_sts_ctx * divekar_output_.tau_sts_Nm;

  /* ------------------------- Stance/swing/sts blending ------------------------- */
  const float tau_gait_Nm =
    divekar_state_.gate_F_grf_u * divekar_output_.tau_st_Nm +
    (1.0f - divekar_state_.gate_F_grf_u) * divekar_output_.tau_sw_Nm;

  divekar_output_.tau_divekar_Nm =
    (1.0f - divekar_state_.gate_sts_ctx) * tau_gait_Nm +
    divekar_output_.tau_sts_mod_Nm;
  divekar_output_.tau_divekar_lpf_Nm = GetTorqueLpf(divekar_output_.tau_divekar_Nm);

  UpdateDivekarMotionControlOutput();
}

void KneeJoint::UpdateDivekarMotionControlOutput()
{
  /* TODO: After the feedforward arbitration is finalized, update this
     equivalent MotionControl derivation to match the gait/STS blending. */
  const bool is_left = ps_.is_left_;
  const bool is_leading = is_left ?
                            bi_leg_ctx_.leading_leg == LeadingLeg::kLeft :
                            bi_leg_ctx_.leading_leg == LeadingLeg::kRight;
  const float xi = is_leading ? 1.0f : 0.0f;
  const float gate_F_heel_contra = is_left ?
                                     bi_leg_ctx_.gate_F_heel_right :
                                     bi_leg_ctx_.gate_F_heel_left;
  const float gate_F_grf_contra_inv = is_left ?
                                        bi_leg_ctx_.gate_F_grf_inv_right :
                                        bi_leg_ctx_.gate_F_grf_inv_left;
  const float gate_F_heel_contra_inv = 1.0f - gate_F_heel_contra;
  const float gate_delta_ajc_dist_plus_F_grf =
    _min(bi_leg_ctx_.gate_delta_ajc_dist_fs + gate_F_grf_contra_inv, 1.0f);

  const float stance_blend = divekar_state_.gate_F_grf_u;
  const float swing_blend = 1.0f - stance_blend;
  const float ascent_gate =
    Step(divekar_state_.theta_k_rad) *
    divekar_state_.gate_theta_la *
    xi *
    divekar_state_.gate_delta_ajc_y_a_fs *
    bi_leg_ctx_.gate_delta_ajc_dist_fs;
  const float non_ascent_gate =
    Step(divekar_state_.theta_kd_rad) *
    divekar_state_.gate_theta_la *
    xi *
    divekar_state_.gate_delta_ajc_y_na_fs *
    gate_F_heel_contra_inv *
    gate_delta_ajc_dist_plus_F_grf;
  const float sts_spring_gate =
    divekar_state_.gate_sts_sit2stand *
    divekar_state_.gate_sts_ctx *
    Step(divekar_state_.theta_k_rad);
  const float sts_damping_gate =
    divekar_state_.gate_sts_stand2sit *
    divekar_state_.gate_sts_ctx;

  const float k_na_eff =
    divekar_params_.k_na * divekar_state_.gate_theta_kd_max;
  const float stiffness_ascent = divekar_params_.k_a * ascent_gate;
  const float stiffness_non_ascent = k_na_eff * non_ascent_gate;
  const float stiffness_sts = divekar_params_.k_sts * sts_spring_gate;
  const float damping_non_ascent =
    divekar_params_.c_na *
    Step(divekar_state_.theta_k_dot_radps) *
    non_ascent_gate;
  const float damping_sts = divekar_params_.c_sts * sts_damping_gate;

  const float stance_stiffness =
    stiffness_ascent + stiffness_non_ascent + stiffness_sts;
  const float stance_damping = damping_non_ascent + damping_sts;
  const float stance_spring_torque_Nm =
    stiffness_ascent * (0.0f - divekar_state_.theta_k_rad) +
    stiffness_non_ascent *
      (divekar_state_.theta_k_hs_rad - divekar_state_.theta_k_rad) +
    stiffness_sts * (0.0f - divekar_state_.theta_k_rad);
  const float stance_damping_torque_Nm =
    damping_non_ascent * (0.0f - divekar_state_.theta_k_dot_radps) +
    damping_sts * (0.0f - divekar_state_.theta_k_dot_radps);

  const float swing_active =
    Step(divekar_params_.theta_k_eq - divekar_state_.theta_k_rad);
  const float swing_damping =
    divekar_params_.c_sw *
    Step(-divekar_state_.theta_k_dot_radps) *
    swing_active;
  const float swing_damping_torque_Nm =
    swing_damping * (0.0f - divekar_state_.theta_k_dot_radps);
  const float exp_arg =
    _constrain(divekar_params_.x4 *
                 (divekar_state_.theta_k_rad - divekar_params_.theta_k_eq),
               -8.0f,
               8.0f);
  const float swing_nonlinear_spring_Nm =
    -divekar_params_.k_sw * expf(exp_arg) * swing_active;

  divekar_output_.mc_stiffness_Nm_per_rad =
    stance_blend * stance_stiffness;
  divekar_output_.mc_damping_Nm_s_per_rad =
    stance_blend * stance_damping + swing_blend * swing_damping;

  const float spring_torque_Nm =
    stance_blend * stance_spring_torque_Nm;
  const float damping_torque_Nm =
    stance_blend * stance_damping_torque_Nm +
    swing_blend * swing_damping_torque_Nm;
  const float feedforward_divekar_Nm =
    stance_blend * divekar_output_.tau_grav_st_mod_Nm +
    swing_blend * (divekar_output_.tau_grav_sw_Nm +
                   divekar_output_.tau_inertial_sw_Nm +
                   swing_nonlinear_spring_Nm);

  divekar_output_.mc_position_ref_rad = pj_.link_pos_rad_;
  if (divekar_output_.mc_stiffness_Nm_per_rad > 1.0e-5f)
  {
    divekar_output_.mc_position_ref_rad +=
      spring_torque_Nm / divekar_output_.mc_stiffness_Nm_per_rad;
  }

  divekar_output_.mc_velocity_ref_radps = pj_.link_vel_radps_;
  if (divekar_output_.mc_damping_Nm_s_per_rad > 1.0e-5f)
  {
    divekar_output_.mc_velocity_ref_radps +=
      damping_torque_Nm / divekar_output_.mc_damping_Nm_s_per_rad;
  }

  /* Divekar-positive torque assists extension; joint flexion is positive. */
  divekar_output_.mc_torque_feedforward_Nm = -feedforward_divekar_Nm;
}

float KneeJoint::GetThetaKddotLpf(float theta_k_dot_radps, float dt_s, bool has_new_sample, float alpha)
{
  if (!has_new_sample)
  {
    return divekar_state_.theta_k_ddot_lpf_radps2;
  }

  dt_s = _constrain(dt_s, 1.0e-5f, 0.1f);
  const float raw_ddot = divekar_state_.theta_k_dot_history_valid ? ((theta_k_dot_radps - divekar_state_.theta_k_dot_prev_radps) / dt_s) : 0.0f;
  divekar_state_.theta_k_dot_prev_radps = theta_k_dot_radps;
  divekar_state_.theta_k_dot_history_valid = true;
  divekar_state_.theta_k_ddot_lpf_radps2 +=
    _constrain(alpha, 0.0f, 1.0f) *
    (raw_ddot - divekar_state_.theta_k_ddot_lpf_radps2);
  return divekar_state_.theta_k_ddot_lpf_radps2;
}

float KneeJoint::GetTorqueLpf(float tau_unfiltered_Nm)
{
  divekar_state_.tau_divekar_lpf_prev_Nm += divekar_params_.torque_lpf_alpha * (tau_unfiltered_Nm - divekar_state_.tau_divekar_lpf_prev_Nm);
  return divekar_state_.tau_divekar_lpf_prev_Nm;
}

float KneeJoint::ApplySafetyLimits(float tau_divekar_Nm, float dt_s)
{
  float tau = tau_divekar_Nm;

  if (!divekar_state_.torque_history_valid)
  {
    divekar_state_.tau_prev_Nm = _constrain(tau, divekar_params_.torque_min_Nm, divekar_params_.torque_max_Nm);
    divekar_state_.torque_history_valid = true;
    return divekar_state_.tau_prev_Nm;
  }

  /* Paper: limit the torque increase rate in the extension direction. */
  const float max_extension_increase = divekar_params_.extension_slew_rate_Nmps * dt_s;
  if (tau > divekar_state_.tau_prev_Nm + max_extension_increase)
  {
    tau = divekar_state_.tau_prev_Nm + max_extension_increase;
  }

  tau = _constrain(tau, divekar_params_.torque_min_Nm, divekar_params_.torque_max_Nm);
  divekar_state_.tau_prev_Nm = tau;
  return tau;
}

void KneeJoint::Calibrate()
{
  /* 连杆角度标定: 仅电机启用时执行 */
  if (pj_.is_actuator_enabled_ && !pj_.is_link_pos_offset_valid_)
  {
    pj_.link_pos_offset_rad_ = motor_.position_;
    pj_.is_link_pos_offset_valid_ = true;
  }

  /* 运动学偏置: 有 IMU 即可 */
  if (ps_.thigh_imu_.IsUsable() && ps_.shank_imu_.IsUsable() && !pj_.is_sagittal_pos_offset_valid_)
  {
    pj_.sagittal_pos_offset_rad_ =
      WrapPi(ps_.thigh_imu_.SagittalRawRad() - ps_.shank_imu_.SagittalRawRad());
    pj_.is_sagittal_pos_offset_valid_ = true;
  }
}

void KneeJoint::PrepareMotorConnectionCheck()
{
  if (!pj_.is_actuator_enabled_) return;

  motor_.StatusFeedbackAutoRequest(false);
  motor_.status_feedback_cnt_ = 0;
}

bool KneeJoint::IsMotorConnect()
{
  if (!pj_.is_actuator_enabled_) return true;

  motor_.EnableMotor();
  if (motor_.status_feedback_cnt_ > 10 &&
      motor_.pattern_ == RobstridePattern::kPatternMotor)
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
  if (!pj_.is_actuator_enabled_) return;

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
  if (!pj_.is_actuator_enabled_) return;

  const float sign = pj_.motor_to_joint_sign_;
  pj_.link_pos_rad_ = sign * (motor_.position_ - pj_.link_pos_offset_rad_);
  pj_.UpdateLinkVelocity(sign * motor_.speed_);
  pj_.tor_output_Nm_ = sign * motor_.torque_;
}

void KneeJoint::Assist()
{
  if (!pj_.is_actuator_enabled_) return;

  float force_profile = 0.0f;
  float force_normalized = 0.0f;

  motor_.torque_forward_ = 0.0f;
  motor_.position_ref_ = 0.0f;
  motor_.speed_ref_ = 0.0f;
  motor_.motion_mode_kp_ = 0.0f;
  motor_.motion_mode_kd_ = 0.0f;

  /* TODO */
  switch (pe_.intention_data_.current_mode_)
  {
  case LocoMode::kSitToStand:
  {
    force_profile = 0.03 * pj_.link_pos_rad_; /* 简单弹簧控制 */
    break;
  }
  case LocoMode::kStandToSit:
  {
    force_profile = 0.03 * pj_.link_vel_radps_; /* 简单阻尼控制 */
    break;
  }
  case LocoMode::kWalking:
  case LocoMode::kRampAscent:
  case LocoMode::kRampDescent:
    if (ps_.fsr_gait_data_.is_phase_valid_)
    {
      float phase_percent = ps_.fsr_gait_data_.percent_gait_;
      force_profile = force_profile_generator_.GetForceProfile(phase_percent, pj_.sagittal_pos_rad_, pj_.sagittal_vel_radps_);
      // force_profile = force_profile_generator_.GetForceProfile(phase_percent, pj_.link_pos_rad_, pj_.link_vel_radps_);
      if (pj_.is_left_)
      {
        force_profile = -force_profile;
      }
    }
    break;
  default:
    break;
  }

  /* 模式切换时力矩从 0 平滑爬升, 避免检测延迟导致的力矩突变 */
  if (pe_.intention_data_.current_mode_ != pe_.intention_data_.prev_mode_)
  {
    force_ramp_ms_ = 0;
  }
  float ramp = (force_ramp_ms_ >= kForceRampDurationMs) ? 1.0f
                                                        : (float)force_ramp_ms_ / (float)kForceRampDurationMs;

  /* 暂时使用开环控制, 并限幅 */
  force_normalized = force_profile * pe_.user_info_.weight_kg_ * ramp;
  pj_.tor_output_ref_Nm_ = _constrain(force_normalized, -100.0f, 100.0f);

  motor_.torque_forward_ = pj_.tor_output_ref_Nm_;
  motor_.MotionControl();
  // motor_.EnableMotor();

  // OpenLoopTorqueControl();

  /* 每周期累加爬升计时 (Assist 在 ~1kHz 主循环中调用, 每周期约 1ms) */
  if (force_ramp_ms_ < kForceRampDurationMs)
    force_ramp_ms_ += 1;
}

void KneeJoint::ApplyDivekarControl()
{
  if (!pj_.is_actuator_enabled_) return;

  float assistance_scale = _constrain(divekar_params_.assistance_scale, 0.0f, 1.0f);
  const float dt_s = _constrain(bi_leg_ctx_.dt_s, 1.0e-5f, 0.1f);

  if (!divekar_params_.use_motion_control_impedance_output)
  {
    divekar_output_.tau_divekar_lpf_limited_Nm =
      ApplySafetyLimits(divekar_output_.tau_divekar_lpf_Nm, dt_s);

    /* Knee flexion is positive; positive Divekar torque assists extension. */
    pj_.tor_output_ref_Nm_ = -assistance_scale * divekar_output_.tau_divekar_lpf_limited_Nm;
    motor_.torque_forward_ = pj_.motor_to_joint_sign_ * pj_.tor_output_ref_Nm_;
    motor_.position_ref_ = 0.0f;
    motor_.speed_ref_ = 0.0f;
    motor_.motion_mode_kp_ = 0.0f;
    motor_.motion_mode_kd_ = 0.0f;
    motor_.MotionControl();
    return;
  }

  static constexpr float kMotorPositionLimitRad = 12.5f;
  static constexpr float kMotorVelocityLimitRadps = 44.0f;
  static constexpr float kMotorKpMax = 500.0f;
  static constexpr float kMotorKdMax = 5.0f;
  static constexpr float kMotorTorqueLimitNm = 17.0f;

  const float sign = pj_.motor_to_joint_sign_;
  const float joint_kp = _constrain(assistance_scale * divekar_output_.mc_stiffness_Nm_per_rad,
                                    0.0f,
                                    kMotorKpMax);
  const float joint_kd = _constrain(assistance_scale * divekar_output_.mc_damping_Nm_s_per_rad,
                                    0.0f,
                                    kMotorKdMax);
  const float joint_torque_feedforward_target_Nm =
    _constrain(assistance_scale * divekar_output_.mc_torque_feedforward_Nm,
               -kMotorTorqueLimitNm,
               kMotorTorqueLimitNm);

  const float motor_position_ref =
    _constrain(pj_.link_pos_offset_rad_ + sign * divekar_output_.mc_position_ref_rad,
               -kMotorPositionLimitRad,
               kMotorPositionLimitRad);
  const float motor_velocity_ref =
    _constrain(sign * divekar_output_.mc_velocity_ref_radps,
               -kMotorVelocityLimitRadps,
               kMotorVelocityLimitRadps);

  const float applied_joint_position_ref_rad =
    sign * (motor_position_ref - pj_.link_pos_offset_rad_);
  const float applied_joint_velocity_ref_radps = sign * motor_velocity_ref;
  const float joint_impedance_torque_Nm =
    joint_kp * (applied_joint_position_ref_rad - pj_.link_pos_rad_) +
    joint_kd * (applied_joint_velocity_ref_radps - pj_.link_vel_radps_);
  const float joint_torque_target_Nm =
    joint_impedance_torque_Nm + joint_torque_feedforward_target_Nm;

  /* ApplySafetyLimits uses Divekar-positive extension torque, whereas the
     joint coordinate is flexion-positive. Only extension-torque rise is
     rate-limited; torque removal remains immediate. */
  const float divekar_torque_limited_Nm =
    ApplySafetyLimits(-joint_torque_target_Nm, dt_s);
  divekar_output_.tau_divekar_lpf_limited_Nm = divekar_torque_limited_Nm;
  const float joint_torque_limited_Nm = -divekar_torque_limited_Nm;
  const float joint_torque_feedforward_applied_Nm =
    _constrain(joint_torque_limited_Nm - joint_impedance_torque_Nm,
               -kMotorTorqueLimitNm,
               kMotorTorqueLimitNm);

  motor_.position_ref_ = motor_position_ref;
  motor_.speed_ref_ = motor_velocity_ref;
  motor_.motion_mode_kp_ = joint_kp;
  motor_.motion_mode_kd_ = joint_kd;
  motor_.torque_forward_ = sign * joint_torque_feedforward_applied_Nm;
  pj_.tor_output_ref_Nm_ =
    joint_impedance_torque_Nm + joint_torque_feedforward_applied_Nm;
  motor_.MotionControl();
}

void KneeJoint::ClosedLoopTorqueControl()
{
  if (!pj_.is_actuator_enabled_) return;

  if (ctrl_mode_ != CtrlMode::kClosedLoopTorque)
  {
    joint_tor_pid_.ResetError();
    ctrl_mode_ = CtrlMode::kClosedLoopTorque;
  }

  /* 运动控制, tref = kp * (theta_set - theta_fb) + kd * (vset - vfb) + tff */

  float tor_err_Nm = pj_.tor_output_ref_Nm_ - pj_.tor_output_Nm_;
  motor_.torque_forward_ = pj_.tor_output_ref_Nm_;
  motor_.position_ref_ = 0.0f;
  motor_.speed_ref_ = joint_tor_pid_.kp_ * tor_err_Nm;
  motor_.motion_mode_kp_ = 0.0f;
  motor_.motion_mode_kd_ = 1.0;
  motor_.MotionControl();
}

void KneeJoint::OpenLoopTorqueControl()
{
  if (!pj_.is_actuator_enabled_) return;

  if (ctrl_mode_ != CtrlMode::kOpenLoopTorque)
  {
    ctrl_mode_ = CtrlMode::kOpenLoopTorque;
  }

  motor_.torque_forward_ = pj_.tor_output_ref_Nm_;
  motor_.position_ref_ = 0.0f;
  motor_.speed_ref_ = 0.0f;
  motor_.motion_mode_kp_ = 0.0f;
  motor_.motion_mode_kd_ = 0.0f;
  // motor_.MotionControl();
  motor_.EnableMotor();
}

void FsrGaitEstimator::PrepareUpdate(uint32_t now_ms)
{
  ClearCycleEvents();

  if (!gait_data_.is_enabled_) return;

  if (!gait_data_.is_data_fresh_) return;

  if (gait_data_.last_update_ms_ == last_contact_sample_ms_) return;
  last_contact_sample_ms_ = gait_data_.last_update_ms_;

  const bool was_contact_inited = gait_data_.heel_.contact_inited && gait_data_.toe_.contact_inited;
  const bool allow_baseline_update = was_contact_inited &&
                                     !gait_data_.prev_foot_contact_state_;

  UpdateContactAdaptive(gait_data_.heel_, allow_baseline_update);
  UpdateContactAdaptive(gait_data_.toe_, allow_baseline_update);
  gait_data_.heel_contact_state_ = gait_data_.heel_.ground_contact;
  gait_data_.toe_contact_state_ = gait_data_.toe_.ground_contact;
  gait_data_.foot_contact_state_ = gait_data_.heel_contact_state_ || gait_data_.toe_contact_state_;

  if (!was_contact_inited)
  {
    gait_data_.current_phase_ = gait_data_.foot_contact_state_ ? GaitPhase::kStance : GaitPhase::kSwing;
    gait_data_.prev_phase_ = gait_data_.current_phase_;
    gait_data_.prev_foot_contact_state_ = gait_data_.foot_contact_state_;
    gait_data_.prev_toe_contact_state_ = gait_data_.toe_contact_state_;
    gait_data_.prev_heel_contact_state_ = gait_data_.heel_contact_state_;
    return;
  }

  const bool stance_end = (!gait_data_.foot_contact_state_) && gait_data_.prev_foot_contact_state_;
  if (stance_end)
  {
    FinalizeStepPeak(gait_data_.heel_);
    FinalizeStepPeak(gait_data_.toe_);
  }

  DetectOwnFsrEvents();
}

void FsrGaitEstimator::FinalizeUpdate(uint32_t now_ms)
{
  if (!gait_data_.is_enabled_ || !gait_data_.is_data_fresh_) return;

  ApplyEventGuards(now_ms);
  UpdateEventTimings(now_ms);
  ResolvePhase();
  UpdatePercentages(now_ms);
  UpdateValidity();
}

void FsrGaitEstimator::CommitUpdate()
{
  if (!gait_data_.is_enabled_) return;

  gait_data_.prev_foot_contact_state_ = gait_data_.foot_contact_state_;
  gait_data_.prev_toe_contact_state_ = gait_data_.toe_contact_state_;
  gait_data_.prev_heel_contact_state_ = gait_data_.heel_contact_state_;
}

void FsrGaitEstimator::ClearCycleEvents()
{
  gait_data_.event_fs_ = false;
  gait_data_.event_hs_ = false;
  gait_data_.event_ts_ = false;
  gait_data_.event_fo_ = false;
  gait_data_.event_ho_ = false;
  gait_data_.event_to_ = false;
  gait_data_.phase_changed_ = false;
}

void FsrGaitEstimator::DetectOwnFsrEvents()
{
  gait_data_.event_fs_ = (gait_data_.foot_contact_state_ && !gait_data_.prev_foot_contact_state_);
  gait_data_.event_fo_ = (!gait_data_.foot_contact_state_ && gait_data_.prev_foot_contact_state_);
  gait_data_.event_hs_ = (gait_data_.heel_contact_state_ && !gait_data_.prev_heel_contact_state_);
  gait_data_.event_ho_ = (!gait_data_.heel_contact_state_ && gait_data_.prev_heel_contact_state_);
  gait_data_.event_ts_ = (gait_data_.toe_contact_state_ && !gait_data_.prev_toe_contact_state_);
  gait_data_.event_to_ = (!gait_data_.toe_contact_state_ && gait_data_.prev_toe_contact_state_);
}

void FsrGaitEstimator::ApplyEventGuards(uint32_t now_ms)
{
  const bool can_enter_stance = (gait_data_.current_phase_ == GaitPhase::kSwing ||
                                 gait_data_.current_phase_ == GaitPhase::kUnknown);
  const bool can_enter_swing = (gait_data_.current_phase_ == GaitPhase::kStance ||
                                gait_data_.current_phase_ == GaitPhase::kUnknown);

  /* Contact hysteresis/debounce validates edges; duration limits only filter statistics. */
  gait_data_.event_fs_ = gait_data_.event_fs_ && can_enter_stance;
  gait_data_.event_fo_ = gait_data_.event_fo_ && can_enter_swing;

  auto gate_local_event = [&](bool &event_flag, uint8_t event_idx)
  {
    if (!event_flag) return;
    if (gait_data_.event_ts_ms_[event_idx] > 0u &&
        (now_ms - gait_data_.event_ts_ms_[event_idx]) < kMinLocalContactEventIntervalMs)
    {
      event_flag = false;
    }
  };

  gate_local_event(gait_data_.event_hs_, kHS);
  gate_local_event(gait_data_.event_ts_, kTS);
  gate_local_event(gait_data_.event_ho_, kHO);
  gate_local_event(gait_data_.event_to_, kTO);
}

void FsrGaitEstimator::CheckDataFreshness(uint32_t now_ms)
{
  if (gait_data_.last_update_ms_ == 0u)
  {
    gait_data_.is_data_fresh_ = false;
    gait_data_.percent_gait_ = -1.0f;
    gait_data_.percent_stance_ = -1.0f;
    gait_data_.percent_swing_ = -1.0f;
    return;
  }

  if (!gait_data_.IsFresh(now_ms))
  {
    gait_data_.is_data_fresh_ = false;
    gait_data_.percent_gait_ = -1.0f;
    gait_data_.percent_stance_ = -1.0f;
    gait_data_.percent_swing_ = -1.0f;
    Reset();
    ResetSensorContactState(gait_data_.heel_);
    ResetSensorContactState(gait_data_.toe_);
  }
}

/**
 * @brief 根据 FS/FO 主事件解析当前支撑/摆动相位
 */

void FsrGaitEstimator::ResolvePhase()
{
  gait_data_.prev_phase_ = gait_data_.current_phase_;

  if (gait_data_.event_fs_)
  {
    gait_data_.current_phase_ = GaitPhase::kStance;
  }
  else if (gait_data_.event_fo_)
  {
    gait_data_.current_phase_ = GaitPhase::kSwing;
  }

  gait_data_.phase_changed_ = (gait_data_.current_phase_ != gait_data_.prev_phase_);
}

void FsrGaitEstimator::UpdatePercentages(uint32_t now_ms)
{
  const float expected_gait = gait_data_.expected_gait_duration_ms_;

  if (expected_gait > 0.0f && gait_data_.event_ts_ms_[kFS] > 0u)
  {
    gait_data_.percent_gait_ = 100.0f * ((float)(now_ms - gait_data_.event_ts_ms_[kFS]) / expected_gait);
    gait_data_.percent_gait_ = _constrain(gait_data_.percent_gait_, 0.0f, 100.0f);
  }
  else
  {
    gait_data_.percent_gait_ = -1.0f;
  }

  if (gait_data_.current_phase_ == GaitPhase::kStance)
  {
    float expected_stance = gait_data_.expected_stance_duration_ms_;
    if (expected_stance <= 0.0f && expected_gait > 0.0f)
    {
      expected_stance = expected_gait * 0.6f;
    }

    if (expected_stance > 0.0f && gait_data_.event_ts_ms_[kFS] > 0u)
    {
      gait_data_.percent_stance_ = 100.0f * ((float)(now_ms - gait_data_.event_ts_ms_[kFS]) / expected_stance);
      gait_data_.percent_stance_ = _constrain(gait_data_.percent_stance_, 0.0f, 100.0f);
    }
    else
    {
      gait_data_.percent_stance_ = -1.0f;
    }
    gait_data_.percent_swing_ = 0.0f;
  }
  else if (gait_data_.current_phase_ == GaitPhase::kSwing)
  {
    float expected_swing = gait_data_.expected_swing_duration_ms_;
    if (expected_swing <= 0.0f && expected_gait > 0.0f)
    {
      expected_swing = expected_gait * 0.4f;
    }

    if (expected_swing > 0.0f && gait_data_.event_ts_ms_[kFO] > 0u)
    {
      gait_data_.percent_swing_ = 100.0f * ((float)(now_ms - gait_data_.event_ts_ms_[kFO]) / expected_swing);
      gait_data_.percent_swing_ = _constrain(gait_data_.percent_swing_, 0.0f, 100.0f);
    }
    else
    {
      gait_data_.percent_swing_ = -1.0f;
    }
    gait_data_.percent_stance_ = 0.0f;
  }
  else
  {
    gait_data_.percent_stance_ = -1.0f;
    gait_data_.percent_swing_ = -1.0f;
  }
}

void FsrGaitEstimator::UpdateValidity()
{
  gait_data_.is_phase_valid_ =
    gait_data_.is_enabled_ &&
    gait_data_.is_data_fresh_ &&
    gait_data_.heel_.contact_inited &&
    gait_data_.toe_.contact_inited &&
    gait_data_.event_ts_ms_[kFS] > 0u &&
    gait_data_.event_ts_ms_[kFO] > 0u &&
    gait_data_.expected_gait_duration_ms_ > 0.0f &&
    gait_data_.percent_gait_ >= 0.0f &&
    gait_data_.percent_gait_ <= 100.0f;
}

void FsrGaitEstimator::UpdateEventTimings(uint32_t now_ms)
{
  if (gait_data_.event_fs_)
  {
    if (gait_data_.event_ts_ms_[kFO] > 0u)
    {
      const uint32_t swing_time = now_ms - gait_data_.event_ts_ms_[kFO];
      if (swing_time >= kMinSwingDurationMs && swing_time <= kMaxStepDurationMs)
      {
        UpdateExpectedDuration(swing_time,
                               gait_data_.swing_duration_window_ms_,
                               gait_data_.expected_swing_duration_ms_);
      }
    }

    if (gait_data_.event_ts_ms_[kFS] > 0u)
    {
      const uint32_t step_time = now_ms - gait_data_.event_ts_ms_[kFS];
      if (step_time >= kMinStepDurationMs && step_time <= kMaxStepDurationMs)
      {
        UpdateExpectedDuration(step_time,
                               gait_data_.gait_duration_window_ms_,
                               gait_data_.expected_gait_duration_ms_);
      }
    }

    RecordEventTimestamp(kFS, now_ms);
  }

  if (gait_data_.event_fo_)
  {
    if (gait_data_.event_ts_ms_[kFS] > 0u)
    {
      const uint32_t stance_time = now_ms - gait_data_.event_ts_ms_[kFS];
      if (stance_time >= kMinStanceDurationMs && stance_time <= kMaxStepDurationMs)
      {
        UpdateExpectedDuration(stance_time,
                               gait_data_.stance_duration_window_ms_,
                               gait_data_.expected_stance_duration_ms_);
      }
    }
    RecordEventTimestamp(kFO, now_ms);
  }

  if (gait_data_.event_hs_) RecordEventTimestamp(kHS, now_ms);
  if (gait_data_.event_ts_) RecordEventTimestamp(kTS, now_ms);
  if (gait_data_.event_ho_) RecordEventTimestamp(kHO, now_ms);
  if (gait_data_.event_to_) RecordEventTimestamp(kTO, now_ms);
}

void FsrGaitEstimator::UpdateExpectedDuration(uint32_t duration_ms,
                                              uint32_t duration_window_ms[],
                                              float &expected_duration_ms)
{
  uint8_t num_uninitialized = 0;
  for (int i = 0; i < FsrGaitData::kNumStepsAvg; i++)
  {
    num_uninitialized += (duration_window_ms[i] == 0u);
  }

  uint32_t *max_val = std::max_element(duration_window_ms, duration_window_ms + FsrGaitData::kNumStepsAvg);
  uint32_t *min_val = std::min_element(duration_window_ms, duration_window_ms + FsrGaitData::kNumStepsAvg);

  if (num_uninitialized > 0)
  {
    for (int i = (FsrGaitData::kNumStepsAvg - 1); i > 0; i--)
    {
      duration_window_ms[i] = duration_window_ms[i - 1];
    }
    duration_window_ms[0] = duration_ms;
    uint32_t sum_times = 0;
    uint8_t valid_count = 0;
    for (int i = 0; i < FsrGaitData::kNumStepsAvg; i++)
    {
      if (duration_window_ms[i] > 0u)
      {
        sum_times += duration_window_ms[i];
        valid_count++;
      }
    }
    if (valid_count > 0u) expected_duration_ms = (float)sum_times / valid_count;
  }
  else if ((duration_ms <= (gait_data_.expected_duration_window_upper_coeff_ * *max_val)) &&
           (duration_ms >= (gait_data_.expected_duration_window_lower_coeff_ * *min_val)))
  {
    uint32_t sum_times = duration_ms;
    for (int i = (FsrGaitData::kNumStepsAvg - 1); i > 0; i--)
    {
      sum_times += duration_window_ms[i - 1];
      duration_window_ms[i] = duration_window_ms[i - 1];
    }
    duration_window_ms[0] = duration_ms;
    expected_duration_ms = (float)sum_times / FsrGaitData::kNumStepsAvg;
  }
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
  last_contact_sample_ms_ = 0u;

  for (uint8_t ev = 0; ev < kNumGaitEvents; ev++)
  {
    gait_data_.event_ts_ms_[ev] = 0;
    gait_data_.prev_event_ts_ms_[ev] = 0;
    gait_data_.event_count_[ev] = 0;
  }
  for (int i = 0; i < FsrGaitData::kNumStepsAvg; i++)
  {
    gait_data_.gait_duration_window_ms_[i] = 0;
    gait_data_.stance_duration_window_ms_[i] = 0;
    gait_data_.swing_duration_window_ms_[i] = 0;
  }
  gait_data_.expected_gait_duration_ms_ = -1.0f;
  gait_data_.expected_stance_duration_ms_ = -1.0f;
  gait_data_.expected_swing_duration_ms_ = -1.0f;

  gait_data_.current_phase_ = GaitPhase::kUnknown;
  gait_data_.prev_phase_ = GaitPhase::kUnknown;
  gait_data_.phase_changed_ = false;
  gait_data_.is_phase_valid_ = false;
  gait_data_.percent_gait_ = -1.0f;
  gait_data_.percent_stance_ = -1.0f;
  gait_data_.percent_swing_ = -1.0f;
  gait_data_.heel_contact_state_ = false;
  gait_data_.toe_contact_state_ = false;
  gait_data_.foot_contact_state_ = false;
  gait_data_.prev_heel_contact_state_ = false;
  gait_data_.prev_toe_contact_state_ = false;
  gait_data_.prev_foot_contact_state_ = false;
  ClearCycleEvents();
}

void FsrGaitEstimator::ResetContact()
{
  last_contact_sample_ms_ = 0u;
  ResetSensorContactState(gait_data_.heel_);
  ResetSensorContactState(gait_data_.toe_);
}

void FsrGaitEstimator::ResetSensorContactState(FsrSensorData &sensor)
{
  sensor.contact_norm = 0.0f;
  sensor.ground_contact = false;
  sensor.contact_baseline = 0.0f;
  sensor.contact_peak_ref = 1.0f;
  sensor.contact_peak_step = 0.0f;
  sensor.contact_on_count = 0;
  sensor.contact_off_count = 0;
  sensor.contact_inited = false;
}

void FsrGaitEstimator::UpdateContactAdaptive(FsrSensorData &sensor, bool allow_baseline_update)
{
  if (!sensor.contact_inited)
  {
    const bool seed_valid = sensor.contact_seed_enabled &&
                            sensor.contact_seed_peak_ref > sensor.contact_seed_baseline;

    if (seed_valid)
    {
      sensor.contact_baseline = sensor.contact_seed_baseline;
      sensor.contact_peak_ref = sensor.contact_seed_peak_ref;
    }
    else
    {
      sensor.contact_baseline = sensor.raw_reading;
      sensor.contact_peak_ref = sensor.raw_reading + sensor.contact_min_range;
    }

    sensor.contact_peak_step = sensor.raw_reading;
    sensor.contact_norm = 0.0f;
    sensor.ground_contact = false;
    sensor.contact_on_count = 0;
    sensor.contact_off_count = 0;
    sensor.contact_inited = true;

    if (seed_valid)
    {
      float init_range = sensor.contact_peak_ref - sensor.contact_baseline;
      if (init_range < sensor.contact_min_range)
      {
        init_range = sensor.contact_min_range;
      }
      sensor.contact_norm = (sensor.raw_reading - sensor.contact_baseline) / init_range;
      sensor.contact_norm = _constrain(sensor.contact_norm, 0.0f, 1.5f);

      sensor.ground_contact = sensor.contact_norm >= sensor.contact_on_ratio;
      if (sensor.ground_contact && sensor.raw_reading > sensor.contact_peak_step)
      {
        sensor.contact_peak_step = sensor.raw_reading;
      }
    }
    return;
  }

  float range = sensor.contact_peak_ref - sensor.contact_baseline;
  if (range < sensor.contact_min_range)
  {
    range = sensor.contact_min_range;
  }

  sensor.contact_norm = (sensor.raw_reading - sensor.contact_baseline) / range;
  sensor.contact_norm = _constrain(sensor.contact_norm, 0.0f, 1.5f);
  const bool contact_candidate = sensor.ground_contact ||
                                 sensor.contact_norm >= sensor.contact_on_ratio;

  if (allow_baseline_update && !contact_candidate)
  {
    float alpha = sensor.contact_baseline_alpha;
    if (sensor.raw_reading < sensor.contact_baseline)
    {
      alpha *= 10.0f;
    }
    sensor.contact_baseline += alpha * (sensor.raw_reading - sensor.contact_baseline);

    range = sensor.contact_peak_ref - sensor.contact_baseline;
    if (range < sensor.contact_min_range)
    {
      range = sensor.contact_min_range;
    }
  }

  sensor.contact_norm = (sensor.raw_reading - sensor.contact_baseline) / range;
  sensor.contact_norm = _constrain(sensor.contact_norm, 0.0f, 1.5f);

  if (!sensor.ground_contact)
  {
    if (sensor.contact_norm >= sensor.contact_on_ratio)
    {
      if (sensor.contact_on_count < 255u)
      {
        sensor.contact_on_count++;
      }
    }
    else
    {
      sensor.contact_on_count = 0;
    }

    if (sensor.contact_on_count >= sensor.contact_on_count_threshold)
    {
      sensor.ground_contact = true;
      sensor.contact_peak_step = sensor.raw_reading;
      sensor.contact_on_count = 0;
      sensor.contact_off_count = 0;
    }
  }
  else
  {
    if (sensor.raw_reading > sensor.contact_peak_step)
    {
      sensor.contact_peak_step = sensor.raw_reading;
    }

    if (sensor.contact_norm <= sensor.contact_off_ratio)
    {
      if (sensor.contact_off_count < 255u)
      {
        sensor.contact_off_count++;
      }
    }
    else
    {
      sensor.contact_off_count = 0;
    }

    if (sensor.contact_off_count >= sensor.contact_off_count_threshold)
    {
      sensor.ground_contact = false;
      sensor.contact_off_count = 0;
      sensor.contact_on_count = 0;
    }
  }
}

void FsrGaitEstimator::FinalizeStepPeak(FsrSensorData &sensor)
{
  if (sensor.contact_peak_step > sensor.contact_baseline + sensor.contact_min_range)
  {
    sensor.contact_peak_ref += sensor.contact_peak_alpha * (sensor.contact_peak_step - sensor.contact_peak_ref);
  }

  sensor.contact_peak_step = sensor.contact_baseline;
}

void Side::CaptureStandPosture()
{
  hip_joint_.Calibrate();
  knee_joint_.Calibrate();
  ankle_joint_.Calibrate();
  knee_sea_joint_.Calibrate();

  ps_.thigh_imu_.CaptureStandPosture();
  ps_.shank_imu_.CaptureStandPosture();

  if (ps_.fsr_gait_data_.IsFresh(GetSysTimeMs()))
  {
    ps_.foot_imu_.CaptureStandPosture();
  }
}

void Side::ClearStandPosture()
{
  ps_.hip_joint_.ClearStandPosture();
  ps_.knee_joint_.ClearStandPosture();
  ps_.ankle_joint_.ClearStandPosture();
  ps_.knee_sea_joint_.ClearStandPosture();

  ps_.thigh_imu_.ClearStandPosture();
  ps_.shank_imu_.ClearStandPosture();
  ps_.foot_imu_.ClearStandPosture();
}

void Side::UpdateCalibrationStatus()
{
  const bool thigh_imu_ok =
    !ps_.thigh_imu_.is_enabled_ ||
    (ps_.thigh_imu_.IsUsable() && ps_.thigh_imu_.is_stand_posture_valid_);
  const bool shank_imu_ok =
    !ps_.shank_imu_.is_enabled_ ||
    (ps_.shank_imu_.IsUsable() && ps_.shank_imu_.is_stand_posture_valid_);
  const bool foot_imu_ok =
    !ps_.foot_imu_.is_enabled_ ||
    (ps_.foot_imu_.IsUsable() && ps_.foot_imu_.is_stand_posture_valid_);

  const bool hip_offset_needed = pe_.body_imu_.is_enabled_ && ps_.thigh_imu_.is_enabled_;
  const bool knee_offset_needed = ps_.thigh_imu_.is_enabled_ && ps_.shank_imu_.is_enabled_;
  const bool ankle_offset_needed = ps_.shank_imu_.is_enabled_ && ps_.foot_imu_.is_enabled_;

  const bool hip_human_ok =
    !hip_offset_needed ||
    (pe_.body_imu_.IsUsable() && ps_.thigh_imu_.IsUsable() && ps_.hip_joint_.is_sagittal_pos_offset_valid_);
  const bool knee_human_ok =
    !knee_offset_needed ||
    (ps_.thigh_imu_.IsUsable() && ps_.shank_imu_.IsUsable() && ps_.knee_joint_.is_sagittal_pos_offset_valid_);
  const bool ankle_human_ok =
    !ankle_offset_needed ||
    (ps_.shank_imu_.IsUsable() && ps_.foot_imu_.IsUsable() && ps_.ankle_joint_.is_sagittal_pos_offset_valid_);

  const bool hip_link_ok =
    !ps_.hip_joint_.is_actuator_enabled_ ||
    ps_.hip_joint_.is_link_pos_offset_valid_;
  const bool knee_link_ok =
    !ps_.knee_joint_.is_actuator_enabled_ ||
    ps_.knee_joint_.is_link_pos_offset_valid_;
  const bool ankle_link_ok =
    !ps_.ankle_joint_.is_actuator_enabled_ ||
    ps_.ankle_joint_.is_link_pos_offset_valid_;
  const bool knee_sea_link_ok =
    !ps_.knee_sea_joint_.is_actuator_enabled_ ||
    ps_.knee_sea_joint_.is_link_pos_offset_valid_;

  ps_.is_calibrated_ =
    thigh_imu_ok &&
    shank_imu_ok &&
    foot_imu_ok &&
    hip_human_ok &&
    knee_human_ok &&
    ankle_human_ok &&
    hip_link_ok &&
    knee_link_ok &&
    ankle_link_ok &&
    knee_sea_link_ok;
}

void Side::Read()
{
  hip_joint_.Read();
  knee_joint_.Read();
  ankle_joint_.Read();
  knee_sea_joint_.Read();
}

bool Side::IsMotorConnect()
{
  bool hip_ok = hip_joint_.IsMotorConnect();
  bool knee_ok = knee_joint_.IsMotorConnect();
  bool ankle_ok = ankle_joint_.IsMotorConnect();

  bool knee_sea_ok = knee_sea_joint_.IsMotorConnect();

  return hip_ok && knee_ok && ankle_ok && knee_sea_ok;
}

void Side::Standby()
{
  /* TODO: optimize standby strategy. */
  hip_joint_.Standby();
  knee_joint_.Standby();
  ankle_joint_.Standby();

  knee_sea_joint_.Standby();
}

void Side::Assist()
{
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

  bool is_stair_mode = pe_.intention_data_.current_mode_ == LocoMode::kStairAscent || pe_.intention_data_.current_mode_ == LocoMode::kStairDescent;
  bool fsr_ok = stair.is_enabled_ && ps_.fsr_gait_data_.IsContactReady();

  bool knee_ok = false;
  float knee_angle = 0.0f;
  float knee_vel = 0.0f;
  if (ps_.thigh_imu_.IsUsable() &&
      ps_.shank_imu_.IsUsable() &&
      ps_.knee_joint_.is_sagittal_pos_offset_valid_)
  {
    knee_angle = ps_.knee_joint_.sagittal_pos_rad_ * RAD_TO_DEG;
    knee_vel = ps_.knee_joint_.sagittal_vel_radps_ * RAD_TO_DEG;
    knee_ok = true;
  }

  if (!is_stair_mode || !fsr_ok || !knee_ok)
  {
    TransitionTo(StairPhase::kSwing);
    stair.prev_is_contact_ = stair.is_contact_;
    stair.is_contact_ = false;
    stair.phase_changed_ = (stair.current_phase_ != stair.prev_phase_);
    return;
  }

  bool is_contact = (ps_.fsr_gait_data_.heel_contact_state_ || ps_.fsr_gait_data_.toe_contact_state_);

  bool prev_contact = stair.is_contact_;
  bool contact_rising = is_contact && !prev_contact;
  bool contact_falling = !is_contact && prev_contact;

  stair.is_valid_ = true;
  stair.prev_is_contact_ = prev_contact;
  stair.is_contact_ = is_contact;

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

  if (pe_.intention_data_.current_mode_ == LocoMode::kStairAscent)
  {
    switch (stair.current_phase_)
    {
    case StairPhase::kSwing:
      if (contact_rising && knee_angle >= kStairStepContactAngleThresh)
      {
        TransitionTo(StairPhase::kWeightAcceptance);
      }
      break;
    case StairPhase::kWeightAcceptance:
      if (contact_falling)
      {
        TransitionTo(StairPhase::kSwing);
        break;
      }
      {
        float extension_delta = stair.contact_start_knee_angle_deg_ - knee_angle;
        bool pullup_by_angle = extension_delta >= kPullUpStartDeltaDeg;
        bool pullup_by_velocity = (extension_delta >= kPullUpStartVelMinDeltaDeg) && (knee_vel <= kPullUpStartVelThresh);
        if (pullup_by_angle || pullup_by_velocity)
        {
          TransitionTo(StairPhase::kPullUp);
        }
      }
      break;
    case StairPhase::kPullUp:
      if (contact_falling)
      {
        TransitionTo(StairPhase::kSwing);
        break;
      }
      if (knee_angle < kKneeStraightAngle)
      {
        TransitionTo(StairPhase::kForwardContinuance);
      }
      break;
    case StairPhase::kForwardContinuance:
      if (contact_falling)
      {
        TransitionTo(StairPhase::kSwing);
        break;
      }
      if ((knee_angle - stair.min_knee_angle_since_contact_deg_) >= kForwardEndFlexionDeltaDeg)
      {
        TransitionTo(StairPhase::kSwing);
      }
      break;
    default:
      TransitionTo(StairPhase::kSwing);
      break;
    }
  }
  else if (pe_.intention_data_.current_mode_ == LocoMode::kStairDescent)
  {
    switch (stair.current_phase_)
    {
    case StairPhase::kSwing:
      // if (contact_rising && opposite_knee_ok && opposite_knee_angle >= kStairDescentOppositeKneeAngleThresh)
      if (contact_rising)
      {
        TransitionTo(StairPhase::kWeightAcceptance);
      }
      break;
    case StairPhase::kWeightAcceptance:
      if (contact_falling)
      {
        TransitionTo(StairPhase::kSwing);
        break;
      }
      {
        float flexion_delta = knee_angle - stair.contact_start_knee_angle_deg_;
        bool lowering_by_angle = flexion_delta >= kControlledLoweringStartDeltaDeg;
        bool lowering_by_velocity = (flexion_delta >= kControlledLoweringVelMinDeltaDeg) && (knee_vel >= kKneeFlexingVelThresh);
        if (lowering_by_angle || lowering_by_velocity)
        {
          TransitionTo(StairPhase::kControlledLowering);
        }
      }
      break;
    case StairPhase::kControlledLowering:
      if (contact_falling)
      {
        TransitionTo(StairPhase::kSwing);
      }
      break;
    default:
      TransitionTo(StairPhase::kSwing);
      break;
    }
  }

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
  stair.contact_start_knee_angle_deg_ = 0.0f;
  stair.min_knee_angle_since_contact_deg_ = 0.0f;
}

void StairPhaseEstimator::TransitionTo(StairPhase next_phase)
{
  StairPhaseData &stair = ps_.stair_phase_data_;
  if (stair.current_phase_ == next_phase) return;

  stair.current_phase_ = next_phase;
}

void StsPhaseEstimator::Update()
{
  StsPhaseData &sts = pe_.sts_phase_data_;
  sts.prev_phase_ = sts.current_phase_;
  sts.phase_changed_ = false;
  sts.is_valid_ = false;
  sts.valid_side_count_ = 0;

  bool is_sts_mode = (pe_.intention_data_.current_mode_ == LocoMode::kSitToStand ||
                      pe_.intention_data_.current_mode_ == LocoMode::kStandToSit);
  if (!is_sts_mode || !pe_.HasFullStandPostureRef())
  {
    sts.percent_transition_ = -1.0f;
    return;
  }

  float thigh_pitch_sum = 0.0f;
  float thigh_vel_sum = 0.0f;
  float knee_angle_sum = 0.0f;
  float knee_vel_sum = 0.0f;

  auto accumulate_side = [&](const SideData &side)
  {
    const ImuData &thigh = side.thigh_imu_;
    const ImuData &shank = side.shank_imu_;
    if (!thigh.is_enabled_ || !shank.is_enabled_)
    {
      return;
    }

    thigh_pitch_sum += thigh.SagittalFromStandRefRad() * RAD_TO_DEG;
    thigh_vel_sum += thigh.SagittalGyroRadps() * RAD_TO_DEG;
    knee_angle_sum += side.knee_joint_.sagittal_pos_rad_ * RAD_TO_DEG;
    knee_vel_sum += side.knee_joint_.sagittal_vel_radps_ * RAD_TO_DEG;
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

  if (pe_.intention_data_.current_mode_ == LocoMode::kSitToStand)
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
      if (fabsf(sts.avg_thigh_pitch_deg_ - (0.5f * (pe_.left_side_.thigh_imu_.SagittalFromStandRefRad() * RAD_TO_DEG + pe_.right_side_.thigh_imu_.SagittalFromStandRefRad() * RAD_TO_DEG))) < kCompleteThreshDeg &&
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
  else if (pe_.intention_data_.current_mode_ == LocoMode::kStandToSit)
  {
    if (sts.current_phase_ != StsPhase::kYielding && sts.current_phase_ != StsPhase::kSeated)
    {
      TransitionTo(StsPhase::kYielding);
    }

    switch (sts.current_phase_)
    {
    case StsPhase::kYielding:
      if (sts.avg_knee_angle_deg_ > kSeatedKneeAngle &&
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

bool AdaptiveOscillator::IsAoImuUsable(const ImuData &imu)
{
  return imu.IsUsable() && imu.is_stand_posture_valid_;
}

bool AdaptiveOscillator::IsAoFsrUsable(const FsrGaitData &fsr)
{
  return fsr.IsContactReady();
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
  total_foot_contact_duration_us_ = 0;

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
 * @brief 自适应频率振荡器更新: 用左右大腿 IMU roll 差估计步态相位
 * AO 的连续相位由 left_thigh_roll - right_thigh_roll 驱动;
 * 左/右足底首次接触分别作为该侧 0 相位锚点。
 * FSR 只负责相位复位, IMU 周期信号负责频率自适应。
 */
void AdaptiveOscillator::Update()
{
  if (!pe_.ao_data_.is_enabled_)
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

  const float left_roll_rad = left_thigh.SagittalFromStandRefRad();
  const float right_roll_rad = right_thigh.SagittalFromStandRefRad();
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

  bool is_left_event = (left_fsr_ok && left_fsr.event_fs_) || left_contact_edge;
  bool is_right_event = (right_fsr_ok && right_fsr.event_fs_) || right_contact_edge;

  const float left_roll_vel_radps = left_thigh.gyro_radps_[0];
  const float right_roll_vel_radps = right_thigh.gyro_radps_[0];
  float vel_energy = 0.0f;
  (void)arm_sqrt_f32(left_roll_vel_radps * left_roll_vel_radps + right_roll_vel_radps * right_roll_vel_radps, &vel_energy);
  const float energy_alpha = 1.0f - expf(-delta_ts_s / kEmaTauS);
  vel_energy_ema_ = energy_alpha * vel_energy + (1.0f - energy_alpha) * vel_energy_ema_;

  if (left_contact && right_contact)
  {
    total_foot_contact_duration_us_ += delta_ts_us;
  }
  else
  {
    total_foot_contact_duration_us_ = 0;
  }

  const bool is_stopping = (total_foot_contact_duration_us_ > kMaxStoppingDurationUs);
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

  auto accept_event = [&](bool event, uint32_t &event_cnt, uint64_t &event_time_us) -> bool
  {
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
                                       float &phi_e) -> float
  {
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

ExoShell::ExoShell(UART_HandleTypeDef &huart, Exo &exo) :
  Shell(huart),
  exo_(exo)
{
  RegisterCommand("setlm", CmdWrapper<ExoShell, &ExoShell::OnCmdSetLocoMode>, this);
  RegisterCommand("setslope", CmdWrapper<ExoShell, &ExoShell::OnCmdSetSlope>, this);

  RegisterCommand("lbvalidsw", [](void *ctx, int, char **)
                  {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.intention_data_.label_is_valid_ = ! shell.exo_.pe_.intention_data_.label_is_valid_;
        if (shell.exo_.pe_.intention_data_.label_is_valid_) shell.SendString("Data label valid: ON\r\n");
        else shell.SendString("Data label valid: OFF\r\n"); },
                  this);

  RegisterCommand("vofasw", [](void *ctx, int, char **)
                  {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.telemetry_config_.enable = ! shell.exo_.pe_.telemetry_config_.enable;
        if (shell.exo_.pe_.telemetry_config_.enable) shell.SendString("VOFA telemetry: ON\r\n");
        else shell.SendString("VOFA telemetry: OFF\r\n"); },
                  this);

  RegisterCommand("estop", [](void *ctx, int, char **)
                  {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kEmergencyStop;
        shell.SendString("!!! EMERGENCY STOP: power cycle required !!!\r\n"); },
                  this);

  RegisterCommand("wakeup", [](void *ctx, int, char **)
                  {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kWakeup;
        shell.SendString("-> Wakeup Requested\r\n"); },
                  this);

  RegisterCommand("calib", [](void *ctx, int, char **)
                  {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kStartCalibrate;
        shell.SendString("-> Calibration Requested\r\n");
        shell.SendString("   Keep shoes flat on level ground and stand still.\r\n"); },
                  this);

  RegisterCommand("recalibstand", [](void *ctx, int, char **)
                  {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.ClearStandPosture();
        shell.exo_.CaptureStandPosture();
        shell.exo_.left_side_.UpdateCalibrationStatus();
        shell.exo_.right_side_.UpdateCalibrationStatus();
        shell.SendString("-> Standing posture re-captured\r\n"); },
                  this);

  RegisterCommand("start", [](void *ctx, int, char **)
                  {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kStartAssist;
        shell.SendString("-> Start Assist Requested\r\n"); },
                  this);

  RegisterCommand("stop", [](void *ctx, int, char **)
                  {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kStopAssist;
        shell.SendString("-> Stop Assist Requested\r\n"); },
                  this);

  RegisterCommand("sleep", [](void *ctx, int, char **)
                  {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kEnterSleep;
        shell.SendString("-> Sleep Requested\r\n"); },
                  this);

  RegisterCommand("clearfaults", [](void *ctx, int, char **)
                  {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.pending_events_ |= ExoData::SysEvent::kClearFaults;
        shell.SendString("-> Clear Faults Requested\r\n"); },
                  this);

  RegisterCommand("testsw", [](void *ctx, int, char **)
                  {
        auto& shell = *static_cast<ExoShell *>(ctx);
        shell.exo_.pe_.do_test = ! shell.exo_.pe_.do_test;
        if (shell.exo_.pe_.do_test) shell.SendString("Test: ON\r\n");
        else shell.SendString("Test: OFF\r\n"); },
                  this);

  /* 注册需要实时调节的参数 */
  RegisterRwParam("assiston", &exo_.left_side_.ankle_joint_.assistance_start_phase_percent_);
  RegisterRwParam("assistoff", &exo_.left_side_.ankle_joint_.assistance_end_phase_percent_);
  RegisterRwParam("cablepre", &exo_.left_side_.ankle_joint_.cable_pre_tensioned_position_);
  RegisterRwParam("cableten", &exo_.left_side_.ankle_joint_.cable_tensioned_position_);
  RegisterRwParam("torkp", &exo_.right_side_.knee_joint_.joint_tor_pid_.kp_);
  RegisterRwParam("torki", &exo_.right_side_.knee_joint_.joint_tor_pid_.ki_);
  RegisterRwParam("divekarscale", &KneeJoint::divekar_params_.assistance_scale, 0.0f, 1.0f);
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
    exo_.intention_recognizer_.override_usr_.enable_locomode_override = false;
    Printf("Mode override: auto, label=%s\r\n", LocoModeToString(exo_.pe_.intention_data_.label_mode_));
    return;
  }

  LocoMode mode = LocoMode::kWalking;
  if (!ParseLocoMode(mode_str, mode))
  {
    Printf("Unknown mode: %s\r\n", mode_str);
    SendString("Modes: sit, sts, stand, stand2sit, walk, ru, rd, su, sd\r\n");
    return;
  }

  exo_.pe_.intention_data_.label_mode_ = mode;
  exo_.intention_recognizer_.override_usr_.enable_locomode_override = true;
  exo_.intention_recognizer_.override_usr_.forced_locomode = mode;
  Printf("Mode override+label: %s\r\n", LocoModeToString(mode));
}

bool ExoShell::StringEquals(const char *lhs, const char *rhs)
{
  return lhs != nullptr && rhs != nullptr && std::strcmp(lhs, rhs) == 0;
}

const char *ExoShell::LocoModeToString(LocoMode mode)
{
  switch (mode)
  {
  case LocoMode::kSitting:
    return "sitting";
  case LocoMode::kSitToStand:
    return "sit2stand";
  case LocoMode::kStanding:
    return "standing";
  case LocoMode::kStandToSit:
    return "stand2sit";
  case LocoMode::kWalking:
    return "walking";
  case LocoMode::kRampAscent:
    return "rampup";
  case LocoMode::kRampDescent:
    return "rampdown";
  case LocoMode::kStairAscent:
    return "stairup";
  case LocoMode::kStairDescent:
    return "stairdown";
  default:
    return "unknown";
  }
}

bool ExoShell::ParseLocoMode(const char *text, LocoMode &mode)
{
  if (StringEquals(text, "sitting") || StringEquals(text, "sit"))
  {
    mode = LocoMode::kSitting;
    return true;
  }
  if (StringEquals(text, "sit2stand") || StringEquals(text, "sit2s"))
  {
    mode = LocoMode::kSitToStand;
    return true;
  }
  if (StringEquals(text, "standing") || StringEquals(text, "stand"))
  {
    mode = LocoMode::kStanding;
    return true;
  }
  if (StringEquals(text, "stand2sit") || StringEquals(text, "stand2s"))
  {
    mode = LocoMode::kStandToSit;
    return true;
  }
  if (StringEquals(text, "walking") || StringEquals(text, "walk") || StringEquals(text, "w"))
  {
    mode = LocoMode::kWalking;
    return true;
  }
  if (StringEquals(text, "rampascent") || StringEquals(text, "ra"))
  {
    mode = LocoMode::kRampAscent;
    return true;
  }
  if (StringEquals(text, "rampdescent") || StringEquals(text, "rd"))
  {
    mode = LocoMode::kRampDescent;
    return true;
  }
  if (StringEquals(text, "stairascent") || StringEquals(text, "sa"))
  {
    mode = LocoMode::kStairAscent;
    return true;
  }
  if (StringEquals(text, "stairdescent") || StringEquals(text, "sd"))
  {
    mode = LocoMode::kStairDescent;
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
  if (!body_imu_.is_enabled_) return;

  BMI088_read(body_imu_.gyro_radps_, body_imu_.accel_mps2_, &body_imu_.chip_temp_c_);
  mahony_filter_.Update6Axis(body_imu_.gyro_radps_, body_imu_.accel_mps2_);
  mahony_filter_.GetQuaternion(body_imu_.q_);
  mahony_filter_.GetEulerAnglesRad(body_imu_.roll_rad_, body_imu_.pitch_rad_, body_imu_.yaw_rad_);
  body_imu_.UpdateSagittalKinematics();
  body_imu_.MarkKinematicsUpdated(GetSysTimeMs());
}

void DaMiaoImuHub::CanRxCallback(uint32_t can_id, const uint8_t *data, uint32_t dlc)
{
  uint8_t id = can_id & 0x0F;
  if (dlc != 8 || id < 1 || id > 4) return;

  ImuData &imu = ImuById(pe_, id);
  uint16_t temp[3];
  int w, x, y, z;

  switch (data[0])
  {
  case 0x01:
    temp[0] = data[3] << 8 | data[2];
    temp[1] = data[5] << 8 | data[4];
    temp[2] = data[7] << 8 | data[6];
    imu.accel_mps2_[0] = UintToFloat(temp[0], kAccelCanMin, kAccelCanMax, 16);
    imu.accel_mps2_[1] = UintToFloat(temp[1], kAccelCanMin, kAccelCanMax, 16);
    imu.accel_mps2_[2] = UintToFloat(temp[2], kAccelCanMin, kAccelCanMax, 16);
    break;

  case 0x02:
    temp[0] = data[3] << 8 | data[2];
    temp[1] = data[5] << 8 | data[4];
    temp[2] = data[7] << 8 | data[6];
    imu.gyro_radps_[0] = UintToFloat(temp[0], kGyroCanMin, kGyroCanMax, 16);
    imu.gyro_radps_[1] = UintToFloat(temp[1], kGyroCanMin, kGyroCanMax, 16);
    imu.gyro_radps_[2] = UintToFloat(temp[2], kGyroCanMin, kGyroCanMax, 16);
    break;

  case 0x03:
    temp[0] = data[3] << 8 | data[2];
    temp[1] = data[5] << 8 | data[4];
    temp[2] = data[7] << 8 | data[6];
    imu.roll_rad_ = UintToFloat(temp[2], kRollCanMin, kRollCanMax, 16) * DEG_TO_RAD;
    imu.pitch_rad_ = UintToFloat(temp[0], kPitchCanMin, kPitchCanMax, 16) * DEG_TO_RAD;
    imu.yaw_rad_ = UintToFloat(temp[1], kYawCanMin, kYawCanMax, 16) * DEG_TO_RAD;
    imu.is_quat_updated_ = false; /* Euler 直接写入, 无需延迟转换 */
    EulerRad2Quaternion(imu.roll_rad_, imu.pitch_rad_, imu.yaw_rad_, imu.q_);
    imu.UpdateSagittalKinematics();
    imu.MarkKinematicsUpdated(GetSysTimeMs());
    break;

  case 0x04:
    w = data[1] << 6 | ((data[2] & 0xFC) >> 2);
    x = (data[2] & 0x03) << 12 | (data[3] << 4) | ((data[4] & 0xF0) >> 4);
    y = (data[4] & 0x0F) << 10 | (data[5] << 2) | (data[6] & 0xC0) >> 6;
    z = (data[6] & 0x3F) << 8 | data[7];
    imu.q_[0] = UintToFloat(w, kQuaternionMin, kQuaternionMax, 14);
    imu.q_[1] = UintToFloat(x, kQuaternionMin, kQuaternionMax, 14);
    imu.q_[2] = UintToFloat(y, kQuaternionMin, kQuaternionMax, 14);
    imu.q_[3] = UintToFloat(z, kQuaternionMin, kQuaternionMax, 14);
    imu.is_quat_updated_ = true; /* 延迟至 Read() 批量转换 */
    break;

  default:
    break;
  }
}

/* ────────────── HiPnucImuHub ────────────── */
void HiPnucImuHub::CanRxCallback(uint32_t can_id, const uint8_t *data, uint32_t dlc)
{
  uint8_t imu_id = can_id & 0x0F;
  if (imu_id < 1 || imu_id > 4) return;
  ImuData &imu = ImuById(pe_, imu_id);
  uint32_t frame_type = can_id & 0x780; /* CANopen TPDO base: id & 0x780 */

  switch (frame_type)
  {
  case 0x180: /* 加速度: int16[3], mG, 6 bytes */
    if (dlc == 6)
    {
      imu.accel_mps2_[0] = (float)unpack_i16(&data[0]) * kAccelMGtoMps2;
      imu.accel_mps2_[1] = (float)unpack_i16(&data[2]) * kAccelMGtoMps2;
      imu.accel_mps2_[2] = (float)unpack_i16(&data[4]) * kAccelMGtoMps2;
    }
    break;

  case 0x280: /* 角速度: int16[3], 0.1°/s, 6 bytes */
    if (dlc == 6)
    {
      imu.gyro_radps_[0] = (float)unpack_i16(&data[0]) * kGyro01DpsToRps;
      imu.gyro_radps_[1] = (float)unpack_i16(&data[2]) * kGyro01DpsToRps;
      imu.gyro_radps_[2] = (float)unpack_i16(&data[4]) * kGyro01DpsToRps;
    }
    break;

  case 0x380: /* 欧拉角: int16[3], 0.01°, 6 bytes: Roll, Pitch, Yaw */
    if (dlc == 6)
    {
      imu.is_quat_updated_ = false;
      imu.roll_rad_ = (float)unpack_i16(&data[0]) * kEuler001DegToRad;
      imu.pitch_rad_ = (float)unpack_i16(&data[2]) * kEuler001DegToRad;
      imu.yaw_rad_ = (float)unpack_i16(&data[4]) * kEuler001DegToRad;
      EulerRad2Quaternion(imu.roll_rad_, imu.pitch_rad_, imu.yaw_rad_, imu.q_);
      imu.UpdateSagittalKinematics();
      imu.MarkKinematicsUpdated(GetSysTimeMs());
    }
    break;

  case 0x480: /* 四元数: int16[4], /10000, 8 bytes: w, x, y, z */
    if (dlc == 8)
    {
      imu.q_[0] = (float)unpack_i16(&data[0]) * kQuatScale;
      imu.q_[1] = (float)unpack_i16(&data[2]) * kQuatScale;
      imu.q_[2] = (float)unpack_i16(&data[4]) * kQuatScale;
      imu.q_[3] = (float)unpack_i16(&data[6]) * kQuatScale;
      imu.is_quat_updated_ = true; /* 延迟至 Read() 批量转换 */
    }
    break;

  default:
    break;
  }
}

bool IntentionRecognizer::IsGaitMode(LocoMode mode)
{
  return mode == LocoMode::kWalking ||
         mode == LocoMode::kRampAscent ||
         mode == LocoMode::kRampDescent ||
         mode == LocoMode::kStairAscent ||
         mode == LocoMode::kStairDescent;
}

bool IntentionRecognizer::IsSvmGaitMode(LocoMode mode)
{
  return IsGaitMode(mode);
}

SvmPrediction IntentionRecognizer::SvmClassifyIntention(const SvmFeatureVector &fv)
{
  SvmPrediction prediction;

  static constexpr bool kEnableLinearSvm = false; /* 模型移植完成后改为 true */
  static constexpr uint8_t kNumClasses = 5;
  static constexpr uint8_t kNumFeatures = 6;
  static constexpr LocoMode kClasses[kNumClasses] = {
    LocoMode::kWalking,
    LocoMode::kRampAscent,
    LocoMode::kRampDescent,
    LocoMode::kStairAscent,
    LocoMode::kStairDescent,
  };
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

  result.mode = static_cast<LocoMode>(best_mode_val);
  result.confidence = best_confidence_sum / (float)best_count;
  result.is_valid = result.confidence >= kSvmConfidenceThreshold;
  return result;
}

bool IntentionRecognizer::IsLegalTransition(LocoMode from, LocoMode to) const
{
  if (from == to) return true;

  if (IsGaitMode(from))
  {
    return IsGaitMode(to) || to == LocoMode::kStanding;
  }

  switch (from)
  {
  case LocoMode::kSitting:
    return to == LocoMode::kSitToStand;
  case LocoMode::kSitToStand:
    return to == LocoMode::kStanding || to == LocoMode::kSitting;
  case LocoMode::kStanding:
    return to == LocoMode::kWalking || to == LocoMode::kStandToSit;
  case LocoMode::kStandToSit:
    return to == LocoMode::kSitting || to == LocoMode::kStanding;
  default:
    return false;
  }
}

bool IntentionRecognizer::TryAcceptMode(LocoMode next_mode, uint32_t now_ms, bool force)
{
  LocoMode current = pe_.intention_data_.detected_mode_;
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
  pe_.intention_data_.sit_to_stand_intent_detected_ = false;
  pe_.intention_data_.stand_to_sit_intent_detected_ = false;
  return true;
}

__attribute__((noinline)) void IntentionRecognizer::UpdateSlopeEstimate(uint32_t now_ms, uint32_t delta_ms)
{
  if (delta_ms == 0u) delta_ms = 1u;

  /* 置信度辅助 */
  auto update_confidence = [&](bool has_new_sample)
  {
    if (has_new_sample)
    {
      pe_.intention_data_.terrain_slope_confidence_ = 1.0f;
      last_slope_sample_ms_ = now_ms;
    }
    else if (last_slope_sample_ms_ > 0u)
    {
      float step_ms = pe_.left_side_.fsr_gait_data_.expected_gait_duration_ms_;
      if (step_ms <= 0.0f) step_ms = 1000.0f;
      float tau_ms = step_ms * 0.8f;
      if (tau_ms > 1.0f)
      {
        pe_.intention_data_.terrain_slope_confidence_ *= expf(-(float)delta_ms / tau_ms);
        if (pe_.intention_data_.terrain_slope_confidence_ < 0.001f) pe_.intention_data_.terrain_slope_confidence_ = 0.0f;
      }
    }
    pe_.intention_data_.terrain_slope_valid_ = pe_.intention_data_.terrain_slope_confidence_ >= kSlopeConfidenceValidThresh;
  };

  if (!pe_.left_side_.foot_imu_.is_stand_posture_valid_ && !pe_.right_side_.foot_imu_.is_stand_posture_valid_)
  {
    left_slope_state_ = SlopeSideState{};
    right_slope_state_ = SlopeSideState{};
    pe_.intention_data_.foot_ref[0].terrain_slope_valid = false;
    pe_.intention_data_.foot_ref[1].terrain_slope_valid = false;
    update_confidence(false);
    return;
  }

  /* 单侧采集 + 步级坡度计算 */
  struct SideInput
  {
    const FsrGaitData &fsr;
    const ImuData &foot;
  };

  auto process_side = [&](SlopeSideState &s, const SideInput &in) -> bool
  {
    bool full_contact = in.fsr.IsContactReady() && in.foot.is_stand_posture_valid_ && in.foot.IsUsable() && in.fsr.heel_contact_state_ && in.fsr.toe_contact_state_;

    if (full_contact)
    {
      s.was_full_contact = true;
      s.contact_duration_ms += delta_ms;
      s.sample_count++;
      s.foot_pitch_sum_deg += in.foot.SagittalFromStandRefRad() * RAD_TO_DEG;
      return false;
    }

    bool produced = false;
    if (s.was_full_contact && s.contact_duration_ms >= kSlopeMinFullContactMs && s.sample_count > 0u)
    {
      float pitch_delta = s.foot_pitch_sum_deg / (float)s.sample_count;
      float slope = kSlopeGain * pitch_delta;
      s.last_step_slope_deg = slope;
      s.has_step_slope = true;
      s.last_sample_ms = now_ms;

      s.smooth_window_deg[s.smooth_window_idx] = slope;
      s.smooth_window_idx = (uint8_t)((s.smooth_window_idx + 1u) % kSlopeSmoothWindowSteps);
      if (s.smooth_window_count < kSlopeSmoothWindowSteps) s.smooth_window_count++;
      produced = true;
    }

    s.was_full_contact = false;
    s.contact_duration_ms = 0;
    s.sample_count = 0;
    s.foot_pitch_sum_deg = 0.0f;
    return produced;
  };

  const SideInput left_in = {pe_.left_side_.fsr_gait_data_, pe_.left_side_.foot_imu_};
  const SideInput right_in = {pe_.right_side_.fsr_gait_data_, pe_.right_side_.foot_imu_};

  bool left_new = process_side(left_slope_state_, left_in);
  bool right_new = process_side(right_slope_state_, right_in);

  /* 2 步因果滑动平均: 左右脚坡度 */
  auto smooth2 = [](const SlopeSideState &s, float &out) -> bool
  {
    if (!s.has_step_slope || s.smooth_window_count == 0u) return false;
    float sum = 0.0f;
    for (uint8_t i = 0; i < s.smooth_window_count; ++i)
      sum += s.smooth_window_deg[i];
    out = sum / (float)s.smooth_window_count;
    return true;
  };
  float left_deg = 0.0f;
  float right_deg = 0.0f;
  bool left_ok = smooth2(left_slope_state_, left_deg);
  bool right_ok = smooth2(right_slope_state_, right_deg);

  pe_.intention_data_.foot_ref[0].terrain_slope_deg = left_slope_state_.last_step_slope_deg;
  pe_.intention_data_.foot_ref[1].terrain_slope_deg = right_slope_state_.last_step_slope_deg;
  pe_.intention_data_.foot_ref[0].terrain_slope_valid = left_ok;
  pe_.intention_data_.foot_ref[1].terrain_slope_valid = right_ok;

  /* 双侧合并 -> terrain_slope_deg_ */
  if (!left_new && !right_new)
  {
    update_confidence(false);
    return;
  }

  uint8_t n = 0;
  float sum = 0.0f;
  if (left_ok)
  {
    sum += left_deg;
    n++;
  }
  if (right_ok)
  {
    sum += right_deg;
    n++;
  }
  if (n == 0u)
  {
    update_confidence(false);
    return;
  }
  if (left_ok && right_ok && fabsf(left_deg - right_deg) > kSlopeSideDisagreeDeg)
  {
    update_confidence(false);
    return;
  }

  pe_.intention_data_.terrain_slope_deg_ = sum / (float)n;
  update_confidence(true);
}

void IntentionRecognizer::UpdateRampModeBySlope(uint32_t now_ms)
{
  if (!pe_.intention_data_.terrain_slope_valid_) return;

  LocoMode current = pe_.intention_data_.detected_mode_;
  float slope_deg = pe_.intention_data_.terrain_slope_deg_;
  LocoMode next = current;
  if (current == LocoMode::kWalking)
  {
    if (slope_deg > kSlopeEnterDeg)
    {
      next = LocoMode::kRampAscent;
    }
    else if (slope_deg < -kSlopeEnterDeg)
    {
      next = LocoMode::kRampDescent;
    }
  }
  else if (current == LocoMode::kRampAscent)
  {
    if (slope_deg < -kSlopeEnterDeg)
    {
      next = LocoMode::kRampDescent;
    }
    else if (fabsf(slope_deg) < kSlopeExitDeg)
    {
      next = LocoMode::kWalking;
    }
  }
  else if (current == LocoMode::kRampDescent)
  {
    if (slope_deg > kSlopeEnterDeg)
    {
      next = LocoMode::kRampAscent;
    }
    else if (fabsf(slope_deg) < kSlopeExitDeg)
    {
      next = LocoMode::kWalking;
    }
  }
  if (next != current)
  {
    TryAcceptMode(next, now_ms);
  }
}

__attribute__((noinline)) void IntentionRecognizer::UpdateRuleBasedDetector(uint32_t now_ms, uint32_t delta_ms)
{
  FsrGaitData *left_fsr = &pe_.left_side_.fsr_gait_data_;
  FsrGaitData *right_fsr = &pe_.right_side_.fsr_gait_data_;
  JointData &left_hip = pe_.left_side_.hip_joint_;
  JointData &right_hip = pe_.right_side_.hip_joint_;
  IntentionData &intent = pe_.intention_data_;

  /* 双支撑看门狗: 步态 -> 站立 -> kStanding */
  bool is_double_support = (left_fsr->heel_contact_state_ || left_fsr->toe_contact_state_) &&
                           (right_fsr->heel_contact_state_ || right_fsr->toe_contact_state_);
  if (is_double_support)
  {
    intent.double_support_timer_ms_ += delta_ms;
    float kinetic_energy = fabsf(left_hip.sagittal_vel_radps_) + fabsf(right_hip.sagittal_vel_radps_);
    float l_knee_deg = pe_.left_side_.knee_joint_.sagittal_pos_rad_ * RAD_TO_DEG;
    float r_knee_deg = pe_.right_side_.knee_joint_.sagittal_pos_rad_ * RAD_TO_DEG;
    bool knees_extended = (fabsf(l_knee_deg) < kKneeExtendedAngleThresh) && (fabsf(r_knee_deg) < kKneeExtendedAngleThresh);
    if ((IsGaitMode(pe_.intention_data_.detected_mode_) || pe_.intention_data_.detected_mode_ == LocoMode::kStanding) &&
        intent.double_support_timer_ms_ > kDoubleSupportTimeoutMs &&
        kinetic_energy * RAD_TO_DEG < kStillEnergyThreshold &&
        knees_extended)
    {
      TryAcceptMode(LocoMode::kStanding, now_ms);
    }
  }
  else
  {
    intent.double_support_timer_ms_ = 0;
  }

  /* 按当前模式检测切换条件 */
  switch (pe_.intention_data_.detected_mode_)
  {
  /* 坐->站: 前倾启动 + 双脚受力 -> kSitToStand */
  case LocoMode::kSitting:
  {
    if (!pe_.HasFullStandPostureRef()) break;
    float l_hip_vel = left_hip.sagittal_vel_radps_ * RAD_TO_DEG;
    float r_hip_vel = right_hip.sagittal_vel_radps_ * RAD_TO_DEG;
    bool any_hip_valid = pe_.left_side_.hip_joint_.is_sagittal_pos_offset_valid_ || pe_.right_side_.hip_joint_.is_sagittal_pos_offset_valid_;
    if (any_hip_valid &&
        (l_hip_vel < kSitLeanVelThresh || !pe_.left_side_.hip_joint_.is_sagittal_pos_offset_valid_) &&
        (r_hip_vel < kSitLeanVelThresh || !pe_.right_side_.hip_joint_.is_sagittal_pos_offset_valid_))
      intent.sit_to_stand_intent_detected_ = true;
    bool both_feet_loaded = (left_fsr->heel_contact_state_ || left_fsr->toe_contact_state_) &&
                            (right_fsr->heel_contact_state_ || right_fsr->toe_contact_state_);
    if (intent.sit_to_stand_intent_detected_ && both_feet_loaded)
      TryAcceptMode(LocoMode::kSitToStand, now_ms, true);
    break;
  }

  /* 起立: 完成 -> kStanding; 放弃(膝仍屈且静止) -> kSitting */
  case LocoMode::kSitToStand:
  {
    float knee_angle_sum = 0.0f, knee_vel_sum = 0.0f;
    uint8_t valid_count = 0;
    auto accum = [&](const SideData &s)
    {
      if (!s.knee_joint_.is_sagittal_pos_offset_valid_) return;
      knee_angle_sum += s.knee_joint_.sagittal_pos_rad_ * RAD_TO_DEG;
      knee_vel_sum += fabsf(s.knee_joint_.sagittal_vel_radps_ * RAD_TO_DEG);
      valid_count++;
    };
    accum(pe_.left_side_);
    accum(pe_.right_side_);
    if (valid_count == 0) break;
    float avg_knee_ang = knee_angle_sum / valid_count;
    float avg_knee_vel = knee_vel_sum / valid_count;

    if (avg_knee_ang < 10.0f && avg_knee_vel < 3.0f)
      TryAcceptMode(LocoMode::kStanding, now_ms, true);

    if (avg_knee_ang > 30.0f && avg_knee_vel < 3.0f)
    {
      if (intent.sit_still_start_ms_ == 0) intent.sit_still_start_ms_ = now_ms;
    }
    else
    {
      intent.sit_still_start_ms_ = 0;
    }
    if (intent.sit_still_start_ms_ > 0 && (now_ms - intent.sit_still_start_ms_) > 500u)
      TryAcceptMode(LocoMode::kSitting, now_ms, true);
    break;
  }

  /* 站->走 / 站->坐: 走优先判断, 排除抬脚/步态事件后才判断坐 */
  case LocoMode::kStanding:
  {
    /* 站->走: 任意脚跟离地, Standing 模式下仅凭接触状态判断 */
    bool left_heel_off = (!left_fsr->heel_contact_state_ && left_fsr->toe_contact_state_);
    bool right_heel_off = (!right_fsr->heel_contact_state_ && right_fsr->toe_contact_state_);
    if (left_heel_off || right_heel_off)
    {
      if (TryAcceptMode(LocoMode::kWalking, now_ms, true))
      {
        intent.double_support_timer_ms_ = 0;
        intent.stand_to_sit_intent_detected_ = false;
      }
      break; /* 已切换到走, 跳过坐检测 */
    }

    /* 站->坐: 屈膝速度 -> 角度到位 */
    bool sts_ref_ready = pe_.HasFullStandPostureRef();
    bool both_feet_flat = (left_fsr->heel_contact_state_ && left_fsr->toe_contact_state_) &&
                          (right_fsr->heel_contact_state_ && right_fsr->toe_contact_state_);
    float l_knee_vel = pe_.left_side_.knee_joint_.sagittal_vel_radps_ * RAD_TO_DEG;
    float r_knee_vel = pe_.right_side_.knee_joint_.sagittal_vel_radps_ * RAD_TO_DEG;
    float l_knee_ang = pe_.left_side_.knee_joint_.sagittal_pos_rad_ * RAD_TO_DEG;
    float r_knee_ang = pe_.right_side_.knee_joint_.sagittal_pos_rad_ * RAD_TO_DEG;

    bool any_knee_valid = pe_.left_side_.knee_joint_.is_sagittal_pos_offset_valid_ || pe_.right_side_.knee_joint_.is_sagittal_pos_offset_valid_;
    if (sts_ref_ready && both_feet_flat && any_knee_valid &&
        (l_knee_vel > kKneeBendVelThresh || !pe_.left_side_.knee_joint_.is_sagittal_pos_offset_valid_) &&
        (r_knee_vel > kKneeBendVelThresh || !pe_.right_side_.knee_joint_.is_sagittal_pos_offset_valid_))
      intent.stand_to_sit_intent_detected_ = true;

    if (sts_ref_ready && intent.stand_to_sit_intent_detected_ &&
        (l_knee_ang > kKneeBendAngleThresh || r_knee_ang > kKneeBendAngleThresh))
      TryAcceptMode(LocoMode::kStandToSit, now_ms, true);
    break;
  }

  /* 坐下完成: 膝屈曲 + 静止 500ms -> kSitting */
  case LocoMode::kStandToSit:
  {
    float knee_angle_sum = 0.0f, knee_vel_sum = 0.0f;
    uint8_t valid_count = 0;
    auto accum = [&](const SideData &s)
    {
      if (!s.knee_joint_.is_sagittal_pos_offset_valid_) return;
      knee_angle_sum += s.knee_joint_.sagittal_pos_rad_ * RAD_TO_DEG;
      knee_vel_sum += fabsf(s.knee_joint_.sagittal_vel_radps_ * RAD_TO_DEG);
      valid_count++;
    };
    accum(pe_.left_side_);
    accum(pe_.right_side_);
    if (valid_count == 0) break;
    if (knee_angle_sum / valid_count > 45.0f && knee_vel_sum / valid_count < 3.0f)
    {
      if (intent.sit_still_start_ms_ == 0) intent.sit_still_start_ms_ = now_ms;
    }
    else
    {
      intent.sit_still_start_ms_ = 0;
    }
    if (intent.sit_still_start_ms_ > 0 && (now_ms - intent.sit_still_start_ms_) > 500u)
      TryAcceptMode(LocoMode::kSitting, now_ms, true);
    break;
  }

  default:
    break;
  }
}

void IntentionRecognizer::UpdateGaitSvmRouter()
{
  if (!IsSvmGaitMode(pe_.intention_data_.detected_mode_)) return;

  auto update_side = [&](FsrGaitData &fsr,
                         ImuData &thigh,
                         ImuData &shank,
                         StreamingFeatureExtractor &extractor,
                         bool &svm_done_this_step)
  {
    if (fsr.event_fo_)
    {
      extractor.Reset();
      svm_done_this_step = false;
    }

    bool sensors_ok =
      fsr.is_data_fresh_ &&
      thigh.IsUsable() &&
      thigh.is_stand_posture_valid_ &&
      shank.IsUsable() &&
      shank.is_stand_posture_valid_;
    bool in_swing = (!fsr.heel_contact_state_ && !fsr.toe_contact_state_);
    if (!sensors_ok || !in_swing)
    {
      return;
    }

    float knee_w_radps = thigh.SagittalGyroRadps() - shank.SagittalGyroRadps();
    extractor.Update(thigh.SagittalFromStandRefRad(),
                     shank.SagittalFromStandRefRad(),
                     thigh.SagittalGyroRadps(),
                     knee_w_radps,
                     thigh.accel_mps2_[2]);

    if (fsr.percent_swing_ > 75.0f && fsr.percent_swing_ < 95.0f && !svm_done_this_step)
    {
      SvmFeatureVector fv = extractor.GetFeatures(shank.SagittalFromStandRefRad());
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

void IntentionRecognizer::Update()
{
  uint32_t now_ms = GetSysTimeMs();
  uint32_t delta_ms = (last_update_ms_ == 0 || now_ms < last_update_ms_) ? 1u : (now_ms - last_update_ms_);
  if (delta_ms == 0u) delta_ms = 1u;
  last_update_ms_ = now_ms;

  UpdateSlopeEstimate(now_ms, delta_ms);
  // UpdateRampModeBySlope(now_ms);

  UpdateRuleBasedDetector(now_ms, delta_ms);

  /* SVM 地形分类: 实验性模块, 独立于物理规则状态机 */
  if (enable_svm_router_)
  {
    UpdateGaitSvmRouter(); /* 填充投票窗 */
    if ((pe_.left_side_.fsr_gait_data_.event_fs_ || pe_.right_side_.fsr_gait_data_.event_fs_) &&
        vote_count_ >= kVoteBufferSize)
    {
      SvmPrediction next = GetMajorityVote();
      if (next.is_valid)
        TryAcceptMode(next.mode, GetSysTimeMs());
    }
  }
}

/**
 * @brief 外骨骼系统初始化: 注册回调、启动通信、配置默认参数
 * @note  在 alt_main.cpp 中电机上电 1s 后调用
 */
void Exo::Initialize()
{
  /* 注册回调, 以便 BSP 回调能转发到当前 Exo 实例 */
  BspCanRegisterRxCallback(this, ExoCanRxBridge);
  BspUsartRegisterRxCallback(this, ExoUartRxBridge);
  BspUsartRegisterErrorCallback(this, ExoUartErrorBridge);
  BspSpiRegisterErrorCallback(this, ExoSpiErrorBridge);
  BspGpioRegisterExtiCallback(this, ExoGpioExtiBridge);

  /* 启动串口接收 */
  // SensorUartReceiveDma(); /* 暂需要, 改为通过 SPI 接收 */
  shell_.UartReceiveDma();

  /* 启用IMU */
  pe_.body_imu_.is_enabled_ = true;
  pe_.left_side_.foot_imu_.is_enabled_ = false;
  pe_.right_side_.foot_imu_.is_enabled_ = false;
  pe_.left_side_.shank_imu_.is_enabled_ = false;
  pe_.right_side_.shank_imu_.is_enabled_ = false;
  pe_.left_side_.thigh_imu_.is_enabled_ = true;
  pe_.right_side_.thigh_imu_.is_enabled_ = true;

  /* 启用FSR */
  pe_.left_side_.fsr_gait_data_.is_enabled_ = false;
  pe_.right_side_.fsr_gait_data_.is_enabled_ = false;
  /* contact_min_range 的单位与映射后的 raw_reading 一致, 当前为 pF。 */
  pe_.left_side_.fsr_gait_data_.heel_.contact_min_range = 30.0f;
  pe_.right_side_.fsr_gait_data_.heel_.contact_min_range = 30.0f;
  pe_.left_side_.fsr_gait_data_.toe_.contact_min_range = 20.0f;
  pe_.right_side_.fsr_gait_data_.toe_.contact_min_range = 20.0f;

  /* 启用关节 */
  pe_.left_side_.hip_joint_.is_actuator_enabled_ = false;
  pe_.right_side_.hip_joint_.is_actuator_enabled_ = false;
  pe_.left_side_.knee_joint_.is_actuator_enabled_ = true;
  pe_.right_side_.knee_joint_.is_actuator_enabled_ = true;
  pe_.left_side_.ankle_joint_.is_actuator_enabled_ = false;
  pe_.right_side_.ankle_joint_.is_actuator_enabled_ = false;
  pe_.left_side_.knee_sea_joint_.is_actuator_enabled_ = false;
  pe_.right_side_.knee_sea_joint_.is_actuator_enabled_ = false;
  pe_.left_side_.knee_joint_.motor_to_joint_sign_ = 1.0f;
  pe_.right_side_.knee_joint_.motor_to_joint_sign_ = -1.0f;

  /* 启用算法 */
  pe_.left_side_.stair_phase_data_.is_enabled_ = false;
  pe_.right_side_.stair_phase_data_.is_enabled_ = false;
  pe_.ao_data_.is_enabled_ = false;
  pe_.sts_phase_data_.is_enabled_ = false;
  pe_.intention_data_.is_enabled_ = true;

  /* 重置标定和估计器运行态。 */
  ResetCalibration();
  ResetEstimations();
  ResetControllerStates();

  /* 调试: 髋关节参数 */

  /* 调试: 膝关节参数 */
  left_side_.knee_joint_.force_profile_generator_.damping_ = 0.01f;
  left_side_.knee_joint_.force_profile_generator_.stiffness_ = 0.1f;

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
 * @brief 外骨骼 1kHz 主控制循环
 * @note
 */
void Exo::Run()
{
#if 0 /* 调试意图识别功能 */
    pe_.intention_data_.current_mode_ = LocoMode::kStairDescent;
    left_side_.stair_phase_estimator_.Update();
    right_side_.stair_phase_estimator_.Update();
    VofaSendTelemetry();
    return;

#elif 0 /* 调试坡度估计功能 */
  UpdateHumanJointKinematics();
  intention_recognizer_.Update();
  VofaSendTelemetry();
  return;

#else
  uint32_t start_ticks = DWT_CYCCNT;

  /* 1. 读取/转换传感器数据 */
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
    /* 没有命令可以撤销 E-stop, 需要断电重启。 */
    return;
  }

  /* 4. 检查是否出现欠压或其他故障
   * 欠压必须在 Sleep 中也检查, 否则休眠放置时可能导致电池过放。
   */
  const auto s = pe_.state_;
  const bool is_active = (s != ExoData::State::kSleep &&
                          s != ExoData::State::kFaultLowBattery &&
                          s != ExoData::State::kFaultSystem &&
                          s != ExoData::State::kEstopped);

  CheckSystemHealth();

  const bool battery_low = ((pe_.error_code_ & ExoData::Error::kBatteryLow) != ExoData::Error::kNone);
  const bool has_any_fault = (pe_.error_code_ != ExoData::Error::kNone);
  if (battery_low)
  {
    if (pe_.state_ != ExoData::State::kFaultLowBattery)
    {
      ClearNonCriticalEvents(pe_);
      pe_.state_ = ExoData::State::kFaultLowBattery;
    }
  }
  else if (is_active && has_any_fault)
  {
    if (pe_.state_ != ExoData::State::kFaultSystem)
    {
      ClearNonCriticalEvents(pe_);
      pe_.state_ = ExoData::State::kFaultSystem;
    }
  }
  else if (is_active && (pe_.pending_events_ & ExoData::SysEvent::kEnterSleep) != ExoData::SysEvent::kNone)
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
      left_side_.knee_joint_.PrepareMotorConnectionCheck();
      right_side_.knee_joint_.PrepareMotorConnectionCheck();

      pe_.pending_events_ &= ~ExoData::SysEvent::kWakeup;
      pe_.state_ = ExoData::State::kWaitMotorComm;
    }
    break;

  case ExoData::State::kWaitMotorComm: /* 接收到 calib 命令且电机通信检查完成则转入 kCalibrating */
    if (IsMotorConnect() && ((pe_.pending_events_ & ExoData::SysEvent::kStartCalibrate) != ExoData::SysEvent::kNone))
    {
      pe_.pending_events_ &= ~ExoData::SysEvent::kStartCalibrate;
      ResetCalibration();
      ResetEstimations();
      pe_.state_ = ExoData::State::kCalibrating;
    }
    break;

  case ExoData::State::kCalibrating: /* 标定完毕则自动转入kReady */
    Calibrate();
    Standby(); /* 为了获取电机/关节状态, 并保持通信 */
    if (IsCalibrateDone())
    {
      pe_.state_ = ExoData::State::kReady;
    }
    break;

  case ExoData::State::kReady: /* 用户发起start命令则转入kAssisting */
    Estimate();
    Standby(); /* 为了获取电机/关节状态, 并保持通信 */
    if ((pe_.pending_events_ & ExoData::SysEvent::kStartCalibrate) != ExoData::SysEvent::kNone) /* 此时如果对标定结果不满意可发起calib命令重新标定 */
    {
      pe_.pending_events_ &= ~ExoData::SysEvent::kStartCalibrate;
      ResetCalibration();
      ResetEstimations();
      pe_.state_ = ExoData::State::kCalibrating;
    }
    else if ((pe_.pending_events_ & ExoData::SysEvent::kStartAssist) != ExoData::SysEvent::kNone)
    {
      pe_.pending_events_ &= ~ExoData::SysEvent::kStartAssist;
      ResetControllerStates();
      pe_.state_ = ExoData::State::kAssisting;
    }
    break;

  case ExoData::State::kAssisting:
    Estimate(); /* 估计运动模式及该模式下的参数, 如 walking 及步态相位 */

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
    /* 充电到大于 19.5V 则重转入 kSleep */
    if (pe_.battery_voltage_ >= 19.5f)
    {
      pe_.state_ = ExoData::State::kSleep;
    }
    break;

  case ExoData::State::kFaultSystem:
    Shutdown();
    /* 最好不用 ClearFaults 命令, 有问题就关机排查 */
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
  dji_esc_hub_.SendAllCanTxData(); /* KneeSeaJoint 的关节指令最终在这里下发 */

  /* 处理上位机蓝牙发下来的命令, 所有命令见 ExoShell 构造函数 */
  const uint32_t now_ms = GetSysTimeMs(); /* 整帧复用, 避免多次调用 */
  if (shell_.ProcessPendingCommand())
  {
    pe_.telemetry_config_.pause_until_ms = now_ms + 100U; // HACK
  }

  /* 处理命令后需要反馈给上位机; 为避免反馈被遥测覆盖, 延时一段时间再发数据 */
  if (pe_.telemetry_config_.enable && (now_ms >= pe_.telemetry_config_.pause_until_ms))
  {
    VofaSendTelemetry();
  }

  /* 指示系统状态机当前状态 */
  state_led_.UpdateColorBDMA(static_cast<uint8_t>(pe_.state_));
  exo_run_time_us = DWTGetDeltaUs(start_ticks);
#endif
}

/**
 * @brief 标定双侧外骨骼: 关节零位 + FSR 踩踏力范围
 */
void Exo::Calibrate()
{
  CaptureStandPosture();
  left_side_.UpdateCalibrationStatus();
  right_side_.UpdateCalibrationStatus();
}

void Exo::ClearStandPosture()
{
  pe_.body_imu_.ClearStandPosture();
  left_side_.ClearStandPosture();
  right_side_.ClearStandPosture();
}

void Exo::CaptureStandPosture()
{
  pe_.body_imu_.CaptureStandPosture();
  left_side_.CaptureStandPosture();
  right_side_.CaptureStandPosture();
}

/**
 * @brief 重置所有标定标志位, 准备重新标定
 * @note  顶层只重置各模块标定状态; FSR 内部标定细节由 FsrGaitEstimator 负责。
 */
void Exo::ResetCalibration()
{
  /* 复位站立姿态参考 */
  ClearStandPosture();
  /* fsr */
  left_side_.fsr_gait_estimator_.ResetContact();
  right_side_.fsr_gait_estimator_.ResetContact();

  /* 更新标定状态 */
  left_side_.UpdateCalibrationStatus();
  right_side_.UpdateCalibrationStatus();
}

void Exo::ResetEstimations()
{
  left_side_.fsr_gait_estimator_.Reset();
  right_side_.fsr_gait_estimator_.Reset();
  left_side_.stair_phase_estimator_.Reset();
  right_side_.stair_phase_estimator_.Reset();
  sts_phase_estimator_.Reset();
  ao_.Reset();
  pe_.intention_data_.Reset();
}

void Exo::ResetControllerStates()
{
  left_side_.knee_joint_.DivekarReset();
  right_side_.knee_joint_.DivekarReset();
  KneeJoint::ResetDivekarBiLegContext();
}

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

  uint32_t now_ms = GetSysTimeMs();
  pe_.left_side_.shank_imu_.ApplyPendingQuaternion(now_ms);
  pe_.left_side_.thigh_imu_.ApplyPendingQuaternion(now_ms);
  pe_.right_side_.shank_imu_.ApplyPendingQuaternion(now_ms);
  pe_.right_side_.thigh_imu_.ApplyPendingQuaternion(now_ms);

  pe_.body_imu_.UpdateFreshness(now_ms);
  pe_.left_side_.foot_imu_.UpdateFreshness(now_ms);
  pe_.left_side_.shank_imu_.UpdateFreshness(now_ms);
  pe_.left_side_.thigh_imu_.UpdateFreshness(now_ms);
  pe_.right_side_.foot_imu_.UpdateFreshness(now_ms);
  pe_.right_side_.shank_imu_.UpdateFreshness(now_ms);
  pe_.right_side_.thigh_imu_.UpdateFreshness(now_ms);

  UpdateHumanJointKinematics();

  left_side_.fsr_gait_estimator_.CheckDataFreshness(now_ms);
  right_side_.fsr_gait_estimator_.CheckDataFreshness(now_ms);
}

void Exo::UpdateHumanJointKinematics()
{
  auto update_side = [&](SideData &side)
  {
    const bool hip_pos_ok =
      pe_.body_imu_.IsUsable() &&
      side.thigh_imu_.IsUsable() &&
      side.hip_joint_.is_sagittal_pos_offset_valid_;
    if (hip_pos_ok)
    {
      const float hip_raw_rad =
        WrapPi(side.thigh_imu_.SagittalRawRad() - pe_.body_imu_.SagittalRawRad());
      side.hip_joint_.sagittal_pos_rad_ =
        WrapPi(hip_raw_rad - side.hip_joint_.sagittal_pos_offset_rad_);
      const uint32_t hip_sample_ms = _min(side.thigh_imu_.last_update_ms_,
                                          pe_.body_imu_.last_update_ms_);
      side.hip_joint_.UpdateSagittalVelocity(
        side.thigh_imu_.SagittalGyroRawRadps() - pe_.body_imu_.SagittalGyroRawRadps(),
        hip_sample_ms);
    }

    const bool knee_pos_ok =
      side.thigh_imu_.IsUsable() &&
      side.shank_imu_.IsUsable() &&
      side.knee_joint_.is_sagittal_pos_offset_valid_;
    if (knee_pos_ok)
    {
      const float knee_raw_rad =
        WrapPi(side.thigh_imu_.SagittalRawRad() - side.shank_imu_.SagittalRawRad());
      side.knee_joint_.sagittal_pos_rad_ =
        WrapPi(knee_raw_rad - side.knee_joint_.sagittal_pos_offset_rad_);
      const uint32_t knee_sample_ms = _min(side.thigh_imu_.last_update_ms_,
                                           side.shank_imu_.last_update_ms_);
      side.knee_joint_.UpdateSagittalVelocity(
        side.thigh_imu_.SagittalGyroRawRadps() - side.shank_imu_.SagittalGyroRawRadps(),
        knee_sample_ms);
    }

    const bool ankle_pos_ok =
      side.shank_imu_.IsUsable() &&
      side.foot_imu_.IsUsable() &&
      side.ankle_joint_.is_sagittal_pos_offset_valid_;
    if (ankle_pos_ok)
    {
      const float ankle_raw_rad =
        WrapPi(side.foot_imu_.SagittalRawRad() - side.shank_imu_.SagittalRawRad());
      side.ankle_joint_.sagittal_pos_rad_ =
        WrapPi(ankle_raw_rad - side.ankle_joint_.sagittal_pos_offset_rad_);
      const uint32_t ankle_sample_ms = _min(side.foot_imu_.last_update_ms_,
                                            side.shank_imu_.last_update_ms_);
      side.ankle_joint_.UpdateSagittalVelocity(
        side.foot_imu_.SagittalGyroRawRadps() - side.shank_imu_.SagittalGyroRawRadps(),
        ankle_sample_ms);
    }
  };

  update_side(pe_.left_side_);
  update_side(pe_.right_side_);
}

/**
 * @brief 运动意图与步态参数估计 (中层控制)
 *
 * 两层架构:
 *  High level: IntentionRecognizer 算法始终运行 -> pe_.intention_data_.detected_mode_
 *              pe_.intention_data_.current_mode_ = override ? forced : detected
 *  Mid level:  根据 current_mode_ 估计相位 (FSR 步态 / 坐立角度 / AO)
 *
 * @note  用户可通过 Shell setlocomode 启用手动覆盖, 覆盖时算法仍在后台运行。
 */
void Exo::Estimate()
{
  pe_.intention_data_.prev_mode_ = pe_.intention_data_.current_mode_;
  intention_recognizer_.Update();
  pe_.intention_data_.current_mode_ = intention_recognizer_.override_usr_.enable_locomode_override ? intention_recognizer_.override_usr_.forced_locomode : pe_.intention_data_.detected_mode_;

  /* 模式转换检测: 按中层估计链路分类 */
  auto is_fsr_gait_phase_mode = [](LocoMode m) -> bool
  {
    return m == LocoMode::kWalking || m == LocoMode::kRampAscent || m == LocoMode::kRampDescent;
  };
  auto is_stair_mode = [](LocoMode m) -> bool
  {
    return m == LocoMode::kStairAscent || m == LocoMode::kStairDescent;
  };
  auto is_sts_mode = [](LocoMode m) -> bool
  {
    return m == LocoMode::kSitToStand || m == LocoMode::kStandToSit;
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

  if (mode_changed &&
      (was_fsr_contact_mode != now_fsr_contact_mode ||
       was_fsr_gait_phase != now_fsr_gait_phase ||
       was_stair != now_stair))
  {
    left_side_.fsr_gait_estimator_.Reset();
    right_side_.fsr_gait_estimator_.Reset();
  }

  /* AO 只在 walking/ramp 链路中更新; 进入/退出该链路时清理旧振荡器状态。 */
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

  uint32_t now_ms = GetSysTimeMs();

  /* FSR 传感器预处理: 所有模式都需要 fresh contact state */
  left_side_.fsr_gait_estimator_.PrepareUpdate(now_ms);
  right_side_.fsr_gait_estimator_.PrepareUpdate(now_ms);
  left_side_.fsr_gait_estimator_.FinalizeUpdate(now_ms);
  right_side_.fsr_gait_estimator_.FinalizeUpdate(now_ms);

  /* 模式特定相位估计 */
  // switch (pe_.intention_data_.current_mode_)
  // {
  // case LocoMode::kWalking:
  // case LocoMode::kRampAscent:
  // case LocoMode::kRampDescent:
  //   left_side_.fsr_gait_estimator_.FinalizeUpdate(now_ms);
  //   right_side_.fsr_gait_estimator_.FinalizeUpdate(now_ms);
  //   ao_.Update();
  //   break;
  // case LocoMode::kStairAscent:
  // case LocoMode::kStairDescent:
  //   left_side_.stair_phase_estimator_.Update();
  //   right_side_.stair_phase_estimator_.Update();
  //   break;
  // case LocoMode::kSitToStand:
  // case LocoMode::kStandToSit:
  //   sts_phase_estimator_.Update();
  //   break;
  // case LocoMode::kSitting:
  // case LocoMode::kStanding:
  //   break;
  // default:
  //   break;
  // }

  KneeJoint::UpdateDivekarBiLegContext(left_side_.knee_joint_, right_side_.knee_joint_, pe_);
  KneeJoint::LatchDivekarLeadingLeg(left_side_.knee_joint_,
                                    right_side_.knee_joint_,
                                    pe_.left_side_.fsr_gait_data_.event_fs_,
                                    pe_.right_side_.fsr_gait_data_.event_fs_);
  left_side_.knee_joint_.DivekarUpdate();
  right_side_.knee_joint_.DivekarUpdate();

  left_side_.fsr_gait_estimator_.CommitUpdate();
  right_side_.fsr_gait_estimator_.CommitUpdate();
}

/**
 * @brief 待机控制: 各关节进入零力/零力矩模式, 保持电机通信
 * @note  在 kCalibrating 和 kReady 状态下每周期调用
 */
void Exo::Standby()
{
  left_side_.Standby();
  right_side_.Standby();
}

/**
 * @brief 助力控制: 各关节根据步态相位输出助力力矩
 * @note  仅在 kAssisting 状态下调用, 内部通过步态百分比驱动各关节力曲线。
 */
void Exo::Assist()
{
#if 1
  left_side_.knee_joint_.ApplyDivekarControl();
  right_side_.knee_joint_.ApplyDivekarControl();
#else
  left_side_.Assist();
  right_side_.Assist();
#endif
}

/**
 * @brief 紧急关断: 所有电机关断 (DisableMotor), 步态估计器重置
 * @note  在 kSleep / kFaultLowBattery / kFaultSystem / E-Stop 路径中调用
 */
void Exo::Shutdown()
{
  left_side_.Shutdown();
  right_side_.Shutdown();
  sts_phase_estimator_.Reset();
}

/**
 * @brief 检查系统健康状态: 电池电压 + 各关节电机故障
 * @note  更新 pe_.error_code_ 位掩码, 但不执行状态转换; 状态转换在 Run() 中完成。
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
  if (pe_.left_side_.knee_joint_.is_actuator_enabled_ &&
      (left_side_.knee_joint_.motor_.error_code_ != 0 || left_side_.knee_joint_.motor_.fault_code_ != 0))
    pe_.error_code_ |= ExoData::Error::kLeftKneeFault;
  if (pe_.right_side_.knee_joint_.is_actuator_enabled_ &&
      (right_side_.knee_joint_.motor_.error_code_ != 0 || right_side_.knee_joint_.motor_.fault_code_ != 0))
    pe_.error_code_ |= ExoData::Error::kRightKneeFault;
  if (pe_.left_side_.ankle_joint_.is_actuator_enabled_ &&
      (left_side_.ankle_joint_.motor_.error_code_ != 0 || left_side_.ankle_joint_.motor_.fault_code_ != 0))
    pe_.error_code_ |= ExoData::Error::kLeftAnkleFault;
  if (pe_.right_side_.ankle_joint_.is_actuator_enabled_ &&
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
  if (fsr.event_fs_) mask |= 1u << kFS;
  if (fsr.event_hs_) mask |= 1u << kHS;
  if (fsr.event_ts_) mask |= 1u << kTS;
  if (fsr.event_fo_) mask |= 1u << kFO;
  if (fsr.event_ho_) mask |= 1u << kHO;
  if (fsr.event_to_) mask |= 1u << kTO;
  return mask;
}

/**
 * @brief 通过 USB CDC 发送 VOFA + JustFloat 格式遥测数据
 * @note  通过 kVofaDownsample 控制下采样率; kVofaDownsample = 1 时 1kHz 全量发送。
 *        可在不同分支间切换平地/坡道、楼梯完整调试或楼梯输入采集。
 */
extern uint32_t g_exo_run_us;
void Exo::VofaSendTelemetry()
{
  static uint8_t downsample_cnt = 1;
  uint16_t idx = 0;

#if 1
  if (downsample_cnt++ < 1) return;
  downsample_cnt = 1;

  DmaUnionBuffer buf = {0};

  /* I0-I3: frame metadata and Divekar configuration. */
  buf.f_data[idx++] = static_cast<float>(GetSysTimeMs()); // I0
  buf.f_data[idx++] = pe_.intention_data_.label_is_valid_ ? 1.0f : 0.0f; // I1
  buf.f_data[idx++] = static_cast<float>(pe_.intention_data_.label_mode_); // I2
  buf.f_data[idx++] = pe_.intention_data_.label_slope_deg_; // I3

  /* I4-I17: left FSR input, adaptive state, contact state and event state. */
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.heel_.raw_reading; // I4
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.toe_.raw_reading; // I5
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.heel_.contact_norm; // I6
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.toe_.contact_norm; // I7
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.heel_.contact_baseline; // I8
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.toe_.contact_baseline; // I9
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.heel_.contact_peak_ref; // I10
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.toe_.contact_peak_ref; // I11
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.heel_contact_state_ ? 1.0f : 0.0f; // I12
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.toe_contact_state_ ? 1.0f : 0.0f; // I13
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.foot_contact_state_ ? 1.0f : 0.0f; // I14
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.event_fs_ ? 1.0f : 0.0f; // I15
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.percent_gait_; // I16
  buf.f_data[idx++] = pe_.left_side_.fsr_gait_data_.IsContactReady() ? 1.0f : 0.0f; // I17

  /* I18-I31: right FSR input, adaptive state, contact state and event state. */
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.heel_.raw_reading; // I18
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.toe_.raw_reading; // I19
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.heel_.contact_norm; // I20
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.toe_.contact_norm; // I21
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.heel_.contact_baseline; // I22
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.toe_.contact_baseline; // I23
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.heel_.contact_peak_ref; // I24
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.toe_.contact_peak_ref; // I25
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.heel_contact_state_ ? 1.0f : 0.0f; // I26
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.toe_contact_state_ ? 1.0f : 0.0f; // I27
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.foot_contact_state_ ? 1.0f : 0.0f; // I28
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.event_fs_ ? 1.0f : 0.0f; // I29
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.percent_gait_; // I30
  buf.f_data[idx++] = pe_.right_side_.fsr_gait_data_.IsContactReady() ? 1.0f : 0.0f; // I31

  /* I32-I61: bilateral Divekar context and shared gates. */
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.delta_ajc_y_l_minus_r_cm; // I32
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.delta_ajc_dist_cm; // I33
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_delta_ajc_dist; // I34
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.left_fs_accepted ? 1.0f : 0.0f; // I35
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.right_fs_accepted ? 1.0f : 0.0f; // I36
  buf.f_data[idx++] = static_cast<float>(KneeJoint::bi_leg_ctx_.leading_leg); // I37
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.left_grf_BW; // I38
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.left_heel_BW; // I39
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.right_grf_BW; // I40
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.right_heel_BW; // I41
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_F_heel_left; // I42
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_F_heel_right; // I43
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_F_grf_inv_left; // I44
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_F_grf_inv_right; // I45
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.theta_k_abs_diff_rad; // I46
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_knee_pos_sym; // I47
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_left_foot_loaded; // I48
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_right_foot_loaded; // I49
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_both_foot_loaded; // I50
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_left_knee_raise_vel; // I51
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_left_knee_lower_vel; // I52
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_right_knee_raise_vel; // I53
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_right_knee_lower_vel; // I54
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_bilateral_knee_raise_vel; // I55
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_bilateral_knee_lower_vel; // I56
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.bilateral_lower_vel_radps; // I57
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_deep_flex_decay; // I58
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_left_thigh_flex; // I59
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_right_thigh_flex; // I60
  buf.f_data[idx++] = KneeJoint::bi_leg_ctx_.gate_sts_bilateral_thigh_flex; // I61

  /* I62-I99: left knee state, gates, torque components and control output. */
  buf.f_data[idx++] = left_side_.knee_joint_.pj_.tor_output_ref_Nm_; // I62
  buf.f_data[idx++] = left_side_.knee_joint_.pj_.tor_output_Nm_; // I63
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.theta_th_rad; // I64
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.theta_sh_rad; // I65
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.theta_la_rad; // I66
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.theta_k_rad; // I67
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.theta_k_dot_radps; // I68
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.theta_k_ddot_lpf_radps2; // I69
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.theta_kd_rad; // I70
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.theta_kd_max_rad; // I71
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.delta_ajc_y_fs_cm; // I72
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.gate_theta_la; // I73
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.gate_theta_kd_max; // I74
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.gate_delta_ajc_y_a_fs; // I75
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.gate_delta_ajc_y_na_fs; // I76
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.gate_theta_k_dot_sw; // I77
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.gate_F_grf_u; // I78
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.gate_sts_stand2sit; // I79
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.gate_sts_sit2stand; // I80
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_state_.gate_sts_ctx; // I81
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_a_mod_Nm; // I82
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_na_mod_Nm; // I83
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_grav_st_mod_Nm; // I84
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_st_Nm; // I85
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_grav_sw_Nm; // I86
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_sd_sw_Nm; // I87
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_sw_Nm; // I88
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_stand2sit_Nm; // I89
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_sit2stand_Nm; // I90
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_sts_mod_Nm; // I91
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_divekar_Nm; // I92
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_divekar_lpf_Nm; // I93
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.tau_divekar_lpf_limited_Nm; // I94
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.mc_stiffness_Nm_per_rad; // I95
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.mc_damping_Nm_s_per_rad; // I96
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.mc_position_ref_rad; // I97
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.mc_velocity_ref_radps; // I98
  buf.f_data[idx++] = left_side_.knee_joint_.divekar_output_.mc_torque_feedforward_Nm; // I99

  /* I100-I137: right knee state, gates, torque components and control output. */
  buf.f_data[idx++] = right_side_.knee_joint_.pj_.tor_output_ref_Nm_; // I100
  buf.f_data[idx++] = right_side_.knee_joint_.pj_.tor_output_Nm_; // I101
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.theta_th_rad; // I102
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.theta_sh_rad; // I103
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.theta_la_rad; // I104
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.theta_k_rad; // I105
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.theta_k_dot_radps; // I106
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.theta_k_ddot_lpf_radps2; // I107
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.theta_kd_rad; // I108
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.theta_kd_max_rad; // I109
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.delta_ajc_y_fs_cm; // I110
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.gate_theta_la; // I111
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.gate_theta_kd_max; // I112
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.gate_delta_ajc_y_a_fs; // I113
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.gate_delta_ajc_y_na_fs; // I114
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.gate_theta_k_dot_sw; // I115
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.gate_F_grf_u; // I116
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.gate_sts_stand2sit; // I117
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.gate_sts_sit2stand; // I118
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_state_.gate_sts_ctx; // I119
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_a_mod_Nm; // I120
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_na_mod_Nm; // I121
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_grav_st_mod_Nm; // I122
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_st_Nm; // I123
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_grav_sw_Nm; // I124
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_sd_sw_Nm; // I125
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_sw_Nm; // I126
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_stand2sit_Nm; // I127
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_sit2stand_Nm; // I128
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_sts_mod_Nm; // I129
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_divekar_Nm; // I130
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_divekar_lpf_Nm; // I131
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.tau_divekar_lpf_limited_Nm; // I132
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.mc_stiffness_Nm_per_rad; // I133
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.mc_damping_Nm_s_per_rad; // I134
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.mc_position_ref_rad; // I135
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.mc_velocity_ref_radps; // I136
  buf.f_data[idx++] = right_side_.knee_joint_.divekar_output_.mc_torque_feedforward_Nm; // I137
  VofaCdcSendJustFloat(buf, idx);
  return;

#elif 0 /* 用于检查传感器反馈 */
  if (downsample_cnt++ < 1) return;
  downsample_cnt = 1;

  DmaUnionBuffer buf = {0};

  ImuData &body = pe_.body_imu_;
  ImuData &left_thigh = pe_.left_side_.thigh_imu_;
  ImuData &right_thigh = pe_.right_side_.thigh_imu_;
  ImuData &left_shank = pe_.left_side_.shank_imu_;
  ImuData &right_shank = pe_.right_side_.shank_imu_;
  ImuData &left_foot = pe_.left_side_.foot_imu_;
  ImuData &right_foot = pe_.right_side_.foot_imu_;

  JointData &left_knee = pe_.left_side_.knee_joint_;
  JointData &right_knee = pe_.right_side_.knee_joint_;
  JointData &left_hip = pe_.left_side_.hip_joint_;
  JointData &right_hip = pe_.right_side_.hip_joint_;
  AnkleData &left_ankle = pe_.left_side_.ankle_joint_;
  AnkleData &right_ankle = pe_.right_side_.ankle_joint_;

  FsrGaitData &left_fsr = pe_.left_side_.fsr_gait_data_;
  FsrGaitData &right_fsr = pe_.right_side_.fsr_gait_data_;

  buf.f_data[idx++] = left_thigh.SagittalFromStandRefDeg();  // 0
  buf.f_data[idx++] = left_shank.SagittalFromStandRefDeg();
  buf.f_data[idx++] = left_knee.sagittal_pos_rad_ * RAD_TO_DEG;  // 2
  buf.f_data[idx++] = left_knee.sagittal_pos_offset_rad_ * RAD_TO_DEG;
  buf.f_data[idx++] = left_knee.sagittal_vel_radps_;
  buf.f_data[idx++] = left_knee.is_sagittal_pos_offset_valid_;

  buf.f_data[idx++] = right_thigh.SagittalFromStandRefDeg();  // 6
  buf.f_data[idx++] = right_shank.SagittalFromStandRefDeg();
  buf.f_data[idx++] = right_knee.sagittal_pos_rad_ * RAD_TO_DEG;  // 8
  buf.f_data[idx++] = right_knee.sagittal_pos_offset_rad_ * RAD_TO_DEG;
  buf.f_data[idx++] = right_knee.sagittal_vel_radps_;
  buf.f_data[idx++] = right_knee.is_sagittal_pos_offset_valid_;

  buf.f_data[idx++] = body.SagittalFromStandRefDeg();  // 12
  buf.f_data[idx++] = left_thigh.SagittalFromStandRefDeg();
  buf.f_data[idx++] = right_thigh.SagittalFromStandRefDeg();
  buf.f_data[idx++] = left_hip.sagittal_pos_rad_ * RAD_TO_DEG;  // 15
  buf.f_data[idx++] = right_hip.sagittal_pos_rad_ * RAD_TO_DEG;  // 16
  buf.f_data[idx++] = left_hip.sagittal_vel_radps_;
  buf.f_data[idx++] = right_hip.sagittal_vel_radps_;
  buf.f_data[idx++] = left_hip.is_sagittal_pos_offset_valid_;
  buf.f_data[idx++] = right_hip.is_sagittal_pos_offset_valid_;

  buf.f_data[idx++] = left_foot.SagittalFromStandRefDeg();  // 21
  buf.f_data[idx++] = right_foot.SagittalFromStandRefDeg();
  buf.f_data[idx++] = left_ankle.sagittal_pos_rad_ * RAD_TO_DEG;
  buf.f_data[idx++] = right_ankle.sagittal_pos_rad_ * RAD_TO_DEG;
  buf.f_data[idx++] = left_ankle.is_sagittal_pos_offset_valid_;
  buf.f_data[idx++] = right_ankle.is_sagittal_pos_offset_valid_;

  VofaCdcSendJustFloat(buf, idx);
  return;

#elif 0    /* 无线发送方式 */
  if (downsample_cnt++ < 5) return;
  downsample_cnt = 1;

  const SideData &left_side = pe_.left_side_;
  const FsrGaitData &left_fsr = left_side.fsr_gait_data_;
  const StairPhaseData &left_stair = left_side.stair_phase_data_;
  const JointData &left_knee = left_side.knee_joint_;

  const SideData &right_side = pe_.right_side_;
  const FsrGaitData &right_fsr = right_side.fsr_gait_data_;
  const StairPhaseData &right_stair = right_side.stair_phase_data_;
  const JointData &right_knee = right_side.knee_joint_;
  const ImuData &left_thigh = left_side.thigh_imu_;
  const ImuData &left_shank = left_side.shank_imu_;
  const ImuData &right_thigh = right_side.thigh_imu_;
  const ImuData &right_shank = right_side.shank_imu_;

  shell_.SetVofaJustFloatData(idx++, downsample_cnt);  // 0
  shell_.SetVofaJustFloatData(idx++, left_fsr.heel_contact_state_ ? 1.0f : 0.0f);  // 0
  shell_.SetVofaJustFloatData(idx++, left_fsr.toe_contact_state_ ? 1.0f : 0.0f);  // 1
  shell_.SetVofaJustFloatData(idx++, left_stair.is_contact_ ? 1.0f : 0.0f);  // 2
  shell_.SetVofaJustFloatData(idx++, left_thigh.SagittalRawRad() * RAD_TO_DEG);  // 3
  shell_.SetVofaJustFloatData(idx++, left_shank.SagittalRawRad() * RAD_TO_DEG);  // 4
  shell_.SetVofaJustFloatData(idx++, left_thigh.SagittalGyroRadps());  // 5
  shell_.SetVofaJustFloatData(idx++, left_shank.SagittalGyroRadps());  // 6
  shell_.SetVofaJustFloatData(idx++, left_knee.sagittal_pos_rad_ * RAD_TO_DEG);  // 7
  shell_.SetVofaJustFloatData(idx++, left_knee.sagittal_vel_radps_ * RAD_TO_DEG);  // 8
  shell_.SetVofaJustFloatData(idx++, (float)static_cast<uint8_t>(left_stair.current_phase_));  // 9
  shell_.SetVofaJustFloatData(idx++, left_stair.phase_changed_ ? 1.0f : 0.0f);  // 10
  shell_.SetVofaJustFloatData(idx++, left_stair.is_valid_ ? 1.0f : 0.0f);  // 11

  shell_.SendVofaJustFloatFrame(idx);
  return;

#endif
}

/**
 * @brief 原始传感器 + 数据集标签 + 意图识别调试帧
 * @note  以 60 个 float 保持原始传感器采集格式。
 *        0..4 通用标签; 5..13 body IMU; 14..36 左侧; 37..59 右侧.
 *        60..68 为意图/坡度估计调试输出。
 */
void Exo::VofaSendSensorTelemetry(uint32_t now_ms, uint32_t loop_cnt)
{
  DmaUnionBuffer buf = {0};
  uint16_t idx = 0;

  buf.f_data[idx++] = (float)now_ms;  // 0
  buf.f_data[idx++] = (float)loop_cnt;  // 1

  buf.f_data[idx++] = pe_.intention_data_.label_is_valid_;  // 2
  buf.f_data[idx++] = static_cast<float>(pe_.intention_data_.label_mode_);  // 3
  buf.f_data[idx++] = pe_.intention_data_.label_slope_deg_;  // 4

  buf.f_data[idx++] = pe_.body_imu_.roll_rad_ * RAD_TO_DEG;  // 5
  buf.f_data[idx++] = pe_.body_imu_.pitch_rad_ * RAD_TO_DEG;  // 6
  buf.f_data[idx++] = pe_.body_imu_.yaw_rad_ * RAD_TO_DEG;  // 7
  buf.f_data[idx++] = pe_.body_imu_.gyro_radps_[0];  // 8
  buf.f_data[idx++] = pe_.body_imu_.gyro_radps_[1];  // 9
  buf.f_data[idx++] = pe_.body_imu_.gyro_radps_[2];  // 10
  buf.f_data[idx++] = pe_.body_imu_.accel_mps2_[0];  // 11
  buf.f_data[idx++] = pe_.body_imu_.accel_mps2_[1];  // 12
  buf.f_data[idx++] = pe_.body_imu_.accel_mps2_[2];  // 13

  SideData *sides[2] = {&pe_.left_side_, &pe_.right_side_};
  for (uint8_t i = 0; i < 2; ++i)
  {
    SideData &side = *sides[i];
    const FsrGaitData &fsr = side.fsr_gait_data_;
    const ImuData &foot_imu = side.foot_imu_;
    const ImuData &shank_imu = side.shank_imu_;
    const ImuData &thigh_imu = side.thigh_imu_;

    buf.f_data[idx++] = fsr.heel_.raw_reading;  // 14 L / 37 R
    buf.f_data[idx++] = fsr.toe_.raw_reading;  // 15 L / 38 R
    buf.f_data[idx++] = fsr.heel_.contact_norm;  // 16 L / 39 R
    buf.f_data[idx++] = fsr.toe_.contact_norm;  // 17 L / 40 R
    buf.f_data[idx++] = fsr.heel_contact_state_ ? 1.f : 0.f;  // 18 L / 41 R
    buf.f_data[idx++] = fsr.toe_contact_state_ ? 1.f : 0.f;  // 19 L / 42 R
    buf.f_data[idx++] = fsr.is_phase_valid_ ? 1.f : 0.f;  // 20 L / 43 R
    buf.f_data[idx++] = fsr.expected_gait_duration_ms_ * 0.001f;  // 21 L / 44 R (s)
    buf.f_data[idx++] = fsr.percent_gait_ * 0.01f;  // 22 L / 45 R ([0,1])

    buf.f_data[idx++] = foot_imu.roll_rad_ * RAD_TO_DEG;  // 23 L / 46 R
    buf.f_data[idx++] = foot_imu.pitch_rad_ * RAD_TO_DEG;  // 24 L / 47 R
    buf.f_data[idx++] = foot_imu.yaw_rad_ * RAD_TO_DEG;  // 25 L / 48 R

    buf.f_data[idx++] = shank_imu.roll_rad_ * RAD_TO_DEG;  // 26 L / 49 R
    buf.f_data[idx++] = shank_imu.pitch_rad_ * RAD_TO_DEG;  // 27 L / 50 R
    buf.f_data[idx++] = shank_imu.yaw_rad_ * RAD_TO_DEG;  // 28 L / 51 R
    buf.f_data[idx++] = shank_imu.gyro_radps_[0];  // 29 L / 52 R
    buf.f_data[idx++] = shank_imu.gyro_radps_[1];  // 30 L / 53 R
    buf.f_data[idx++] = shank_imu.gyro_radps_[2];  // 31 L / 54 R

    buf.f_data[idx++] = thigh_imu.roll_rad_ * RAD_TO_DEG;  // 32 L / 55 R
    buf.f_data[idx++] = thigh_imu.pitch_rad_ * RAD_TO_DEG;  // 33 L / 56 R
    buf.f_data[idx++] = thigh_imu.yaw_rad_ * RAD_TO_DEG;  // 34 L / 57 R
    buf.f_data[idx++] = thigh_imu.gyro_radps_[0];  // 35 L / 58 R
    buf.f_data[idx++] = thigh_imu.gyro_radps_[1];  // 36 L / 59 R
    buf.f_data[idx++] = thigh_imu.gyro_radps_[2];  // 37 L / 61 R
  }

  VofaCdcSendJustFloat(buf, idx);
}

/**
 * @brief 上下楼梯相位估计调试帧
 * @note  ? 52 ? float:
 *        0..4 通用; 5..26 左侧; 27..48 右侧; 49..51 数据集标签(valid/mode/slope)。
 */
void Exo::VofaSendStairTelemetry(uint32_t now_ms, uint32_t loop_cnt)
{
  DmaUnionBuffer buf = {0};
  uint16_t idx = 0;

  buf.f_data[idx++] = (float)now_ms;  // 0,
  buf.f_data[idx++] = (float)loop_cnt;  // 1,
  buf.f_data[idx++] = (float)static_cast<uint8_t>(pe_.state_);  // 2,
  buf.f_data[idx++] = (float)static_cast<uint8_t>(pe_.intention_data_.current_mode_);  // 3,
  buf.f_data[idx++] = (float)static_cast<uint32_t>(pe_.error_code_);  // 4,

  SideData *sides[2] = {&pe_.left_side_, &pe_.right_side_};
  for (uint8_t i = 0; i < 2; ++i)
  {
    SideData &side = *sides[i];
    const FsrGaitData &fsr = side.fsr_gait_data_;
    const StairPhaseData &stair = side.stair_phase_data_;
    const JointData &knee = side.knee_joint_;
    const ImuData &foot_imu = side.foot_imu_;
    const ImuData &thigh_imu = side.thigh_imu_;
    const ImuData &shank_imu = side.shank_imu_;

    uint8_t sensor_mask = 0;
    if (fsr.IsContactReady()) sensor_mask |= 1u << 0;
    if (foot_imu.is_enabled_) sensor_mask |= 1u << 1;
    if (thigh_imu.is_enabled_) sensor_mask |= 1u << 2;
    if (shank_imu.is_enabled_) sensor_mask |= 1u << 3;

    buf.f_data[idx++] = fsr.heel_.raw_reading;  // 5, 27
    buf.f_data[idx++] = fsr.toe_.raw_reading;  // 6, 28
    buf.f_data[idx++] = fsr.heel_.contact_norm;  // 7, 29
    buf.f_data[idx++] = fsr.toe_.contact_norm;  // 8, 30

    buf.f_data[idx++] = fsr.heel_contact_state_ ? 1.0f : 0.0f;  // 9, 31
    buf.f_data[idx++] = fsr.toe_contact_state_ ? 1.0f : 0.0f;  // 10, 32
    buf.f_data[idx++] = fsr.is_data_fresh_ ? 1.0f : 0.0f;  // 11, 33
    buf.f_data[idx++] = (float)sensor_mask;  // 12, 34

    buf.f_data[idx++] = (float)static_cast<uint8_t>(stair.current_phase_);  // 13, 35
    buf.f_data[idx++] = stair.phase_changed_ ? 1.0f : 0.0f;  // 14, 36
    buf.f_data[idx++] = stair.is_valid_ ? 1.0f : 0.0f;  // 15, 37
    buf.f_data[idx++] = stair.is_contact_ ? 1.0f : 0.0f;  // 16, 38
    buf.f_data[idx++] = knee.sagittal_pos_rad_ * RAD_TO_DEG;  // 17, 39
    buf.f_data[idx++] = knee.sagittal_vel_radps_ * RAD_TO_DEG;  // 18, 40

    buf.f_data[idx++] = thigh_imu.roll_rad_ * RAD_TO_DEG;  // 19, 41
    buf.f_data[idx++] = shank_imu.roll_rad_ * RAD_TO_DEG;  // 20, 42
    buf.f_data[idx++] = thigh_imu.gyro_radps_[0];  // 21, 43
    buf.f_data[idx++] = shank_imu.gyro_radps_[0];  // 22, 44

    buf.f_data[idx++] = foot_imu.roll_rad_ * RAD_TO_DEG;  // 23, 45
    buf.f_data[idx++] = foot_imu.pitch_rad_ * RAD_TO_DEG;  // 24, 46
    buf.f_data[idx++] = foot_imu.yaw_rad_ * RAD_TO_DEG;  // 25, 47
  }

  buf.f_data[idx++] = pe_.intention_data_.label_is_valid_ ? 1.0f : 0.0f;
  buf.f_data[idx++] = (float)static_cast<uint8_t>(pe_.intention_data_.label_mode_);
  buf.f_data[idx++] = pe_.intention_data_.label_slope_deg_;
  VofaCdcSendJustFloat(buf, idx);
}

bool Exo::IsMotorConnect()
{
  const bool left_ok = left_side_.IsMotorConnect();
  const bool right_ok = right_side_.IsMotorConnect();
  return left_ok && right_ok;
}

bool Exo::IsCalibrateDone()
{
  bool body_ref_ok = !pe_.body_imu_.is_enabled_ ||
                     (pe_.body_imu_.IsUsable() && pe_.body_imu_.is_stand_posture_valid_);
  return pe_.left_side_.is_calibrated_ && pe_.right_side_.is_calibrated_ && body_ref_ok;
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

  case ExoData::State::kEstopped:
    /* E-Stop 是终态, 不接受任何事件(包括再次 E-Stop) */
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
 * @brief CAN 鎬荤嚎鎺ユ敹鍥炶皟鍒嗗彂
 * @note  根据 CAN 外设句柄分发到对应总线:
 *        motor_can1 -> 左侧髋/膝/踝电机 + DJI ESC
 *        motor_can2 -> 右侧髋/膝/踝电机
 *        sensor_can -> 达妙四肢 IMU
 */
void Exo::CanRxCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, const uint8_t *data, uint32_t dlc)
{
  if (hfdcan == &hw_.motor_can1 && dlc == 8)
  {
    left_side_.hip_joint_.motor_.CanRxCallBack(can_id, data);
    left_side_.knee_joint_.motor_.CanRxCallBack(can_id, data);
    left_side_.ankle_joint_.motor_.CanRxCallBack(can_id, data);
    dji_esc_hub_.CanRxCallBack(can_id, data);
  }
  if (hfdcan == &hw_.motor_can2 && dlc == 8)
  {
    right_side_.hip_joint_.motor_.CanRxCallBack(can_id, data);
    right_side_.knee_joint_.motor_.CanRxCallBack(can_id, data);
    right_side_.ankle_joint_.motor_.CanRxCallBack(can_id, data);
  }
  if (hfdcan == &hw_.sensor_can)
  {
    if (can_id < 0x100)
      dm_imu_hub_.CanRxCallback(can_id, data, dlc);
    else
      hipnuc_imu_hub_.CanRxCallback(can_id, data, dlc);
  }
}
void Exo::SensorUartReceiveDma(void)
{
  /* DMA 接收双足无线传感数据, 波特率 1000000 Bits/s */
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
 * @brief SPI CS 下降沿中断 -> 启动 DMA 接收
 * @note  先 Abort 上一次传输(如果还在进行), 然后启动新一轮 DMA 接收。
 *        双缓冲索引 spi_dma_reading_idx_ 在 CS 上升沿回调中翻转。
 */
/* 启动 SPI 接收, 在 CS 下降沿中断时调用 */
void Exo::SpiRxStart(void)
{
  HAL_SPI_Abort(&hw_.sensor_spi);
  HAL_SPI_Receive_DMA(&hw_.sensor_spi, spi_rx_dma_buf[spi_dma_reading_idx_], SENSOR_SPI_RX_BUF_SIZE);
}

/**
 * @brief SPI CS 上升沿中断 -> 完成 DMA 接收, 翻转双缓冲索引
 * @note  双缓冲机制: 一个 buffer 正在 DMA 写入 (reading_idx),
 *        另一个 buffer 正在被主循环处理 (handling_idx)。
 *        CS 上升沿时: 记录本次传输字节数 -> 切换 handling_idx -> 翻转 reading_idx
 */
/* 完成 SPI 接收, 在 CS 上升沿中断时调用 */
void Exo::SpiRxCallback(void)
{
  uint8_t remaining_bytes = __HAL_DMA_GET_COUNTER(hw_.sensor_spi.hdmarx);
  spi_dma_readed_size_ = SENSOR_SPI_RX_BUF_SIZE - remaining_bytes;

  HAL_SPI_Abort(&hw_.sensor_spi);
  spi_dma_handling_idx_ = spi_dma_reading_idx_;
  spi_dma_reading_idx_ = (spi_dma_reading_idx_ + 1) % 2;
  spi_data_ready_ = true;
}

/* SPI 错误回调函数 */
void Exo::SpiErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi != &hw_.sensor_spi) return;
  HAL_SPI_Abort(hspi);
}

void Exo::ProcessSpiData()
{
  const uint8_t *data = spi_rx_dma_buf[spi_dma_handling_idx_];
  const uint8_t packet_size = spi_dma_readed_size_;

  if (packet_size == sizeof(exo_sensor_packet_v2_t))
  {
    const exo_sensor_packet_v2_t *packet = reinterpret_cast<const exo_sensor_packet_v2_t *>(data);

    /* Common heel/toe mapping identified from capgrf.csv after aligning the
       right sensors to their left-side equivalents. The mean robust gait peak
       is normalized to approximately 1 BW. */
    static constexpr float kHeelBaselinePf = 88.856218f;
    static constexpr float kToeBaselinePf = 41.462096f;
    static constexpr float kHeelNPerPf = 0.004944110f * 600.0f;
    static constexpr float kToeNPerPf = 0.010395912f * 600.0f;

    const float left_heel_raw_reading = packet->left_foot.heel_raw_reading;
    const float left_toe_raw_reading = packet->left_foot.toe_raw_reading;
    const float left_heel_mapped_pf = left_heel_raw_reading;
    const float left_toe_mapped_pf = left_toe_raw_reading;
    const float left_force_heel_N = _max(left_heel_mapped_pf - kHeelBaselinePf, 0.0f) * kHeelNPerPf;
    const float left_force_toe_N = _max(left_toe_mapped_pf - kToeBaselinePf, 0.0f) * kToeNPerPf;

    pe_.left_side_.fsr_gait_data_.heel_.raw_reading = left_heel_mapped_pf;
    pe_.left_side_.fsr_gait_data_.toe_.raw_reading = left_toe_mapped_pf;
    pe_.left_side_.fsr_gait_data_.heel_.force_N = left_force_heel_N;
    pe_.left_side_.fsr_gait_data_.toe_.force_N = left_force_toe_N;
    pe_.left_side_.fsr_gait_data_.cop_x_mm_ = packet->left_foot.cop_x_mm;
    pe_.left_side_.fsr_gait_data_.cop_y_mm_ = packet->left_foot.cop_y_mm;
    pe_.left_side_.fsr_gait_data_.vgrf_N_ = left_force_heel_N + left_force_toe_N;

    pe_.left_side_.foot_imu_.q_[1] = packet->left_foot.quatI;
    pe_.left_side_.foot_imu_.q_[2] = packet->left_foot.quatJ;
    pe_.left_side_.foot_imu_.q_[3] = packet->left_foot.quatK;
    pe_.left_side_.foot_imu_.q_[0] = packet->left_foot.quatReal;
    const bool left_foot_imu_valid =
      pe_.left_side_.foot_imu_.UpdateEulerFromQuaternion();
    if (left_foot_imu_valid)
    {
      pe_.left_side_.foot_imu_.UpdateSagittalKinematics();
    }

    const float right_heel_raw_reading = packet->right_foot.heel_raw_reading;
    const float right_toe_raw_reading = packet->right_foot.toe_raw_reading;
    const float right_heel_mapped_pf =
      0.001209486957f * right_heel_raw_reading * right_heel_raw_reading +
      0.55743072f * right_heel_raw_reading +
      14.949217f;
    const float right_toe_mapped_pf =
      1.09803407f * right_toe_raw_reading -
      2.364295f;
    const float right_force_heel_N = _max(right_heel_mapped_pf - kHeelBaselinePf, 0.0f) * kHeelNPerPf;
    const float right_force_toe_N = _max(right_toe_mapped_pf - kToeBaselinePf, 0.0f) * kToeNPerPf;

    // pe_.right_side_.fsr_gait_data_.heel_.raw_reading = right_heel_mapped_pf;
    // pe_.right_side_.fsr_gait_data_.toe_.raw_reading = right_toe_mapped_pf;
    /* HACK: just for debug */
    pe_.right_side_.fsr_gait_data_.heel_.raw_reading = packet->right_foot.heel_raw_reading;
    pe_.right_side_.fsr_gait_data_.toe_.raw_reading = packet->right_foot.toe_raw_reading;
    pe_.right_side_.fsr_gait_data_.heel_.force_N = right_force_heel_N;
    pe_.right_side_.fsr_gait_data_.toe_.force_N = right_force_toe_N;
    pe_.right_side_.fsr_gait_data_.cop_x_mm_ = packet->right_foot.cop_x_mm;
    pe_.right_side_.fsr_gait_data_.cop_y_mm_ = packet->right_foot.cop_y_mm;
    pe_.right_side_.fsr_gait_data_.vgrf_N_ = right_force_heel_N + right_force_toe_N;

    pe_.right_side_.foot_imu_.q_[1] = packet->right_foot.quatI;
    pe_.right_side_.foot_imu_.q_[2] = packet->right_foot.quatJ;
    pe_.right_side_.foot_imu_.q_[3] = packet->right_foot.quatK;
    pe_.right_side_.foot_imu_.q_[0] = packet->right_foot.quatReal;
    const bool right_foot_imu_valid =
      pe_.right_side_.foot_imu_.UpdateEulerFromQuaternion();
    if (right_foot_imu_valid)
    {
      pe_.right_side_.foot_imu_.UpdateSagittalKinematics();
    }

    uint32_t t_now = GetSysTimeMs();
    if (left_foot_imu_valid)
    {
      pe_.left_side_.foot_imu_.MarkKinematicsUpdated(t_now);
    }
    if (right_foot_imu_valid)
    {
      pe_.right_side_.foot_imu_.MarkKinematicsUpdated(t_now);
    }
    pe_.left_side_.fsr_gait_data_.last_update_ms_ = t_now;
    pe_.right_side_.fsr_gait_data_.last_update_ms_ = t_now;
    pe_.left_side_.fsr_gait_data_.is_data_fresh_ = true;
    pe_.right_side_.fsr_gait_data_.is_data_fresh_ = true;
  }
}
