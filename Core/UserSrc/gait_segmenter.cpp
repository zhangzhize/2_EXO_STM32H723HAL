#include "gait_segmenter.hpp"
#include <cmath>
#include <algorithm>

GaitSegmenter::GaitSegmenter(const GaitSegmenterConfig &config) : config_(config)
{
    // 初始化时保证阈值不低于下限
    state_.w_norm_th = std::max(config_.w_norm_th, config_.threshold_floor);
}

void GaitSegmenter::Reset()
{
    float current_th = std::max(state_.w_norm_th, config_.threshold_floor);
    // 重置所有状态，但保留当前学习到的阈值
    state_ = GaitState();
    state_.w_norm_th = current_th;
}

GaitSegmenterOutput GaitSegmenter::Update(const IMUSample &sample)
{
    // 防止时间倒流
    if (state_.has_last_sample_ms && sample.t_ms < state_.last_sample_ms)
    {
        return BuildOutput(0.0f, false);
    }

    // ---------- 1) 信号计算 ----------
    float w_norm = std::sqrt(sample.gyro_x * sample.gyro_x +
                             sample.gyro_y * sample.gyro_y +
                             sample.gyro_z * sample.gyro_z);

    state_.w_norm_filtered = LowPass(state_.has_w_norm_filtered, state_.w_norm_filtered, w_norm);
    state_.has_w_norm_filtered = true;

    state_.gyro_z_filtered = LowPass(state_.has_gyro_z_filtered, state_.gyro_z_filtered, sample.gyro_z);
    state_.has_gyro_z_filtered = true;

    if (sample.has_thigh_angle)
    {
        state_.thigh_angle = -sample.thigh_angle_x;
    }
    if (sample.has_thigh_angle && sample.has_foot_angle)
    {
        state_.ankle_angle = 180.0f - (sample.foot_angle_x - sample.thigh_angle_x);
    }

    state_.last_sample_ms = sample.t_ms;
    state_.has_last_sample_ms = true;

    bool is_valid_step = false;

    // ---------- 2) 冷却期检查 ----------
    if (IsInRefractory(sample.t_ms))
    {
        state_.prev_gyro_z_filtered = state_.gyro_z_filtered;
        state_.has_prev_gyro_z_filtered = true;
        return BuildOutput(w_norm, false);
    }

    // ---------- 3) 状态机 ----------
    if (state_.event == GaitEvent::kNone || state_.event == GaitEvent::kIC)
    {
        Transition(GaitEvent::kFF, sample.t_ms);
        state_.peak_w_norm = 0.0f;
        state_.ic_positive_count = 0;
    }
    else if (state_.event == GaitEvent::kFF)
    {
        if (state_.w_norm_filtered > state_.w_norm_th)
        {
            Transition(GaitEvent::kHO, sample.t_ms);
            state_.peak_w_norm = state_.w_norm_filtered;
        }
    }
    else if (state_.event == GaitEvent::kHO)
    {
        if (state_.w_norm_filtered > state_.peak_w_norm)
        {
            state_.peak_w_norm = state_.w_norm_filtered;
        }

        uint32_t ho_duration_ms = sample.t_ms - state_.ho_time;

        if (state_.peak_w_norm > 0.0f &&
            state_.w_norm_filtered < 0.8f * state_.peak_w_norm &&
            state_.gyro_z_filtered < -config_.to_negative_gz_th &&
            ho_duration_ms >= config_.to_min_duration_ms)
        {

            Transition(GaitEvent::kTO, sample.t_ms);
            state_.ic_positive_count = 0;
        }
    }
    else if (state_.event == GaitEvent::kTO)
    {
        uint32_t to_elapsed_ms = sample.t_ms - state_.to_time;

        bool crossed = false;
        if (config_.use_strict_ic_crossing)
        {
            crossed = (state_.has_prev_gyro_z_filtered && state_.prev_gyro_z_filtered < -config_.gz_zero_deadband && state_.gyro_z_filtered >= config_.gz_zero_deadband);
        }
        else
        {
            crossed = (state_.gyro_z_filtered >= config_.gz_zero_deadband);
        }

        bool positive_now = (state_.gyro_z_filtered >= config_.gz_zero_deadband);
        bool ic_ready = (to_elapsed_ms >= config_.ic_min_duration_ms);

        if (ic_ready && (crossed || (state_.ic_positive_count > 0 && positive_now)))
        {
            state_.ic_positive_count++;
        }
        else if (!positive_now)
        {
            state_.ic_positive_count = 0;
        }

        if (state_.ic_positive_count >= config_.ic_confirm_samples)
        {
            Transition(GaitEvent::kIC, sample.t_ms);
            ComputeCycleMetrics();
            state_.ic_positive_count = 0;

            if (state_.cycle_ms >= config_.cycle_min_ms && state_.cycle_ms <= config_.cycle_max_ms)
            {
                state_.stride++;
                is_valid_step = true;
                UpdateThresholdByCalibration(state_.peak_w_norm);
            }
        }
    }

    state_.prev_gyro_z_filtered = state_.gyro_z_filtered;
    state_.has_prev_gyro_z_filtered = true;

    return BuildOutput(w_norm, is_valid_step);
}

// ------------------- 辅助私有方法实现 -------------------

void GaitSegmenter::Transition(GaitEvent to_event, uint32_t t_ms)
{
    state_.event = to_event;
    state_.last_event_ms = t_ms;
    state_.has_last_event_ms = true;

    switch (to_event)
    {
    case GaitEvent::kFF:
        state_.ff_time = t_ms;
        break;
    case GaitEvent::kHO:
        state_.ho_time = t_ms;
        break;
    case GaitEvent::kTO:
        state_.to_time = t_ms;
        break;
    case GaitEvent::kIC:
        state_.ic_time = t_ms;
        break;
    default:
        break;
    }
}

bool GaitSegmenter::IsInRefractory(uint32_t now_ms) const
{
    if (!state_.has_last_event_ms)
        return false;
    return (now_ms - state_.last_event_ms) < config_.event_refract_ms;
}

float GaitSegmenter::LowPass(bool has_prev, float prev, float x) const
{
    if (!has_prev)
        return x;
    return (1.0f - config_.a) * prev + config_.a * x;
}

void GaitSegmenter::ComputeCycleMetrics()
{
    // 如果还没经历完整的周期，暂不计算
    if (state_.ff_time == 0 && state_.event != GaitEvent::kFF)
        return;

    state_.cycle_ms = state_.ic_time - state_.ff_time;
    state_.stance_ms = state_.to_time - state_.ff_time;
    state_.swing_ms = state_.ic_time - state_.to_time;
}

void GaitSegmenter::UpdateThresholdByCalibration(float peak_w_norm)
{
    if (peak_w_norm <= 0.0f)
        return;

    if (state_.num_calib_peaks < config_.calib_steps &&
        state_.num_calib_peaks < GaitState::kMaxCalibSteps)
    {

        state_.calibration_peaks[state_.num_calib_peaks] = peak_w_norm;
        state_.num_calib_peaks++;

        float sum = 0.0f;
        for (uint8_t i = 0; i < state_.num_calib_peaks; i++)
        {
            sum += state_.calibration_peaks[i];
        }
        float mean_peak = sum / state_.num_calib_peaks;
        state_.w_norm_th = std::max(mean_peak * config_.threshold_scale, config_.threshold_floor);
    }
}

uint8_t GaitSegmenter::GetSegmentOutMap(GaitEvent event) const
{
    switch (event)
    {
    case GaitEvent::kHO:
        return 1;
    case GaitEvent::kTO:
        return 2;
    case GaitEvent::kIC:
        return 3;
    case GaitEvent::kFF:
        return 4;
    default:
        return 0;
    }
}

GaitSegmenterOutput GaitSegmenter::BuildOutput(float w_norm, bool is_valid_step) const
{
    GaitSegmenterOutput out;
    out.event = state_.event;
    out.seg_out = GetSegmentOutMap(state_.event);
    out.stride = state_.stride;
    out.w_norm = w_norm;
    out.w_norm_filtered = state_.w_norm_filtered;
    out.gyro_z_filtered = state_.gyro_z_filtered;
    out.w_norm_th = state_.w_norm_th;
    out.peak_w_norm = state_.peak_w_norm;
    out.ff_time = state_.ff_time;
    out.ho_time = state_.ho_time;
    out.to_time = state_.to_time;
    out.ic_time = state_.ic_time;
    out.cycle_ms = state_.cycle_ms;
    out.stance_ms = state_.stance_ms;
    out.swing_ms = state_.swing_ms;
    out.thigh_angle = state_.thigh_angle;
    out.ankle_angle = state_.ankle_angle;
    out.is_valid_step = is_valid_step;
    return out;
}