#ifndef GAIT_SEGMENTER_HPP
#define GAIT_SEGMENTER_HPP

#include <cstdint>

// 定义状态机的事件枚举
enum class GaitEvent : uint8_t
{
    kNone = 0,
    kFF = 1, // Foot Flat
    kHO = 2, // Heel Off
    kTO = 3, // Toe Off
    kIC = 4  // Initial Contact
};

// 输入样本结构体 (对应 Python 中的 IMUSample)
struct IMUSample
{
    uint32_t t_ms = 0;
    
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    float thigh_angle_x = 0.0f;
    bool has_thigh_angle = false;

    float foot_angle_x = 0.0f;
    bool has_foot_angle = false;
};

// 算法配置项结构体 (对应 Python 中 __init__ 的参数)
struct GaitSegmenterConfig {
    float a = 0.15f;
    float w_norm_th = 0.61f;
    uint32_t event_refract_ms = 120;
    uint32_t cycle_min_ms = 300;
    uint32_t cycle_max_ms = 2000;
    uint8_t calib_steps = 5;
    float threshold_scale = 0.6f;
    float threshold_floor = 0.61f;
    bool use_strict_ic_crossing = true;
    float gz_zero_deadband = 0.03f;
    float to_negative_gz_th = 0.05f;
    uint32_t to_min_duration_ms = 80;
    uint32_t ic_min_duration_ms = 80;
    uint32_t ic_confirm_samples = 1;
};

// 算法内部状态结构体
struct GaitState {
    GaitEvent event = GaitEvent::kNone;
    uint32_t stride = 0;

    float w_norm_th = 0.61f;
    float peak_w_norm = 0.0f;
    
    // 采用静态数组替代 Python 的 List，避免动态内存分配
    static constexpr uint8_t kMaxCalibSteps = 10;
    float calibration_peaks[kMaxCalibSteps] = {0};
    uint8_t num_calib_peaks = 0;

    uint32_t last_event_ms = 0;
    bool has_last_event_ms = false;

    uint32_t last_sample_ms = 0;
    bool has_last_sample_ms = false;

    uint32_t ff_time = 0;
    uint32_t ho_time = 0;
    uint32_t to_time = 0;
    uint32_t ic_time = 0;

    uint32_t cycle_ms = 0;
    uint32_t stance_ms = 0;
    uint32_t swing_ms = 0;

    float w_norm_filtered = 0.0f;
    bool has_w_norm_filtered = false;

    float gyro_z_filtered = 0.0f;
    bool has_gyro_z_filtered = false;

    float prev_gyro_z_filtered = 0.0f;
    bool has_prev_gyro_z_filtered = false;

    uint32_t ic_positive_count = 0;

    float thigh_angle = 0.0f;
    float ankle_angle = 0.0f;
};

// 算法输出结构体 (替代 Python 里的 Return Dict)
struct GaitSegmenterOutput {
    GaitEvent event;
    uint8_t seg_out; // 1:HO, 2:TO, 3:IC, 4:FF
    uint32_t stride;
    float w_norm;
    float w_norm_filtered;
    float gyro_z_filtered;
    float w_norm_th;
    float peak_w_norm;
    
    uint32_t ff_time;
    uint32_t ho_time;
    uint32_t to_time;
    uint32_t ic_time;
    
    uint32_t cycle_ms;
    uint32_t stance_ms;
    uint32_t swing_ms;
    
    float thigh_angle;
    float ankle_angle;
    bool is_valid_step;
};

// 核心估算器类
class GaitSegmenter {
public:
    explicit GaitSegmenter(const GaitSegmenterConfig& config = GaitSegmenterConfig());
    ~GaitSegmenter() = default;

    void Reset();
    GaitSegmenterOutput Update(const IMUSample& sample);
    
    // Getter for read-only access to state
    const GaitState& GetState() const { return state_; }

private:
    GaitSegmenterConfig config_;
    GaitState state_;

    // 私有辅助方法
    void Transition(GaitEvent to_event, uint32_t t_ms);
    bool IsInRefractory(uint32_t now_ms) const;
    float LowPass(bool has_prev, float prev, float x) const;
    void ComputeCycleMetrics();
    void UpdateThresholdByCalibration(float peak_w_norm);
    GaitSegmenterOutput BuildOutput(float w_norm, bool is_valid_step) const;
    uint8_t GetSegmentOutMap(GaitEvent event) const;
};

#endif // GAIT_SEGMENTER_HPP