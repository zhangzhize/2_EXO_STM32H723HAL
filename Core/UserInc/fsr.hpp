#ifndef FSR_HPP
#define FSR_HPP

#include <cstdint>

class Fsr
{
public:
    Fsr(bool is_left = true);
    ~Fsr() = default;

    bool Calibrate(bool do_calibrate);
    bool RefineCalibration(bool do_refinement);
    float Read();
    bool GetGroundContact();
    void GetContactThresholds(float &lower_threshold_percent_ground_contact, float &upper_threshold_percent_ground_contact);
    void SetContactThresholds(float lower_threshold_percent_ground_contact, float upper_threshold_percent_ground_contact);

    bool CalcGroundContact();

    bool is_left_;
    float raw_reading_;
    float calibrated_reading_;

    const uint32_t kCalibrationDurationMs = 5000u;
    uint32_t calibration_start_sys_ms_;
    bool last_do_calibrate_;
    float calibration_min_;
    float calibration_max_;

    const uint8_t num_refinement_steps_ = 7u;
    const float kLowerThresholdPercentCalibrationRefinement = 0.33f;
    const float kUpperThresholdPercentCalibrationRefinement = 0.66f;
    bool state_;
    bool last_do_refinement_;
    float step_max_sum_;
    float step_max_;
    float step_min_sum_;
    float step_min_;
    uint8_t step_count_;
    float calibration_refinement_min_;
    float calibration_refinement_max_;

    bool ground_contact_;
    const uint8_t kGroundStateCountThreshold = 4u;
    float lower_threshold_percent_ground_contact_ = 0.15f;
    float upper_threshold_percent_ground_contact_ = 0.25f;
};

#endif