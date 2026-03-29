#include "fsr.hpp"
#include "utils.h"
#include <cmath>

bool Fsr::SchmittTrigger(float value, bool is_high, float lower_threshold, float upper_threshold)
{
    bool trigger = 0;
    if (is_high)
    {
        trigger = value > lower_threshold;
    }
    else
    {
        trigger = value > upper_threshold;
    }
    return trigger;
}

// ----------------------------- Fsr ---------------------------------
Fsr::Fsr(bool is_left) : is_left_(is_left)
{
    raw_reading_ = 0.0f;
    calibrated_reading_ = 0.0f;

    calibration_start_sys_ms_ = 0;
    last_do_calibrate_ = false;
    calibration_min_ = 0.0f;
    calibration_max_ = 0.0f;

    state_ = false;
    last_do_refinement_ = false;
    step_max_sum_ = 0.0f;
    step_max_ = 0.0f;
    step_min_sum_ = 0.0f;
    step_min_ = 0.0f;
    step_count_ = 0;
    calibration_refinement_min_ = 0.0f;
    calibration_refinement_max_ = 0.0f;

    ground_contact_ = false;
    lower_threshold_percent_ground_contact_ = 0.15f;
    upper_threshold_percent_ground_contact_ = 0.25f;
}


bool Fsr::Calibrate(bool do_calibrate)
{
    if (do_calibrate && !last_do_calibrate_)
    {
        calibration_start_sys_ms_ = GetSysTimeMs();
        calibration_max_ = raw_reading_;
        calibration_min_ = calibration_max_;
    }

    uint32_t delta = GetSysTimeMs() - calibration_start_sys_ms_;

    if ((do_calibrate) && (delta <= kCalibrationDurationMs))
    {
            float current_raw_reading = raw_reading_;
            calibration_max_ = _max(calibration_max_, current_raw_reading);
            calibration_min_ = _min(calibration_min_, current_raw_reading);
    }
    else if (do_calibrate)
    {
        do_calibrate = false;
    }

    last_do_calibrate_ = do_calibrate;

    return do_calibrate;
}

bool Fsr::RefineCalibration(bool do_refinement)
{
    if (do_refinement && !last_do_refinement_)
    {
        step_count_ = 0u;

        step_max_ = (calibration_max_ + calibration_min_) / 2;
        step_min_ = (calibration_max_ + calibration_min_) / 2;

        step_max_sum_ = 0u;
        step_min_sum_ = 0u;
        // state_ = false;  /**< why not? */
    }

    if (do_refinement)
    {
        if (step_count_ < num_refinement_steps_)
        {
            float current_raw_reading = raw_reading_;

            step_max_ = _max(step_max_, current_raw_reading);
            step_min_ = _min(step_min_, current_raw_reading);

            bool last_state = state_;
            float lower = kLowerThresholdPercentCalibrationRefinement * (calibration_max_ - calibration_min_) + calibration_min_;
            float upper = kUpperThresholdPercentCalibrationRefinement * (calibration_max_ - calibration_min_) + calibration_min_;
            state_ = SchmittTrigger(current_raw_reading, last_state, lower, upper);
            
            if (state_ && !last_state)
            {
                step_max_sum_ += step_max_;
                step_min_sum_ += step_min_;

                float mid2 = (calibration_max_ + calibration_min_) / 2.0f;
                step_max_ = mid2;
                step_min_ = mid2;

                step_count_++;
            }
        }
        else
        {
            if (step_count_ > 0)
            {
                calibration_refinement_max_ = step_max_sum_ / num_refinement_steps_;
                calibration_refinement_min_ = step_min_sum_ / num_refinement_steps_;
            }
            else
            {
                calibration_refinement_max_ = calibration_max_;
                calibration_refinement_min_ = calibration_min_;
            }
            do_refinement = false;
        }
    }

    last_do_refinement_ = do_refinement;

    return do_refinement;
}

float Fsr::Read()
{
    /** raw data has been readed by nrf54(uart) */

    if (calibration_refinement_max_ > calibration_refinement_min_ + 1e-6f)
    {
        calibrated_reading_ = (raw_reading_ - calibration_refinement_min_) / (calibration_refinement_max_ - calibration_refinement_min_);
    }
    else if (calibration_max_ > calibration_min_ + 1e-6f)
    {
        calibrated_reading_ = (raw_reading_ - calibration_min_) / (calibration_max_ - calibration_min_);
    }
    else
    {
        calibrated_reading_ = raw_reading_;
    }

    // clamp normalized when calib done
    if (calibration_refinement_max_ > calibration_refinement_min_ + 1e-6f || calibration_max_ > calibration_min_ + 1e-6f)
    {
        if (calibrated_reading_ < 0.0f) calibrated_reading_ = 0.0f;
        if (calibrated_reading_ > 1.0f) calibrated_reading_ = 1.0f;
    }

    CalcGroundContact();

    return calibrated_reading_;
}


bool Fsr::CalcGroundContact()
{
    bool current_state = false;

    // Only when refinement available use schmitt on normalized value
    if (calibration_refinement_max_ > calibration_refinement_min_ + 1e-6f)
    {
        current_state = SchmittTrigger(calibrated_reading_, ground_contact_, lower_threshold_percent_ground_contact_, upper_threshold_percent_ground_contact_);
    }
    ground_contact_ = current_state;
    return ground_contact_;
}

bool Fsr::GetGroundContact()
{
    return ground_contact_;
}

void Fsr::GetContactThresholds(float &lower_threshold_percent_ground_contact, float &upper_threshold_percent_ground_contact)
{
    lower_threshold_percent_ground_contact = lower_threshold_percent_ground_contact_;
    upper_threshold_percent_ground_contact = upper_threshold_percent_ground_contact_;
}

void Fsr::SetContactThresholds(float lower_threshold_percent_ground_contact, float upper_threshold_percent_ground_contact)
{
    lower_threshold_percent_ground_contact_ = lower_threshold_percent_ground_contact;
    upper_threshold_percent_ground_contact_ = upper_threshold_percent_ground_contact;
}
