#include "bike_controller.hpp"

#include "utils.hpp"

static constexpr float TWO_PI = 6.28318530718f;
static constexpr float TURNS_PER_SEC_TO_RAD_PER_SEC = TWO_PI;
static constexpr float RAD_PER_SEC_TO_RPM = 60.0f / TWO_PI;
static constexpr float MICRO_TO_SEC = 1.0e-6f;

BikeController::BikeController() {
}

void BikeController::SetAxes(Axis* pedalAxis, Axis* driveAxis) {
    pedalAxis_ = pedalAxis;
    driveAxis_ = driveAxis;
}

void BikeController::start_bike_controller(void) {
    // Any startup stuff we need to do goes here. it could be configuring the axes / motor controllers

    // Set Limits
    pedalAxis_->motor_.config_.current_lim = 5.0f;          // Amps
    pedalAxis_->controller_.config_.vel_limit = 25.0f;      // turns/s
    pedalAxis_->motor_.config_.calibration_current = 2.0f;  // Amps
    pedalAxis_->motor_.config_.pole_pairs = 7.0f;
    pedalAxis_->motor_.config_.torque_constant = 0.05907f;  // 8.27/kv = 8.27/140
    pedalAxis_->motor_.config_.motor_type = ODriveIntf::MotorIntf::MOTOR_TYPE_HIGH_CURRENT;

    driveAxis_->motor_.config_.current_lim = 5.0f;          // Amps
    driveAxis_->controller_.config_.vel_limit = 25.0f;      // turns/s
    driveAxis_->motor_.config_.calibration_current = 2.0f;  // Amps
    driveAxis_->motor_.config_.pole_pairs = 7.0f;
    driveAxis_->motor_.config_.torque_constant = 0.05907f;  // 8.27/kv = 8.27/140
    driveAxis_->motor_.config_.motor_type = ODriveIntf::MotorIntf::MOTOR_TYPE_HIGH_CURRENT;

    // Encoder Config
    pedalAxis_->encoder_.config_.mode = ODriveIntf::EncoderIntf::MODE_INCREMENTAL;
    pedalAxis_->encoder_.config_.cpr = 8192;

    driveAxis_->encoder_.config_.mode = ODriveIntf::EncoderIntf::MODE_INCREMENTAL;
    driveAxis_->encoder_.config_.cpr = 8192;

    // Motor / Control Config

    pedalAxis_->controller_.config_.control_mode = ODriveIntf::ControllerIntf::CONTROL_MODE_TORQUE_CONTROL;
    pedalAxis_->controller_.config_.input_mode = ODriveIntf::ControllerIntf::INPUT_MODE_PASSTHROUGH;

    driveAxis_->controller_.config_.control_mode = ODriveIntf::ControllerIntf::CONTROL_MODE_TORQUE_CONTROL;
    driveAxis_->controller_.config_.input_mode = ODriveIntf::ControllerIntf::INPUT_MODE_PASSTHROUGH;
}

void BikeController::update_measurements(float delta_t) {
    update_crank_speed(delta_t);
    update_wheel_speed(delta_t);

    update_crank_motor_torque();
    update_wheel_motor_torque();
}

void BikeController::update_crank_motor_torque(void) {
    const float motor_torque =
        pedalAxis_->motor_.config_.torque_constant *
        pedalAxis_->motor_.current_control_.Iq_measured_;

    crank_motor_torque_ = motor_torque * config_.gear_ratio_pedal;
}

void BikeController::update_wheel_motor_torque(void) {
    const float motor_torque =
        driveAxis_->motor_.config_.torque_constant *
        driveAxis_->motor_.current_control_.Iq_measured_;

    wheel_motor_torque_ =
        abs(motor_torque) *
        config_.gear_ratio_drive;
}

void BikeController::update_values(void) {
    const unsigned long now = micros();

    float delta_t =
        (now - _last_update_time) *
        MICRO_TO_SEC;

    _last_update_time = now;

    // 1. Hardware measurements
    update_measurements(delta_t);

    // 2. Derived estimates
    update_rider_torques(delta_t);
    update_drive_torque(delta_t);

    // 3. Existing control calculations
    calculate_input_output_gear_ratio();

    target_wheel_speed_ =
        crank_speed_estimate_ *
        input_output_gear_ratio_;

    target_resistance_torque_ =
        drive_torque_estimate_ *
        input_output_gear_ratio_ *
        -1.0f;
}

void BikeController::run_control_loop(void) {
    if (pedalAxis_ == nullptr || driveAxis_ == nullptr) return;

    // Check for axis errors as this may impact our state
    check_axis_states();

    // Update our values (measured values and targets)
    update_values();

    // State machine
    switch (current_state_) {
        case BIKE_STATE_UNDEFINED: {
            // This is the first time into this so we can do any startup we need and then move to the next state
            start_bike_controller();

            requested_state_ = BIKE_STATE_CALIBRATION;  // Go to Calibration

            pedalAxis_->requested_state_ = ODriveIntf::AxisIntf::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
            driveAxis_->requested_state_ = ODriveIntf::AxisIntf::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
        } break;

        case BIKE_STATE_CALIBRATION: {
            // We just wait for the axes calibration to finish
            bool calibration_done =
                pedalAxis_->current_state_ == ODriveIntf::AxisIntf::AXIS_STATE_IDLE &&
                driveAxis_->current_state_ == ODriveIntf::AxisIntf::AXIS_STATE_IDLE &&
                pedalAxis_->current_state_ != ODriveIntf::AxisIntf::AXIS_STATE_FULL_CALIBRATION_SEQUENCE &&
                driveAxis_->current_state_ != ODriveIntf::AxisIntf::AXIS_STATE_FULL_CALIBRATION_SEQUENCE &&
                pedalAxis_->error_ == ODriveIntf::AxisIntf::ERROR_NONE &&
                driveAxis_->error_ == ODriveIntf::AxisIntf::ERROR_NONE;
            if (calibration_done) {
                // Double check
                // bool ready = pedalAxis_->motor_.is_calibrated_ && pedalAxis_->encoder_.is_ready_ && driveAxis_->motor_.is_calibrated_ && driveAxis_->encoder_.is_ready_;
                // We should now be able to move on
                requested_state_ = BIKE_STATE_IDLE;
            }
        } break;

        case BIKE_STATE_IDLE: {
            // In this state we just look for movement of the pedals
            if (get_cadence_rpm() >= config_.min_cadence) {
                pedalAxis_->requested_state_ = ODriveIntf::AxisIntf::AXIS_STATE_CLOSED_LOOP_CONTROL;
                pedalAxis_->controller_.input_torque_ = 0.0;
                driveAxis_->requested_state_ = ODriveIntf::AxisIntf::AXIS_STATE_CLOSED_LOOP_CONTROL;
                driveAxis_->controller_.input_torque_ = 0.0f;
                requested_state_ = BIKE_STATE_CONTROL;
            } else {
            }
        } break;

        case BIKE_STATE_CONTROL: {
            // We have done all the calculations, so here we decide what to do and set the motor controllers values

            if (should_freewheel()) {
                // We want to set both motors to idle state to freewheel
                pedalAxis_->controller_.input_torque_ = 0.0;
                pedalAxis_->requested_state_ = ODriveIntf::AxisIntf::AXIS_STATE_IDLE;

                driveAxis_->controller_.input_torque_ = 0.0f;
                driveAxis_->requested_state_ = ODriveIntf::AxisIntf::AXIS_STATE_IDLE;
                requested_state_ = BIKE_STATE_IDLE;
            } else {
                // We want to set the targets
                pedalAxis_->controller_.input_torque_ = (target_resistance_torque_ / config_.gear_ratio_pedal);
                driveAxis_->controller_.input_torque_ = 0.0f;
            }
        } break;

        case BIKE_STATE_BRAKING: {
            // We have seen that we are in a braking scenario so we need to decide how much breaking to do and set the negative torque
            requested_state_ = BIKE_STATE_IDLE;  // For now I don't know what to do so am just ignoring this
        } break;

        case BIKE_STATE_ERROR: {
            // Handle errors
        } break;
    }

    update_bike_state();
}

float BikeController::get_cadence_rpm() const {
    return crank_speed_estimate_ * RAD_PER_SEC_TO_RPM;
}

void BikeController::update_crank_speed(float delta_t) {
    std::optional<float> maybe_vel =
        pedalAxis_->encoder_.vel_estimate_.any();

    if (maybe_vel.has_value()) {
        const float motor_speed_rad_s =
            maybe_vel.value() * TURNS_PER_SEC_TO_RAD_PER_SEC;

        float newVal =
            motor_speed_rad_s / config_.gear_ratio_pedal;

        newVal = newVal < 0.0f ? 0.0f : newVal;

        crank_speed_estimate_ =
            config_.cadence_smoothing_alpha * newVal +
            (1.0f - config_.cadence_smoothing_alpha) *
                crank_speed_estimate_;

        if (delta_t > 0.0f) {
            crank_accel_estimate_ =
                (crank_speed_estimate_ -
                 _last_crank_speed_estimate) /
                delta_t;
        }

        _last_crank_speed_estimate =
            crank_speed_estimate_;

        cadence_estimate_ = get_cadence_rpm();
    }
}

void BikeController::update_wheel_speed(float delta_t) {
    std::optional<float> maybe_vel =
        driveAxis_->encoder_.vel_estimate_.any();

    if (maybe_vel.has_value()) {
        // ODrive encoder velocity is in turns/s.
        const float motor_speed_rad_s =
            maybe_vel.value() * TURNS_PER_SEC_TO_RAD_PER_SEC;

        // Convert motor speed to wheel speed through the fixed physical gearbox.
        const float newVal =
            motor_speed_rad_s / config_.gear_ratio_drive;

        wheel_speed_estimate_ =
            config_.wheel_speed_smoothing_alpha * newVal +
            (1.0f - config_.wheel_speed_smoothing_alpha) *
                wheel_speed_estimate_;

        if (delta_t > 0.0f) {
            wheel_accel_estimate_ =
                (wheel_speed_estimate_ -
                 _last_wheel_speed_estimate) /
                delta_t;
        }

        _last_wheel_speed_estimate =
            wheel_speed_estimate_;
    }
}

void BikeController::update_rider_torques(float delta_t) {
    // Positive resistance magnitude for legacy telemetry/control.
    const float raw_resistance_torque =
        std::max(0.0f, -crank_motor_torque_);

    resistance_torque_ =
        config_.torque_smoothing_alpha *
            raw_resistance_torque +
        (1.0f - config_.torque_smoothing_alpha) *
            resistance_torque_;

    const float resistance_delta =
        resistance_torque_ -
        _last_resistance_torque;

    if (delta_t > 0.0f) {
        resistance_torque_gradient_ =
            resistance_delta / delta_t;
    }

    _last_resistance_torque =
        resistance_torque_;

    // Rider torque from crank dynamics.
    const float raw_rider_torque =
        (config_.crank_inertia *
         crank_accel_estimate_) -
        crank_motor_torque_;

    rider_torque_estimate_ =
        config_.torque_smoothing_alpha *
            raw_rider_torque +
        (1.0f - config_.torque_smoothing_alpha) *
            rider_torque_estimate_;

    rider_torque_estimate_ =
        std::max(0.0f,
                 rider_torque_estimate_);

    const float rider_delta =
        rider_torque_estimate_ -
        last_rider_torque;

    if (delta_t > 0.0f) {
        rider_torque_gradient_ =
            rider_delta / delta_t;
    }

    last_rider_torque =
        rider_torque_estimate_;

    const float rider_power =
        rider_torque_estimate_ *
        crank_speed_estimate_;

    rider_power_estimate_ =
        0.5f * rider_power +
        0.5f * rider_power_estimate_;
}

void BikeController::update_drive_torque(float delta_t) {
    const float newVal = wheel_motor_torque_;

    // Low pass filter
    drive_torque_estimate_ = config_.torque_smoothing_alpha * newVal + (1.0f - config_.torque_smoothing_alpha) * drive_torque_estimate_;

    float delta_Torque = drive_torque_estimate_ - _last_drive_torque;
    if (delta_t > 0.0f) {
        drive_torque_gradient_ = delta_Torque / delta_t;
    }

    _last_drive_torque = drive_torque_estimate_;

    drive_power_estimate_ = drive_torque_estimate_ * wheel_speed_estimate_;
}

void BikeController::calculate_input_output_gear_ratio(void) {
    switch (mode_) {
        case ODriveIntf::BikeControllerIntf::BikeMode::BIKE_MODE_AUTO_CADENCE: {
            float cadenceDelta = fabs(get_cadence_rpm() - config_.target_cadence);
            if (cadenceDelta > config_.cadence_tolerance) {
                float newGearRatio = (wheel_speed_estimate_ * RAD_PER_SEC_TO_RPM) / config_.target_cadence;
                input_output_gear_ratio_ = std::clamp(newGearRatio, config_.min_i_o_gear_ratio, config_.max_i_o_gear_ratio);

                if (input_output_gear_ratio_ < 1.0f || input_output_gear_ratio_ > 5.0f) {
                    error_ = ERROR_CONTROLLER_FAILED;
                }
            }
            break;
        }

        case ODriveIntf::BikeControllerIntf::BikeMode::BIKE_MODE_MANUAL: {
            input_output_gear_ratio_ = get_fixed_gear(currentGear_);
            break;
        }

        case ODriveIntf::BikeControllerIntf::BikeMode::BIKE_MODE_AUTO_POWER: {
            float new_gear_ratio = config_.target_power / rider_power_estimate_;
            input_output_gear_ratio_ = std::clamp(new_gear_ratio, config_.min_i_o_gear_ratio, config_.max_i_o_gear_ratio);
            break;
        }

        default: {
            input_output_gear_ratio_ = get_fixed_gear(currentGear_);
            break;
        }
    }
}

bool BikeController::should_freewheel(void) {
    return false;
    // We arent pedaling
    if ((get_cadence_rpm() < config_.min_cadence)) {
        return true;
    }

    // Look for coasting
    if ((crank_speed_estimate_ * input_output_gear_ratio_) < wheel_speed_estimate_) {
        // Is the rider struggling or just reducing cadence
        if ((crank_accel_estimate_ < 0.0f) && (rider_torque_gradient_ >= config_.pedal_torque_gradient_threshold)) {
            // Rider cadence droping and they are having to input more torque, we need to review the gear ratio
            return false;
        } else if ((crank_accel_estimate_ < 0.0f) && (rider_torque_gradient_ <= 0.0f)) {
            // Rider cadence dropping and they are applying less or the same torque, probably just want to coast
            return true;
        } else {
            // Not sure, I think it is best to review the current situation
            return false;
        }
    } else {
        return false;
    }
}

void BikeController::check_axis_states(void) {
    switch (pedalAxis_->error_) {
        case ODriveIntf::AxisIntf::ERROR_NONE:
            break;

        case ODriveIntf::AxisIntf::ERROR_INVALID_STATE:
        case ODriveIntf::AxisIntf::ERROR_MOTOR_FAILED:
        case ODriveIntf::AxisIntf::ERROR_SENSORLESS_ESTIMATOR_FAILED:
        case ODriveIntf::AxisIntf::ERROR_ENCODER_FAILED:
        case ODriveIntf::AxisIntf::ERROR_CONTROLLER_FAILED:
        case ODriveIntf::AxisIntf::ERROR_OVER_TEMP:
        case ODriveIntf::AxisIntf::ERROR_UNKNOWN_POSITION: {
            pedalAxis_->requested_state_ = ODriveIntf::AxisIntf::AXIS_STATE_IDLE;
            driveAxis_->requested_state_ = ODriveIntf::AxisIntf::AXIS_STATE_IDLE;
            current_state_ = BIKE_STATE_ERROR;
            break;
        }

        default:
            break;
    }

    switch (driveAxis_->error_) {
        case ODriveIntf::AxisIntf::ERROR_NONE:
            break;

        case ODriveIntf::AxisIntf::ERROR_INVALID_STATE:
        case ODriveIntf::AxisIntf::ERROR_MOTOR_FAILED:
        case ODriveIntf::AxisIntf::ERROR_SENSORLESS_ESTIMATOR_FAILED:
        case ODriveIntf::AxisIntf::ERROR_ENCODER_FAILED:
        case ODriveIntf::AxisIntf::ERROR_CONTROLLER_FAILED:
        case ODriveIntf::AxisIntf::ERROR_OVER_TEMP:
        case ODriveIntf::AxisIntf::ERROR_UNKNOWN_POSITION: {
            pedalAxis_->requested_state_ = ODriveIntf::AxisIntf::AXIS_STATE_IDLE;
            driveAxis_->requested_state_ = ODriveIntf::AxisIntf::AXIS_STATE_IDLE;
            current_state_ = BIKE_STATE_ERROR;
            break;
        }

        default:
            break;
    }
}

void BikeController::update_bike_state(void) {
    if (error_ != ERROR_NONE) {
        requested_state_ = BIKE_STATE_ERROR;
    }

    switch (requested_state_) {
        case BIKE_STATE_UNDEFINED:
        case BIKE_STATE_CALIBRATION: {
            current_state_ = BIKE_STATE_CALIBRATION;
            break;
        }

        case BIKE_STATE_IDLE: {
            current_state_ = BIKE_STATE_IDLE;
            break;
        }

        case BIKE_STATE_CONTROL: {
            current_state_ = BIKE_STATE_CONTROL;
            break;
        }

        case BIKE_STATE_BRAKING: {
            current_state_ = BIKE_STATE_BRAKING;
            break;
        }

        case BIKE_STATE_ERROR: {
            current_state_ = BIKE_STATE_ERROR;
            break;
        }

        default:
            break;
    }

    check_axis_states();
}