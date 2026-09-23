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
#ifdef BIKE_SINGLE_MOTOR_TEST
    driveAxis_ = nullptr;
#else
    driveAxis_ = driveAxis;
#endif
}

/*
NOTE: Encoder config has been removed from here as it should persist on the O-Drive
This should be done once to configure it:
    odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
    odrv0.axis0.encoder.config.cpr = 16384
    odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 3

    odrv0.axis1.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
    odrv0.axis1.encoder.config.cpr = 16384
    odrv0.axis1.encoder.config.abs_spi_cs_gpio_pin = 4

    odrv0.save_configuration()
    odrv0.reboot()
*/

void BikeController::start_bike_controller(void) {
    // Any startup stuff we need to do goes here. it could be configuring the axes / motor controllers

    // Set Limits
    pedalAxis_->motor_.config_.current_lim = 5.0f;          // Amps
    pedalAxis_->controller_.config_.vel_limit = 25.0f;      // turns/s
    pedalAxis_->motor_.config_.calibration_current = 2.0f;  // Amps
    pedalAxis_->motor_.config_.pole_pairs = 7.0f;
    pedalAxis_->motor_.config_.torque_constant = 0.05907f;  // 8.27/kv = 8.27/140
    pedalAxis_->motor_.config_.motor_type = ODriveIntf::MotorIntf::MOTOR_TYPE_HIGH_CURRENT;

#ifndef BIKE_SINGLE_MOTOR_TEST
    driveAxis_->motor_.config_.current_lim = 5.0f;          // Amps
    driveAxis_->controller_.config_.vel_limit = 25.0f;      // turns/s
    driveAxis_->motor_.config_.calibration_current = 2.0f;  // Amps
    driveAxis_->motor_.config_.pole_pairs = 7.0f;
    driveAxis_->motor_.config_.torque_constant = 0.05907f;  // 8.27/kv = 8.27/140
    driveAxis_->motor_.config_.motor_type = ODriveIntf::MotorIntf::MOTOR_TYPE_HIGH_CURRENT;
#endif

    // Motor / Control Config

    pedalAxis_->controller_.config_.control_mode = ODriveIntf::ControllerIntf::CONTROL_MODE_TORQUE_CONTROL;
    pedalAxis_->controller_.config_.input_mode = ODriveIntf::ControllerIntf::INPUT_MODE_PASSTHROUGH;

#ifndef BIKE_SINGLE_MOTOR_TEST
    driveAxis_->controller_.config_.control_mode = ODriveIntf::ControllerIntf::CONTROL_MODE_TORQUE_CONTROL;
    driveAxis_->controller_.config_.input_mode = ODriveIntf::ControllerIntf::INPUT_MODE_PASSTHROUGH;
#endif

    target_input_output_gear_ratio_ =
        get_fixed_gear(currentGear_);

    input_output_gear_ratio_ =
        target_input_output_gear_ratio_;
}

void BikeController::update_measurements(float delta_t) {
    update_crank_speed(delta_t);
    update_crank_motor_torque();

#ifdef BIKE_SINGLE_MOTOR_TEST
    update_simulated_wheel(delta_t);
#else
    update_wheel_speed(delta_t);
    update_wheel_motor_torque();
#endif
}

#ifdef BIKE_SINGLE_MOTOR_TEST
void BikeController::update_simulated_wheel(float delta_t) {
    if (delta_t <= 0.0f) {
        return;
    }

    const float drive_torque =
        wheel_torque_command_;

    const float load_torque =
        config_.simulated_wheel_load *
        wheel_speed_estimate_;

    const float raw_accel =
        (drive_torque - load_torque) /
        config_.wheel_inertia;

    lowpassfilter(
        wheel_accel_estimate_,
        raw_accel,
        config_.wheel_accel_smoothing_alpha);

    wheel_speed_estimate_ +=
        wheel_accel_estimate_ * delta_t;

    wheel_speed_estimate_ =
        std::clamp(
            wheel_speed_estimate_,
            0.0f,
            50.0f);
}
#endif

void BikeController::update_crank_motor_torque(void) {
    const float measured_motor_torque =
        pedalAxis_->motor_.config_.torque_constant *
        pedalAxis_->motor_.current_control_.Iq_measured_;

    const float geared_motor_torque =
        measured_motor_torque *
        config_.gear_ratio_pedal;
    lowpassfilter(crank_motor_torque_, geared_motor_torque, config_.crank_torque_smoothing_alpha);
}

void BikeController::update_wheel_motor_torque(void) {
    const float measured_motor_torque =
        driveAxis_->motor_.config_.torque_constant *
        driveAxis_->motor_.current_control_.Iq_measured_;

    const float geared_motor_torque =
        measured_motor_torque *
        config_.gear_ratio_drive;
    lowpassfilter(wheel_motor_torque_, geared_motor_torque, config_.wheel_torque_smoothing_alpha);
}

void BikeController::update_sync_speed_error(void) {
    virtual_wheel_speed_ =
        input_output_gear_ratio_ *
        crank_speed_estimate_;

    sync_speed_error_ =
        virtual_wheel_speed_ -
        wheel_speed_estimate_;
}

void BikeController::update_virtual_drivetrain_pi(float delta_t) {
    if (current_state_ != BIKE_STATE_CONTROL) {
        sync_proportional_ = 0.0f;
        virtual_torque_request_ = 0.0f;
        return;
    }

    sync_proportional_ =
        sync_kp_ *
        sync_speed_error_;

    if (delta_t > 0.0f) {
        const float candidate_integral =
            sync_integral_ +
            (sync_ki_ *
             sync_speed_error_ *
             delta_t);

        const float candidate_torque =
            sync_proportional_ +
            candidate_integral;

        const bool pushing_below_lower_limit =
            candidate_torque < 0.0f &&
            sync_speed_error_ < 0.0f;

        const bool pushing_above_upper_limit =
            candidate_torque > virtual_torque_max_ &&
            sync_speed_error_ > 0.0f;

        if (!pushing_below_lower_limit &&
            !pushing_above_upper_limit) {
            sync_integral_ =
                candidate_integral;
        }
    }

    virtual_torque_request_ =
        sync_proportional_ +
        sync_integral_;
}

void BikeController::update_assistance_state(void) {
    const float assist_ratio =
        std::max(0.0f, config_.assist_ratio);

    human_fraction_ =
        1.0f / (1.0f + assist_ratio);
}

void BikeController::update_virtual_torque_authority(void) {
    if (current_state_ != BIKE_STATE_CONTROL) {
        virtual_torque_max_ = 0.0f;
        return;
    }

    if (input_output_gear_ratio_ <= 0.0f) {
        virtual_torque_max_ = 0.0f;
        return;
    }

    const float assist_ratio =
        std::max(0.0f, config_.assist_ratio);

    virtual_torque_max_ =
        ((1.0f + assist_ratio) *
         rider_torque_estimate_) /
        input_output_gear_ratio_;
}

void BikeController::limit_virtual_torque(void) {
    if (current_state_ != BIKE_STATE_CONTROL) {
        virtual_torque_limited_ = 0.0f;
        return;
    }

    virtual_torque_limited_ =
        std::clamp(
            virtual_torque_request_,
            0.0f,
            virtual_torque_max_);
}

void BikeController::update_virtual_torque_commands(void) {
    if (current_state_ != BIKE_STATE_CONTROL) {
        crank_torque_command_ = 0.0f;
        wheel_torque_command_ = 0.0f;
        return;
    }

    wheel_torque_command_ =
        virtual_torque_limited_;

    crank_torque_command_ =
        -human_fraction_ *
        input_output_gear_ratio_ *
        virtual_torque_limited_;
}

void BikeController::update_virtual_drivetrain_gains(void) {
    const float crank_inertia =
        std::max(config_.crank_inertia, 0.0001f);

    const float wheel_inertia =
        std::max(config_.wheel_inertia, 0.0001f);

    const float gear_ratio =
        std::max(input_output_gear_ratio_, 0.0001f);

    drivetrain_inertia_gain_ =
        (human_fraction_ *
         gear_ratio *
         gear_ratio /
         crank_inertia) +
        (1.0f / wheel_inertia);

    const float natural_frequency =
        std::max(
            0.0f,
            config_.sync_natural_frequency);

    const float damping_ratio =
        std::max(
            0.0f,
            config_.sync_damping_ratio);

    sync_kp_ =
        (2.0f *
         damping_ratio *
         natural_frequency) /
        drivetrain_inertia_gain_;

    sync_ki_ =
        (natural_frequency *
         natural_frequency) /
        drivetrain_inertia_gain_;
}

void BikeController::update_values(void) {
    const unsigned long now = micros();

    float delta_t =
        (now - _last_update_time) *
        MICRO_TO_SEC;

    delta_t_ = delta_t;

    _last_update_time = now;

    // 1. Hardware measurements
    update_measurements(delta_t);

    // 2. Derived estimates
    update_rider_torques(delta_t);
    update_assistance_state();

    // 3. Existing control calculations
    calculate_target_gear_ratio(delta_t);

    // 4. Virtual drivetrain measurements
    update_virtual_drivetrain_gains();
    update_sync_speed_error();
    update_virtual_torque_authority();
    update_virtual_drivetrain_pi(delta_t);
    limit_virtual_torque();
    update_virtual_torque_commands();
}

void BikeController::run_control_loop(void) {
#ifdef BIKE_SINGLE_MOTOR_TEST
    if (pedalAxis_ == nullptr) return;
#else
    if (pedalAxis_ == nullptr || driveAxis_ == nullptr) return;
#endif
    // Check for axis errors as this may impact our state
    check_axis_states();

    // Update our values (measured values and targets)
    update_values();

    // State machine
    switch (current_state_) {
        case BIKE_STATE_UNDEFINED: {
            // This is the first time into this so we can do any startup we need and then move to the next state
            start_bike_controller();

            const bool pedal_ready =
                pedalAxis_->motor_.is_calibrated_ &&
                pedalAxis_->encoder_.is_ready_;

#ifndef BIKE_SINGLE_MOTOR_TEST
            const bool drive_ready =
                driveAxis_->motor_.is_calibrated_ &&
                driveAxis_->encoder_.is_ready_;

            const bool calibration_required =
                !pedal_ready || !drive_ready;
#else
            const bool calibration_required =
                !pedal_ready;
#endif

            if (calibration_required) {
                requested_state_ = BIKE_STATE_CALIBRATION;

                if (!pedal_ready) {
                    pedalAxis_->requested_state_ =
                        ODriveIntf::AxisIntf::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
                }

#ifndef BIKE_SINGLE_MOTOR_TEST
                if (!drive_ready) {
                    driveAxis_->requested_state_ =
                        ODriveIntf::AxisIntf::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
                }
#endif
            } else {
                requested_state_ = BIKE_STATE_IDLE;
                reset_control_state();
            }
        } break;

        case BIKE_STATE_CALIBRATION: {
            // We just wait for the axes calibration to finish
            const bool pedal_ready =
                pedalAxis_->motor_.is_calibrated_ &&
                pedalAxis_->encoder_.is_ready_ &&
                pedalAxis_->error_ == ODriveIntf::AxisIntf::ERROR_NONE &&
                pedalAxis_->current_state_ == ODriveIntf::AxisIntf::AXIS_STATE_IDLE;

#ifndef BIKE_SINGLE_MOTOR_TEST
            const bool drive_ready =
                driveAxis_->motor_.is_calibrated_ &&
                driveAxis_->encoder_.is_ready_ &&
                driveAxis_->error_ == ODriveIntf::AxisIntf::ERROR_NONE &&
                driveAxis_->current_state_ == ODriveIntf::AxisIntf::AXIS_STATE_IDLE;

            const bool calibration_done =
                pedal_ready &&
                drive_ready;
#else
            const bool calibration_done =
                pedal_ready;
#endif

            if (calibration_done) {
                requested_state_ = BIKE_STATE_IDLE;
                reset_control_state();
            }
        } break;

        case BIKE_STATE_IDLE: {
            // IDLE means no commanded torque.
            pedalAxis_->controller_.input_torque_ = 0.0f;
#ifndef BIKE_SINGLE_MOTOR_TEST
            driveAxis_->controller_.input_torque_ = 0.0f;
#endif

            if (rider_active()) {
                pedalAxis_->requested_state_ =
                    ODriveIntf::AxisIntf::AXIS_STATE_CLOSED_LOOP_CONTROL;

#ifndef BIKE_SINGLE_MOTOR_TEST
                driveAxis_->requested_state_ =
                    ODriveIntf::AxisIntf::AXIS_STATE_CLOSED_LOOP_CONTROL;
#endif

                requested_state_ = BIKE_STATE_CONTROL;
            } else {
                // Keep axes physically idle until the rider starts pedalling.
                pedalAxis_->requested_state_ =
                    ODriveIntf::AxisIntf::AXIS_STATE_IDLE;

#ifndef BIKE_SINGLE_MOTOR_TEST
                driveAxis_->requested_state_ =
                    ODriveIntf::AxisIntf::AXIS_STATE_IDLE;
#endif
            }
        } break;

        case BIKE_STATE_CONTROL: {
            if (rider_stopped()) {
                pedalAxis_->controller_.input_torque_ = 0.0f;
#ifndef BIKE_SINGLE_MOTOR_TEST
                driveAxis_->controller_.input_torque_ = 0.0f;
#endif

                pedalAxis_->requested_state_ =
                    ODriveIntf::AxisIntf::AXIS_STATE_IDLE;

#ifndef BIKE_SINGLE_MOTOR_TEST
                driveAxis_->requested_state_ =
                    ODriveIntf::AxisIntf::AXIS_STATE_IDLE;
#endif

                requested_state_ = BIKE_STATE_IDLE;
                reset_control_state();
            } else {
                pedalAxis_->controller_.input_torque_ =
                    crank_torque_command_ /
                    config_.gear_ratio_pedal;

#ifndef BIKE_SINGLE_MOTOR_TEST
                driveAxis_->controller_.input_torque_ =
                    wheel_torque_command_ /
                    config_.gear_ratio_drive;
#endif
            }
        } break;

        case BIKE_STATE_BRAKING: {
            // We have seen that we are in a braking scenario so we need to decide how much breaking to do and set the negative torque
            requested_state_ = BIKE_STATE_IDLE;  // For now I don't know what to do so am just ignoring this
            reset_control_state();
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
        lowpassfilter(crank_speed_estimate_, newVal, config_.crank_speed_smoothing_alpha);

        if (delta_t > 0.0f) {
            const float raw_crank_accel =
                (crank_speed_estimate_ -
                 _last_crank_speed_estimate) /
                delta_t;
            lowpassfilter(crank_accel_estimate_, raw_crank_accel, config_.crank_accel_smoothing_alpha);
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
        lowpassfilter(wheel_speed_estimate_, newVal, config_.wheel_speed_smoothing_alpha);

        if (delta_t > 0.0f) {
            float raw_accel = (wheel_speed_estimate_ -
                               _last_wheel_speed_estimate) /
                              delta_t;
            lowpassfilter(wheel_accel_estimate_, raw_accel, config_.wheel_accel_smoothing_alpha);
        }

        _last_wheel_speed_estimate =
            wheel_speed_estimate_;
    }
}

void BikeController::update_rider_torques(float delta_t) {
    const float raw_rider_torque =
        (config_.crank_inertia * crank_accel_estimate_) -
        crank_motor_torque_;

    lowpassfilter(rider_torque_estimate_, raw_rider_torque, config_.rider_torque_smoothing_alpha);

    rider_torque_estimate_ =
        std::max(0.0f, rider_torque_estimate_);

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

void BikeController::calculate_target_gear_ratio(float delta_t) {
    switch (mode_) {
        case ODriveIntf::BikeControllerIntf::BikeMode::BIKE_MODE_AUTO_CADENCE: {
            update_auto_cadence(delta_t);
            break;
        }

        case ODriveIntf::BikeControllerIntf::BikeMode::BIKE_MODE_MANUAL: {
            target_input_output_gear_ratio_ = get_fixed_gear(currentGear_);
            update_active_gear_ratio(delta_t);
            break;
        }

        case ODriveIntf::BikeControllerIntf::BikeMode::BIKE_MODE_AUTO_POWER: {
            float new_gear_ratio = config_.target_power / rider_power_estimate_;
            target_input_output_gear_ratio_ = std::clamp(new_gear_ratio, config_.min_i_o_gear_ratio, config_.max_i_o_gear_ratio);
            update_active_gear_ratio(delta_t);
            break;
        }

        default: {
            target_input_output_gear_ratio_ = get_fixed_gear(currentGear_);
            update_active_gear_ratio(delta_t);
            break;
        }
    }
}

void BikeController::update_auto_cadence(float delta_t) {
    if (delta_t <= 0.0f) {
        return;
    }

    cadence_error_ =
        get_cadence_rpm() -
        config_.target_cadence;

    if (fabs(cadence_error_) <= config_.cadence_tolerance) {
        gear_ratio_rate_command_ = 0.0f;
        return;
    }

    gear_ratio_rate_command_ =
        config_.cadence_kp *
        cadence_error_;

    bool drivetrain_torque_limited =
        virtual_torque_request_ >
        virtual_torque_max_;

    float available_rate = drivetrain_torque_limited ? config_.max_gear_ratio_rate * 0.1f : config_.max_gear_ratio_rate;

    gear_ratio_rate_command_ =
        std::clamp(
            gear_ratio_rate_command_,
            -available_rate,
            available_rate);

    target_input_output_gear_ratio_ +=
        gear_ratio_rate_command_ * delta_t;

    input_output_gear_ratio_ =
        std::clamp(
            target_input_output_gear_ratio_,
            config_.min_i_o_gear_ratio,
            config_.max_i_o_gear_ratio);
}

void BikeController::update_active_gear_ratio(float delta_t) {
    if (delta_t <= 0.0f) {
        return;
    }

    const float max_change =
        config_.max_gear_ratio_rate * delta_t;

    const float difference =
        target_input_output_gear_ratio_ -
        input_output_gear_ratio_;

    const float change =
        std::clamp(
            difference,
            -max_change,
            max_change);

    input_output_gear_ratio_ += change;

    input_output_gear_ratio_ =
        std::clamp(
            input_output_gear_ratio_,
            config_.min_i_o_gear_ratio,
            config_.max_i_o_gear_ratio);
}

bool BikeController::rider_active(void) const {
    return get_cadence_rpm() >= config_.engage_cadence;
}

bool BikeController::rider_stopped(void) const {
    return get_cadence_rpm() <= config_.disengage_cadence;
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
#ifndef BIKE_SINGLE_MOTOR_TEST
            driveAxis_->requested_state_ = ODriveIntf::AxisIntf::AXIS_STATE_IDLE;
#endif
            current_state_ = BIKE_STATE_ERROR;
            break;
        }

        default:
            break;
    }

#ifndef BIKE_SINGLE_MOTOR_TEST
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
#endif
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

void BikeController::reset_control_state(void) {
    sync_speed_error_ = 0.0f;
    sync_integral_ = 0.0f;
    sync_proportional_ = 0.0f;
    virtual_torque_request_ = 0.0f;
    virtual_torque_max_ = 0.0f;
    virtual_torque_limited_ = 0.0f;

    crank_torque_command_ = 0.0f;
    wheel_torque_command_ = 0.0f;

#ifdef BIKE_SINGLE_MOTOR_TEST
    wheel_speed_estimate_ = 0.0f;
    wheel_accel_estimate_ = 0.0f;
    _last_wheel_speed_estimate = 0.0f;
#endif
}

void BikeController::lowpassfilter(float& oldVal, const float& newVal, const float& alpha) {
    oldVal += alpha * (newVal - oldVal);
}