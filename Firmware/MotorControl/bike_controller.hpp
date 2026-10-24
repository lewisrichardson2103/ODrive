#ifndef __BIKE_CONTROLLER_HPP
#define __BIKE_CONTROLLER_HPP

class BikeController;

#include <autogen/interfaces.hpp>

#include "axis.hpp"

#define BIKE_SINGLE_MOTOR_TEST

class ODrive;

class BikeController : public ODriveIntf::BikeControllerIntf {
   public:
    BikeController();

    void SetAxes(ODrive* odrive, Axis* pedalAxis, Axis* driveAxis);
    void run_control_loop(void);

    inline float get_fixed_gear(uint32_t idx) const {
        switch (idx) {
            case 0: return config_.fixed_gear_0;
            case 1: return config_.fixed_gear_1;
            case 2: return config_.fixed_gear_2;
            case 3: return config_.fixed_gear_3;
            case 4: return config_.fixed_gear_4;
            case 5: return config_.fixed_gear_5;
            case 6: return config_.fixed_gear_6;
            case 7: return config_.fixed_gear_7;
            default: return 1.0f;
        }
    }

   public:
    struct Config_t {
        float target_cadence = 70.0f;
        float target_power = 100.0f;  // watts

        float fixed_gear_0 = 1.0f;
        float fixed_gear_1 = 1.3f;
        float fixed_gear_2 = 1.6f;
        float fixed_gear_3 = 2.0f;
        float fixed_gear_4 = 2.5f;
        float fixed_gear_5 = 3.1f;
        float fixed_gear_6 = 3.8f;
        float fixed_gear_7 = 4.0f;

        float engage_cadence = 5.0f;     // rpm
        float disengage_cadence = 2.0f;  // rpm
        float cadence_kp = 0.00000001f;  // ratio/s per RPM
        float cadence_tolerance = 2.0f;  // RPM

        float gear_ratio_pedal = 1.0f;
        float gear_ratio_drive = 1.0f;
        float crank_speed_smoothing_alpha = 0.001f;
        float crank_accel_smoothing_alpha = 0.01f;
        float crank_torque_smoothing_alpha = 1.0f;
        float rider_torque_smoothing_alpha = 0.0001f;
        float wheel_speed_smoothing_alpha = 0.01f;
        float wheel_accel_smoothing_alpha = 0.0001f;
        float wheel_torque_smoothing_alpha = 0.01f;

        float max_i_o_gear_ratio = 4.0f;
        float min_i_o_gear_ratio = 1.0f;
        float max_gear_ratio_rate = 0.0001f;  // ratio units per second

        float pedal_torque_gradient_threshold = 1.0f;  // Nm/s

        float crank_inertia = 0.05f;  // kg m^2, effective inertia referred to crank
        float wheel_inertia = 1.0f;   // kg m^2, effective inertia referred to wheel

        float sync_natural_frequency = 5.0f;  // rad/s
        float sync_damping_ratio = 1.0f;

        float assist_ratio = 0.0f;  // 0 = no assist, 1 = 100% additional assist

        float simulated_wheel_load = 0.25f;  // Nm per rad/s - Only used in single motor testing

        float crank_torque_cutoff_rpm = 8.0f;
        float crank_torque_full_rpm = 15.0f;

        // power management
        bool enable_power_limits = true;
        // Positive battery discharge into the DC bus.
        float max_battery_discharge_power = 40.0f;  // W
        // Maximum power allowed back into the battery.
        // Zero for a bench PSU that cannot absorb regeneration.
        float max_battery_charge_power = 0.0f;  // W
        // Conservative continuous resistor power budget.
        float max_brake_resistor_power = 15.0f;  // W
        // Conservative wheel-drive efficiency estimate.
        float drive_efficiency_estimate = 0.70f;
    };

    struct TaskTimes {
        TaskTimer update;
    };

    ODriveIntf::BikeControllerIntf::Error error_ = ODriveIntf::BikeControllerIntf::Error::ERROR_NONE;
    ODriveIntf::BikeControllerIntf::BikeMode mode_ = ODriveIntf::BikeControllerIntf::BikeMode::BIKE_MODE_AUTO_CADENCE;
    ODriveIntf::BikeControllerIntf::BikeState current_state_ = ODriveIntf::BikeControllerIntf::BikeState::BIKE_STATE_UNDEFINED;
    ODriveIntf::BikeControllerIntf::BikeState requested_state_ = ODriveIntf::BikeControllerIntf::BikeState::BIKE_STATE_CALIBRATION;

    uint32_t currentGear_ = 0;
    float crank_speed_estimate_ = 0.0f;  // rad/s
    float crank_accel_estimate_ = 0.0f;  // rad/s^2
    float cadence_estimate_ = 0.0f;      // rpm

    float rider_torque_estimate_ = 0.0f;
    float rider_torque_gradient_ = 0.0f;
    float rider_power_estimate_ = 0.0f;

    float cadence_error_ = 0.0f;
    float gear_ratio_rate_command_ = 0.0f;

    float input_output_gear_ratio_ = 1.0f;  // 1:1 at the start
    float target_input_output_gear_ratio_ = 1.0f;

    float delta_t_ = 0.0f;

    float sync_kp_ = 0.0f;
    float sync_ki_ = 0.0f;
    float drivetrain_inertia_gain_ = 0.0f;
    float sync_speed_error_ = 0.0f;        // rad/s
    float sync_integral_ = 0.0f;           // Nm
    float sync_proportional_ = 0.0f;       // Nm
    float virtual_torque_request_ = 0.0f;  // Nm, wheel-side virtual torque - raw PI request
    float virtual_torque_max_ = 0.0f;      // Nm, wheel-side rider authority
    float virtual_torque_limited_ = 0.0f;  // Nm, PI request after authority limit

    float human_fraction_ = 1.0f;

    float crank_torque_command_ = 0.0f;  // Nm at crank
    float wheel_torque_command_ = 0.0f;  // Nm at wheel

    float wheel_speed_estimate_ = 0.0f;  // rad/s
    float wheel_accel_estimate_ = 0.0f;  // rad/s^2

    float crank_motor_torque_ = 0.0f;  // Nm at crank after physical gearbox
    float wheel_motor_torque_ = 0.0f;  // Nm at wheel after physical gearbox

    float virtual_wheel_speed_ = 0.0f;

    Config_t config_;
    TaskTimes task_times_;

   public:  // Power management
    struct PowerMonitor_t {
        // DC Bus
        float bus_voltage = 0.0f;      // V
        float battery_current = 0.0f;  // A, Estimated
        float battery_power = 0.0f;    // W, + discharge

        // Individual motor electrical power
        float pedal_electrical_power = 0.0f;  // W
        float wheel_electrical_power = 0.0f;  // W

        // Mechanical power at the pedal and wheel
        float pedal_mechanical_power = 0.0f;  // W
        float wheel_mechanical_power = 0.0f;  // W

        // Brake resistor power
        float brake_resistor_power = 0.0f;  // W

        // Diagnostic
        float power_balance_error = 0.0f;  // W
    };
    PowerMonitor_t power_monitor_;

    // power-limit diagnostics
    float power_limit_factor_ = 1.0f;
    float battery_discharge_power_request_ = 0.0f;
    float generator_absorption_power_request_ = 0.0f;
    float available_regen_sink_power_ = 0.0f;
    bool battery_discharge_limited_ = false;
    bool generator_power_limited_ = false;

   private:
    void start_bike_controller(void);

    void update_measurements(float delta_t);
    void update_power_monitor();
    void update_crank_speed(float delta_t);
#ifdef BIKE_SINGLE_MOTOR_TEST
    void update_simulated_wheel(float delta_t);
#endif
    void update_wheel_speed(float delta_t);
    void update_crank_motor_torque(void);
    void update_wheel_motor_torque(void);
    void update_sync_speed_error(void);
    void update_virtual_drivetrain_pi(float delta_t);
    void update_virtual_torque_authority(void);
    void limit_virtual_torque(void);

    void update_assistance_state(void);

    void update_virtual_torque_commands(void);
    void update_virtual_drivetrain_gains(void);

    void apply_power_limits();
    void reset_power_limits();

    float limit_crank_torque(float requested_torque) const;

    float get_cadence_rpm() const;
    void update_values(void);

    void update_rider_torques(float delta_t);

    void calculate_target_gear_ratio(float delta_t);
    void update_active_gear_ratio(float delta_t);
    void update_auto_cadence(float delta_t);

    void check_axis_states(void);
    void update_bike_state(void);
    void reset_control_state(void);

    bool rider_active(void) const;
    bool rider_stopped(void) const;

    void lowpassfilter(float& value, const float& newVal, const float& alpha);

   private:
    float _last_crank_speed_estimate = 0.0f;
    float last_rider_torque = 0.0f;
    float _last_wheel_speed_estimate = 0.0f;
    unsigned long _last_update_time = 0.0;
    bool virtual_drivetrain_torque_limited = false;

    Axis* pedalAxis_ = nullptr;
    Axis* driveAxis_ = nullptr;
    ODrive* odrive_ = nullptr;
};

#endif