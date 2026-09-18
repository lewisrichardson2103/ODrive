#ifndef __BIKE_CONTROLLER_HPP
#define __BIKE_CONTROLLER_HPP

class BikeController;

#include <autogen/interfaces.hpp>

#include "axis.hpp"

class BikeController : public ODriveIntf::BikeControllerIntf {
   public:
    BikeController();

    void SetAxes(Axis* pedalAxis, Axis* driveAxis);
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

        float gear_ratio_pedal = 1.0f;
        float gear_ratio_drive = 1.0f;
        float cadence_smoothing_alpha = 1.0f;
        float wheel_speed_smoothing_alpha = 1.0f;
        float torque_smoothing_alpha = 1.0f;
        float max_i_o_gear_ratio = 4.0f;
        float min_i_o_gear_ratio = 1.0f;
        float max_gear_ratio_rate = 1.0f;  // ratio units per second

        float cadence_tolerance = 5.0f;
        float pedal_torque_gradient_threshold = 1.0f;  // Nm/s

        float crank_inertia = 0.05f;  // kg m^2, effective inertia referred to crank
        float wheel_inertia = 1.0f;   // kg m^2, effective inertia referred to wheel

        float sync_natural_frequency = 5.0f;  // rad/s
        float sync_damping_ratio = 1.0f;

        float assist_ratio = 0.0f;  // 0 = no assist, 1 = 100% additional assist

        float simulated_wheel_load = 1.0f;  // Nm - Only used in single motor testing
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

    float input_output_gear_ratio_ = 1.0f;  // 1:1 at the start
    float target_input_output_gear_ratio_ = 1.0f;

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

    float virtual_wheel_speed_ = 0.0f;

    Config_t config_;
    TaskTimes task_times_;

   private:
    void start_bike_controller(void);

    void update_measurements(float delta_t);
    void update_crank_speed(float delta_t);
    void update_simulated_wheel(float delta_t);
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

    float get_cadence_rpm() const;
    void update_values(void);

    void update_rider_torques(float delta_t);

    void calculate_target_gear_ratio(void);
    void update_active_gear_ratio(float delta_t);

    void check_axis_states(void);
    void update_bike_state(void);
    void reset_control_state(void);

    bool rider_active(void) const;
    bool rider_stopped(void) const;

   private:
    float _last_crank_speed_estimate = 0.0f;
    float last_rider_torque = 0.0f;
    float _last_wheel_speed_estimate = 0.0f;
    unsigned long _last_update_time = 0.0;

    float crank_motor_torque_ = 0.0f;  // Nm at crank after physical gearbox
    float wheel_motor_torque_ = 0.0f;  // Nm at wheel after physical gearbox

    Axis* pedalAxis_ = nullptr;
    Axis* driveAxis_ = nullptr;
};

#endif