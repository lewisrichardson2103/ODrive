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

        float min_cadence = 5.0f;
        float gear_ratio_pedal = 18.0f;
        float gear_ratio_drive = 1.0f;
        float cadence_smoothing_alpha = 1.0f;
        float wheel_speed_smoothing_alpha = 1.0f;
        float torque_smoothing_alpha = 1.0f;
        float max_i_o_gear_ratio = 4.0f;
        float min_i_o_gear_ratio = 1.0f;
        float cadence_tolerance = 5.0f;
        float pedal_torque_gradient_threshold = 1.0f;  // Nm/s
    };

    struct TaskTimes {
        TaskTimer update;
    };

    ODriveIntf::BikeControllerIntf::Error error_ = ODriveIntf::BikeControllerIntf::Error::ERROR_NONE;
    ODriveIntf::BikeControllerIntf::BikeMode mode_ = ODriveIntf::BikeControllerIntf::BikeMode::BIKE_MODE_AUTO_CADENCE;
    ODriveIntf::BikeControllerIntf::BikeState current_state_ = ODriveIntf::BikeControllerIntf::BikeState::BIKE_STATE_UNDEFINED;
    ODriveIntf::BikeControllerIntf::BikeState requested_state_ = ODriveIntf::BikeControllerIntf::BikeState::BIKE_STATE_CALIBRATION;

    uint32_t currentGear_ = 0;
    float cadence_estimate_ = 0.0f;
    float cadence_accel_estimate_ = 0.0f;
    float resistance_torque_ = 0.0f;
    float resistance_torque_gradient_ = 0.0f;
    float rider_torque_estimate_ = 0.0f;
    float rider_torque_gradient_ = 0.0f;
    float rider_power_estimate_ = 0.0f;
    float target_resistance_torque_ = 0.0f;

    float input_output_gear_ratio_ = 1.0f;  // 1:1 at the start

    float wheel_speed_estimate_ = 0.0f;
    float wheel_accel_estimate_ = 0.0f;
    float target_wheel_speed_ = 0.0f;
    float drive_torque_estimate_ = 0.0f;
    float drive_torque_gradient_ = 0.0f;
    float drive_power_estimate_ = 0.0f;

    Config_t config_;
    TaskTimes task_times_;

   private:
    void start_bike_controller(void);

    void update_values(void);
    void update_cadence(float delta_t);
    void update_rider_torques(float delta_t);
    void calculate_input_output_gear_ratio(void);
    void update_wheel_speed(float delta_t);
    void update_drive_torque(float delta_t);

    void check_axis_states(void);
    void update_bike_state(void);

    bool should_freewheel(void);

   private:
    float _last_cadence_estimate = 0.0f;
    float _last_resistance_torque = 0.0f;
    float last_rider_torque = 0.0f;
    float _last_drive_torque = 0.0f;
    float _last_wheel_speed_estimate = 0.0f;
    unsigned long _last_update_time = 0.0;

    Axis* pedalAxis_ = nullptr;
    Axis* driveAxis_ = nullptr;
};

#endif