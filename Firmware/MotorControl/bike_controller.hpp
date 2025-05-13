#ifndef __BIKE_CONTROLLER_HPP
#define __BIKE_CONTROLLER_HPP

class BikeController;

#include <autogen/interfaces.hpp>

#include "axis.hpp"

class BikeController : public ODriveIntf::BikeControllerIntf {
   public:
    BikeController(Axis& pedalAxis, Axis& driveAxis);

    void run_control_loop(void);

   public:
    struct Config_t {
        float target_cadence = 70.0f;
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

    Error error_ = ERROR_NONE;
    BikeState current_state_ = BIKE_STATE_UNDEFINED;
    BikeState requested_state_ = BIKE_STATE_CALIBRATION;

    float cadence_estimate_ = 0.0f;
    float cadence_accel_estimate = 0.0f;
    float resistance_torque_ = 0.0f;
    float resistance_torque_gradient = 0.0f;
    float rider_torque_estimate_ = 0.0f;
    float rider_torque_gradient = 0.0f;
    float target_resistance_torque = 0.0f;

    float input_output_gear_ratio = 1.0f;  // 1:1 at the start

    float wheel_speed_estimate = 0.0f;
    float wheel_accel_estimate = 0.0f;
    float target_wheel_speed = 0.0f;
    float drive_torque_estimate_ = 0.0f;
    float drive_torque_gradient = 0.0f;

    Config_t config_;
    Axis& pedalAxis_;
    Axis& driveAxis_;
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
};

#endif