#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"

#include <chrono>
#include <cmath>

#include <eigen3/Eigen/Core>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller {

class MotorTestSingleController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MotorTestSingleController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        using rmcs_msgs::Switch;

        
        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);


        register_input("/test/motor/velocity", motor_velocity_, false);
        register_output("/test/motor/control_torque", control_torque_, 0.0);

        //可视化
        register_output("/test/motor/target_velocity", target_velocity_output_, 0.0);
        register_output("/test/motor/velocity_error", velocity_error_output_, 0.0);

        max_velocity_ = 5.0;
        get_parameter("max_velocity", max_velocity_);

        // ---- 方波目标速度：目标在 -A 和 +A(rad/s) 之间每 square_wave_period 秒切换一次 ----
        square_wave_enabled_ = true;
        get_parameter("square_wave_enabled", square_wave_enabled_);
        square_wave_amplitude_ = 1.0;
        get_parameter("target_velocity_amplitude", square_wave_amplitude_);
        square_wave_half_period_ = 1.0;
        get_parameter("square_wave_period", square_wave_half_period_);

        
        velocity_pid_.kp = 0.5;
        get_parameter("kp", velocity_pid_.kp);
        velocity_pid_.ki = 0.0;
        get_parameter("ki", velocity_pid_.ki);
        velocity_pid_.kd = 0.0;
        get_parameter("kd", velocity_pid_.kd);

        double integral_limit = 1.0;
        get_parameter("integral_limit", integral_limit);
        velocity_pid_.integral_min = -integral_limit;
        velocity_pid_.integral_max = integral_limit;

        velocity_filter_cutoff_ = 100.0;
        get_parameter("velocity_filter_cutoff", velocity_filter_cutoff_);
        filter_sampling_frequency_ = 1000.0;
        get_parameter("filter_sampling_frequency", filter_sampling_frequency_);

        RCLCPP_INFO(
            get_logger(),
            "[MotorTestSingleController] initialized (max_velocity=%.1f, kp=%.3f, ki=%.4f, "
            "kd=%.4f, vel_filter=%.1fHz)",
            max_velocity_, velocity_pid_.kp, velocity_pid_.ki, velocity_pid_.kd,
            velocity_filter_cutoff_);
    }

    void before_updating() override {
        if (!joystick_left_.ready()) {
            joystick_left_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/joystick/left\". Set to zero.");
        }
        if (!switch_left_.ready()) {
            switch_left_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }
        if (!motor_velocity_.ready()) {
            motor_velocity_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/test/motor/velocity\". Set to 0.");
        }
    }

    void update() override {
        get_parameter("max_velocity", max_velocity_);
        get_parameter("square_wave_enabled", square_wave_enabled_);
        get_parameter("target_velocity_amplitude", square_wave_amplitude_);
        get_parameter("square_wave_period", square_wave_half_period_);
        get_parameter("kp", velocity_pid_.kp);
        get_parameter("ki", velocity_pid_.ki);
        get_parameter("kd", velocity_pid_.kd);
        get_parameter("velocity_filter_cutoff", velocity_filter_cutoff_);
        get_parameter("filter_sampling_frequency", filter_sampling_frequency_);

        if (filter_sampling_frequency_ <= 0.0) // 防除零
            filter_sampling_frequency_ = 1000.0;
        if (square_wave_half_period_ <= 0.0)   // 防除零
            square_wave_half_period_ = 0.001;

        
        const double measured_velocity = filter_measurement(
            velocity_filter_, velocity_filter_active_, velocity_filter_cutoff_, *motor_velocity_);

        // 双下停止
        const bool remote_ok =
            *switch_left_ != rmcs_msgs::Switch::UNKNOWN
            && *switch_right_ != rmcs_msgs::Switch::UNKNOWN;
        const bool both_switches_down =
            *switch_left_ == rmcs_msgs::Switch::DOWN
            && *switch_right_ == rmcs_msgs::Switch::DOWN;
        const bool enabled = remote_ok && !both_switches_down;

        double torque = 0.0;
        double target_velocity = 0.0;
        double velocity_error = 0.0;
        if (enabled) {
            if (square_wave_enabled_) {
                // 方波目标速度：在 -A 和 +A(rad/s) 之间切换(参考 motor_test_double)
                const auto now = std::chrono::steady_clock::now();
                if (!wave_started_) {
                    wave_started_ = true;
                    wave_start_time_ = now;
                }

                const double elapsed = std::chrono::duration<double>(now - wave_start_time_).count();
                const double cycle = 2.0 * square_wave_half_period_;
                target_velocity = std::fmod(elapsed, cycle) < square_wave_half_period_
                                      ? -square_wave_amplitude_
                                      : square_wave_amplitude_;
            } else {
                // 左摇杆 y(前/后) 映射目标速度
                target_velocity = joystick_left_->y() * max_velocity_;
            }
            velocity_error = target_velocity - measured_velocity;
            torque = velocity_pid_.update(velocity_error);
        } else {
            velocity_pid_.reset();
        }

        *control_torque_ = torque;
        *target_velocity_output_ = target_velocity;
        *velocity_error_output_ = velocity_error;

        if (enabled != last_enabled_) {
            last_enabled_ = enabled;
            RCLCPP_INFO(
                get_logger(),
                enabled ? "[MotorTestSingleController] enabled (left stick controls motor speed)"
                        : "[MotorTestSingleController] disabled (both switches down or no remote)");
        }
    }

private:
    double filter_measurement(
        filter::LowPassFilter<1>& filter, bool& active, double cutoff_hz, double raw) {
        if (cutoff_hz > 0.0) {
            filter.set_cutoff(cutoff_hz, filter_sampling_frequency_);
            if (!active) {
                filter.reset();
                active = true;
            }
            return filter.update(raw);
        }
        active = false;
        return raw;
    }

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<double> motor_velocity_;
    OutputInterface<double> control_torque_;
    OutputInterface<double> target_velocity_output_;
    OutputInterface<double> velocity_error_output_;

    pid::PidCalculator velocity_pid_;  

    filter::LowPassFilter<1> velocity_filter_{1.0};  // 速度反馈低通
    bool velocity_filter_active_ = false;
    double velocity_filter_cutoff_ = 100.0;
    double filter_sampling_frequency_ = 1000.0;

    double max_velocity_ = 5.0;

    bool square_wave_enabled_ = false;
    double square_wave_amplitude_ = 1.0;
    double square_wave_half_period_ = 1.0;
    bool wave_started_ = false;
    std::chrono::steady_clock::time_point wave_start_time_{};

    bool last_enabled_ = false;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::MotorTestSingleController, rmcs_executor::Component)
