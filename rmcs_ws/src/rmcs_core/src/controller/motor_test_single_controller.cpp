#include "filter/low_pass_filter.hpp"

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


        register_output("/test/motor/target_velocity", target_velocity_output_, 0.0);
        register_output("/test/motor/measured_velocity", measured_velocity_output_, 0.0);
        register_output("/test/motor/velocity_error", velocity_error_output_, 0.0);

        max_velocity_ = 5.0;
        get_parameter("max_velocity", max_velocity_);

        fixed_velocity_enabled_ = false;
        get_parameter("fixed_velocity_enabled", fixed_velocity_enabled_);
        target_velocity_ = 0.0;
        get_parameter("target_velocity", target_velocity_);

        velocity_filter_cutoff_ = 100.0;
        get_parameter("velocity_filter_cutoff", velocity_filter_cutoff_);
        filter_sampling_frequency_ = 1000.0;
        get_parameter("filter_sampling_frequency", filter_sampling_frequency_);
        if (filter_sampling_frequency_ <= 0.0) // 防除零
            filter_sampling_frequency_ = 1000.0;

        RCLCPP_INFO(
            get_logger(),
            "[MotorTestSingleController] initialized (max_velocity=%.1f, vel_filter=%.1fHz)",
            max_velocity_, velocity_filter_cutoff_);
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
        const double measured_velocity = filter_measurement(
            velocity_filter_, velocity_filter_active_, velocity_filter_cutoff_, *motor_velocity_);

        const bool remote_ok =
            *switch_left_ != rmcs_msgs::Switch::UNKNOWN
            && *switch_right_ != rmcs_msgs::Switch::UNKNOWN;
        const bool both_switches_down =
            *switch_left_ == rmcs_msgs::Switch::DOWN
            && *switch_right_ == rmcs_msgs::Switch::DOWN;
        const bool enabled = remote_ok && !both_switches_down;

        double target_velocity = 0.0;
        if (enabled) {
            if (fixed_velocity_enabled_) {
                // 固定转速模式：按 target_velocity(rad/s) 恒速转动，方便调试
                target_velocity = target_velocity_;
            } else {
                // 左摇杆 y(前/后) 映射目标速度
                target_velocity = joystick_left_->y() * max_velocity_;
            }
        }

        // 目标速度(供 PID)与过滤后的测量速度(供 PID)，速度误差仅用于可视化
        *target_velocity_output_ = target_velocity;
        *measured_velocity_output_ = measured_velocity;
        *velocity_error_output_ = target_velocity - measured_velocity;

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
    OutputInterface<double> target_velocity_output_;
    OutputInterface<double> measured_velocity_output_;
    OutputInterface<double> velocity_error_output_;

    filter::LowPassFilter<1> velocity_filter_{1.0}; // 速度反馈低通
    bool velocity_filter_active_ = false;
    double velocity_filter_cutoff_ = 100.0;
    double filter_sampling_frequency_ = 1000.0;

    double max_velocity_ = 5.0;

    bool fixed_velocity_enabled_ = false;
    double target_velocity_ = 0.0;

    bool last_enabled_ = false;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::MotorTestSingleController, rmcs_executor::Component)
