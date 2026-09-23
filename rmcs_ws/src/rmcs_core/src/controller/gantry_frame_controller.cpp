#include <cmath>

#include <eigen3/Eigen/Core>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller {

// 左摇杆 x 轴控制 pitch 双电机，右摇杆 y 轴控制 yaw 电机。实际速度闭环由
// YAML 中的 RMCS PidController 组件完成。两个开关同时拨到下挡时停止输出速度。
class GantryFrameController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryFrameController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/remote/joystick/right", joystick_right_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);

        register_output("/test/frame_left_motor/target_velocity", left_target_velocity_output_, 0.0);
        register_output(
            "/test/frame_right_motor/target_velocity", right_target_velocity_output_, 0.0);
        register_output("/test/frame_yaw_motor/target_velocity", yaw_target_velocity_output_, 0.0);

        max_velocity_ = 5.0;
        get_parameter("max_velocity", max_velocity_);
        max_velocity_ = std::abs(max_velocity_);
        yaw_max_velocity_ = 5.0;
        get_parameter("yaw_max_velocity", yaw_max_velocity_);
        yaw_max_velocity_ = std::abs(yaw_max_velocity_);

        RCLCPP_INFO(
            get_logger(),
            "[GantryFrameController] initialized (pitch_max=%.3f rad/s, yaw_max=%.3f rad/s, "
            "DR16 control)",
            max_velocity_, yaw_max_velocity_);
    }

    void before_updating() override {
        if (!joystick_left_.ready()) {
            joystick_left_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/joystick/left\". Set to zero.");
        }
        if (!joystick_right_.ready()) {
            joystick_right_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/joystick/right\". Set to zero.");
        }
        if (!switch_left_.ready()) {
            switch_left_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }
    }

    void update() override {
        const bool remote_ok =
            *switch_left_ != rmcs_msgs::Switch::UNKNOWN
            && *switch_right_ != rmcs_msgs::Switch::UNKNOWN;
        const bool both_switches_down =
            *switch_left_ == rmcs_msgs::Switch::DOWN
            && *switch_right_ == rmcs_msgs::Switch::DOWN;
        const bool enabled = remote_ok && !both_switches_down;

        const double pitch_target_velocity = enabled ? joystick_left_->x() * max_velocity_ : 0.0;
        const double yaw_target_velocity = enabled ? joystick_right_->y() * yaw_max_velocity_ : 0.0;

        *left_target_velocity_output_ = pitch_target_velocity;
        *right_target_velocity_output_ = pitch_target_velocity;
        *yaw_target_velocity_output_ = yaw_target_velocity;

        if (enabled != last_enabled_) {
            last_enabled_ = enabled;
            RCLCPP_INFO(
                get_logger(),
                enabled
                    ? "[GantryFrameController] enabled, left stick controls pitch, right stick "
                      "controls yaw"
                    : "[GantryFrameController] disabled, target velocities set to zero");
        }
    }

private:
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    OutputInterface<double> left_target_velocity_output_;
    OutputInterface<double> right_target_velocity_output_;
    OutputInterface<double> yaw_target_velocity_output_;

    double max_velocity_ = 5.0;
    double yaw_max_velocity_ = 5.0;
    bool last_enabled_ = false;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::GantryFrameController, rmcs_executor::Component)
