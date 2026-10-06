#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Core>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller {

// 启动后先将左右 pitch 电机分别向顶点方向转动。每一侧检测到堵转后停止，
// 两侧均到顶后进入遥控模式：左摇杆 x 控制 pitch，右摇杆 y 控制 yaw。
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

        register_input("/test/frame_left_motor/velocity", left_motor_velocity_, false);
        register_input("/test/frame_left_motor/torque", left_motor_torque_, false);
        register_input("/test/frame_right_motor/velocity", right_motor_velocity_, false);
        register_input("/test/frame_right_motor/torque", right_motor_torque_, false);

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

        homing_velocity_ = 4.0;
        get_parameter("homing_velocity", homing_velocity_);
        stall_velocity_threshold_ = 0.2;
        get_parameter("stall_velocity_threshold", stall_velocity_threshold_);
        stall_velocity_threshold_ = std::abs(stall_velocity_threshold_);
        stall_torque_threshold_ = 0.15;
        get_parameter("stall_torque_threshold", stall_torque_threshold_);
        stall_torque_threshold_ = std::abs(stall_torque_threshold_);
        stall_ticks_ = 200;
        get_parameter("stall_ticks", stall_ticks_);
        stall_ticks_ = std::max(1, stall_ticks_);

        RCLCPP_INFO(
            get_logger(),
            "[GantryFrameController] initialized (homing_velocity=%.3f rad/s, "
            "stall_velocity=%.3f rad/s, stall_torque=%.3f Nm, stall_ticks=%d)",
            homing_velocity_, stall_velocity_threshold_, stall_torque_threshold_, stall_ticks_);
    }

    void before_updating() override {
        bind_zero_if_needed(joystick_left_, "/remote/joystick/left");
        bind_zero_if_needed(joystick_right_, "/remote/joystick/right");
        bind_unknown_if_needed(switch_left_, "/remote/switch/left");
        bind_unknown_if_needed(switch_right_, "/remote/switch/right");
        bind_zero_if_needed(left_motor_velocity_, "/test/frame_left_motor/velocity");
        bind_zero_if_needed(left_motor_torque_, "/test/frame_left_motor/torque");
        bind_zero_if_needed(right_motor_velocity_, "/test/frame_right_motor/velocity");
        bind_zero_if_needed(right_motor_torque_, "/test/frame_right_motor/torque");
    }

    void update() override {
        if (stage_ == Stage::kHoming)
            update_homing();
        else
            update_remote_control();
    }

private:
    enum class Stage { kHoming, kRemoteControl };

    void bind_zero_if_needed(InputInterface<Eigen::Vector2d>& input, const char* name) {
        if (!input.ready()) {
            input.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(get_logger(), "Failed to fetch \"%s\". Set to zero.", name);
        }
    }

    void bind_zero_if_needed(InputInterface<double>& input, const char* name) {
        if (!input.ready()) {
            input.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"%s\". Set to zero.", name);
        }
    }

    void bind_unknown_if_needed(
        InputInterface<rmcs_msgs::Switch>& input, const char* name) {
        if (!input.ready()) {
            input.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"%s\". Set to UNKNOWN.", name);
        }
    }

    void update_homing() {
        *left_target_velocity_output_ = left_homed_ ? 0.0 : homing_velocity_;
        *right_target_velocity_output_ = right_homed_ ? 0.0 : homing_velocity_;
        *yaw_target_velocity_output_ = 0.0;

        update_stall_detection(
            *left_motor_velocity_, *left_motor_torque_, left_homed_, left_stall_ticks_, "left");
        update_stall_detection(
            *right_motor_velocity_, *right_motor_torque_, right_homed_, right_stall_ticks_, "right");

        if (left_homed_ && right_homed_) {
            stage_ = Stage::kRemoteControl;
            RCLCPP_INFO(
                get_logger(),
                "[GantryFrameController] both pitch motors reached the top; remote control enabled");
        }
    }

    void update_stall_detection(
        double velocity, double torque, bool& homed, int& stall_ticks, const char* motor_name) {
        if (homed)
            return;

        const bool stalled = std::abs(velocity) < stall_velocity_threshold_
                             && std::abs(torque) > stall_torque_threshold_;
        stall_ticks = stalled ? stall_ticks + 1 : 0;

        if (stall_ticks >= stall_ticks_) {
            homed = true;
            stall_ticks = 0;
            RCLCPP_INFO(
                get_logger(), "[GantryFrameController] %s pitch motor stall detected; stopped",
                motor_name);
        }
    }

    void update_remote_control() {
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

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    InputInterface<double> left_motor_velocity_;
    InputInterface<double> left_motor_torque_;
    InputInterface<double> right_motor_velocity_;
    InputInterface<double> right_motor_torque_;

    OutputInterface<double> left_target_velocity_output_;
    OutputInterface<double> right_target_velocity_output_;
    OutputInterface<double> yaw_target_velocity_output_;

    Stage stage_ = Stage::kHoming;
    bool left_homed_ = false;
    bool right_homed_ = false;
    int left_stall_ticks_ = 0;
    int right_stall_ticks_ = 0;

    double max_velocity_ = 5.0;
    double yaw_max_velocity_ = 5.0;
    double homing_velocity_ = 4.0;
    double stall_velocity_threshold_ = 0.2;
    double stall_torque_threshold_ = 0.15;
    int stall_ticks_ = 200;
    bool last_enabled_ = false;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::GantryFrameController, rmcs_executor::Component)
