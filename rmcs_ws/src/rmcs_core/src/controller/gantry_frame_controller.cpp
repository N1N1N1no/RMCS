#include <algorithm>
#include <cmath>
#include <numbers>

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
        register_input("/test/frame_left_motor/angle", left_motor_angle_, false);
        register_input("/test/frame_right_motor/velocity", right_motor_velocity_, false);
        register_input("/test/frame_right_motor/torque", right_motor_torque_, false);
        register_input("/test/frame_right_motor/angle", right_motor_angle_, false);

        register_output("/test/frame_left_motor/target_velocity", left_target_velocity_output_, 0.0);
        register_output(
            "/test/frame_right_motor/target_velocity", right_target_velocity_output_, 0.0);
        register_output("/test/frame_yaw_motor/target_velocity", yaw_target_velocity_output_, 0.0);
        register_output("/test/gantry/pitch_sync_error", sync_error_output_, 0.0);
        register_output("/test/gantry/pitch_sync_correction", sync_correction_output_, 0.0);

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

        screw_lead_mm_ = 2.0;
        get_parameter("screw_lead_mm", screw_lead_mm_);
        if (screw_lead_mm_ <= 0.0) {
            screw_lead_mm_ = 2.0;
            RCLCPP_WARN(get_logger(), "Invalid screw_lead_mm, fallback to 2.0 mm");
        }
        sync_kp_ = 5.0;
        get_parameter("sync_kp", sync_kp_);
        sync_kp_ = std::abs(sync_kp_);
        sync_kd_ = 0.0;
        get_parameter("sync_kd", sync_kd_);
        sync_kd_ = std::abs(sync_kd_);
        sync_max_velocity_ = 2.0;
        get_parameter("sync_max_velocity", sync_max_velocity_);
        sync_max_velocity_ = std::abs(sync_max_velocity_);
        sync_deadband_mm_ = 0.1;
        get_parameter("sync_deadband_mm", sync_deadband_mm_);
        sync_deadband_mm_ = std::abs(sync_deadband_mm_);

        RCLCPP_INFO(
            get_logger(),
            "[GantryFrameController] initialized (homing_velocity=%.3f rad/s, "
            "stall_velocity=%.3f rad/s, stall_torque=%.3f Nm, stall_ticks=%d, "
            "sync_kp=%.3f, sync_kd=%.3f, sync_max=%.3f rad/s, sync_deadband=%.3f mm)",
            homing_velocity_, stall_velocity_threshold_, stall_torque_threshold_, stall_ticks_,
            sync_kp_, sync_kd_, sync_max_velocity_, sync_deadband_mm_);
    }

    void before_updating() override {
        bind_zero_if_needed(joystick_left_, "/remote/joystick/left");
        bind_zero_if_needed(joystick_right_, "/remote/joystick/right");
        bind_unknown_if_needed(switch_left_, "/remote/switch/left");
        bind_unknown_if_needed(switch_right_, "/remote/switch/right");
        bind_zero_if_needed(left_motor_velocity_, "/test/frame_left_motor/velocity");
        bind_zero_if_needed(left_motor_torque_, "/test/frame_left_motor/torque");
        bind_zero_if_needed(left_motor_angle_, "/test/frame_left_motor/angle");
        bind_zero_if_needed(right_motor_velocity_, "/test/frame_right_motor/velocity");
        bind_zero_if_needed(right_motor_torque_, "/test/frame_right_motor/torque");
        bind_zero_if_needed(right_motor_angle_, "/test/frame_right_motor/angle");
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
        *sync_error_output_ = 0.0;
        *sync_correction_output_ = 0.0;

        update_stall_detection(
            *left_motor_velocity_, *left_motor_torque_, *left_motor_angle_, left_homed_,
            left_stall_ticks_, left_zero_angle_, "left");
        update_stall_detection(
            *right_motor_velocity_, *right_motor_torque_, *right_motor_angle_, right_homed_,
            right_stall_ticks_, right_zero_angle_, "right");

        if (left_homed_ && right_homed_) {
            stage_ = Stage::kRemoteControl;
            RCLCPP_INFO(
                get_logger(),
                "[GantryFrameController] both pitch motors reached the top; remote control enabled");
        }
    }

    void update_stall_detection(
        double velocity, double torque, double angle, bool& homed, int& stall_ticks,
        double& zero_angle, const char* motor_name) {
        if (homed)
            return;

        const bool stalled = std::abs(velocity) < stall_velocity_threshold_
                             && std::abs(torque) > stall_torque_threshold_;
        stall_ticks = stalled ? stall_ticks + 1 : 0;

        if (stall_ticks >= stall_ticks_) {
            zero_angle = angle;
            homed = true;
            stall_ticks = 0;
            RCLCPP_INFO(
                get_logger(),
                "[GantryFrameController] %s pitch motor stall detected; zero angle latched at "
                "%.3f rad",
                motor_name, zero_angle);
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

        const double left_relative_angle = *left_motor_angle_ - left_zero_angle_;
        const double right_relative_angle = *right_motor_angle_ - right_zero_angle_;
        sync_error_mm_ = angle_to_height(left_relative_angle - right_relative_angle);

        double sync_correction = 0.0;
        if (enabled && std::abs(sync_error_mm_) > sync_deadband_mm_) {
            const double sync_error_rate_mm_s =
                angle_to_height(*left_motor_velocity_ - *right_motor_velocity_);
            sync_correction =
                sync_kp_ * sync_error_mm_ + sync_kd_ * sync_error_rate_mm_s;
            sync_correction =
                std::clamp(sync_correction, -sync_max_velocity_, sync_max_velocity_);
        }

        // 差速修正不改变两侧平均速度，只消除左右高度差。
        *left_target_velocity_output_ = pitch_target_velocity - sync_correction;
        *right_target_velocity_output_ = pitch_target_velocity + sync_correction;
        *yaw_target_velocity_output_ = yaw_target_velocity;
        *sync_error_output_ = sync_error_mm_;
        *sync_correction_output_ = sync_correction;

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

    double angle_to_height(double angle_rad) const {
        return angle_rad * screw_lead_mm_ / (2.0 * std::numbers::pi);
    }

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    InputInterface<double> left_motor_velocity_;
    InputInterface<double> left_motor_torque_;
    InputInterface<double> left_motor_angle_;
    InputInterface<double> right_motor_velocity_;
    InputInterface<double> right_motor_torque_;
    InputInterface<double> right_motor_angle_;

    OutputInterface<double> left_target_velocity_output_;
    OutputInterface<double> right_target_velocity_output_;
    OutputInterface<double> yaw_target_velocity_output_;
    OutputInterface<double> sync_error_output_;
    OutputInterface<double> sync_correction_output_;

    Stage stage_ = Stage::kHoming;
    bool left_homed_ = false;
    bool right_homed_ = false;
    int left_stall_ticks_ = 0;
    int right_stall_ticks_ = 0;
    double left_zero_angle_ = 0.0;
    double right_zero_angle_ = 0.0;

    double max_velocity_ = 5.0;
    double yaw_max_velocity_ = 5.0;
    double homing_velocity_ = 4.0;
    double stall_velocity_threshold_ = 0.2;
    double stall_torque_threshold_ = 0.15;
    int stall_ticks_ = 200;
    double screw_lead_mm_ = 2.0;
    double sync_kp_ = 5.0;
    double sync_kd_ = 0.0;
    double sync_max_velocity_ = 2.0;
    double sync_deadband_mm_ = 0.1;
    double sync_error_mm_ = 0.0;
    bool last_enabled_ = false;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::GantryFrameController, rmcs_executor::Component)
