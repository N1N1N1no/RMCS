#include <algorithm>

// #include <eigen3/Eigen/Core>        // [无遥控器] 恢复摇杆映射时取消注释
// #include <rmcs_msgs/switch.hpp>     // [无遥控器] 恢复遥控器拨杆时取消注释
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller {

class MotorTestController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MotorTestController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        // [无遥控器] 不再读取 /remote/* 话题，改用参数 target_velocity 直接给定目标速度
        // register_input("/remote/joystick/left", joystick_left_, false);
        // register_input("/remote/switch/right", switch_right_, false);
        register_input("/test/motor/velocity", motor_velocity_, false);
        register_output("/test/motor/control_torque", control_torque_, 0.0);

        target_velocity_ = 0.0;
        get_parameter("target_velocity", target_velocity_);

        kp_ = 0.5;
        get_parameter("kp", kp_);
        ki_ = 0.0;
        get_parameter("ki", ki_);
        kd_ = 0.0;
        get_parameter("kd", kd_);
        integral_limit_ = 1.0;
        get_parameter("integral_limit", integral_limit_);

        RCLCPP_INFO(
            get_logger(),
            "[MotorTestController] initialized (target_velocity=%.2f, kp=%.3f, ki=%.4f, kd=%.4f)",
            target_velocity_, kp_, ki_, kd_);
    }

    void before_updating() override {
        // [无遥控器] 无需对 joystick/switch 兜底
        if (!motor_velocity_.ready()) {
            motor_velocity_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/test/motor/velocity\". Set to 0.");
        }
    }

    void update() override {
        // [无遥控器] 始终使能：PID 让电机实际转速收敛到目标转速 target_velocity_
        const double err = target_velocity_ - *motor_velocity_;

        err_integral_ = std::clamp(err_integral_ + err, -integral_limit_, integral_limit_);
        *control_torque_ = kp_ * err + ki_ * err_integral_ + kd_ * (err - last_err_);
        last_err_ = err;
    }

private:
    // InputInterface<Eigen::Vector2d> joystick_left_;   // [无遥控器]
    // InputInterface<rmcs_msgs::Switch> switch_right_;  // [无遥控器]
    InputInterface<double> motor_velocity_;
    OutputInterface<double> control_torque_;

    double target_velocity_ = 0.0;
    double kp_ = 0.5;
    double ki_ = 0.0;
    double kd_ = 0.0;
    double integral_limit_ = 1.0;

    double err_integral_ = 0.0;
    double last_err_ = 0.0;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::MotorTestController, rmcs_executor::Component)
