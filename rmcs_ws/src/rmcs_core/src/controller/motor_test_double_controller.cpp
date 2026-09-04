#include "controller/pid/pid_calculator.hpp"
#include "filter/low_pass_filter.hpp"

// #include <eigen3/Eigen/Core>       // [未启用 DR16] 恢复摇杆映射时取消注释
// #include <rmcs_msgs/switch.hpp>    // [未启用 DR16] 恢复遥控器拨杆时取消注释
#include <chrono>
#include <cmath>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller {

class MotorTestDoubleController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MotorTestDoubleController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        // [未启用 DR16] 不再读取 /remote/* 话题
        // register_input("/remote/joystick/left", joystick_left_, false);
        // register_input("/remote/switch/left", switch_left_, false);
        // register_input("/remote/switch/right", switch_right_, false);

        // 角度环/速度环所需的反馈 + 输出力矩
        register_input("/test/motor/angle", motor_angle_, false);
        register_input("/test/motor/velocity", motor_velocity_, false);
        register_output("/test/motor/control_torque", control_torque_, 0.0);

        // 供调试/可视化使用的话题：目标角度、角度误差（实际角度 /test/motor/angle 由 DjiMotor 发布）
        register_output("/test/motor/target_angle", target_angle_output_, 0.0);
        register_output("/test/motor/angle_error", angle_error_output_, 0.0);

        // ---- 固定目标角度(rad)，方波关闭时使用 ----
        target_angle_ = 0.0;
        get_parameter("target_angle", target_angle_);

        // ---- 方波目标角度：目标在 -A 和 +A 之间每 half_period 秒切换一次 ----
        square_wave_enabled_ = true;
        get_parameter("square_wave_enabled", square_wave_enabled_);
        square_wave_amplitude_ = 1.0;
        get_parameter("target_angle_amplitude", square_wave_amplitude_);
        square_wave_half_period_ = 1.0;
        get_parameter("square_wave_period", square_wave_half_period_);

        // ---- 反馈低通滤波(截止频率 Hz，<=0 表示关闭) ----
        angle_filter_cutoff_ = 0.0;
        get_parameter("angle_filter_cutoff", angle_filter_cutoff_);
        velocity_filter_cutoff_ = 100.0;
        get_parameter("velocity_filter_cutoff", velocity_filter_cutoff_);
        filter_sampling_frequency_ = 1000.0;
        get_parameter("filter_sampling_frequency", filter_sampling_frequency_);

        // 位置环输出(目标速度)的限幅
        max_velocity_ = 5.0;
        get_parameter("max_velocity", max_velocity_);

        // 角度环(外环)-
        angle_pid_.kp = 8.0;
        get_parameter("angle_kp", angle_pid_.kp);
        angle_pid_.ki = 0.0;
        get_parameter("angle_ki", angle_pid_.ki);
        angle_pid_.kd = 0.0;
        get_parameter("angle_kd", angle_pid_.kd);
        angle_pid_.output_min = -max_velocity_;
        angle_pid_.output_max = max_velocity_;

        // 速度环(内环):
        velocity_pid_.kp = 0.5;
        get_parameter("vel_kp", velocity_pid_.kp);
        velocity_pid_.ki = 0.0;
        get_parameter("vel_ki", velocity_pid_.ki);
        velocity_pid_.kd = 0.0;
        get_parameter("vel_kd", velocity_pid_.kd);

        double integral_limit = 1.0;
        get_parameter("integral_limit", integral_limit);
        velocity_pid_.integral_min = -integral_limit;
        velocity_pid_.integral_max = integral_limit;

        RCLCPP_INFO(
            get_logger(),
            "[MotorTestDoubleController] initialized (square_wave=%s, amplitude=%.2f, period=%.2fs, "
            "angle_filter=%.1fHz, vel_filter=%.1fHz, angle_kp=%.2f, vel_kp=%.3f)",
            square_wave_enabled_ ? "on" : "off", square_wave_amplitude_, square_wave_half_period_,
            angle_filter_cutoff_, velocity_filter_cutoff_, angle_pid_.kp, velocity_pid_.kp);
    }

    void before_updating() override {
        if (!motor_angle_.ready()) {
            motor_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/test/motor/angle\". Set to 0.");
        }
        if (!motor_velocity_.ready()) {
            motor_velocity_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/test/motor/velocity\". Set to 0.");
        }
    }

    void update() override {
        get_parameter("square_wave_enabled", square_wave_enabled_);
        get_parameter("target_angle_amplitude", square_wave_amplitude_);
        get_parameter("square_wave_period", square_wave_half_period_);
        get_parameter("target_angle", target_angle_);
        get_parameter("max_velocity", max_velocity_);
        get_parameter("angle_kp", angle_pid_.kp);
        get_parameter("angle_ki", angle_pid_.ki);
        get_parameter("angle_kd", angle_pid_.kd);
        get_parameter("vel_kp", velocity_pid_.kp);
        get_parameter("vel_ki", velocity_pid_.ki);
        get_parameter("vel_kd", velocity_pid_.kd);
        get_parameter("angle_filter_cutoff", angle_filter_cutoff_);
        get_parameter("velocity_filter_cutoff", velocity_filter_cutoff_);
        get_parameter("filter_sampling_frequency", filter_sampling_frequency_);

        if (square_wave_half_period_ <= 0.0)   // 防除零：周期非法时退化为最小周期
            square_wave_half_period_ = 0.001;
        if (filter_sampling_frequency_ <= 0.0) // 防除零
            filter_sampling_frequency_ = 1000.0;

        angle_pid_.output_min = -max_velocity_;
        angle_pid_.output_max = max_velocity_;

        // 反馈低通滤波
        const double measured_angle = filter_measurement(
            angle_filter_, angle_filter_active_, angle_filter_cutoff_, *motor_angle_);
        const double measured_velocity = filter_measurement(
            velocity_filter_, velocity_filter_active_, velocity_filter_cutoff_, *motor_velocity_);

        // 计算当前目标角度：默认固定值 target_angle_，启用方波则在 -A/+A 之间切换
        double target_angle = target_angle_;
        if (square_wave_enabled_) {
            const auto now = std::chrono::steady_clock::now();
            if (!wave_started_) {
                wave_started_ = true;
                wave_start_time_ = now;
            }

            const double elapsed = std::chrono::duration<double>(now - wave_start_time_).count();
            const double cycle = 2.0 * square_wave_half_period_;
            target_angle = std::fmod(elapsed, cycle) < square_wave_half_period_
                               ? -square_wave_amplitude_
                               : square_wave_amplitude_;
        }

        const double angle_error = target_angle - measured_angle;

        // 角度环(外环）
        const double target_velocity = angle_pid_.update(angle_error);

        // 速度环(内环)
        *control_torque_ = velocity_pid_.update(target_velocity - measured_velocity);

        // 可视化
        *target_angle_output_ = target_angle;
        *angle_error_output_ = angle_error;
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

    InputInterface<double> motor_angle_;
    InputInterface<double> motor_velocity_;
    OutputInterface<double> control_torque_;
    OutputInterface<double> target_angle_output_;
    OutputInterface<double> angle_error_output_;

    // [未启用 DR16] 注释保留
    // InputInterface<Eigen::Vector2d> joystick_left_;
    // InputInterface<rmcs_msgs::Switch> switch_left_;
    // InputInterface<rmcs_msgs::Switch> switch_right_;

    pid::PidCalculator angle_pid_;     // 角度环(位置环)
    pid::PidCalculator velocity_pid_;  // 速度环

    filter::LowPassFilter<1> angle_filter_{1.0};     // 角度反馈低通(默认直通)
    filter::LowPassFilter<1> velocity_filter_{1.0};  // 速度反馈低通
    bool angle_filter_active_ = false;
    bool velocity_filter_active_ = false;
    double angle_filter_cutoff_ = 0.0;
    double velocity_filter_cutoff_ = 100.0;
    double filter_sampling_frequency_ = 1000.0;

    double target_angle_ = 0.0;
    bool square_wave_enabled_ = false;
    double square_wave_amplitude_ = 1.0;
    double square_wave_half_period_ = 1.0;
    bool wave_started_ = false;
    std::chrono::steady_clock::time_point wave_start_time_{};

    double max_velocity_ = 5.0;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::MotorTestDoubleController, rmcs_executor::Component)
