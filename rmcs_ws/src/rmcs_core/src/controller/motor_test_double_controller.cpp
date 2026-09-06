#include "filter/low_pass_filter.hpp"

#include <chrono>
#include <cmath>
#include <numbers>

#include <eigen3/Eigen/Core>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller {


class MotorTestDoubleController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MotorTestDoubleController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {


        register_input("/test/motor/angle", motor_angle_, false);
        register_input("/test/motor/velocity", motor_velocity_, false);

        
        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);

        register_output("/test/motor/measured_angle", measured_angle_output_, 0.0);
        register_output("/test/motor/measured_velocity", measured_velocity_output_, 0.0);

        register_output("/test/motor/target_angle", target_angle_output_, 0.0);
        register_output("/test/motor/angle_error", angle_error_output_, 0.0);

        target_angle_ = 0.0;
        get_parameter("target_angle", target_angle_);

        trapezoid_enabled_ = true;
        get_parameter("trapezoid_enabled", trapezoid_enabled_);
        trapezoid_amplitude_ = 1.0;
        get_parameter("target_angle_amplitude", trapezoid_amplitude_);
        trapezoid_rise_time_ = 1.0;
        get_parameter("trapezoid_rise_time", trapezoid_rise_time_);
        trapezoid_hold_time_ = 1.0;
        get_parameter("trapezoid_hold_time", trapezoid_hold_time_);
        if (trapezoid_rise_time_ < 0.0) // 防负值：非法时退化为 0（瞬间换向，等效方波）
            trapezoid_rise_time_ = 0.0;
        if (trapezoid_hold_time_ < 0.0) // 防负值
            trapezoid_hold_time_ = 0.0;

        //优弧
        major_arc_enabled_ = false;
        get_parameter("major_arc_enabled", major_arc_enabled_);
        major_arc_tolerance_ = 0.05; // rad：离设定角多近就算“已到达”（避免已在设定角时还空转一整圈）
        get_parameter("major_arc_tolerance", major_arc_tolerance_);
        if (major_arc_tolerance_ < 0.0) // 防负值
            major_arc_tolerance_ = 0.0;
        if (major_arc_enabled_ && trapezoid_enabled_) {
            RCLCPP_WARN(
                get_logger(),
                "[MotorTestDoubleController] major_arc_enabled 与 trapezoid_enabled 同时开启；"
                "走优弧仅在固定目标模式(trapezoid_enabled=false)下生效");
        }
        startup_time_ = std::chrono::steady_clock::now();


        remote_angle_enabled_ = false;
        get_parameter("remote_angle_enabled", remote_angle_enabled_);
        remote_angle_setpoint_ = 3.0; 
        get_parameter("remote_angle_setpoint", remote_angle_setpoint_);
        remote_angle_deadband_ = 0.1; 
        get_parameter("remote_angle_deadband", remote_angle_deadband_);
        if (remote_angle_deadband_ < 0.0) 
            remote_angle_deadband_ = 0.0;
        if (remote_angle_enabled_ && trapezoid_enabled_) {
            RCLCPP_WARN(
                get_logger(),
                "[MotorTestDoubleController] remote_angle_enabled 与 trapezoid_enabled 同时开启；"
                "遥控给定角仅在梯形波关闭(trapezoid_enabled=false)时生效");
        }

        angle_filter_cutoff_ = 0.0;
        get_parameter("angle_filter_cutoff", angle_filter_cutoff_);
        velocity_filter_cutoff_ = 100.0;
        get_parameter("velocity_filter_cutoff", velocity_filter_cutoff_);
        filter_sampling_frequency_ = 1000.0;
        get_parameter("filter_sampling_frequency", filter_sampling_frequency_);
        if (filter_sampling_frequency_ <= 0.0) 
            filter_sampling_frequency_ = 1000.0;

        RCLCPP_INFO(
            get_logger(),
            "[MotorTestDoubleController] initialized (trapezoid=%s, amplitude=%.2f rad, rise=%.2fs, "
            "hold=%.2fs, major_arc=%s, angle_filter=%.1fHz, vel_filter=%.1fHz)",
            trapezoid_enabled_ ? "on" : "off", trapezoid_amplitude_, trapezoid_rise_time_,
            trapezoid_hold_time_, major_arc_enabled_ ? "on" : "off", angle_filter_cutoff_,
            velocity_filter_cutoff_);
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
        if (!joystick_left_.ready()) {
            joystick_left_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/joystick/left\". Set to zero.");
        }
        if (!switch_left_.ready()) {
            switch_left_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }
    }

    void update() override {
        const double measured_angle = filter_measurement(
            angle_filter_, angle_filter_active_, angle_filter_cutoff_, *motor_angle_);
        const double measured_velocity = filter_measurement(
            velocity_filter_, velocity_filter_active_, velocity_filter_cutoff_, *motor_velocity_);

        // 目标角度（梯形波/固定）
        double target_angle = target_angle_;
        if (trapezoid_enabled_) {
            const auto now = std::chrono::steady_clock::now();
            if (!wave_started_) {
                wave_started_ = true;
                wave_start_time_ = now;
            }

            const double elapsed = std::chrono::duration<double>(now - wave_start_time_).count();
            const double rise = trapezoid_rise_time_;
            const double hold = trapezoid_hold_time_;
            const double cycle = 2.0 * (rise + hold); // 一个完整周期：-A -> +A -> -A
            if (cycle > 0.0) {
                const double amplitude = trapezoid_amplitude_;
                const double phase = std::fmod(elapsed, cycle);
                if (rise <= 0.0) {
                    // 上升时间为 0：退化为方波，半周期 = hold
                    target_angle = phase < hold ? -amplitude : amplitude;
                } else if (phase < rise) {
                    target_angle = -amplitude + phase / rise * 2.0 * amplitude; // 斜坡 -A -> +A
                } else if (phase < rise + hold) {
                    target_angle = amplitude; // 保持 +A
                } else if (phase < 2.0 * rise + hold) {
                    target_angle = amplitude - (phase - rise - hold) / rise * 2.0 * amplitude;
                } else {
                    target_angle = -amplitude; // 保持 -A
                }
            }
        } else if (remote_angle_enabled_) {
            // 与 motor_test_single 相同的安全逻辑：左右拨杆都非 UNKNOWN 且不同时为 DOWN 才使能
            const bool remote_ok = *switch_left_ != rmcs_msgs::Switch::UNKNOWN
                                   && *switch_right_ != rmcs_msgs::Switch::UNKNOWN;
            const bool both_switches_down = *switch_left_ == rmcs_msgs::Switch::DOWN
                                            && *switch_right_ == rmcs_msgs::Switch::DOWN;
            const bool enabled = remote_ok && !both_switches_down;

            if (enabled) {
                remote_hold_latched_ = false; 
                const double stick_norm = joystick_left_->norm();
                if (stick_norm > remote_angle_deadband_)
                    remote_stick_moved_ = true;
                else if (stick_norm < remote_angle_deadband_ * 0.5)
                    remote_stick_moved_ = false;
                const double command_angle = remote_stick_moved_ ? remote_angle_setpoint_ : 0.0;
                if (major_arc_enabled_) {
                    target_angle = update_major_arc_target(
                        measured_angle, command_angle, remote_angle_deadband_);
                } else {
                    target_angle = command_angle; 
                }
            } else {
                const bool feedback_settled =
                    std::chrono::steady_clock::now() - startup_time_ >= kMajorArcStartupDelay;
                if (!remote_hold_latched_ || !feedback_settled) {
                    remote_hold_target_ = measured_angle;
                    remote_hold_latched_ = feedback_settled;
                    target_latched_ = false; 
                }
                target_angle = remote_hold_target_;
            }

            if (enabled != remote_enabled_prev_) {
                remote_enabled_prev_ = enabled;
                RCLCPP_INFO(
                    get_logger(),
                    enabled
                        ? "[MotorTestDoubleController] 遥控角度模式使能（拨动左摇杆给定目标角）"
                        : "[MotorTestDoubleController] 遥控角度模式禁用（双拨下/无遥控），保持当前位置");
            }
        } else if (major_arc_enabled_) {
            double command_angle = target_angle_;
            get_parameter("target_angle", command_angle);
            target_angle = update_major_arc_target(measured_angle, command_angle);
        }

        *measured_angle_output_ = measured_angle;
        *measured_velocity_output_ = measured_velocity;
        *target_angle_output_ = target_angle;
        *angle_error_output_ = target_angle - measured_angle;
    }

private:

    static double wrap_angle(double angle) {
        angle = std::remainder(angle, 2.0 * std::numbers::pi); 
        if (angle <= -std::numbers::pi) 
            angle += 2.0 * std::numbers::pi;
        return angle;
    }


    double resolve_major_arc_target(double measured, double command_angle) const {
        const double shortest_error = wrap_angle(command_angle - measured); 
        if (std::abs(shortest_error) <= major_arc_tolerance_) {
            return measured;
        }
        const double major_error = shortest_error > 0.0
                                       ? shortest_error - 2.0 * std::numbers::pi
                                       : shortest_error + 2.0 * std::numbers::pi;
        return measured + major_error;
    }

 
    double update_major_arc_target(
        double measured, double command,
        double command_change_tolerance = kMajorArcCommandTolerance) {
        if (!std::isfinite(command))
            return target_latched_ ? latched_target_angle_ : measured;


        if (!target_latched_
            && std::chrono::steady_clock::now() - startup_time_ < kMajorArcStartupDelay) {
            return measured; 
        }

        const double command_angle = wrap_angle(command);
        if (!target_latched_
            || std::abs(wrap_angle(command_angle - resolved_command_angle_))
                   > command_change_tolerance) {
            latched_target_angle_ = resolve_major_arc_target(measured, command_angle);
            resolved_command_angle_ = command_angle;
            target_latched_ = true;
            RCLCPP_INFO(
                get_logger(),
                "[MotorTestDoubleController] 走优弧：设定角 %.3f rad，锁存目标(多圈) %.3f rad",
                command_angle, latched_target_angle_);
        }
        return latched_target_angle_;
    }

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
    OutputInterface<double> measured_angle_output_;
    OutputInterface<double> measured_velocity_output_;
    OutputInterface<double> target_angle_output_;
    OutputInterface<double> angle_error_output_;

    filter::LowPassFilter<1> angle_filter_{1.0};
    filter::LowPassFilter<1> velocity_filter_{1.0};
    bool angle_filter_active_ = false;
    bool velocity_filter_active_ = false;
    double angle_filter_cutoff_ = 0.0;
    double velocity_filter_cutoff_ = 100.0;
    double filter_sampling_frequency_ = 1000.0;

    double target_angle_ = 0.0;
    bool trapezoid_enabled_ = false;
    double trapezoid_amplitude_ = 1.0;
    double trapezoid_rise_time_ = 1.0;
    double trapezoid_hold_time_ = 1.0;
    bool wave_started_ = false;
    std::chrono::steady_clock::time_point wave_start_time_{};

  
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    bool remote_angle_enabled_ = false;
    double remote_angle_setpoint_ = 3.0; 
    double remote_angle_deadband_ = 0.1;

    bool remote_enabled_prev_ = false;
    bool remote_stick_moved_ = false; 
    bool remote_hold_latched_ = false;
    double remote_hold_target_ = 0.0; 

    // —— 走优弧状态 ——
    bool major_arc_enabled_ = false;
    double major_arc_tolerance_ = 0.05; 

    static constexpr auto kMajorArcStartupDelay = std::chrono::milliseconds{100};
    static constexpr double kMajorArcCommandTolerance = 1e-6; 

    bool target_latched_ = false;         
    double latched_target_angle_ = 0.0;   
    double resolved_command_angle_ = 0.0; 
    std::chrono::steady_clock::time_point startup_time_{};
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::MotorTestDoubleController, rmcs_executor::Component)
