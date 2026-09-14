#include "filter/low_pass_filter.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

#include <eigen3/Eigen/Core>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_core::controller {

// 龙门架（飞镖 Pitch）双电机升降：共模高度/Pitch 共用同一轨迹，
// 差模 δ = 左角 - 右角 由交叉耦合同步环消除。
class GantryFrameController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    GantryFrameController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/test/frame_left_motor/angle", left_motor_angle_, false);
        register_input("/test/frame_left_motor/velocity", left_motor_velocity_, false);
        register_input("/test/frame_right_motor/angle", right_motor_angle_, false);
        register_input("/test/frame_right_motor/velocity", right_motor_velocity_, false);

        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);

        register_output("/test/frame_left_motor/target_angle", left_target_angle_output_, 0.0);
        register_output("/test/frame_right_motor/target_angle", right_target_angle_output_, 0.0);
        register_output(
            "/test/frame_left_motor/feedforward_velocity", left_feedforward_velocity_output_,
            0.0);
        register_output(
            "/test/frame_right_motor/feedforward_velocity", right_feedforward_velocity_output_,
            0.0);
        register_output(
            "/test/frame_left_motor/feedforward_torque", left_feedforward_torque_output_, 0.0);
        register_output(
            "/test/frame_right_motor/feedforward_torque", right_feedforward_torque_output_, 0.0);

        register_output("/test/gantry/height", measured_height_output_, 0.0);
        register_output("/test/gantry/target_height", target_height_output_, 0.0);
        register_output("/test/gantry/reference_velocity", reference_velocity_output_, 0.0);
        register_output("/test/gantry/sync_error", sync_error_output_, 0.0);           // mm
        register_output("/test/gantry/sync_correction", sync_correction_output_, 0.0); // rad
        register_output("/test/gantry/left_angle_error", left_angle_error_output_, 0.0);
        register_output("/test/gantry/right_angle_error", right_angle_error_output_, 0.0);
        register_output("/test/gantry/enabled", enabled_output_, 0.0);

        load_parameters();

        RCLCPP_INFO(
            get_logger(),
            "[GantryFrameController] initialized (lead=%.2fmm, height=[%.1f, %.1f]mm, "
            "v_max=%.1fmm/s, a_max=%.1fmm/s^2, jerk=%.1f, remote=%s, sync=%s)",
            screw_lead_mm_, min_height_, max_height_, max_velocity_, max_acceleration_,
            max_jerk_, remote_enabled_ ? "on" : "off", sync_enabled_ ? "on" : "off");
    }

    void before_updating() override {
        if (!left_motor_angle_.ready()) {
            left_motor_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/test/frame_left_motor/angle\". Set to 0.");
        }
        if (!left_motor_velocity_.ready()) {
            left_motor_velocity_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/test/frame_left_motor/velocity\". Set to 0.");
        }
        if (!right_motor_angle_.ready()) {
            right_motor_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/test/frame_right_motor/angle\". Set to 0.");
        }
        if (!right_motor_velocity_.ready()) {
            right_motor_velocity_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/test/frame_right_motor/velocity\". Set to 0.");
        }
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
    }

    void update() override {
        update_feedback();

        const bool enabled = calculate_enabled();
        const double target_height = calculate_target_height(enabled);

        update_reference(target_height, 1.0 / filter_sampling_frequency_);

        const double sync_correction = calculate_sync_correction();
        publish_outputs(target_height, enabled, sync_correction);
    }

private:
    void load_parameters() {
        screw_lead_mm_ = 4.0; // mm/rev（1204 滚珠丝杆）
        get_parameter("screw_lead_mm", screw_lead_mm_);
        if (screw_lead_mm_ <= 0.0) {
            screw_lead_mm_ = 4.0;
            RCLCPP_WARN(
                get_logger(), "[GantryFrameController] screw_lead_mm 非法，回退为 4.0 mm");
        }

        // 龙门停在已知零点时读 /test/frame_*_motor/angle 填入
        left_zero_angle_ = 0.0;
        get_parameter("left_zero_angle", left_zero_angle_);
        right_zero_angle_ = 0.0;
        get_parameter("right_zero_angle", right_zero_angle_);

        min_height_ = 0.0;
        get_parameter("min_height", min_height_);
        max_height_ = 100.0;
        get_parameter("max_height", max_height_);
        if (max_height_ < min_height_)
            std::swap(max_height_, min_height_);

        target_height_ = 0.5 * (min_height_ + max_height_);
        get_parameter("target_height", target_height_);

        max_velocity_ = 30.0;
        get_parameter("max_velocity", max_velocity_);
        max_acceleration_ = 120.0;
        get_parameter("max_acceleration", max_acceleration_);
        max_jerk_ = 600.0;
        get_parameter("max_jerk", max_jerk_);
        height_tolerance_ = 0.1;
        get_parameter("height_tolerance", height_tolerance_);
        velocity_tolerance_ = 0.5;
        get_parameter("velocity_tolerance", velocity_tolerance_);

        max_velocity_ = std::abs(max_velocity_);
        max_acceleration_ = std::abs(max_acceleration_);
        max_jerk_ = std::max(0.0, max_jerk_);
        height_tolerance_ = std::max(0.0, height_tolerance_);
        velocity_tolerance_ = std::max(0.0, velocity_tolerance_);
        if (max_velocity_ <= 0.0)
            max_velocity_ = 30.0;
        if (max_acceleration_ <= 0.0)
            max_acceleration_ = 120.0;

        remote_enabled_ = true;
        get_parameter("remote_enabled", remote_enabled_);
        remote_center_height_ = 0.5 * (min_height_ + max_height_);
        get_parameter("remote_center_height", remote_center_height_);
        remote_height_range_ = 0.5 * (max_height_ - min_height_);
        get_parameter("remote_height_range", remote_height_range_);
        remote_deadband_ = 0.08;
        get_parameter("remote_deadband", remote_deadband_);
        if (remote_deadband_ < 0.0)
            remote_deadband_ = 0.0;

        sync_enabled_ = false;
        get_parameter("sync_enabled", sync_enabled_);
        sync_kp_ = 0.6;
        get_parameter("sync_kp", sync_kp_);
        sync_kd_ = 0.02;
        get_parameter("sync_kd", sync_kd_);
        sync_output_limit_ = 0.03;
        get_parameter("sync_output_limit", sync_output_limit_);
        sync_error_limit_mm_ = 0.0;
        get_parameter("sync_error_limit_mm", sync_error_limit_mm_);
        sync_output_limit_ = std::abs(sync_output_limit_);
        sync_error_limit_mm_ = std::max(0.0, sync_error_limit_mm_);

        left_gravity_ff_ = 0.0;
        get_parameter("left_gravity_ff", left_gravity_ff_);
        right_gravity_ff_ = 0.0;
        get_parameter("right_gravity_ff", right_gravity_ff_);
        friction_ff_ = 0.0;
        get_parameter("friction_ff", friction_ff_);
        friction_ff_velocity_scale_ = 5.0;
        get_parameter("friction_ff_velocity_scale", friction_ff_velocity_scale_);
        friction_ff_ = std::abs(friction_ff_);
        if (friction_ff_velocity_scale_ <= 0.0)
            friction_ff_velocity_scale_ = 5.0;

        angle_filter_cutoff_ = 0.0;
        get_parameter("angle_filter_cutoff", angle_filter_cutoff_);
        velocity_filter_cutoff_ = 100.0;
        get_parameter("velocity_filter_cutoff", velocity_filter_cutoff_);
        filter_sampling_frequency_ = 1000.0;
        get_parameter("filter_sampling_frequency", filter_sampling_frequency_);
        if (filter_sampling_frequency_ <= 0.0)
            filter_sampling_frequency_ = 1000.0;
    }

    void update_feedback() {
        left_angle_ = filter_measurement(
            left_angle_filter_, left_angle_filter_active_, angle_filter_cutoff_,
            *left_motor_angle_);
        right_angle_ = filter_measurement(
            right_angle_filter_, right_angle_filter_active_, angle_filter_cutoff_,
            *right_motor_angle_);
        left_velocity_ = filter_measurement(
            left_velocity_filter_, left_velocity_filter_active_, velocity_filter_cutoff_,
            *left_motor_velocity_);
        right_velocity_ = filter_measurement(
            right_velocity_filter_, right_velocity_filter_active_, velocity_filter_cutoff_,
            *right_motor_velocity_);

        const double left_height = angle_to_height(left_angle_ - left_zero_angle_);
        const double right_height = angle_to_height(right_angle_ - right_zero_angle_);
        measured_height_ = 0.5 * (left_height + right_height);
        sync_error_rad_ = (left_angle_ - left_zero_angle_) - (right_angle_ - right_zero_angle_);
        sync_error_mm_ = angle_to_height(sync_error_rad_);

        // 首次反馈锁存当前位置，避免上电跳变
        if (!reference_initialized_) {
            reference_height_ = measured_height_;
            reference_velocity_ = 0.0;
            last_acceleration_ = 0.0;
            reference_initialized_ = true;
            RCLCPP_INFO(
                get_logger(), "[GantryFrameController] 首次反馈到位，参考高度锁存为 %.2f mm",
                reference_height_);
        }
    }

    bool calculate_enabled() const {
        const bool remote_ok = *switch_left_ != rmcs_msgs::Switch::UNKNOWN
                               && *switch_right_ != rmcs_msgs::Switch::UNKNOWN;
        const bool both_switches_down = *switch_left_ == rmcs_msgs::Switch::DOWN
                                        && *switch_right_ == rmcs_msgs::Switch::DOWN;
        return remote_enabled_ ? (remote_ok && !both_switches_down) : true;
    }

    double calculate_target_height(bool enabled) {
        double target_height = target_height_;
        if (remote_enabled_) {
            if (enabled) {
                double stick = joystick_left_->y();
                if (std::abs(stick) < remote_deadband_)
                    stick = 0.0;
                target_height = remote_center_height_ + stick * remote_height_range_;
            } else {
                target_height = measured_height_;
            }
        }
        target_height = std::clamp(target_height, min_height_, max_height_);

        const bool sync_over_limit =
            sync_error_limit_mm_ > 0.0 && std::abs(sync_error_mm_) > sync_error_limit_mm_;
        if (sync_over_limit) {
            target_height = measured_height_;
            if (!sync_over_limit_prev_) {
                RCLCPP_WARN(
                    get_logger(),
                    "[GantryFrameController] 同步误差 %.3f mm 超过阈值 %.3f mm，暂停升降并保持",
                    sync_error_mm_, sync_error_limit_mm_);
            }
        } else if (sync_over_limit_prev_) {
            RCLCPP_INFO(get_logger(), "[GantryFrameController] 同步误差恢复，继续跟踪目标高度");
        }
        sync_over_limit_prev_ = sync_over_limit;

        return target_height;
    }

    double calculate_sync_correction() const {
        if (!sync_enabled_)
            return 0.0;

        const double sync_error_rate = left_velocity_ - right_velocity_; // rad/s
        const double correction = sync_kp_ * sync_error_rad_ + sync_kd_ * sync_error_rate;
        return std::clamp(correction, -sync_output_limit_, sync_output_limit_);
    }

    void publish_outputs(double target_height, bool enabled, double sync_correction) {
        const double reference_angle = height_to_angle(reference_height_);
        const double reference_velocity_rad = height_to_angle(reference_velocity_);

        const double left_target_angle =
            reference_angle + left_zero_angle_ - sync_correction;
        const double right_target_angle =
            reference_angle + right_zero_angle_ + sync_correction;

        const double friction_ff =
            friction_ff_ * std::tanh(reference_velocity_ / friction_ff_velocity_scale_);

        *left_target_angle_output_ = left_target_angle;
        *right_target_angle_output_ = right_target_angle;
        *left_feedforward_velocity_output_ = reference_velocity_rad;
        *right_feedforward_velocity_output_ = reference_velocity_rad;
        *left_feedforward_torque_output_ = left_gravity_ff_ + friction_ff;
        *right_feedforward_torque_output_ = right_gravity_ff_ + friction_ff;

        *measured_height_output_ = measured_height_;
        *target_height_output_ = target_height;
        *reference_velocity_output_ = reference_velocity_;
        *sync_error_output_ = sync_error_mm_;
        *sync_correction_output_ = sync_correction;
        *left_angle_error_output_ = left_target_angle - left_angle_;
        *right_angle_error_output_ = right_target_angle - right_angle_;
        *enabled_output_ = enabled ? 1.0 : 0.0;
    }

    double height_to_angle(double height_mm) const {
        return height_mm * 2.0 * std::numbers::pi / screw_lead_mm_;
    }

    double angle_to_height(double angle_rad) const {
        return angle_rad * screw_lead_mm_ / (2.0 * std::numbers::pi);
    }

    void update_reference(double target_height, double dt) {
        const double error = target_height - reference_height_;
        const double braking_speed = std::sqrt(2.0 * max_acceleration_ * std::abs(error));
        const double desired_velocity =
            std::copysign(std::min(max_velocity_, braking_speed), error);

        double acceleration = (desired_velocity - reference_velocity_) / dt;
        acceleration = std::clamp(acceleration, -max_acceleration_, max_acceleration_);
        if (max_jerk_ > 0.0) {
            acceleration = std::clamp(
                acceleration, last_acceleration_ - max_jerk_ * dt,
                last_acceleration_ + max_jerk_ * dt);
        }
        last_acceleration_ = acceleration;

        reference_velocity_ =
            std::clamp(reference_velocity_ + acceleration * dt, -max_velocity_, max_velocity_);
        reference_height_ += reference_velocity_ * dt;

        if (std::abs(target_height - reference_height_) <= height_tolerance_
            && std::abs(reference_velocity_) <= velocity_tolerance_) {
            reference_height_ = target_height;
            reference_velocity_ = 0.0;
            last_acceleration_ = 0.0;
        }
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

    InputInterface<double> left_motor_angle_;
    InputInterface<double> left_motor_velocity_;
    InputInterface<double> right_motor_angle_;
    InputInterface<double> right_motor_velocity_;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    OutputInterface<double> left_target_angle_output_;
    OutputInterface<double> right_target_angle_output_;
    OutputInterface<double> left_feedforward_velocity_output_;
    OutputInterface<double> right_feedforward_velocity_output_;
    OutputInterface<double> left_feedforward_torque_output_;
    OutputInterface<double> right_feedforward_torque_output_;

    OutputInterface<double> measured_height_output_;
    OutputInterface<double> target_height_output_;
    OutputInterface<double> reference_velocity_output_;
    OutputInterface<double> sync_error_output_;
    OutputInterface<double> sync_correction_output_;
    OutputInterface<double> left_angle_error_output_;
    OutputInterface<double> right_angle_error_output_;
    OutputInterface<double> enabled_output_;

    filter::LowPassFilter<1> left_angle_filter_{1.0};
    filter::LowPassFilter<1> right_angle_filter_{1.0};
    filter::LowPassFilter<1> left_velocity_filter_{1.0};
    filter::LowPassFilter<1> right_velocity_filter_{1.0};
    bool left_angle_filter_active_ = false;
    bool right_angle_filter_active_ = false;
    bool left_velocity_filter_active_ = false;
    bool right_velocity_filter_active_ = false;

    double angle_filter_cutoff_ = 0.0;
    double velocity_filter_cutoff_ = 100.0;
    double filter_sampling_frequency_ = 1000.0;

    double screw_lead_mm_ = 4.0;
    double left_zero_angle_ = 0.0;
    double right_zero_angle_ = 0.0;
    double min_height_ = 0.0;
    double max_height_ = 100.0;

    double target_height_ = 0.0;
    double max_velocity_ = 30.0;
    double max_acceleration_ = 120.0;
    double max_jerk_ = 600.0;
    double height_tolerance_ = 0.1;
    double velocity_tolerance_ = 0.5;

    bool remote_enabled_ = true;
    double remote_center_height_ = 0.0;
    double remote_height_range_ = 0.0;
    double remote_deadband_ = 0.08;

    bool sync_enabled_ = false;
    double sync_kp_ = 0.6;
    double sync_kd_ = 0.02;
    double sync_output_limit_ = 0.03;
    double sync_error_limit_mm_ = 0.0;
    bool sync_over_limit_prev_ = false;

    double left_gravity_ff_ = 0.0;
    double right_gravity_ff_ = 0.0;
    double friction_ff_ = 0.0;
    double friction_ff_velocity_scale_ = 5.0;

    double left_angle_ = 0.0;
    double right_angle_ = 0.0;
    double left_velocity_ = 0.0;
    double right_velocity_ = 0.0;
    double measured_height_ = 0.0;
    double sync_error_rad_ = 0.0;
    double sync_error_mm_ = 0.0;

    double reference_height_ = 0.0;
    double reference_velocity_ = 0.0;
    double last_acceleration_ = 0.0;
    bool reference_initialized_ = false;
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::GantryFrameController, rmcs_executor::Component)
