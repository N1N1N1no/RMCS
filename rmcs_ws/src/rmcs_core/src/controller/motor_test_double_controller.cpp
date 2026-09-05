#include "filter/low_pass_filter.hpp"

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


        register_input("/test/motor/angle", motor_angle_, false);
        register_input("/test/motor/velocity", motor_velocity_, false);

        register_output("/test/motor/measured_angle", measured_angle_output_, 0.0);
        register_output("/test/motor/measured_velocity", measured_velocity_output_, 0.0);

        register_output("/test/motor/target_angle", target_angle_output_, 0.0);
        register_output("/test/motor/angle_error", angle_error_output_, 0.0);

        target_angle_ = 0.0;
        get_parameter("target_angle", target_angle_);

        square_wave_enabled_ = true;
        get_parameter("square_wave_enabled", square_wave_enabled_);
        square_wave_amplitude_ = 1.0;
        get_parameter("target_angle_amplitude", square_wave_amplitude_);
        square_wave_half_period_ = 1.0;
        get_parameter("square_wave_period", square_wave_half_period_);
        if (square_wave_half_period_ <= 0.0) // 防除零：周期非法时退化为最小周期
            square_wave_half_period_ = 0.001;

        angle_filter_cutoff_ = 0.0;
        get_parameter("angle_filter_cutoff", angle_filter_cutoff_);
        velocity_filter_cutoff_ = 100.0;
        get_parameter("velocity_filter_cutoff", velocity_filter_cutoff_);
        filter_sampling_frequency_ = 1000.0;
        get_parameter("filter_sampling_frequency", filter_sampling_frequency_);
        if (filter_sampling_frequency_ <= 0.0) // 防除零
            filter_sampling_frequency_ = 1000.0;

        RCLCPP_INFO(
            get_logger(),
            "[MotorTestDoubleController] initialized (square_wave=%s, amplitude=%.2f, period=%.2fs, "
            "angle_filter=%.1fHz, vel_filter=%.1fHz)",
            square_wave_enabled_ ? "on" : "off", square_wave_amplitude_, square_wave_half_period_,
            angle_filter_cutoff_, velocity_filter_cutoff_);
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
        const double measured_angle = filter_measurement(
            angle_filter_, angle_filter_active_, angle_filter_cutoff_, *motor_angle_);
        const double measured_velocity = filter_measurement(
            velocity_filter_, velocity_filter_active_, velocity_filter_cutoff_, *motor_velocity_);

        // 目标角度（方波/固定）
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

        *measured_angle_output_ = measured_angle;
        *measured_velocity_output_ = measured_velocity;
        *target_angle_output_ = target_angle;
        *angle_error_output_ = target_angle - measured_angle;
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
    bool square_wave_enabled_ = false;
    double square_wave_amplitude_ = 1.0;
    double square_wave_half_period_ = 1.0;
    bool wave_started_ = false;
    std::chrono::steady_clock::time_point wave_start_time_{};
};

} // namespace rmcs_core::controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_core::controller::MotorTestDoubleController, rmcs_executor::Component)
