#include "controller/gimbal/pitch_load_feedforward.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <rmcs_msgs/switch.hpp>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::controller::gimbal {

using namespace rmcs_description;

class PitchGravityFeedforward
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    PitchGravityFeedforward()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , load_(load_config()) {
        get_parameter_or("pitch_gravity_ff_gain", gain_, 0.);
        get_parameter_or("pitch_gravity_ff_phase", phase_, 0.);
        register_input("/tf", tf_);
        if (load_.enabled()) {
            register_input("/gimbal/pitch/angle", joint_angle_);
            register_input("/remote/switch/left", left_);
            register_input("/remote/switch/right", right_);
        }
        register_output("/gimbal/pitch/gravity_torque_ff", gravity_, 0.);
        register_output("/gimbal/pitch/load_torque_ff", load_torque_, 0.);
        register_output("/gimbal/pitch/torque_ff", torque_, 0.);
    }

    void update() override {
        const auto direction =
            fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        const double pitch = std::asin(std::clamp(direction->z(), -1., 1.));
        *gravity_ = gain_ * std::sin(pitch - phase_);
        *load_torque_ = 0.;
        const auto now = std::chrono::steady_clock::now();
        if (load_.enabled()) {
            using rmcs_msgs::Switch;
            if (*left_ == Switch::UNKNOWN || *right_ == Switch::UNKNOWN
                || (*left_ == Switch::DOWN && *right_ == Switch::DOWN))
                load_.reset();
            else {
                *load_torque_ = load_.update(
                    *joint_angle_, std::chrono::duration<double>(now - last_update_).count());
            }
        }
        last_update_ = now;
        *torque_ = *gravity_ + *load_torque_;
    }

private:
    PitchLoadFeedforward::Config load_config() {
        PitchLoadFeedforward::Config c;
        c.scale = get_parameter_or<double>("pitch_load_ff_scale", 0.);
        c.torque_limit = get_parameter_or<double>("pitch_load_ff_torque_limit", c.torque_limit);
        c.slew_rate = get_parameter_or<double>("pitch_load_ff_slew_rate", c.slew_rate);
        c.direction_deadband =
            get_parameter_or<double>("pitch_load_ff_direction_deadband", c.direction_deadband);
        c.angles = get_parameter_or<std::vector<double>>("pitch_load_ff_angles", {});
        c.up_torques = get_parameter_or<std::vector<double>>("pitch_load_ff_up_torques", {});
        c.down_torques = get_parameter_or<std::vector<double>>("pitch_load_ff_down_torques", {});
        return c;
    }
    PitchLoadFeedforward load_;
    std::chrono::steady_clock::time_point last_update_ = std::chrono::steady_clock::now();
    InputInterface<double> joint_angle_;
    InputInterface<rmcs_msgs::Switch> left_, right_;
    InputInterface<Tf> tf_;
    OutputInterface<double> torque_, gravity_, load_torque_;
    double gain_ = 0., phase_ = 0.;
};

} // namespace rmcs_core::controller::gimbal

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::gimbal::PitchGravityFeedforward, rmcs_executor::Component)
