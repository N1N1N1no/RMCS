#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <numbers>
#include <ranges>
#include <sstream>
#include <span>
#include <string>
#include <tuple>

#include <eigen3/Eigen/Dense>
#include <rclcpp/node.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <librmcs/board/rmcs_board_lite.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/board_clock.hpp>
#include <rmcs_msgs/imu_snapshot.hpp>
#include <rmcs_msgs/mouse.hpp>
#include <rmcs_msgs/serial_interface.hpp>
#include <rmcs_msgs/switch.hpp>
#include <rmcs_utility/ring_buffer.hpp>

#include "hardware/device/bmi088.hpp"
#include "hardware/device/bmi088_ekf.hpp"
#include "hardware/device/board_clock_lifter.hpp"
#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/lk_motor.hpp"
#include "hardware/device/remote_control.hpp"
#include "hardware/device/supercap.hpp"
#include "hardware/device/vt13.hpp"
#include "hardware/util/status_monitor.hpp"

namespace rmcs_core::hardware {

using Clock = std::chrono::steady_clock;

class DeformableInfantryOmniC
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DeformableInfantryOmniC()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , command_(create_partner_component<Command>(get_component_name() + "_command", *this)) {
        using namespace rmcs_description;

        register_input("/predefined/timestamp", timestamp_);
        register_output("/tf", tf_);
        register_output(
            "/auto_aim/camera_transform", camera_transform_, Eigen::Isometry3d::Identity());
        register_output("/auto_aim/barrel_direction", barrel_direction_, Eigen::Vector3d::UnitX());
        register_output("/auto_aim/yaw_velocity", auto_aim_yaw_velocity_, 0.0);
        // The auto-aim component is disabled in the yaw-only configuration. Its direction
        // interface is reused for a bounded, manually triggered baseline sweep.
        register_output("/auto_aim/should_control", yaw_sweep_should_control_, false);
        register_output(
            "/auto_aim/control_direction", yaw_sweep_control_direction_,
            Eigen::Vector3d::Constant(kNaN));
        register_output("/auto_aim/ff_v", yaw_sweep_ff_velocity_, Eigen::Vector3d::Zero());
        register_output("/auto_aim/ff_a", yaw_sweep_ff_acceleration_, Eigen::Vector3d::Zero());
        register_output("/gimbal/yaw_sweep/target_offset", yaw_sweep_target_offset_, kNaN);
        register_output("/gimbal/yaw_sweep/active", yaw_sweep_active_output_, 0.0);
        register_output("/gimbal/yaw_sweep/remote_ready", yaw_sweep_remote_ready_output_, 0.0);
        register_output("/gimbal/yaw_sweep/right_switch", yaw_sweep_right_switch_output_, 0.0);
        register_output("/gimbal/yaw_sweep/reference_velocity", yaw_sweep_reference_velocity_, 0.0);
        register_output(
            "/gimbal/yaw_sweep/reference_acceleration", yaw_sweep_reference_acceleration_, 0.0);
        register_output("/gimbal/yaw_sweep/feedforward_scale", yaw_sweep_ff_scale_output_, 0.0);

        yaw_sweep_amplitude_rad_ = std::clamp(
            std::abs(get_parameter_or("yaw_sweep_amplitude_rad", 0.10)), 0.01, 0.15);
        yaw_sweep_hold_seconds_ =
            std::clamp(get_parameter_or("yaw_sweep_hold_seconds", 1.5), 0.5, 5.0);
        yaw_sweep_cycles_ = std::clamp(get_parameter_or("yaw_sweep_cycles", 4), 1, 10);
        yaw_sweep_transition_seconds_ =
            std::clamp(get_parameter_or("yaw_sweep_transition_seconds", 0.0), 0.0, 1.0);
        yaw_sweep_feedforward_scale_ =
            std::clamp(get_parameter_or("yaw_sweep_feedforward_scale", 0.0), 0.0, 1.0);
        yaw_sweep_torque_limit_nm_ =
            std::clamp(get_parameter_or("yaw_sweep_torque_limit_nm", 0.0), 0.0, 4.5);
        yaw_plant_torque_nm_ =
            std::clamp(std::abs(get_parameter_or("yaw_plant_torque_nm", 0.9)), 0.1, 1.0);
        yaw_plant_phase_seconds_ =
            std::clamp(get_parameter_or("yaw_plant_phase_seconds", 0.12), 0.08, 0.20);
        yaw_plant_cycles_ = std::clamp(get_parameter_or("yaw_plant_cycles", 8), 2, 10);

        tf_->set_transform<PitchLink, CameraLink>(Eigen::Translation3d{0.058, -0.08, 0.0});

        remote_control_ = std::make_unique<device::RemoteControl>(*this);

        bottom_board_ = std::make_unique<BottomBoard>(
            *this, *command_, get_parameter("serial_filter_bottom_board").as_string());
        top_board_ = std::make_unique<TopBoard>(
            *this, *command_, get_parameter("serial_filter_top_board").as_string());

        // For command: remote-status
        using Srv = std_srvs::srv::Trigger;
        status_service_ = create_service<Srv>(
            "/rmcs/service/robot_status",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                status_service_callback(response);
            });
        yaw_sweep_start_service_ = create_service<Srv>(
            "/rmcs/service/yaw_sweep/start",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                if (!yaw_sweep_remote_ready_.load()) {
                    response->success = false;
                    response->message =
                        "Yaw sweep not started: hold the right switch UP or right mouse button "
                        "with both remote switches valid. Current right switch value: "
                        + std::to_string(yaw_sweep_right_switch_state_.load());
                    return;
                }
                yaw_sweep_start_requested_.store(true);
                response->success = true;
                response->message = "Yaw sweep queued. Keep the auto-aim request active to run it.";
            });
        yaw_sweep_stop_service_ = create_service<Srv>(
            "/rmcs/service/yaw_sweep/stop",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                yaw_sweep_stop_requested_.store(true);
                response->success = true;
                response->message = "Yaw sweep stop queued.";
            });
        yaw_plant_start_service_ = create_service<Srv>(
            "/rmcs/service/yaw_plant/start",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                if (!yaw_sweep_remote_ready_.load()) {
                    response->success = false;
                    response->message = "Yaw plant test requires the right switch UP or right mouse "
                                        "button and valid remote switches.";
                    return;
                }
                yaw_plant_start_requested_.store(true);
                response->success = true;
                response->message = "Yaw plant test queued; keep the remote request active.";
            });
        yaw_plant_stop_service_ = create_service<Srv>(
            "/rmcs/service/yaw_plant/stop",
            [this](const Srv::Request::SharedPtr&, const Srv::Response::SharedPtr& response) {
                yaw_plant_stop_requested_.store(true);
                response->success = true;
                response->message = "Yaw plant test stop queued.";
            });
    }

    ~DeformableInfantryOmniC() override = default;

    void before_updating() override { top_board_->request_hard_sync_read(); }

    void update() override {
        bottom_board_->update();
        top_board_->update();
        remote_control_->update();

        using namespace rmcs_description;
        *camera_transform_ = fast_tf::lookup_transform<OdomImu, CameraLink>(*tf_);
        *barrel_direction_ =
            *fast_tf::cast<OdomImu>(PitchLink::DirectionVector{Eigen::Vector3d::UnitX()}, *tf_);
        *auto_aim_yaw_velocity_ = top_board_->gimbal_yaw_velocity();
        update_yaw_sweep();
        update_yaw_plant();
    }

    void command_update() {
        const bool even = ((cmd_tick_++ & 1u) == 0u);
        bottom_board_->command_update(even);
        top_board_->command_update();
    }

private:
    static constexpr auto kNaN = std::numeric_limits<double>::quiet_NaN();
    static constexpr auto kLeftFront = 0;
    static constexpr auto kLeftBack = 1;
    static constexpr auto kRightBack = 2;
    static constexpr auto kRightFront = 3;
    static constexpr auto kJointName = std::array{
        "left_front",
        "left_back",
        "right_back",
        "right_front",
    };

    void update_yaw_sweep() {
        const auto right_switch = command_->right_switch();
        const bool remote_ready = command_->sweep_allowed();
        yaw_sweep_right_switch_state_.store(static_cast<int>(right_switch));
        yaw_sweep_remote_ready_.store(remote_ready);
        *yaw_sweep_remote_ready_output_ = remote_ready ? 1.0 : 0.0;
        *yaw_sweep_right_switch_output_ = static_cast<double>(static_cast<int>(right_switch));
        *yaw_sweep_should_control_ = false;
        *yaw_sweep_control_direction_ = Eigen::Vector3d::Constant(kNaN);
        *yaw_sweep_target_offset_ = kNaN;
        *yaw_sweep_active_output_ = 0.0;
        *yaw_sweep_ff_velocity_ = Eigen::Vector3d::Zero();
        *yaw_sweep_ff_acceleration_ = Eigen::Vector3d::Zero();
        *yaw_sweep_reference_velocity_ = 0.0;
        *yaw_sweep_reference_acceleration_ = 0.0;
        *yaw_sweep_ff_scale_output_ = 0.0;

        if (yaw_plant_active_) {
            yaw_sweep_start_requested_.store(false);
            yaw_sweep_active_ = false;
            return;
        }

        if (yaw_sweep_stop_requested_.exchange(false)) {
            yaw_sweep_start_requested_.store(false);
            yaw_sweep_active_ = false;
            // Capture the current direction once so the gimbal holds where it was stopped.
            if (barrel_direction_->allFinite() && !barrel_direction_->isZero()) {
                *yaw_sweep_should_control_ = true;
                *yaw_sweep_control_direction_ = *barrel_direction_;
            }
            return;
        }

        // Match the gimbal controller's auto-aim request gate. Cancel when it drops so
        // the sweep cannot unexpectedly resume when the operator enables it again.
        if (!remote_ready) {
            yaw_sweep_start_requested_.store(false);
            yaw_sweep_active_ = false;
            return;
        }

        if (yaw_sweep_start_requested_.exchange(false)) {
            if (barrel_direction_->allFinite() && !barrel_direction_->isZero()) {
                yaw_sweep_center_direction_ = barrel_direction_->normalized();
                yaw_sweep_start_angle_ = bottom_board_->gimbal_yaw_motor_.angle();
                yaw_sweep_start_time_ = Clock::now();
                yaw_sweep_active_ = true;
            } else {
                RCLCPP_WARN(get_logger(), "Yaw sweep start ignored: invalid gimbal direction");
            }
        }

        if (!yaw_sweep_active_)
            return;

        const double elapsed_seconds =
            std::chrono::duration<double>(Clock::now() - yaw_sweep_start_time_).count();
        const int phase = static_cast<int>(elapsed_seconds / yaw_sweep_hold_seconds_);
        const int final_center_phase = 2 * yaw_sweep_cycles_ + 1;
        if (phase > final_center_phase) {
            yaw_sweep_active_ = false;
            return;
        }

        const double displacement = std::remainder(
            bottom_board_->gimbal_yaw_motor_.angle() - yaw_sweep_start_angle_,
            2.0 * std::numbers::pi);
        const double velocity = bottom_board_->gimbal_yaw_motor_.velocity();
        if (!std::isfinite(displacement) || !std::isfinite(velocity)
            || std::abs(displacement) > 0.18 || std::abs(velocity) > 1.2) {
            yaw_sweep_active_ = false;
            *yaw_sweep_should_control_ = true;
            *yaw_sweep_control_direction_ = *barrel_direction_;
            RCLCPP_WARN(get_logger(), "Yaw sweep stopped by motion limit");
            return;
        }

        // One center interval, alternating right/left intervals, then one center interval.
        const auto phase_offset = [this, final_center_phase](int p) {
            return p <= 0 || p >= final_center_phase
                     ? 0.0
                     : (p % 2 == 1 ? yaw_sweep_amplitude_rad_ : -yaw_sweep_amplitude_rad_);
        };
        const double start_offset = phase_offset(phase - 1);
        const double target_offset = phase_offset(phase);
        double offset = target_offset;
        double reference_velocity = 0.0;
        double reference_acceleration = 0.0;
        if (phase > 0 && yaw_sweep_transition_seconds_ > 0.0) {
            const double phase_seconds =
                elapsed_seconds - static_cast<double>(phase) * yaw_sweep_hold_seconds_;
            const double s = std::clamp(
                phase_seconds / yaw_sweep_transition_seconds_, 0.0, 1.0);
            const double s2 = s * s;
            const double s3 = s2 * s;
            const double s4 = s3 * s;
            const double s5 = s4 * s;
            const double delta = target_offset - start_offset;
            offset = start_offset + delta * (10.0 * s3 - 15.0 * s4 + 6.0 * s5);
            reference_velocity = delta * (30.0 * s2 - 60.0 * s3 + 30.0 * s4)
                               / yaw_sweep_transition_seconds_;
            reference_acceleration = delta * (60.0 * s - 180.0 * s2 + 120.0 * s3)
                                   / (yaw_sweep_transition_seconds_
                                      * yaw_sweep_transition_seconds_);
        }
        *yaw_sweep_should_control_ = true;
        *yaw_sweep_control_direction_ =
            Eigen::AngleAxisd{offset, Eigen::Vector3d::UnitZ()} * yaw_sweep_center_direction_;
        *yaw_sweep_target_offset_ = offset;
        *yaw_sweep_active_output_ = 1.0;
        *yaw_sweep_reference_velocity_ = reference_velocity;
        *yaw_sweep_reference_acceleration_ = reference_acceleration;
        *yaw_sweep_ff_scale_output_ = yaw_sweep_feedforward_scale_;
        *yaw_sweep_ff_velocity_ =
            yaw_sweep_feedforward_scale_ * reference_velocity * Eigen::Vector3d::UnitZ();
        *yaw_sweep_ff_acceleration_ =
            yaw_sweep_feedforward_scale_ * reference_acceleration * Eigen::Vector3d::UnitZ();
    }

    void finish_yaw_plant(const char* reason) {
        yaw_plant_active_ = false;
        yaw_plant_override_torque_.store(kNaN);
        if (yaw_plant_csv_.is_open())
            yaw_plant_csv_.close();
        // Let the ordinary yaw controller catch the current position, without a stale target.
        if (command_->sweep_allowed() && barrel_direction_->allFinite()) {
            *yaw_sweep_should_control_ = true;
            *yaw_sweep_control_direction_ = *barrel_direction_;
        }
        RCLCPP_INFO(get_logger(), "Yaw plant test ended (%s); CSV: %s", reason,
                    yaw_plant_csv_path_.c_str());
    }

    void update_yaw_plant() {
        if (yaw_plant_stop_requested_.exchange(false)) {
            yaw_plant_start_requested_.store(false);
            if (yaw_plant_active_)
                finish_yaw_plant("operator stop");
            return;
        }

        if (yaw_plant_start_requested_.exchange(false)) {
            if (yaw_plant_active_ || !command_->sweep_allowed() || yaw_sweep_active_)
                return;
            const double angle = bottom_board_->gimbal_yaw_motor_.angle();
            const double velocity = bottom_board_->gimbal_yaw_motor_.velocity();
            if (!std::isfinite(angle) || !std::isfinite(velocity)
                || std::abs(velocity) > 0.15 || !barrel_direction_->allFinite()) {
                RCLCPP_WARN(get_logger(), "Yaw plant test refused: gimbal is not stationary");
                return;
            }
            const auto epoch_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                      std::chrono::system_clock::now().time_since_epoch())
                                      .count();
            yaw_plant_csv_path_ = "/tmp/yaw_plant_" + std::to_string(epoch_ms) + ".csv";
            yaw_plant_csv_.open(yaw_plant_csv_path_);
            if (!yaw_plant_csv_) {
                RCLCPP_ERROR(get_logger(), "Cannot open yaw plant CSV: %s",
                             yaw_plant_csv_path_.c_str());
                return;
            }
            yaw_plant_csv_ << std::setprecision(10)
                           << "t_s,phase,torque_command_nm,torque_measured_nm,angle_rad,"
                              "velocity_rad_s,gimbal_imu_rad_s,chassis_imu_rad_s\n";
            yaw_plant_start_angle_ = angle;
            yaw_plant_start_time_ = Clock::now();
            yaw_plant_active_ = true;
            RCLCPP_INFO(get_logger(), "Yaw plant test started; CSV: %s",
                        yaw_plant_csv_path_.c_str());
        }

        if (!yaw_plant_active_)
            return;

        const double t =
            std::chrono::duration<double>(Clock::now() - yaw_plant_start_time_).count();
        const double angle = bottom_board_->gimbal_yaw_motor_.angle();
        const double velocity = bottom_board_->gimbal_yaw_motor_.velocity();
        const double displacement = std::remainder(angle - yaw_plant_start_angle_,
                                                    2.0 * std::numbers::pi);
        if (!command_->sweep_allowed() || !std::isfinite(displacement)
            || !std::isfinite(velocity) || std::abs(displacement) > 0.12
            || std::abs(velocity) > 0.9) {
            finish_yaw_plant("remote or motion limit");
            return;
        }
        const int phase = static_cast<int>(t / yaw_plant_phase_seconds_);
        if (phase >= 4 * yaw_plant_cycles_) {
            finish_yaw_plant("completed");
            return;
        }

        // + - - + gives zero net speed and displacement in each ideal cycle.
        const double torque = (phase % 4 == 0 || phase % 4 == 3)
                                  ? yaw_plant_torque_nm_
                                  : -yaw_plant_torque_nm_;
        yaw_plant_override_torque_.store(torque);
        *yaw_sweep_should_control_ = true;
        *yaw_sweep_control_direction_ = *barrel_direction_;
        yaw_plant_csv_ << t << ',' << phase << ',' << torque << ','
                       << bottom_board_->gimbal_yaw_motor_.torque() << ',' << angle << ','
                       << velocity << ',' << top_board_->gimbal_yaw_velocity() << ','
                       << bottom_board_->chassis_yaw_velocity() << '\n';
    }

    class Command : public Component {
    public:
        explicit Command(DeformableInfantryOmniC& deformableInfantry)
            : deformableInfantry(deformableInfantry) {
            register_input("/remote/switch/right", right_switch_);
            register_input("/remote/switch/left", left_switch_);
            register_input("/remote/mouse", mouse_);
        }

        void update() override { deformableInfantry.command_update(); }

        rmcs_msgs::Switch right_switch() const {
            return right_switch_.ready() ? *right_switch_ : rmcs_msgs::Switch::UNKNOWN;
        }

        bool sweep_allowed() const {
            const auto right = right_switch();
            const auto left = left_switch_.ready() ? *left_switch_ : rmcs_msgs::Switch::UNKNOWN;
            const bool controls_enabled = right != rmcs_msgs::Switch::UNKNOWN
                                       && left != rmcs_msgs::Switch::UNKNOWN
                                       && !(left == rmcs_msgs::Switch::DOWN
                                            && right == rmcs_msgs::Switch::DOWN);
            return controls_enabled
                && (right == rmcs_msgs::Switch::UP || (mouse_.ready() && mouse_->right));
        }

        DeformableInfantryOmniC& deformableInfantry;

    private:
        InputInterface<rmcs_msgs::Switch> right_switch_;
        InputInterface<rmcs_msgs::Switch> left_switch_;
        InputInterface<rmcs_msgs::Mouse> mouse_;
    };

    struct TopBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        explicit TopBoard(
            DeformableInfantryOmniC& status, Component& command,
            const std::string& serial_filter = {})
            : status_{status}
            , tf_{status.tf_}
            , bmi088_{device::Bmi088Ekf::Config{
                  .body_to_sensor =
                      Eigen::AngleAxisd{std::numbers::pi / 2.0, Eigen::Vector3d::UnitX()}
                          .toRotationMatrix()}}
            , gimbal_pitch_motor_(status, command, "/gimbal/pitch")
            , gimbal_left_friction_(status, command, "/gimbal/left_friction")
            , gimbal_right_friction_(status, command, "/gimbal/right_friction") {

            gimbal_pitch_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}
                    .set_reversed()
                    .set_encoder_zero_point(
                        static_cast<int>(status.get_parameter("pitch_motor_zero_point").as_int())));

            gimbal_left_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                    .set_reduction_ratio(1.)
                    .set_reversed());
            gimbal_right_friction_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 2}.set_reduction_ratio(
                    1.));

            status.register_output("/gimbal/yaw/velocity_imu", gimbal_yaw_velocity_bmi088_);
            status.register_output("/gimbal/pitch/velocity_imu", gimbal_pitch_velocity_bmi088_);
            status.register_output("/gimbal/auto_aim/imu_snapshot", imu_snapshot_output_);
            status.register_output("/gimbal/auto_aim/exposure_signal", camera_signal_output_);

            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = true;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial_filter, options);

            board_->start_transmit().gpio_digital_read(
                Spec::kGpios.kUart1Rx, //
                {
                    .period_ms = 0,
                    .asap = false,
                    .rising_edge = false,
                    .falling_edge = true,
                    .capture_timestamp = true,
                    .pull = librmcs::data::GpioPull::kUp,
                });

            board_->start_transmit().uart_config(Spec::kUarts.kUart0, {.baudrate = 921600});

            status_.remote_control_->register_vt13(&vt13_);
        }

        ~TopBoard() override = default;

        [[nodiscard]] auto gimbal_yaw_velocity() const -> double {
            return *gimbal_yaw_velocity_bmi088_;
        }

        void request_hard_sync_read() {
            // RMCS-lite top board variant currently has no GPIO hard-sync request
            // path.
        }

        void update() {
            vt13_.update_status();
            gimbal_pitch_motor_.update_status();
            gimbal_left_friction_.update_status();
            gimbal_right_friction_.update_status();

            const double pitch_encoder_angle = gimbal_pitch_motor_.angle();

            if (auto snapshot = bmi088_.snapshot()) {
                *gimbal_pitch_velocity_bmi088_ = snapshot->gyro_body.y();
                *gimbal_yaw_velocity_bmi088_ = snapshot->gyro_body.z();
                tf_->set_transform<rmcs_description::PitchLink, rmcs_description::OdomImu>(
                    snapshot->orientation.conjugate());
            }

            tf_->set_state<rmcs_description::YawLink, rmcs_description::PitchLink>(
                pitch_encoder_angle);
        }

        void command_update() {
            auto builder = board_->start_transmit();
            {
                auto packet = gimbal_pitch_motor_.generate_torque_command();
                builder.can_transmit(
                    Spec::kCans.kCan0, //
                    {
                        .can_id = 0x141,
                        .can_data = packet.as_bytes(),
                    });
            }
            {
                auto packet = device::CanPacket8{uint64_t{0}};
                packet << gimbal_left_friction_;
                builder.can_transmit(
                    Spec::kCans.kCan1, //
                    {
                        .can_id = gimbal_left_friction_.send_id(),
                        .can_data = packet.as_bytes(),
                    });
            }
            {
                auto packet = device::CanPacket8{uint64_t{0}};
                packet << gimbal_right_friction_;
                builder.can_transmit(
                    Spec::kCans.kCan2, //
                    {
                        .can_id = gimbal_right_friction_.send_id(),
                        .can_data = packet.as_bytes(),
                    });
            }
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;
            if (can == Spec::kCans.kCan0) {
                if (data.can_id == 0x141)
                    gimbal_pitch_motor_.store_status(data.can_data);
                monitor_.tick("Top::Can0", data.can_id);
            } else if (can == Spec::kCans.kCan1) {
                gimbal_left_friction_.match_then_store_status(data.can_id, data.can_data);
                monitor_.tick("Top::Can1", data.can_id);
            } else if (can == Spec::kCans.kCan2) {
                gimbal_right_friction_.match_then_store_status(data.can_id, data.can_data);
                monitor_.tick("Top::Can2", data.can_id);
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kUart0)
                vt13_.store_status(data.uart_data);
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            const auto timestamp = board_clock_lifter_.advance_timebase(data.timestamp_quarter_us);
            bmi088_.push_accelerometer_sample(data.x, data.y, data.z, timestamp);
            monitor_.tick("Top::Imu", "Acc");
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            const auto timestamp = board_clock_lifter_.lift_timestamp(data.timestamp_quarter_us);
            monitor_.tick("Top::Imu", "Gyr");
            if (!timestamp.has_value())
                return;
            auto snapshot =
                bmi088_.try_update_with_gyroscope_sample(data.x, data.y, data.z, *timestamp);
            if (snapshot)
                imu_snapshot_output_.emit(*snapshot);
        }

        void gpio_digital_read_result_callback(
            const Spec::Gpio& gpio, const View::GpioDigital& data) override {
            if (gpio != Spec::kGpios.kUart1Rx)
                return;
            if (!data.timestamp_quarter_us)
                return;

            const auto timestamp = board_clock_lifter_.lift_timestamp(*data.timestamp_quarter_us);
            if (!timestamp.has_value())
                return;

            camera_signal_output_.emit(*timestamp);
            monitor_.tick("Top::CameraSync", "Active");
        }

        auto status() const -> std::vector<std::string> { return monitor_.text(); }

        DeformableInfantryOmniC& status_;
        OutputInterface<rmcs_description::Tf>& tf_;
        OutputInterface<double> gimbal_yaw_velocity_bmi088_;
        OutputInterface<double> gimbal_pitch_velocity_bmi088_;

        EventOutputInterface<rmcs_msgs::ImuSnapshot> imu_snapshot_output_;
        EventOutputInterface<rmcs_msgs::BoardClock::time_point> camera_signal_output_;

        device::Bmi088Ekf bmi088_;
        device::BoardClockLifter board_clock_lifter_;
        device::Vt13 vt13_;
        device::LkMotor gimbal_pitch_motor_;
        device::DjiMotor gimbal_left_friction_;
        device::DjiMotor gimbal_right_friction_;

        StatusMonitor monitor_{};
        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;
    };

    struct BottomBoard final : public librmcs::board::RmcsBoardLite::Callback {
    public:
        explicit BottomBoard(
            DeformableInfantryOmniC& status, Component& command,
            const std::string& serial_filter = {})
            : status_{status}
            , command_{command}
            , kChassisRadiusBase(status.get_parameter("chassis_radius").as_double())
            , kRodLength(status.get_parameter("rod_length").as_double())
            , kDefaultRadius(kChassisRadiusBase + kRodLength) {

            status.register_output("/referee/serial", referee_serial_);
            referee_serial_->read = [this](std::byte* buffer, size_t size) {
                return referee_ring_buffer_receive_.pop_front_n(
                    [&buffer](std::byte byte) noexcept { *buffer++ = byte; }, size);
            };
            referee_serial_->write = [this](const std::byte* buffer, size_t size) {
                board_->start_transmit().uart_transmit(
                    Spec::kUarts.kUart0, {.uart_data = std::span<const std::byte>{buffer, size}});
                return size;
            };

            gimbal_yaw_motor_.configure(
                device::LkMotor::Config{device::LkMotor::Type::kMG4010Ei10}.set_encoder_zero_point(
                    static_cast<int>(status.get_parameter("yaw_motor_zero_point").as_int())));

            for (auto& motor : chassis_wheel_motors_)
                motor.configure(
                    device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 1}
                        .set_reduction_ratio(19.0)
                        .enable_multi_turn_angle());

            for (auto& motor : chassis_joint_motors_)
                motor.configure(
                    device::LkMotor::Config{device::LkMotor::Type::kMG5010Ei36}
                        .set_reversed()
                        .enable_multi_turn_angle());

            imu_.set_coordinate_mapping([](double x, double y, double z) {
                // Keep the existing chassis yaw axis mapping explicit until the bottom-board IMU
                // installation is re-validated on hardware.
                return std::make_tuple(-y, x, z);
            });

            gimbal_bullet_feeder_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 3}
                    .enable_multi_turn_angle());

            status.register_output("/chassis/yaw/velocity_imu", chassis_yaw_velocity_imu_, 0);
            status.register_output("/chassis/imu/pitch", chassis_imu_pitch_, 0.0);
            status.register_output("/chassis/imu/roll", chassis_imu_roll_, 0.0);
            status.register_output("/chassis/imu/pitch_rate", chassis_imu_pitch_rate_, 0.0);
            status.register_output("/chassis/imu/roll_rate", chassis_imu_roll_rate_, 0.0);
            for (size_t i = 0; i < 4; ++i) {
                status.register_output(
                    std::format(
                        "/chassis/{}_joint/physical_angle", DeformableInfantryOmniC::kJointName[i]),
                    joint_physical_angle_[i], kNaN);
                status.register_output(
                    std::format(
                        "/chassis/{}_joint/physical_velocity",
                        DeformableInfantryOmniC::kJointName[i]),
                    joint_physical_velocity_[i], kNaN);
            }
            status.register_output("/chassis/encoder/alpha", encoder_alpha_, kNaN);
            status.register_output("/chassis/encoder/alpha_dot", encoder_alpha_dot_, kNaN);
            status.register_output("/chassis/radius", radius_, kDefaultRadius);

            auto options = librmcs::board::AdvancedOptions{};
            options.dangerously_skip_version_checks = true;
            board_ = std::make_unique<librmcs::board::RmcsBoardLite>(*this, serial_filter, options);

            status_.remote_control_->register_dr16(&dr16_);
        }

        void update() {
            imu_.update_status();
            *chassis_yaw_velocity_imu_ = imu_.gz();
            {
                const double q0 = imu_.q0();
                const double q1 = imu_.q1();
                const double q2 = imu_.q2();
                const double q3 = imu_.q3();

                double sin_pitch = 2.0 * (q0 * q2 - q3 * q1);
                sin_pitch = std::clamp(sin_pitch, -1.0, 1.0);

                const double standard_pitch = std::asin(sin_pitch);
                const double standard_roll =
                    std::atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));

                // Export chassis attitude using the requested convention:
                // pitch < 0 when the front is higher, roll > 0 when the left side is higher.
                *chassis_imu_pitch_ = -standard_pitch;
                *chassis_imu_roll_ = standard_roll;
                *chassis_imu_pitch_rate_ = -imu_.gy();
                *chassis_imu_roll_rate_ = imu_.gx();
            }

            for (auto& motor : chassis_wheel_motors_)
                motor.update_status();
            for (auto& motor : chassis_joint_motors_)
                motor.update_status();

            for (size_t i = 0; i < 4; ++i)
                update_joint_physical_feedback_(
                    i, joint_physical_angle_[i], joint_physical_velocity_[i]);

            update_geometry_feedback_();

            dr16_.update_status();
            gimbal_yaw_motor_.update_status();
            if (supercap_status_received_.load(std::memory_order_relaxed))
                supercap_.update_status();
            gimbal_bullet_feeder_.update_status();

            tf_->set_state<rmcs_description::GimbalCenterLink, rmcs_description::YawLink>(
                gimbal_yaw_motor_.angle());
        }

        double chassis_yaw_velocity() const { return *chassis_yaw_velocity_imu_; }

        void command_update(bool even) {
            auto builder = board_->start_transmit();
            if (even) {
                builder.can_transmit(
                    Spec::kCans.kCan0,         //
                    {
                        .can_id = 0x200,
                        .can_data =
                            device::CanPacket8{
                                chassis_wheel_motors_[kLeftFront].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    });
                builder.can_transmit(
                    Spec::kCans.kCan1,         //
                    {
                        .can_id = 0x200,
                        .can_data =
                            device::CanPacket8{
                                chassis_wheel_motors_[kLeftBack].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    });
                auto packet_can2_200 = device::CanPacket8{
                    chassis_wheel_motors_[kRightBack].generate_command(),
                    device::CanPacket8::PaddingQuarter{},
                    gimbal_bullet_feeder_.generate_command(),
                    device::CanPacket8::PaddingQuarter{},
                };
                builder.can_transmit(
                    Spec::kCans.kCan2,         //
                    {
                        .can_id = 0x200,
                        .can_data = packet_can2_200.as_bytes(),
                    });
                builder.can_transmit(
                    Spec::kCans.kCan3,         //
                    {
                        .can_id = 0x200,
                        .can_data =
                            device::CanPacket8{
                                chassis_wheel_motors_[kRightFront].generate_command(),
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                            }
                                .as_bytes(),
                    });
                const double plant_torque = status_.yaw_plant_override_torque_.load();
                auto packet_can2_142 = gimbal_yaw_motor_.generate_command();
                if (std::isfinite(plant_torque)) {
                    packet_can2_142 = gimbal_yaw_motor_.generate_torque_command(plant_torque);
                } else if (status_.yaw_sweep_active_.load()
                           && status_.yaw_sweep_torque_limit_nm_ > 0.0) {
                    const double torque = gimbal_yaw_motor_.control_torque();
                    if (std::isfinite(torque)) {
                        const double limit = status_.yaw_sweep_torque_limit_nm_;
                        packet_can2_142 = gimbal_yaw_motor_.generate_torque_command(
                            std::clamp(torque, -limit, limit));
                    }
                }
                builder.can_transmit(
                    Spec::kCans.kCan2,         //
                    {
                        .can_id = 0x142,
                        .can_data = packet_can2_142.as_bytes(),
                    });
                builder.can_transmit(
                    Spec::kCans.kCan1,         //
                    {
                        .can_id = 0x1FE,
                        .can_data =
                            device::CanPacket8{
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                                device::CanPacket8::PaddingQuarter{},
                                supercap_.generate_command(),
                            }
                                .as_bytes(),
                    });
            } else {
                for (size_t i = 0; i < 4; ++i) {
                    switch (i) {
                    case kLeftFront:
                        builder.can_transmit(
                            Spec::kCans.kCan0, //
                            {
                                .can_id = 0x141,
                                .can_data = chassis_joint_motors_[i].generate_command().as_bytes(),
                            });
                        break;
                    case kLeftBack:
                        builder.can_transmit(
                            Spec::kCans.kCan1, //
                            {
                                .can_id = 0x141,
                                .can_data = chassis_joint_motors_[i].generate_command().as_bytes(),
                            });
                        break;
                    case kRightBack: {
                        auto packet = chassis_joint_motors_[i].generate_command();
                        builder.can_transmit(
                            Spec::kCans.kCan2, //
                            {
                                .can_id = 0x141,
                                .can_data = packet.as_bytes(),
                            });
                        break;
                    }
                    case kRightFront:
                        builder.can_transmit(
                            Spec::kCans.kCan3, //
                            {
                                .can_id = 0x141,
                                .can_data = chassis_joint_motors_[i].generate_command().as_bytes(),
                            });
                        break;
                    default: break;
                    }
                }
            }
        }

        static constexpr double kJointZeroPhysicalAngleRad = 62.5 * std::numbers::pi / 180.0;

        DeformableInfantryOmniC& status_;
        Component& command_;

        std::unique_ptr<librmcs::board::RmcsBoardLite> board_;

        // Interfaces

        OutputInterface<rmcs_description::Tf>& tf_{status_.tf_};

        OutputInterface<double> chassis_yaw_velocity_imu_;
        OutputInterface<double> chassis_imu_pitch_;
        OutputInterface<double> chassis_imu_roll_;
        OutputInterface<double> chassis_imu_pitch_rate_;
        OutputInterface<double> chassis_imu_roll_rate_;

        std::array<OutputInterface<double>, 4> joint_physical_angle_;
        std::array<OutputInterface<double>, 4> joint_physical_velocity_;

        OutputInterface<double> encoder_alpha_;
        OutputInterface<double> encoder_alpha_dot_;
        OutputInterface<double> radius_;

        rmcs_utility::RingBuffer<std::byte> referee_ring_buffer_receive_{256};
        OutputInterface<rmcs_msgs::SerialInterface> referee_serial_;

        // State

        std::atomic<bool> joint_status_received_[4] = {false, false, false, false};

        const double kChassisRadiusBase;
        const double kRodLength;
        const double kDefaultRadius;

        // Device

        device::Bmi088 imu_{1000, 0.2, 0.0};
        device::LkMotor gimbal_yaw_motor_{status_, command_, "/gimbal/yaw"};
        device::Dr16 dr16_;

        device::DjiMotor chassis_wheel_motors_[4]{
            device::DjiMotor{status_, command_, "/chassis/left_front_wheel"},
            device::DjiMotor{status_, command_, "/chassis/left_back_wheel"},
            device::DjiMotor{status_, command_, "/chassis/right_back_wheel"},
            device::DjiMotor{status_, command_, "/chassis/right_front_wheel"},
        };
        device::LkMotor chassis_joint_motors_[4]{
            device::LkMotor{status_, command_, "/chassis/left_front_joint"},
            device::LkMotor{status_, command_, "/chassis/left_back_joint"},
            device::LkMotor{status_, command_, "/chassis/right_back_joint"},
            device::LkMotor{status_, command_, "/chassis/right_front_joint"},
        };

        std::atomic<bool> supercap_status_received_{false};
        device::Supercap supercap_{status_, command_};

        device::DjiMotor gimbal_bullet_feeder_{status_, command_, "/gimbal/bullet_feeder"};

        void process_chassis_can_receive_(size_t index, const View::Can& data) {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (data.can_id == 0x201) {
                chassis_wheel_motors_[index].store_status(data.can_data);
            } else if (data.can_id == 0x141) {
                chassis_joint_motors_[index].store_status(data.can_data);
                joint_status_received_[index].store(true, std::memory_order_relaxed);
            }
        }

        void update_joint_physical_feedback_(
            size_t index, OutputInterface<double>& angle_output,
            OutputInterface<double>& velocity_output) {

            if (!joint_status_received_[index].load(std::memory_order_relaxed)) {
                *angle_output = kNaN;
                *velocity_output = kNaN;
                return;
            }

            const auto to_physical_angle = [](double motor_angle) {
                return kJointZeroPhysicalAngleRad - motor_angle;
            };
            const auto to_physical_velocity = [](double motor_velocity) { return -motor_velocity; };

            *angle_output = to_physical_angle(chassis_joint_motors_[index].angle());
            *velocity_output = to_physical_velocity(chassis_joint_motors_[index].velocity());
        }

        void update_geometry_feedback_() {
            const Eigen::Vector4d alpha_rad{
                *joint_physical_angle_[kLeftFront], *joint_physical_angle_[kLeftBack],
                *joint_physical_angle_[kRightBack], *joint_physical_angle_[kRightFront]};
            const Eigen::Vector4d alpha_dot_rad{
                *joint_physical_velocity_[kLeftFront], *joint_physical_velocity_[kLeftBack],
                *joint_physical_velocity_[kRightBack], *joint_physical_velocity_[kRightFront]};

            if (!alpha_rad.array().isFinite().all() || !alpha_dot_rad.array().isFinite().all()) {
                *encoder_alpha_ = kNaN;
                *encoder_alpha_dot_ = kNaN;
                *radius_ = kDefaultRadius;
                RCLCPP_WARN_THROTTLE(
                    status_.get_logger(), *status_.get_clock(), 1000,
                    "deformable joint feedback invalid, fallback chassis radius to default %.3f m",
                    kDefaultRadius);
                return;
            }

            *encoder_alpha_ = alpha_rad.mean();
            *encoder_alpha_dot_ = alpha_dot_rad.mean();
            *radius_ = (kChassisRadiusBase + kRodLength * alpha_rad.array().cos()).mean();
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission)
                return;
            if (can == Spec::kCans.kCan0) {
                process_chassis_can_receive_(0, data);
                monitor_.tick("Bottom::Can0", data.can_id);
            } else if (can == Spec::kCans.kCan1) {
                process_chassis_can_receive_(1, data);
                if (!data.is_extended_can_id && !data.is_remote_transmission
                    && data.can_id == 0x300) {
                    supercap_.store_status(data.can_data);
                    supercap_status_received_.store(true, std::memory_order_relaxed);
                }
                monitor_.tick("Bottom::Can1", data.can_id);
            } else if (can == Spec::kCans.kCan2) {
                process_chassis_can_receive_(2, data);
                if (data.is_extended_can_id || data.is_remote_transmission)
                    return;
                if (data.can_id == 0x142) {
                    gimbal_yaw_motor_.store_status(data.can_data);
                } else if (data.can_id == 0x203)
                    gimbal_bullet_feeder_.store_status(data.can_data);
                monitor_.tick("Bottom::Can2", data.can_id);
            } else if (can == Spec::kCans.kCan3) {
                process_chassis_can_receive_(3, data);
                monitor_.tick("Bottom::Can3", data.can_id);
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            if (uart == Spec::kUarts.kDbus) {
                dr16_.store_status(data.uart_data.data(), data.uart_data.size());
                monitor_.tick("Bottom::Dbus", "Active");
            } else if (uart == Spec::kUarts.kUart0) {
                const std::byte* ptr = data.uart_data.data();
                referee_ring_buffer_receive_.emplace_back_n(
                    [&ptr](std::byte* storage) noexcept { *storage = *ptr++; },
                    data.uart_data.size());
                monitor_.tick("Bottom::Uart0", "Active");
            }
        }

        void accelerometer_receive_callback(const View::ImuAccelerometer& data) override {
            imu_.store_accelerometer_status(data.x, data.y, data.z);
            monitor_.tick("Bottom::Imu", "Acc");
        }

        void gyroscope_receive_callback(const View::ImuGyroscope& data) override {
            imu_.store_gyroscope_status(data.x, data.y, data.z);
            monitor_.tick("Bottom::Imu", "Gyr");
        }

        auto status() const -> std::vector<std::string> { return monitor_.text(); }

        StatusMonitor monitor_{};
    };

    auto status_service_callback(const std::shared_ptr<std_srvs::srv::Trigger::Response>& response)
        -> void {
        response->success = true;

        auto feedback_message = std::ostringstream{};
        auto text = [&]<typename... Args>(std::format_string<Args...> format, Args&&... args) {
            std::println(feedback_message, format, std::forward<Args>(args)...);
        };

        text("    yaw_motor_zero_point: {}", bottom_board_->gimbal_yaw_motor_.last_raw_angle());
        text("    pitch_motor_zero_point: {}", top_board_->gimbal_pitch_motor_.last_raw_angle());
        constexpr auto kPosition =
            std::array<std::string_view, 4>{"left_front", "left_back", "right_back", "right_front"};

        text("");
        for (auto&& [index, motor] :
             std::views::zip(kPosition, bottom_board_->chassis_joint_motors_)) {
            text("    {}_zero_point: {}", index, motor.last_raw_angle());
        }

        text("\nBottomBoard Status:");
        for (const auto& line : bottom_board_->status())
            text("> {}", line);

        text("\nTopBoard Status:");
        for (const auto& line : top_board_->status())
            text("> {}", line);

        response->message = feedback_message.str();
    }

    OutputInterface<rmcs_description::Tf> tf_;
    OutputInterface<Eigen::Isometry3d> camera_transform_;
    OutputInterface<Eigen::Vector3d> barrel_direction_;
    OutputInterface<double> auto_aim_yaw_velocity_;
    OutputInterface<bool> yaw_sweep_should_control_;
    OutputInterface<Eigen::Vector3d> yaw_sweep_control_direction_;
    OutputInterface<Eigen::Vector3d> yaw_sweep_ff_velocity_;
    OutputInterface<Eigen::Vector3d> yaw_sweep_ff_acceleration_;
    OutputInterface<double> yaw_sweep_target_offset_;
    OutputInterface<double> yaw_sweep_active_output_;
    OutputInterface<double> yaw_sweep_remote_ready_output_;
    OutputInterface<double> yaw_sweep_right_switch_output_;
    OutputInterface<double> yaw_sweep_reference_velocity_;
    OutputInterface<double> yaw_sweep_reference_acceleration_;
    OutputInterface<double> yaw_sweep_ff_scale_output_;
    InputInterface<Clock::time_point> timestamp_;

    std::atomic<bool> yaw_sweep_start_requested_{false};
    std::atomic<bool> yaw_sweep_stop_requested_{false};
    std::atomic<bool> yaw_sweep_remote_ready_{false};
    std::atomic<int> yaw_sweep_right_switch_state_{0};
    std::atomic<bool> yaw_sweep_active_{false};
    double yaw_sweep_amplitude_rad_ = 0.10;
    double yaw_sweep_hold_seconds_ = 1.5;
    int yaw_sweep_cycles_ = 4;
    double yaw_sweep_transition_seconds_ = 0.0;
    double yaw_sweep_feedforward_scale_ = 0.0;
    double yaw_sweep_torque_limit_nm_ = 0.0;
    double yaw_sweep_start_angle_ = 0.0;
    Clock::time_point yaw_sweep_start_time_{};
    Eigen::Vector3d yaw_sweep_center_direction_ = Eigen::Vector3d::UnitX();

    std::atomic<bool> yaw_plant_start_requested_{false};
    std::atomic<bool> yaw_plant_stop_requested_{false};
    std::atomic<double> yaw_plant_override_torque_{kNaN};
    bool yaw_plant_active_ = false;
    double yaw_plant_torque_nm_ = 0.9;
    double yaw_plant_phase_seconds_ = 0.12;
    int yaw_plant_cycles_ = 8;
    double yaw_plant_start_angle_ = 0.0;
    Clock::time_point yaw_plant_start_time_{};
    std::string yaw_plant_csv_path_;
    std::ofstream yaw_plant_csv_;

    std::unique_ptr<BottomBoard> bottom_board_;
    std::unique_ptr<TopBoard> top_board_;
    std::unique_ptr<device::RemoteControl> remote_control_;

    std::shared_ptr<Command> command_;
    uint32_t cmd_tick_ = 0;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> status_service_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> yaw_sweep_start_service_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> yaw_sweep_stop_service_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> yaw_plant_start_service_;
    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> yaw_plant_stop_service_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::DeformableInfantryOmniC, rmcs_executor::Component)
