#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"
#include <librmcs/board/c_board.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class GantryFrame
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    GantryFrame()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<GantryFrameCommand>(
                  get_component_name() + "_command", *this))
        , left_motor_(*this, *command_component_, "/test/frame_left_motor")
        , right_motor_(*this, *command_component_, "/test/frame_right_motor") {
        left_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 1}
                .enable_multi_turn_angle());
        right_motor_.configure(
            device::DjiMotor::Config{device::DjiMotor::Type::kM2006, 2}
                .enable_multi_turn_angle());

        remote_control_ = std::make_unique<device::RemoteControl>(*this);
        remote_control_->register_dr16(&dr16_);

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this, get_parameter("board_serial").as_string());
    }

    GantryFrame(const GantryFrame&) = delete;
    GantryFrame& operator=(const GantryFrame&) = delete;
    GantryFrame(GantryFrame&&) = delete;
    GantryFrame& operator=(GantryFrame&&) = delete;

    ~GantryFrame() override = default;

    void update() override {
        left_motor_.update_status();
        right_motor_.update_status();
        dr16_.update_status();
        remote_control_->update();
    }

    void command_update() {
        auto builder = board_->start_transmit();

        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = left_motor_.send_id(),
                .can_data =
                    device::CanPacket8{
                        left_motor_.generate_command(),
                        right_motor_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                    }
                        .as_bytes(),
            });
    }

private:
    class GantryFrameCommand : public rmcs_executor::Component {
    public:
        explicit GantryFrameCommand(GantryFrame& gantry_frame)
            : gantry_frame_(gantry_frame) {}

        void update() override { gantry_frame_.command_update(); }

    private:
        GantryFrame& gantry_frame_;
    };

    void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
        if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
            return;

        if (can != Spec::kCans.kCan1)
            return;

        const auto can_id = data.can_id;
        if (can_id == left_motor_.recv_id()) {
            left_motor_.store_status(data.can_data);
        } else if (can_id == right_motor_.recv_id()) {
            right_motor_.store_status(data.can_data);
        }
    }

    void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
        if (uart == Spec::kUarts.kDbus)
            dr16_.store_status(data.uart_data.data(), data.uart_data.size());
    }

    std::shared_ptr<GantryFrameCommand> command_component_;

    device::DjiMotor left_motor_;
    device::DjiMotor right_motor_;
    device::Dr16 dr16_;

    std::unique_ptr<device::RemoteControl> remote_control_;
    std::unique_ptr<librmcs::board::CBoard> board_;
};

} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::GantryFrame, rmcs_executor::Component)
