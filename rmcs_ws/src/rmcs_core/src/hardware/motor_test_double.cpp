#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "librmcs/board/c_board.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class MotorTestDouble
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MotorTestDouble()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<MotorTestDoubleCommand>(
                  get_component_name() + "_command", *this)) {


        board_ = std::make_unique<MotorBoard>(
            *this, *command_component_, get_parameter("board_serial").as_string());
    }

    MotorTestDouble(const MotorTestDouble&) = delete;
    MotorTestDouble& operator=(const MotorTestDouble&) = delete;
    MotorTestDouble(MotorTestDouble&&) = delete;
    MotorTestDouble& operator=(MotorTestDouble&&) = delete;

    ~MotorTestDouble() override = default;

    void update() override {
        board_->update();

    }

    void command_update() { board_->command_update(); }

private:
    class MotorTestDoubleCommand : public rmcs_executor::Component {
    public:
        explicit MotorTestDoubleCommand(MotorTestDouble& motor_test)
            : motor_test_(motor_test) {}

        void update() override { motor_test_.command_update(); }

        MotorTestDouble& motor_test_;
    };
    std::shared_ptr<MotorTestDoubleCommand> command_component_;

    struct MotorBoard final : librmcs::board::CBoard::Callback {
        explicit MotorBoard(
            MotorTestDouble& motor_test, MotorTestDoubleCommand& motor_test_command,
            std::string_view board_serial = {})
            : logger_(motor_test.get_logger())
            , motor_(motor_test, motor_test_command, "/test/motor") {

            motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kGM6020, 1}
                    .enable_multi_turn_angle());

            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
        }

        void update() {
            motor_.update_status();

        }

        void command_update() {
            auto builder = board_->start_transmit();

            builder.can_transmit(
                Spec::kCans.kCan1,
                {
                    .can_id = motor_.send_id(),
                    .can_data =
                        device::CanPacket8{
                            motor_.generate_command(),
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                        }
                            .as_bytes(),
                });
        }

        void can_receive_callback(const Spec::Can& can, const View::Can& data) override {
            if (data.is_extended_can_id || data.is_remote_transmission) [[unlikely]]
                return;

            auto can_id = data.can_id;

            if (can == Spec::kCans.kCan1) {
                if (can_id == motor_.recv_id()) {
                    motor_.store_status(data.can_data);
                }
            }
        }


        rclcpp::Logger logger_;

        device::DjiMotor motor_;

        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    std::shared_ptr<MotorBoard> board_;
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::MotorTestDouble, rmcs_executor::Component)
