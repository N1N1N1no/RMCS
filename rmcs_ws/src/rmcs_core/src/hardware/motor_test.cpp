#include "hardware/device/dji_motor.hpp"
// #include "hardware/device/dr16.hpp"           // [无遥控器] 暂时注释，恢复遥控时取消注释
// #include "hardware/device/remote_control.hpp"  // [无遥控器] 暂时注释，恢复遥控时取消注释
#include "librmcs/board/c_board.hpp"
#include "librmcs/data/datas.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_core::hardware {

class MotorTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    MotorTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<MotorTestCommand>(
                  get_component_name() + "_command", *this)) {

        // [无遥控器] 不需要创建 RemoteControl / 接入 DR16
        // remote_control_ = std::make_unique<device::RemoteControl>(*this);

        board_ = std::make_unique<MotorBoard>(
            *this, *command_component_, get_parameter("board_serial").as_string());
    }

    MotorTest(const MotorTest&) = delete;
    MotorTest& operator=(const MotorTest&) = delete;
    MotorTest(MotorTest&&) = delete;
    MotorTest& operator=(MotorTest&&) = delete;

    ~MotorTest() override = default;

    void update() override {
        board_->update();
        // [无遥控器] 不需要刷新/发布 /remote/* 话题
        // remote_control_->update();
    }

    void command_update() { board_->command_update(); }

private:
    class MotorTestCommand : public rmcs_executor::Component {
    public:
        explicit MotorTestCommand(MotorTest& motor_test)
            : motor_test_(motor_test) {}

        void update() override { motor_test_.command_update(); }

        MotorTest& motor_test_;
    };
    std::shared_ptr<MotorTestCommand> command_component_;

    struct MotorBoard final : librmcs::board::CBoard::Callback {
        explicit MotorBoard(
            MotorTest& motor_test, MotorTestCommand& motor_test_command,
            std::string_view board_serial = {})
            : logger_(motor_test.get_logger())
            , motor_(motor_test, motor_test_command, "/test/motor") {

            motor_.configure(
                device::DjiMotor::Config{device::DjiMotor::Type::kM3508, 3}
                    .enable_multi_turn_angle());

            // [无遥控器] 不把 DR16 注册给 RemoteControl
            // motor_test.remote_control_->register_dr16(&dr16_);

            board_ = std::make_unique<librmcs::board::CBoard>(*this, board_serial);
        }

        void update() {
            motor_.update_status();
            // [无遥控器] 不需要刷新 DR16 状态
            // dr16_.update_status();
        }

        void command_update() {
            auto builder = board_->start_transmit();

            builder.can_transmit(
                Spec::kCans.kCan1,
                {
                    .can_id = 0x200,
                    .can_data =
                        device::CanPacket8{
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            motor_.generate_command(),
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
                if (can_id == 0x203) {
                    motor_.store_status(data.can_data);
                }
            }
        }

        void uart_receive_callback(const Spec::Uart& uart, const View::Uart& data) override {
            // [无遥控器] 不处理 DBUS 遥控数据
            // if (uart == Spec::kUarts.kDbus) {
            //     dr16_.store_status(data.uart_data.data(), data.uart_data.size());
            // }
            (void)uart;
            (void)data;
        }

        rclcpp::Logger logger_;

        device::DjiMotor motor_;
        // device::Dr16 dr16_;   // [无遥控器]

        std::unique_ptr<librmcs::board::CBoard> board_;
    };

    std::shared_ptr<MotorBoard> board_;
    // std::unique_ptr<device::RemoteControl> remote_control_;  // [无遥控器]
};
} // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_core::hardware::MotorTest, rmcs_executor::Component)
