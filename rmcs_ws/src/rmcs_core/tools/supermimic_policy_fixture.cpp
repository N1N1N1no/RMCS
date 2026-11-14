// Offline sensor-to-policy fixture: no executor, CAN, USB or hardware plugins.
#include "controller/chassis/wheel_leg/policy/supermimic.hpp"
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <span>

using namespace rmcs_core::controller::chassis;
int main() {
    Supermimic adapter;
    SupermimicFullState geometry;
    std::string token;
    std::uint64_t tick = 0;
    std::cout << std::setprecision(17);
    while (std::cin >> token) {
        if (token == "R") {
            adapter.reset();
            std::cout << "RESET" << std::endl;
            continue;
        }
        std::array<double, 28> input{};
        if (token != "S")
            return 2;
        for (double& value : input) {
            if (!(std::cin >> token))
                return 2;
            value = std::stod(token);
        }
        JointVector q{}, dq{}, previous{};
        std::array<double, 3> gyro{}, gravity{}, command{};
        std::copy_n(input.begin(), 6, q.begin());
        std::copy_n(input.begin() + 6, 6, dq.begin());
        std::copy_n(input.begin() + 12, 3, gyro.begin());
        std::copy_n(input.begin() + 15, 3, gravity.begin());
        std::copy_n(input.begin() + 18, 3, command.begin());
        std::copy_n(input.begin() + 22, 6, previous.begin());
        const auto frame = adapter.sample(++tick, q, dq, gyro, gravity, command, input[21], previous, true);
        const auto state = geometry.reconstruct(q, dq);
        if (!frame || !state) {
            std::cout << "INVALID" << std::endl;
            continue;
        }
        const auto target = action_target(kSupermimic, previous, q);
        for (const auto& values : {std::span<const double>(state->q), std::span<const double>(state->dq),
                                   std::span<const double>(*frame), std::span<const double>(target)})
            for (double value : values)
                std::cout << value << ' ';
        std::cout << std::endl;
    }
}
