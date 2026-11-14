#pragma once
#include "wheelbipe.hpp"
#include "../model/supermimic_full_state.hpp"

namespace rmcs_core::controller::chassis {
inline constexpr PolicyParameters kSupermimic = [] {
    auto p = kWheelbipe;
    p.name = "supermimic";
    p.observation_version = supermimic_data::kVersion;
    p.observation_sha = supermimic_data::kContractSha;
    p.winding_bounds = supermimic_data::kWindingBounds;
    p.model_spec_sha256 = "794232d38b1511d9665cc95fbb2f20cc12bbc6ce0162fd050a9085e1d4620177";
    p.height_max = .44;
    p.high_height_threshold = .42;
    p.high_height_forward_limit = .5;
    p.high_height_yaw_limit = .5;
    return p;
}();

class Supermimic {
public:
    void reset() { initialized_ = false; }
    std::optional<std::array<double, 500>> sample(
        std::uint64_t, const JointVector& position, const JointVector& velocity,
        const std::array<double, 3>& gyro, const std::array<double, 3>& gravity,
        const std::array<double, 3>& command, double height, const JointVector& previous_action,
        bool) {
        if (!finite(gyro) || !finite(gravity) || !finite(command) || !finite(previous_action)
            || !std::isfinite(height)) {
            reset();
            return std::nullopt;
        }
        const auto state = geometry_.reconstruct(position, velocity);
        if (!state) {
            reset();
            return std::nullopt;
        }
        std::array<double, 46> obs{};
        std::size_t cursor = 0;
        for (int i = 0; i < 18; ++i)
            if (i != 8 && i != 10)
                obs[cursor++] = state->q[i] - supermimic_data::kNominal[i];
        for (double v : state->dq)
            obs[cursor++] = v;
        for (double v : gyro)
            obs[cursor++] = v;
        for (double v : gravity)
            obs[cursor++] = v;
        // Controller feedback is in LH,LK,RH,RK,LW,RW; actor uses LH,RH,LK,RK,LW,RW.
        constexpr std::array<int, 6> order{0, 2, 1, 3, 4, 5};
        for (int i : order)
            obs[cursor++] = previous_action[i];
        if (!initialized_) {
            for (int i = 0; i < 10; ++i)
                std::copy(obs.begin(), obs.end(), history_.begin() + i * 46);
            initialized_ = true;
        } else {
            std::move(history_.begin() + 46, history_.end(), history_.begin());
            std::copy(obs.begin(), obs.end(), history_.end() - 46);
        }
        std::array<double, 500> frame{};
        std::copy(history_.begin(), history_.end(), frame.begin());
        for (int i = 0; i < 10; ++i) {
            frame[460 + i * 4] = command[0];
            frame[461 + i * 4] = 0.;
            frame[462 + i * 4] = command[2];
            frame[463 + i * 4] = height;
        }
        return frame;
    }

private:
    template <std::size_t N> static bool finite(const std::array<double, N>& values) {
        return std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); });
    }
    SupermimicFullState geometry_;
    std::array<double, 460> history_{};
    bool initialized_ = false;
};
} // namespace rmcs_core::controller::chassis
