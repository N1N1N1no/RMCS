#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>
#include <vector>

namespace rmcs_core::controller::gimbal {

// Empirical joint-load residual, including cable and friction. No error input or PID integral.
class PitchLoadFeedforward {
public:
    struct Config {
        double scale = 0., torque_limit = 1., slew_rate = .2;
        double direction_deadband = .5 * std::numbers::pi / 180.;
        std::vector<double> angles, up_torques, down_torques;
        void validate() const {
            for (double v : {scale, torque_limit, slew_rate, direction_deadband})
                if (!std::isfinite(v))
                    throw std::invalid_argument("non-finite Pitch load parameter");
            if (scale < 0. || scale > 1. || torque_limit <= 0. || torque_limit > 1.
                || slew_rate <= 0. || direction_deadband <= 0.)
                throw std::invalid_argument("invalid Pitch load bounds");
            if (scale == 0.)
                return;
            if (angles.size() < 2 || up_torques.size() != angles.size()
                || down_torques.size() != angles.size())
                throw std::invalid_argument("Pitch load tables must have matching sizes");
            for (std::size_t i = 0; i < angles.size(); ++i) {
                if (!std::isfinite(angles[i]) || std::abs(angles[i]) >= std::numbers::pi
                    || (i && angles[i] <= angles[i - 1]) || !std::isfinite(up_torques[i])
                    || !std::isfinite(down_torques[i]) || std::abs(up_torques[i]) > 1.
                    || std::abs(down_torques[i]) > 1.)
                    throw std::invalid_argument("invalid Pitch load table entries");
            }
        }
    };
    explicit PitchLoadFeedforward(Config c)
        : config_(std::move(c)) {
        config_.validate();
    }
    bool enabled() const { return config_.scale > 0.; }
    void reset() {
        direction_ = 0;
        initialized_ = false;
        output_ = 0.;
    }
    double update(double angle, double dt) {
        if (!enabled())
            return 0.;
        if (!std::isfinite(angle) || !std::isfinite(dt)
            || dt <= 0.) {
            reset();
            return std::numeric_limits<double>::quiet_NaN();
        }
        const double q = std::remainder(angle, 2. * std::numbers::pi);
        if (!initialized_) {
            initialized_ = true;
            anchor_ = q;
        } else if (direction_ == -1) {
            if (q < anchor_)
                anchor_ = q;
            else if (q - anchor_ >= config_.direction_deadband) {
                direction_ = 1;
                anchor_ = q;
            }
        } else if (direction_ == 1) {
            if (q > anchor_)
                anchor_ = q;
            else if (anchor_ - q >= config_.direction_deadband) {
                direction_ = -1;
                anchor_ = q;
            }
        } else if (std::abs(q - anchor_) >= config_.direction_deadband) {
            direction_ = q > anchor_ ? 1 : -1;
            anchor_ = q;
        }
        const double up = interpolate(q, config_.up_torques),
                     down = interpolate(q, config_.down_torques);
        const double target = std::clamp(
            config_.scale
                * (direction_ < 0   ? up
                   : direction_ > 0 ? down
                                    : .5 * (up + down)),
            -config_.torque_limit, config_.torque_limit);
        // Bound wall-time jumps; this limits table-output slew, never accumulates tracking error.
        const double step = config_.slew_rate * std::min(dt, .01);
        output_ += std::clamp(target - output_, -step, step);
        return output_;
    }
    int direction() const { return direction_; }

private:
    double interpolate(double q, const std::vector<double>& values) const {
        if (q <= config_.angles.front())
            return values.front();
        if (q >= config_.angles.back())
            return values.back();
        const auto hi = static_cast<std::size_t>(
            std::upper_bound(config_.angles.begin(), config_.angles.end(), q)
            - config_.angles.begin());
        const double f =
            (q - config_.angles[hi - 1]) / (config_.angles[hi] - config_.angles[hi - 1]);
        return values[hi - 1] + f * (values[hi] - values[hi - 1]);
    }
    Config config_;
    bool initialized_ = false;
    int direction_ = 0;
    double anchor_ = 0., output_ = 0.;
};
} // namespace rmcs_core::controller::gimbal
