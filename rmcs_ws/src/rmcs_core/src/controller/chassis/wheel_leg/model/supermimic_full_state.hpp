#pragma once

#include "supermimic_geometry.hpp"
#include "../joint_coordinates.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace rmcs_core::controller::chassis {

// Six encoders determine all twelve passive coordinates on the rigid CAD
// assembly branch. Seeds select that branch; Newton correction solves the
// actual 3-D spherical closures, including the small CAD axis misalignments.
// No finite differencing of noisy 1 kHz encoders is used for passive velocities.
class SupermimicFullState {
public:
    struct State {
        std::array<double, 18> q, dq;
        double closure_error_m = 0.;
    };

    std::optional<State> reconstruct(const JointVector& measured, const JointVector& speed) const {
        using namespace supermimic_data;
        constexpr std::array<int, 6> order{0, 2, 1, 3, 4, 5};
        if (!finite(measured) || !finite(speed))
            return std::nullopt;
        State state{kNominal, {}};
        for (int i = 0; i < 6; ++i) {
            state.q[kActive[i]] = measured[order[i]];
            state.dq[kActive[i]] = speed[order[i]];
        }
        for (int side = 0; side < 2; ++side) {
            const double delta = (side == 0 ? 1. : -1.)
                                 * (measured[2 * side + 1] - measured[2 * side]);
            const auto& bounds = kWindingBounds[side];
            if (delta < bounds[0] - 1e-6 || delta > bounds[1] + 1e-6)
                return std::nullopt;
            const double u = std::clamp((delta - bounds[0]) / (bounds[1] - bounds[0]) * 70., 0., 70.);
            const int index = std::min(static_cast<int>(u), 69);
            const double blend = u - index;
            const auto& seeds = side == 0 ? kSeed0 : kSeed1;
            const auto& indices = side == 0 ? kSide0 : kSide1;
            for (int j = 0; j < 6; ++j)
                state.q[indices[j]] = (1. - blend) * seeds[index][j] + blend * seeds[index + 1][j];
        }
        Residual error;
        Jacobian jacobian;
        PassiveJacobian passive;
        for (int iteration = 0; iteration < 8; ++iteration) {
            evaluate(state.q, error, jacobian);
            for (int j = 0; j < 12; ++j)
                passive.col(j) = jacobian.col(kPassive[j]);
            if (error.cwiseAbs().maxCoeff() < 1e-10)
                break;
            const Eigen::Matrix<double, 12, 1> step = passive.colPivHouseholderQr().solve(-error);
            if (!step.allFinite() || step.cwiseAbs().maxCoeff() > .25)
                return std::nullopt;
            for (int j = 0; j < 12; ++j)
                state.q[kPassive[j]] += step[j];
        }
        // Recompute after the final step, then differentiate the same closures.
        evaluate(state.q, error, jacobian);
        state.closure_error_m = error.cwiseAbs().maxCoeff();
        if (state.closure_error_m > 2e-7)
            return std::nullopt;
        for (int j = 0; j < 18; ++j)
            if (!std::isfinite(state.q[j]) || state.q[j] < kLower[j] - 1e-5
                || state.q[j] > kUpper[j] + 1e-5)
                return std::nullopt;
        Residual rhs = Residual::Zero();
        for (int j : kActive)
            rhs -= jacobian.col(j) * state.dq[j];
        for (int j = 0; j < 12; ++j)
            passive.col(j) = jacobian.col(kPassive[j]);
        const Eigen::Matrix<double, 12, 1> rates = passive.colPivHouseholderQr().solve(rhs);
        if (!rates.allFinite() || (passive * rates - rhs).cwiseAbs().maxCoeff() > 1e-5)
            return std::nullopt;
        for (int j = 0; j < 12; ++j)
            state.dq[kPassive[j]] = rates[j];
        return state;
    }

private:
    using Residual = Eigen::Matrix<double, 18, 1>;
    using Jacobian = Eigen::Matrix<double, 18, 18>;
    using PassiveJacobian = Eigen::Matrix<double, 18, 12>;
    template <std::size_t N> static bool finite(const std::array<double, N>& values) {
        return std::all_of(values.begin(), values.end(), [](double x) { return std::isfinite(x); });
    }
    static Eigen::Vector3d vector(const std::array<double, 3>& v) { return {v[0], v[1], v[2]}; }

    static void evaluate(const std::array<double, 18>& q, Residual& error, Jacobian& jacobian) {
        using namespace supermimic_data;
        std::array<Eigen::Isometry3d, kJoints.size() + 1> frames;
        std::array<Eigen::Vector3d, 18> axes, pivots;
        frames[0].setIdentity();
        for (const auto& j : kJoints) {
            Eigen::Isometry3d origin;
            origin.matrix() = Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(j.origin.data());
            auto frame = Eigen::Isometry3d(frames[j.parent] * origin);
            if (j.index >= 0) {
                axes[j.index] = frame.linear() * vector(j.axis);
                pivots[j.index] = frame.translation();
                if (j.slide)
                    frame.translate(vector(j.axis) * q[j.index]);
                else
                    frame.rotate(Eigen::AngleAxisd(q[j.index], vector(j.axis)));
            }
            frames[j.child] = frame;
        }
        jacobian.setZero();
        for (int row = 0; row < 6; ++row) {
            const auto& loop = kLoops[row];
            const std::array<int, 2> bodies{loop.body0, loop.body1};
            const std::array<Eigen::Vector3d, 2> points{
                frames[loop.body0] * vector(loop.point0), frames[loop.body1] * vector(loop.point1)};
            error.segment<3>(3 * row) = points[0] - points[1];
            for (int end = 0; end < 2; ++end) {
                for (int body = bodies[end]; body != 0;) {
                    const auto& joint = kJoints[body - 1];
                    if (joint.index >= 0) {
                        const Eigen::Vector3d derivative = joint.slide
                            ? axes[joint.index]
                            : Eigen::Vector3d(axes[joint.index].cross(points[end] - pivots[joint.index]));
                        jacobian.block<3, 1>(3 * row, joint.index) += (end == 0 ? 1. : -1.) * derivative;
                    }
                    body = joint.parent;
                }
            }
        }
    }
};
} // namespace rmcs_core::controller::chassis
