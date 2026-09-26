#pragma once

#include "riptide_mpc/mpc.hpp"

#include <optional>

namespace riptide_mpc {
// "Has the vehicle settled on its setpoint?" in [0, 1], published on
// controller/scale/trust for autonomy's WaitForController (every mission tree
// waits for all axes > 0.8-0.99). Same shape as complete_controller's steady-state
// trust: restarts at 0 when the setpoint moves, grows toward 1 while settled,
// decays otherwise. Settled here means the motion profile has reached the
// setpoint and the vehicle is on it and still, with thresholds matched to what
// the MPC actually holds rather than the old controller's.
struct SettleTrustSettings {
    double growth_tau = 0.25;               // s
    double decay_tau = 1.0;                 // s; one noisy sample barely dents it
    double position_threshold = 0.02;       // m
    double attitude_threshold = 0.035;      // rad (2 deg)
    double linear_velocity_threshold = 0.03; // m/s
    double angular_velocity_threshold = 0.03; // rad/s
};

class SettleTrust {
  public:
    explicit SettleTrust(SettleTrustSettings settings = SettleTrustSettings()) : settings_(settings) {}

    // position/orientation of base_link, twist = [base_link velocity, body rates].
    double update(double dt, const Reference &r, const MotionProfile &profile, const Vector3d &position,
                  const Quaterniond &orientation, const Vector6d &twist) {
        if (r.linear_mode != Mode::POSITION || r.angular_mode != Mode::POSITION) {
            reset();
            return trust_;
        }
        const bool moved = !last_position_ || (r.position - *last_position_).norm() > 1e-4 ||
                           last_orientation_->angularDistance(r.orientation) > 1e-4;
        last_position_ = r.position;
        last_orientation_ = r.orientation;
        const bool arrived = (profile.position - r.position).norm() < 1e-6 &&
                             profile.orientation.angularDistance(r.orientation) < 1e-6;
        const bool settled = arrived && (position - r.position).norm() < settings_.position_threshold &&
                             orientation.angularDistance(r.orientation) < settings_.attitude_threshold &&
                             twist.head<3>().norm() < settings_.linear_velocity_threshold &&
                             twist.tail<3>().norm() < settings_.angular_velocity_threshold;
        if (moved)
            trust_ = 0;
        else if (settled)
            trust_ = 1 - std::exp(-dt / settings_.growth_tau) * (1 - trust_);
        else
            trust_ *= std::exp(-dt / settings_.decay_tau);
        return trust_;
    }

    void reset() {
        trust_ = 0;
        last_position_.reset();
        last_orientation_.reset();
    }
    double trust() const {
        return trust_;
    }

  private:
    SettleTrustSettings settings_;
    double trust_ = 0;
    std::optional<Vector3d> last_position_;
    std::optional<Quaterniond> last_orientation_;
};
} // namespace riptide_mpc
