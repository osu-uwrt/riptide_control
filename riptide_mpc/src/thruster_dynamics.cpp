// Copied from c_simulator (riptide_simulator) so the controller does not depend on
// the simulator. Keep it identical to the simulator's copy: test_dynamics_parity
// compares the two whenever c_simulator is installed.
#include "riptide_mpc/thruster_dynamics.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace riptide_mpc {
void ThrusterDynamics::configure(const std::vector<ThrusterParameters> &p, double timeout) {
    if (p.empty() || !std::isfinite(timeout) || timeout < 0)
        throw std::invalid_argument("Invalid actuator count/timeout");
    for (const auto &a : p) {
        for (double value : {a.delay, a.rise, a.fall, a.slew, a.deadband, a.forwardLimit, a.reverseLimit,
                             a.forwardScale, a.reverseScale, a.efficiency})
            if (!std::isfinite(value) || value < 0)
                throw std::invalid_argument("Thruster parameters must be finite/nonnegative");
        if (a.forwardLimit <= 0 || a.reverseLimit <= 0 || a.efficiency > 1)
            throw std::invalid_argument("Invalid force limits/efficiency");
    }
    parameters_ = p;
    timeout_ = timeout;
    queues_.resize(p.size());
    targets_ = Eigen::VectorXd::Zero(p.size());
    forces_ = targets_;
    reset();
}
void ThrusterDynamics::reset() {
    time_ = 0;
    lastCommand_ = -1e9;
    for (auto &q : queues_)
        q.clear();
    targets_.setZero();
    forces_.setZero();
}
void ThrusterDynamics::stop() {
    for (auto &q : queues_)
        q.clear();
    targets_.setZero();
    lastCommand_ = -1e9;
}
void ThrusterDynamics::command(const Eigen::VectorXd &f) {
    if (f.size() != forces_.size() || !f.allFinite())
        throw std::invalid_argument("Invalid thruster command");
    lastCommand_ = time_;
    for (int i = 0; i < f.size(); ++i) {
        const auto &p = parameters_[i];
        double force = std::abs(f[i]) < p.deadband ? 0 : f[i];
        force *= force >= 0 ? p.forwardScale : p.reverseScale;
        force = std::clamp(force, -p.reverseLimit, p.forwardLimit) * p.efficiency;
        // Latest command at a given physics instant wins, bounding queues at high
        // ROS rates.
        auto &q = queues_[i];
        double due = time_ + p.delay;
        if (!q.empty() && q.back().time == due)
            q.back().force = force;
        else
            q.push_back({due, force});
    }
}
void ThrusterDynamics::evolve(int i, double dt) {
    if (dt <= 0)
        return;
    const auto &p = parameters_[i];
    const double tau =
        (targets_[i] * forces_[i] >= 0 && std::abs(targets_[i]) > std::abs(forces_[i])) ? p.rise : p.fall;
    double delta = tau > 0 ? (targets_[i] - forces_[i]) * (-std::expm1(-dt / tau)) : targets_[i] - forces_[i];
    if (p.slew > 0)
        delta = std::clamp(delta, -p.slew * dt, p.slew * dt);
    forces_[i] += delta;
}
void ThrusterDynamics::advance(double dt) {
    if (!std::isfinite(dt) || dt <= 0)
        throw std::invalid_argument("Actuator step must be finite/positive");
    const double end = time_ + dt;
    // Split exactly at the watchdog boundary before processing delayed commands.
    const double expiry = lastCommand_ + timeout_;
    if (timeout_ > 0 && expiry > time_ && expiry < end) {
        advance(expiry - time_);
        stop();
        advance(end - time_);
        return;
    }
    if (timeout_ > 0 && time_ >= expiry)
        stop();
    for (size_t i = 0; i < parameters_.size(); ++i) {
        auto &q = queues_[i];
        double cursor = time_;
        while (!q.empty() && q.front().time <= end) {
            evolve(i, std::max(0., q.front().time - cursor));
            cursor = std::max(cursor, q.front().time);
            targets_[i] = q.front().force;
            q.pop_front();
        }
        evolve(i, end - cursor);
    }
    time_ = end;
}
} // namespace riptide_mpc
