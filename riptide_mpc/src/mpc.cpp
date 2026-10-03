#include "riptide_mpc/mpc.hpp"
#include "riptide_mpc/box_qp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace riptide_mpc {
namespace {
bool samePath(const std::vector<Waypoint> &a, const std::vector<Waypoint> &b) {
    if (a.size() != b.size())
        return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i].position != b[i].position || !a[i].orientation.coeffs().isApprox(b[i].orientation.coeffs(), 0))
            return false;
    return true;
}
} // namespace

MpcController::MpcController(FossenModel model, MpcSettings settings)
    : model_(std::move(model)), settings_(settings), actuator_(model_.makeActuator()) {
    if (!(settings_.dt > 0) || settings_.horizon < 1 || !(settings_.model_step > 0) ||
        !(settings_.linearization_step > 0) || settings_.sqp_iterations < 1)
        throw std::invalid_argument("Invalid MPC settings");
    nu_ = model_.thrusterCount();
    nx_ = 12 + nu_;
    lb_ = model_.commandLowerBound().replicate(settings_.horizon, 1);
    ub_ = model_.commandUpperBound().replicate(settings_.horizon, 1);
    motion_ = settings_.motion;
    last_command_ = VectorXd::Zero(nu_);
    last_deviation_ = issued_feedforward_ = VectorXd::Zero(nu_);
}

void MpcController::advance(double dt) {
    if (dt <= 0)
        return;
    actuator_.advance(dt);
    stepReference(live_, target_, dt);
}

void MpcController::stepReference(StageReference &s, const Reference &r, double dt) const {
    const MotionLimits &m = motion_;
    const bool path = !r.path.empty();
    if (path) { // turn toward the next waypoint once within the corner radius and angle of this one
        s.waypoint = std::min(s.waypoint, r.path.size() - 1);
        while (s.waypoint + 1 < r.path.size() &&
               (s.pose.position - r.path[s.waypoint].position).norm() < r.corner_radius &&
               s.pose.orientation.angularDistance(r.path[s.waypoint].orientation) < r.corner_angle)
            ++s.waypoint;
    }
    if (r.linear_mode == Mode::POSITION) {
        if (settings_.profile_motion && path)
            s.pose.stepLinear(r.path[s.waypoint].position, m, dt, cornerAllowance(s, r));
        else if (settings_.profile_motion)
            s.pose.stepLinear(r.position, m, dt);
        else {
            s.pose.position = r.position;
            s.pose.velocity.setZero();
            s.pose.acceleration.setZero();
        }
    } else if (r.linear_mode == Mode::VELOCITY) {
        s.linear_velocity = settings_.profile_motion
                                ? MotionProfile::rateLimit(s.linear_velocity, r.linear_velocity,
                                                           linearLimitsAlong(m, r.linear_velocity - s.linear_velocity).accel, dt)
                                : r.linear_velocity;
    }
    if (r.angular_mode == Mode::POSITION) {
        if (settings_.profile_motion)
            s.pose.stepAngular(path ? legAttitude(s, r) : r.orientation, m, dt);
        else {
            s.pose.orientation = r.orientation;
            s.pose.angular_velocity.setZero();
            s.pose.angular_acceleration.setZero();
        }
    } else if (r.angular_mode == Mode::VELOCITY) {
        s.angular_velocity =
            settings_.profile_motion
                ? MotionProfile::rateLimit(s.angular_velocity, r.angular_velocity, m.angular_accel, dt)
                : r.angular_velocity;
    }
}

// Attitude on the leg into the current waypoint: the previous waypoint's,
// blended toward this one's as the profile covers the leg, so a heading change
// spreads over the move instead of finishing first at full rate. A leg shorter
// than the corner radius (a turn in place) aims straight at the waypoint.
Quaterniond MpcController::legAttitude(const StageReference &s, const Reference &r) const {
    const Waypoint &to = r.path[s.waypoint];
    const Waypoint &from = s.waypoint > 0 ? r.path[s.waypoint - 1] : r.path_start;
    const double length = (to.position - from.position).norm();
    if (length < r.corner_radius)
        return to.orientation;
    const double covered = std::clamp(1 - (to.position - s.pose.position).norm() / length, 0., 1.);
    return from.orientation.slerp(covered, to.orientation);
}

// Braking allowance past the waypoint being approached: 0 for the last one;
// otherwise arrive at the corner at the speed whose sideways swing after
// turning, (v sin(turn))^2 / 2a, stays within the corner radius, and never plan
// to stop beyond the next waypoint.
double MpcController::cornerAllowance(const StageReference &s, const Reference &r) const {
    if (s.waypoint + 1 >= r.path.size())
        return 0;
    const MotionLimits &m = motion_;
    const Vector3d corner = r.path[s.waypoint].position;
    const Vector3d in = corner - s.pose.position, out = r.path[s.waypoint + 1].position - corner;
    if (in.norm() < 1e-9 || out.norm() < 1e-9)
        return 0;
    const double c = in.normalized().dot(out.normalized());
    const double sin_turn = c > 0 ? std::sqrt(std::max(0., 1 - c * c)) : 1.; // past 90 deg all speed must go
    const AxisLimits l_in = linearLimitsAlong(m, in), l_out = linearLimitsAlong(m, out);
    const double corner_speed = std::min(
        l_in.speed, std::sqrt(2 * std::min(l_in.accel, l_out.accel) * r.corner_radius) / std::max(sin_turn, 1e-3));
    return std::min(MotionProfile::brakingDistance(corner_speed, m, in) - r.corner_radius, out.norm());
}

// Starts the profile from where the vehicle is and how it is moving, on mode
// entry or when the vehicle has fallen too far behind it.
void MpcController::seedReference(const State13d &x, const Reference &r) {
    const Quaterniond q = Quaterniond(x[3], x[4], x[5], x[6]).normalized();
    const Vector3d p = model_.baseLinkPosition(x), v_body = model_.baseLinkVelocity(x), w_body = x.tail<3>();
    if (r.linear_mode != target_.linear_mode) {
        linear_seeded_ = false;
        live_.linear_velocity = r.linear_velocity_in_body ? v_body : Vector3d(q * v_body);
    }
    if (r.angular_mode != target_.angular_mode) {
        angular_seeded_ = false;
        live_.angular_velocity = w_body;
    }
    if (!samePath(r.path, target_.path))
        live_.waypoint = 0;
    if (r.linear_mode == Mode::POSITION &&
        (!linear_seeded_ || (live_.pose.position - p).norm() > settings_.max_reference_lag)) {
        live_.pose.position = p;
        live_.pose.velocity = q * v_body;
        live_.pose.acceleration.setZero();
        linear_seeded_ = true;
    }
    if (r.angular_mode == Mode::POSITION &&
        (!angular_seeded_ || live_.pose.orientation.angularDistance(q) > settings_.max_reference_lag_angle)) {
        live_.pose.orientation = q;
        live_.pose.angular_velocity = q * w_body;
        live_.pose.angular_acceleration.setZero();
        angular_seeded_ = true;
    }
    target_ = r;
}

void MpcController::setActuatorParameters(const std::vector<ThrusterParameters> &parameters) {
    model_.setActuatorParameters(parameters);
    lb_ = model_.commandLowerBound().replicate(settings_.horizon, 1);
    ub_ = model_.commandUpperBound().replicate(settings_.horizon, 1);
    actuator_ = model_.settledActuator(last_command_);
}

void MpcController::setModel(FossenModel model) {
    if (model.thrusterCount() != nu_)
        throw std::invalid_argument("New model has a different thruster count");
    model.setActuatorParameters(model_.actuatorParameters());
    model.setDisturbance(model_.disturbance());
    model_ = std::move(model);
}

void MpcController::issue(const VectorXd &command) {
    actuator_.command(command);
    last_command_ = command;
    last_deviation_ = command - issued_feedforward_;
}

void MpcController::stopActuators() {
    actuator_.stop();
    last_command_.setZero();
    clearWarmStart();
}

void MpcController::resetActuators() {
    actuator_.reset();
    last_command_.setZero();
    clearWarmStart();
}

void MpcController::clearWarmStart() {
    warm_.resize(0);
    issued_feedforward_.setZero();
    linear_seeded_ = angular_seeded_ = false;
}

MpcController::Point MpcController::boxplus(const Point &p, const VectorXd &d) const {
    Point out = p;
    out.x.head<3>() += d.head<3>();
    const Quaterniond q =
        (Quaterniond(p.x[3], p.x[4], p.x[5], p.x[6]) * quaternionExp(d.segment<3>(3))).normalized();
    out.x.segment<4>(3) << q.w(), q.x(), q.y(), q.z();
    out.x.tail<6>() += d.segment<6>(6);
    out.f += d.tail(nu_);
    return out;
}

VectorXd MpcController::boxminus(const Point &a, const Point &b) const {
    VectorXd d(nx_);
    d.head<3>() = a.x.head<3>() - b.x.head<3>();
    d.segment<3>(3) =
        quaternionLog(Quaterniond(b.x[3], b.x[4], b.x[5], b.x[6]).conjugate() * Quaterniond(a.x[3], a.x[4], a.x[5], a.x[6]));
    d.segment<6>(6) = a.x.tail<6>() - b.x.tail<6>();
    d.tail(nu_) = a.f - b.f;
    return d;
}

// One control interval with the command held. `fine` is the exact simulator
// integration; the coarse variant is only used for Jacobians.
MpcController::Point MpcController::stage(const Point &p, const VectorXd &command, bool fine) const {
    const double h_request = fine ? settings_.model_step : settings_.linearization_step;
    const int steps = std::max(1, static_cast<int>(std::lround(settings_.dt / h_request)));
    const double h = settings_.dt / steps;
    const VectorXd target = model_.commandToTarget(command, !fine);
    Point out = p;
    for (int i = 0; i < steps; ++i)
        model_.step(out.x, out.f, target, h);
    return out;
}

MpcController::Output MpcController::output(const State13d &x, const Reference &r, const StageReference &s) const {
    const Quaterniond q = Quaterniond(x[3], x[4], x[5], x[6]).normalized();
    Output y;
    y.segment<3>(0) = model_.baseLinkPosition(x) - s.pose.position;
    y.segment<3>(3) = quaternionLog(s.pose.orientation.conjugate() * q);
    const Vector3d v = model_.baseLinkVelocity(x);
    if (r.linear_mode == Mode::VELOCITY)
        y.segment<3>(6) = r.linear_velocity_in_body ? Vector3d(v - s.linear_velocity)
                                                    : Vector3d(q * v - s.linear_velocity);
    else // POSITION: follow the profile's velocity (zero once it has arrived)
        y.segment<3>(6) = v - q.conjugate() * s.pose.velocity;
    y.segment<3>(9) = r.angular_mode == Mode::VELOCITY
                          ? Vector3d(x.tail<3>() - s.angular_velocity)
                          : Vector3d(x.tail<3>() - q.conjugate() * s.pose.angular_velocity);
    return y;
}

// Inverse dynamics along the reference: the thruster command whose wrench gives
// the reference acceleration at the reference state,
//   tau = M (nu_dot_ref - f(x_ref, tau = 0)),   u = allocate(tau).
// Effort is penalized about this, so the MPC is never paid to lag the reference.
VectorXd MpcController::feedforwardInput(const StageReference &s, const Reference &r, const State13d &x0) const {
    const bool linear_pose = r.linear_mode == Mode::POSITION, angular_pose = r.angular_mode == Mode::POSITION;
    const Quaterniond q = angular_pose ? s.pose.orientation.normalized()
                                       : Quaterniond(x0[3], x0[4], x0[5], x0[6]).normalized();
    const Vector3d p = linear_pose ? s.pose.position : model_.baseLinkPosition(x0);
    Vector3d v_body = Vector3d::Zero(), a_world = Vector3d::Zero();
    if (linear_pose) {
        v_body = q.conjugate() * s.pose.velocity;
        a_world = s.pose.acceleration;
    } else if (r.linear_mode == Mode::VELOCITY) {
        v_body = r.linear_velocity_in_body ? s.linear_velocity : Vector3d(q.conjugate() * s.linear_velocity);
    }
    Vector3d w_body = Vector3d::Zero(), w_dot_body = Vector3d::Zero();
    if (angular_pose) {
        w_body = q.conjugate() * s.pose.angular_velocity;
        w_dot_body = q.conjugate() * s.pose.angular_acceleration;
    } else if (r.angular_mode == Mode::VELOCITY) {
        w_body = s.angular_velocity;
    }
    const State13d x_ref = model_.fromBaseLink(p, q, v_body, w_body);
    // Body-frame derivatives: d/dt(q v) = a  ->  v_dot = q^T a - w x v; COM = base_link - w x r.
    const Vector3d v_dot_base = q.conjugate() * a_world - w_body.cross(v_body);
    Vector6d nu_dot;
    nu_dot << v_dot_base - w_dot_body.cross(model_.baseLinkOffset()), w_dot_body;
    const Vector6d unforced = model_.derivative(x_ref, VectorXd::Zero(nu_)).tail<6>();
    return allocate(x_ref, model_.dynamics().mass() * (nu_dot - unforced));
}

MpcController::Output MpcController::weights(const Reference &r) const {
    Output w = Output::Zero();
    if (r.linear_mode == Mode::POSITION) {
        w.segment<3>(0) = settings_.q_position;
        w.segment<3>(6) = settings_.q_linear_damping;
    } else if (r.linear_mode == Mode::VELOCITY) {
        w.segment<3>(6) = settings_.q_linear_velocity;
    }
    if (r.angular_mode == Mode::POSITION) {
        w.segment<3>(3) = settings_.q_attitude;
        w.segment<3>(9) = settings_.q_angular_damping;
    } else if (r.angular_mode == Mode::VELOCITY) {
        w.segment<3>(9) = settings_.q_angular_velocity;
    }
    return w;
}

// Body wrench per newton of COMMAND, including efficiency and surface losses.
MatrixXd MpcController::effectiveThrusterMatrix(const State13d &x) const {
    MatrixXd T(6, nu_);
    const VectorXd per_newton = model_.commandToTarget(VectorXd::Ones(nu_), true);
    for (int i = 0; i < nu_; ++i) {
        VectorXd unit = VectorXd::Zero(nu_);
        unit[i] = per_newton[i];
        T.col(i) = model_.propulsionWrench(x, unit);
    }
    return T;
}

VectorXd MpcController::allocate(const State13d &x, const Vector6d &wrench) const {
    const MatrixXd T = effectiveThrusterMatrix(x);
    const VectorXd u = T.completeOrthogonalDecomposition().solve(wrench);
    return model_.limitTotalThrust(u.cwiseMax(model_.commandLowerBound()).cwiseMin(model_.commandUpperBound()));
}

MpcOutput MpcController::compute(const State13d &measured, const Reference &reference) {
    const auto start = std::chrono::steady_clock::now();
    MpcOutput out;
    out.command = VectorXd::Zero(nu_);
    auto finish = [&]() {
        out.command = model_.limitTotalThrust(out.command); // hardware power budget
        out.wrench = model_.thrusterMatrix() * model_.commandToTarget(out.command);
        out.solve_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
        return out;
    };

    if (reference.linear_mode == Mode::DISABLED || reference.angular_mode == Mode::DISABLED) {
        clearWarmStart();
        return finish();
    }
    out.active = true;
    if (reference.linear_mode == Mode::FEEDFORWARD && reference.angular_mode == Mode::FEEDFORWARD) {
        clearWarmStart();
        out.command = allocate(measured, reference.feedforward);
        return finish();
    }

    // Initial point: where the vehicle will be when this command starts acting.
    // Everything already sent is known, so roll it through the delay exactly.
    Point x0{measured, actuator_.forces()};
    if (settings_.compensate_delay) {
        ThrusterDynamics actuator = actuator_;
        const double delay = model_.actuatorParameters().front().delay;
        const int steps = static_cast<int>(std::lround(delay / settings_.model_step));
        for (int i = 0; i < steps; ++i)
            model_.step(x0.x, actuator, delay / steps);
        x0.f = actuator.forces();
    }

    // The profile lives on the same clock as x0 (now + delay): a new setpoint
    // starts moving at the first instant thrust can respond, so the reference
    // never gets a head start the vehicle cannot physically recover.
    seedReference(x0.x, reference);
    {
        const auto scale = [](double lag, double start, double stop) {
            return stop > start ? std::clamp(1 - (lag - start) / (stop - start), 0.1, 1.) : 1.;
        };
        const Quaterniond q0(x0.x[3], x0.x[4], x0.x[5], x0.x[6]);
        motion_ = settings_.motion;
        // Near the final stop the profile is left alone: it cannot run away there,
        // and cutting its speed while it brakes makes it overshoot and hunt (the
        // measured lag includes model error in the delay compensation).
        const bool final_leg = reference.path.empty() || live_.waypoint + 1 >= reference.path.size();
        const Vector3d goal = reference.path.empty() ? reference.position : reference.path.back().position;
        const bool braking = final_leg && (goal - live_.pose.position).norm() <=
                                              MotionProfile::brakingDistance(live_.pose.velocity.norm(), motion_,
                                                                             goal - live_.pose.position) +
                                                  settings_.governor_stop_lag;
        if (reference.linear_mode == Mode::POSITION && !braking) {
            const double s_lin = scale((model_.baseLinkPosition(x0.x) - live_.pose.position).norm(),
                                       settings_.governor_lag, settings_.governor_stop_lag);
            motion_.linear_speed *= s_lin;
            motion_.linear_speed_vertical *= s_lin; // non-positive (same as horizontal) stays so
        }
        if (reference.angular_mode == Mode::POSITION)
            motion_.angular_speed *= scale(q0.normalized().angularDistance(live_.pose.orientation),
                                           settings_.governor_angle, settings_.governor_stop_angle);
    }

    // Per-stage references: the profile at the end of each stage.
    const int N = settings_.horizon, nU = N * nu_;
    std::vector<StageReference> stage_reference(N);
    {
        StageReference s = live_;
        for (int k = 0; k < N; ++k) {
            stepReference(s, reference, settings_.dt);
            stage_reference[k] = s;
        }
    }

    // Feedforward per stage, sampled a thruster rise time after the stage
    // midpoint so the command leads the lagging thrust.
    VectorXd U_ref(nU);
    {
        const auto &actuator = model_.actuatorParameters().front();
        StageReference s = live_;
        stepReference(s, reference, settings_.dt / 2 + actuator.rise);
        for (int k = 0; k < N; ++k) {
            U_ref.segment(k * nu_, nu_) = feedforwardInput(s, reference, x0.x);
            stepReference(s, reference, settings_.dt);
        }
    }

    if (warm_.size() != nU) {
        warm_ = U_ref;
    } else { // shift the previous solution by one interval
        warm_.head(nU - nu_) = warm_.tail(nU - nu_).eval();
    }
    VectorXd U = warm_.cwiseMax(lb_).cwiseMin(ub_);
    const Output w_stage = weights(reference);

    std::vector<Point> nominal(N + 1);
    for (int iteration = 0; iteration < settings_.sqp_iterations; ++iteration) {
        nominal[0] = x0;
        for (int k = 0; k < N; ++k)
            nominal[k + 1] = stage(nominal[k], U.segment(k * nu_, nu_), true);

        MatrixXd H = MatrixXd::Zero(nU, nU);
        VectorXd g = VectorXd::Zero(nU);
        MatrixXd P = MatrixXd::Zero(nx_, nU); // d(stage state)/dU
        MatrixXd A(nx_, nx_), B(nx_, nu_), C(12, nx_);
        const double eps = 1e-6;
        for (int k = 0; k < N; ++k) {
            const VectorXd u = U.segment(k * nu_, nu_);
            const Point base = stage(nominal[k], u, false);
            for (int i = 0; i < nx_; ++i) {
                VectorXd d = VectorXd::Zero(nx_);
                d[i] = eps;
                A.col(i) = boxminus(stage(boxplus(nominal[k], d), u, false), base) / eps;
            }
            for (int i = 0; i < nu_; ++i) {
                VectorXd du = u;
                du[i] += eps;
                B.col(i) = boxminus(stage(nominal[k], du, false), base) / eps;
            }
            const int cols = (k + 1) * nu_; // P is zero to the right (causality)
            P.leftCols(cols) = (A * P.leftCols(cols)).eval();
            P.middleCols(k * nu_, nu_) += B;

            const Point &next = nominal[k + 1];
            const Output y = output(next.x, reference, stage_reference[k]);
            for (int i = 0; i < nx_; ++i) {
                VectorXd d = VectorXd::Zero(nx_);
                d[i] = eps;
                C.col(i) = (output(boxplus(next, d).x, reference, stage_reference[k]) - y) / eps;
            }
            const Output w = (k == N - 1 ? settings_.terminal_factor : 1.) * w_stage;
            const MatrixXd M = C * P.leftCols(cols);
            H.topLeftCorner(cols, cols).noalias() += M.transpose() * w.asDiagonal() * M;
            g.head(cols).noalias() += M.transpose() * (w.asDiagonal() * y);
        }
        // Effort and smoothness both act on the deviation from the feedforward,
        // so following the (smooth, jerk-limited) feedforward itself is free.
        H.diagonal().array() += settings_.r_thrust;
        g += settings_.r_thrust * (U - U_ref);
        const VectorXd D = U - U_ref;
        for (int k = 0; k < N; ++k) {
            const VectorXd previous = k == 0 ? last_deviation_ : VectorXd(D.segment((k - 1) * nu_, nu_));
            const VectorXd rate = D.segment(k * nu_, nu_) - previous;
            H.block(k * nu_, k * nu_, nu_, nu_).diagonal().array() += settings_.r_thrust_rate;
            g.segment(k * nu_, nu_) += settings_.r_thrust_rate * rate;
            if (k > 0) {
                H.block((k - 1) * nu_, (k - 1) * nu_, nu_, nu_).diagonal().array() += settings_.r_thrust_rate;
                H.block(k * nu_, (k - 1) * nu_, nu_, nu_).diagonal().array() -= settings_.r_thrust_rate;
                H.block((k - 1) * nu_, k * nu_, nu_, nu_).diagonal().array() -= settings_.r_thrust_rate;
                g.segment((k - 1) * nu_, nu_) -= settings_.r_thrust_rate * rate;
            }
        }

        VectorXd delta = VectorXd::Zero(nU);
        const BoxQpResult qp = solveBoxQp(H, g, lb_ - U, ub_ - U, delta, settings_.qp_max_iterations);
        out.qp_iterations += qp.iterations;
        out.converged = out.converged && qp.converged;
        U = (U + delta).cwiseMax(lb_).cwiseMin(ub_);
    }

    warm_ = U;
    out.command = U.head(nu_);
    issued_feedforward_ = U_ref.head(nu_);
    Point p = x0;
    out.prediction.reserve(N + 1);
    out.prediction.push_back(p.x);
    for (int k = 0; k < N; ++k) {
        p = stage(p, U.segment(k * nu_, nu_), true);
        out.prediction.push_back(p.x);
    }
    return finish();
}
} // namespace riptide_mpc
