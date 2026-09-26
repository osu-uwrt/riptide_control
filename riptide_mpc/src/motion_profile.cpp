#include "riptide_mpc/motion_profile.hpp"
#include "riptide_mpc/fossen_model.hpp"

#include <algorithm>
#include <cmath>

namespace riptide_mpc {
namespace {
constexpr double kSubstep = 0.002; // s; switching resolution of the jerk decisions

using Limits = AxisLimits;

// Near the target, time-optimal bang-bang jerk switching chatters at the
// sampling resolution. Inside this capture region a critically damped linear
// law (triple pole at -kCapturePole) takes over and settles smoothly.
constexpr double kCapturePole = 10.0; // 1/s
struct Capture {
    double distance, speed, accel;
    // Wider region for states not heading at the target (circling it after a
    // corner or retarget, or backing off it), where bang-bang braking orbits
    // the target instead of settling.
    double stray_distance = 0, stray_speed = 0;
};

// Constant-jerk motion for time t; returns the distance covered.
double advance(double &v, double &a, double j, double t) {
    const double d = v * t + a * t * t / 2 + j * t * t * t / 6;
    v += a * t + j * t * t / 2;
    a += j * t;
    return d;
}

// Distance needed to come to rest (v = a = 0) from (v >= 0, a), braking as hard
// as the limits allow: ramp deceleration up, hold, ramp it back to zero.
double stoppingDistance(double v, double a, const Limits &l) {
    double d = 0;
    if (a > 0) { // first stop accelerating
        d += advance(v, a, -l.jerk, a / l.jerk);
        a = 0;
    }
    if (v <= 0)
        return d;
    const double a0 = -a;
    double peak = std::sqrt((2 * l.jerk * v + a0 * a0) / 2);
    if (peak < a0) { // already braking harder than needed: ease off until stopped
        const double t = (a0 - std::sqrt(std::max(0., a0 * a0 - 2 * l.jerk * v))) / l.jerk;
        return d + advance(v, a, l.jerk, t);
    }
    peak = std::min(peak, l.accel);
    d += advance(v, a, -l.jerk, (peak - a0) / l.jerk);
    const double hold = (v - peak * peak / (2 * l.jerk)) / peak;
    if (hold > 0)
        d += advance(v, a, 0, hold);
    return d + advance(v, a, l.jerk, peak / l.jerk);
}

double clampJerk(double j, double a, const Limits &l, double h) {
    return std::clamp(j, (-l.accel - a) / h, (l.accel - a) / h);
}

// Jerk that drives velocity to `target` with acceleration ending at zero.
double velocityJerk(double v, double a, double target, const Limits &l, double h) {
    const double v_end = v + a * std::abs(a) / (2 * l.jerk); // velocity once a is ramped to 0
    double j;
    if (std::abs(v_end - target) < l.jerk * h * h)
        j = std::clamp(-a / h, -l.jerk, l.jerk);
    else
        j = v_end < target ? l.jerk : -l.jerk;
    return clampJerk(j, a, l, h);
}

// Jerk that follows the hardest-allowed braking plan from (v > 0, a).
double brakingJerk(double v, double a, const Limits &l, double h) {
    if (a > 0)
        return clampJerk(-l.jerk, a, l, h);
    const double a0 = -a;
    double peak = std::sqrt((2 * l.jerk * v + a0 * a0) / 2);
    if (peak < a0)
        return l.jerk;
    peak = std::min(peak, l.accel);
    if (v <= peak * peak / (2 * l.jerk) + 1e-12) // final ramp back to zero
        return l.jerk;
    if (a0 < peak)
        return clampJerk(-std::min(l.jerk, (peak - a0) / h), a, l, h);
    return 0;
}

// One substep toward a point `offset` away (distance and direction); `v`/`a`
// are the rate and its derivative. Braking is planned to stop `extra` beyond
// the point (a path corner is crossed at speed). Returns the displacement to apply.
template <typename LimitsAlong>
Vector3d substep(const Vector3d &offset, Vector3d &v, Vector3d &a, const LimitsAlong &limitsAlong, const Capture &c,
                 double h, double extra = 0) {
    const double distance = offset.norm();
    const bool stray =
        distance < c.stray_distance && v.norm() < c.stray_speed && offset.dot(v) < 0.5 * distance * v.norm();
    if ((distance < c.distance && v.norm() < c.speed && a.norm() < c.accel) || stray) {
        const double p = kCapturePole;
        Vector3d j = p * p * p * offset - 3 * p * p * v - 3 * p * a;
        const double jerk = limitsAlong(j).jerk;
        if (j.norm() > jerk)
            j *= jerk / j.norm();
        const Vector3d d = v * h + a * (h * h / 2) + j * (h * h * h / 6);
        v += a * h + j * (h * h / 2);
        a += j * h;
        return d;
    }
    // Orthonormal basis: toward the target, then the sideways motion to cancel.
    Vector3d e[3];
    const bool travelling = distance > 1e-9;
    e[0] = travelling ? Vector3d(offset / distance)
           : v.norm() > 1e-12 ? Vector3d(v.normalized())
           : a.norm() > 1e-12 ? Vector3d(a.normalized())
                              : Vector3d::UnitX();
    Vector3d side = v - e[0] * e[0].dot(v);
    if (side.norm() < 1e-12)
        side = a - e[0] * e[0].dot(a);
    e[1] = side.norm() > 1e-12 ? Vector3d(side.normalized()) : Vector3d(e[0].unitOrthogonal());
    e[2] = e[0].cross(e[1]);

    Vector3d displacement = Vector3d::Zero(), v_new = Vector3d::Zero(), a_new = Vector3d::Zero();
    for (int i = 0; i < 3; ++i) {
        const Limits l = limitsAlong(e[i]);
        double vi = e[i].dot(v), ai = e[i].dot(a), j;
        bool braking = false;
        if (i == 0 && travelling) {
            j = velocityJerk(vi, ai, l.speed, l, h);
            double vc = vi, ac = ai;
            const double dc = advance(vc, ac, j, h);
            if (vc > 0 && dc + stoppingDistance(vc, std::clamp(ac, -l.accel, l.accel), l) > distance + extra) {
                j = brakingJerk(vi, ai, l, h);
                braking = true;
            }
        } else {
            j = velocityJerk(vi, ai, 0, l, h);
        }
        double d = advance(vi, ai, j, h);
        if (braking && vi < 0) { // came to rest a hair early; never reverse
            vi = ai = 0;
            d = std::max(d, 0.);
        }
        displacement += e[i] * d;
        v_new += e[i] * vi;
        a_new += e[i] * ai;
    }
    v = v_new;
    a = a_new;
    return displacement;
}

bool atRest(double distance, const Vector3d &v, const Vector3d &a) {
    return distance < 2e-4 && v.norm() < 2e-3 && a.norm() < 2e-2;
}
} // namespace

Vector3d MotionProfile::rateLimit(const Vector3d &current, const Vector3d &target, double accel, double dt) {
    const Vector3d change = target - current;
    const double limit = accel * dt;
    return change.norm() > limit ? Vector3d(current + change * (limit / change.norm())) : target;
}

AxisLimits linearLimitsAlong(const MotionLimits &m, const Vector3d &direction) {
    const double n = direction.norm();
    const double z = n > 1e-12 ? std::min(1., std::abs(direction.z()) / n) : 0., h = std::sqrt(1 - z * z);
    const auto blend = [&](double horizontal, double vertical) {
        return vertical > 0 ? 1 / std::hypot(h / horizontal, z / vertical) : horizontal;
    };
    return {blend(m.linear_speed, m.linear_speed_vertical), blend(m.linear_accel, m.linear_accel_vertical),
            blend(m.linear_jerk, m.linear_jerk_vertical)};
}

double MotionProfile::brakingDistance(double speed, const MotionLimits &m, const Vector3d &direction) {
    return stoppingDistance(speed, 0, linearLimitsAlong(m, direction));
}

Vector3d MotionProfile::stopPoint(const MotionLimits &m) const {
    const double speed = velocity.norm();
    if (speed < 1e-9)
        return position;
    const Vector3d direction = velocity / speed;
    const Limits l = linearLimitsAlong(m, direction);
    return position + direction * stoppingDistance(speed, std::clamp(direction.dot(acceleration), -l.accel, l.accel), l);
}

void MotionProfile::stepLinear(const Vector3d &target, const MotionLimits &m, double dt, double extra) {
    const auto l = [&m](const Vector3d &direction) { return linearLimitsAlong(m, direction); };
    const int steps = static_cast<int>(std::ceil(dt / kSubstep - 1e-9));
    for (int i = 0; i < steps; ++i) {
        if (extra == 0 && atRest((target - position).norm(), velocity, acceleration)) {
            position = target;
            velocity.setZero();
            acceleration.setZero();
            return;
        }
        position += substep(target - position, velocity, acceleration, l, {2e-3, 2e-2, 1e-1, 2e-2, 1e-1}, dt / steps, extra);
    }
}

void MotionProfile::stepAngular(const Quaterniond &target_input, const MotionLimits &m, double dt) {
    const Limits limits{m.angular_speed, m.angular_accel, m.angular_jerk};
    const auto l = [&limits](const Vector3d &) { return limits; };
    const Quaterniond target = target_input.normalized();
    const int steps = static_cast<int>(std::ceil(dt / kSubstep - 1e-9));
    for (int i = 0; i < steps; ++i) {
        const Vector3d offset = quaternionLog(target * orientation.conjugate()); // world frame
        if (atRest(offset.norm(), angular_velocity, angular_acceleration)) {
            orientation = target;
            angular_velocity.setZero();
            angular_acceleration.setZero();
            return;
        }
        orientation =
            (quaternionExp(substep(offset, angular_velocity, angular_acceleration, l, {5e-3, 3e-2, 1.5e-1}, dt / steps)) *
             orientation)
                .normalized();
    }
}
} // namespace riptide_mpc
