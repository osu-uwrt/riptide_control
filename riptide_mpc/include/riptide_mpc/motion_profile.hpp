#pragma once

#include <Eigen/Dense>

namespace riptide_mpc {
using Eigen::Quaterniond;
using Eigen::Vector3d;

struct MotionLimits {
    double linear_speed = 0.7;  // m/s; Talos tops out near 0.9 m/s in the sim model
    double linear_accel = 0.5;  // m/s^2
    double linear_jerk = 2.0;   // m/s^3
    // Straight up/down (world z); the linear_* values above are then the
    // horizontal limits, and a diagonal move gets the ellipse between the two.
    // Non-positive: same as horizontal.
    double linear_speed_vertical = -1, linear_accel_vertical = -1, linear_jerk_vertical = -1;
    double angular_speed = 0.5; // rad/s
    double angular_accel = 0.6; // rad/s^2
    double angular_jerk = 3.0;  // rad/s^3
};

// Moving reference that travels to a setpoint with bounded speed, acceleration
// and jerk (S-curve): straight line for position, shortest rotation for
// attitude. It brakes exactly when its jerk-limited stopping distance reaches
// the remaining distance, so it comes to rest on the target with zero velocity
// and acceleration; any sideways motion (a retarget mid-move) is brought to
// zero under the same limits. Everything is in the odometry (world) frame.
struct AxisLimits {
    double speed, accel, jerk;
};
// Linear limits for motion along `direction` (world frame, any length).
AxisLimits linearLimitsAlong(const MotionLimits &limits, const Vector3d &direction);

struct MotionProfile {
    Vector3d position = Vector3d::Zero();
    Vector3d velocity = Vector3d::Zero();
    Vector3d acceleration = Vector3d::Zero();
    Quaterniond orientation = Quaterniond::Identity();
    Vector3d angular_velocity = Vector3d::Zero();
    Vector3d angular_acceleration = Vector3d::Zero();

    // `extra` > 0 plans braking to stop that far beyond `target`, so the target
    // is passed at speed (a path corner); 0 comes to rest on it.
    void stepLinear(const Vector3d &target, const MotionLimits &limits, double dt, double extra = 0);
    void stepAngular(const Quaterniond &target, const MotionLimits &limits, double dt);
    // Where the position would come to rest braking as hard as the limits allow.
    Vector3d stopPoint(const MotionLimits &limits) const;
    // Distance to brake from `speed` (at zero acceleration) to rest, moving along `direction`.
    static double brakingDistance(double speed, const MotionLimits &limits,
                                  const Vector3d &direction = Vector3d::UnitX());
    // Velocity setpoints are only acceleration-limited.
    static Vector3d rateLimit(const Vector3d &current, const Vector3d &target, double accel, double dt);
};
} // namespace riptide_mpc
