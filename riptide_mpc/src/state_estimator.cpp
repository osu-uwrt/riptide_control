#include "riptide_mpc/state_estimator.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace riptide_mpc {
namespace {
struct Pose {
    Vector3d position;
    Quaterniond orientation;
};

Pose pose(const YAML::Node &node, const char *name) {
    if (!node)
        throw std::invalid_argument(std::string("vehicle config has no ") + name + " pose");
    const auto v = node.as<std::vector<double>>();
    if (v.size() != 3 && v.size() != 6)
        throw std::invalid_argument(std::string(name) + " pose needs three or six values");
    return {Vector3d(v[0], v[1], v[2]), v.size() == 6 ? rpyToQuaternion(v[3], v[4], v[5]) : Quaterniond::Identity()};
}

// Rotation about world z from `from` to `to`: the twist of the relative
// rotation (swing-twist decomposition). Unlike comparing nose headings, it is
// defined at every attitude, including with the nose straight up or down.
double headingError(const Quaterniond &to, const Quaterniond &from) {
    Quaterniond d = to * from.conjugate();
    if (d.w() < 0)
        d.coeffs() *= -1;
    return 2 * std::atan2(d.z(), d.w());
}
} // namespace

SensorMounts SensorMounts::load(const std::string &vehicle_yaml) {
    const YAML::Node vehicle = YAML::LoadFile(vehicle_yaml);
    const auto com = vehicle["com"].as<std::vector<double>>();
    SensorMounts m;
    // Same interpretation as c_simulator Robot::storeConfigData.
    m.imu = pose(vehicle["imu"]["pose"], "imu").orientation;
    const Pose dvl = pose(vehicle["dvl"]["pose"], "dvl");
    m.dvl = dvl.orientation;
    m.dvl_position = dvl.position - Vector3d(com[0], com[1], com[2]);
    if (vehicle["fog"])
        m.fog_axis = pose(vehicle["fog"]["pose"], "fog").orientation * Vector3d::UnitZ();
    return m;
}

StateEstimator::StateEstimator(FossenModel model, SensorMounts mounts, EstimatorSettings settings)
    : model_(std::move(model)), mounts_(mounts), settings_(settings), actuator_(model_.makeActuator()) {
    if (!(settings_.model_step > 0))
        throw std::invalid_argument("Estimator model_step must be positive");
    mounts_.fog_axis.normalize();
}

double StateEstimator::gain(double elapsed, double time_constant) {
    if (!(time_constant > 0))
        return 1.;
    // A long gap (first sample, dropout) is capped so one sample never jumps the state.
    return -std::expm1(-std::clamp(elapsed, 0., time_constant) / time_constant);
}

Quaterniond StateEstimator::orientation() const {
    return Quaterniond(x_[3], x_[4], x_[5], x_[6]).normalized();
}

void StateEstimator::setOrientation(const Quaterniond &input) {
    const Quaterniond q = input.normalized();
    x_.segment<4>(3) << q.w(), q.x(), q.y(), q.z();
}

void StateEstimator::reset(const State13d &x, double t) {
    x_ = x;
    t_ = t;
    initialized_ = true;
}

void StateEstimator::propagate(double t) {
    if (!initialized_ || t <= t_)
        return;
    const int steps = std::max(1, static_cast<int>(std::ceil((t - t_) / settings_.model_step - 1e-9)));
    const double h = (t - t_) / steps;
    for (int i = 0; i < steps; ++i)
        model_.step(x_, actuator_, h);
    t_ = t;
}

bool StateEstimator::advanceTo(double t) {
    if (!initialized_)
        return false;
    if (t - t_ > settings_.max_propagation) { // stalled: let the EKF re-seed
        initialized_ = false;
        return false;
    }
    propagate(t); // a late measurement is applied to the current estimate
    return true;
}

void StateEstimator::command(double t, const VectorXd &command) {
    advanceTo(t);
    actuator_.command(command);
}

void StateEstimator::stopActuators(double t) {
    advanceTo(t);
    actuator_.stop();
}

double StateEstimator::gate() const {
    const auto factor = [](double value, double scale) {
        return scale > 0 ? 1 / (1 + (value / scale) * (value / scale)) : 1.;
    };
    return factor(model_.baseLinkVelocity(x_).norm(), settings_.disturbance_gate_speed) *
           factor(x_.tail<3>().norm(), settings_.disturbance_gate_rate);
}

void StateEstimator::learnForce(const Vector3d &e) {
    if (!settings_.estimate_disturbance || !(settings_.disturbance_force_time_constant > 0))
        return;
    Vector6d d = model_.disturbance();
    const Vector3d force = model_.dynamics().mass().topLeftCorner<3, 3>() * e;
    d.head<3>() += gate() * (orientation() * force) / settings_.disturbance_force_time_constant;
    if (d.head<3>().norm() > settings_.max_disturbance_force)
        d.head<3>() *= settings_.max_disturbance_force / d.head<3>().norm();
    model_.setDisturbance(d);
}

void StateEstimator::learnTorque(const Vector3d &e) {
    if (!settings_.estimate_disturbance || !(settings_.disturbance_torque_time_constant > 0))
        return;
    Vector6d d = model_.disturbance();
    d.tail<3>() +=
        gate() * model_.dynamics().mass().bottomRightCorner<3, 3>() * e / settings_.disturbance_torque_time_constant;
    if (d.tail<3>().norm() > settings_.max_disturbance_torque)
        d.tail<3>() *= settings_.max_disturbance_torque / d.tail<3>().norm();
    model_.setDisturbance(d);
}

void StateEstimator::imuRate(double t, const Vector3d &rate_imu) {
    if (!advanceTo(t))
        return;
    Vector3d innovation = mounts_.imu * rate_imu - x_.tail<3>();
    if (fresh(last_fog_, settings_.fog_timeout, t)) // the FOG owns its axis
        innovation -= mounts_.fog_axis * mounts_.fog_axis.dot(innovation);
    if (fresh(last_gyro_, settings_.gyro_timeout, t)) // a dropout gap is not model error
        learnTorque(innovation);
    x_.tail<3>() += settings_.gyro_gain * innovation;
    last_gyro_ = t;
}

void StateEstimator::fogRate(double t, double rate) {
    if (!advanceTo(t))
        return;
    const Vector3d innovation = (rate - mounts_.fog_axis.dot(x_.tail<3>())) * mounts_.fog_axis;
    if (fresh(last_fog_, settings_.fog_timeout, t))
        learnTorque(innovation);
    x_.tail<3>() += settings_.fog_gain * innovation;
    last_fog_ = t;
}

void StateEstimator::correctTilt(const Quaterniond &measured, double k) {
    // Gravity direction in body axes; rotating about their cross product fixes
    // roll/pitch without touching heading.
    const Quaterniond q = orientation();
    const Vector3d up_measured = measured.conjugate() * Vector3d::UnitZ();
    const Vector3d up_estimated = q.conjugate() * Vector3d::UnitZ();
    const Vector3d axis = up_measured.cross(up_estimated);
    const double angle = std::atan2(axis.norm(), up_measured.dot(up_estimated));
    if (axis.norm() > 1e-12)
        setOrientation(q * quaternionExp(k * angle * axis.normalized()));
}

void StateEstimator::imuOrientation(double t, const Quaterniond &orientation_imu) {
    if (!advanceTo(t))
        return;
    const double k = gain(t - last_tilt_, settings_.tilt_time_constant);
    correctTilt(orientation_imu.normalized() * mounts_.imu.conjugate(), k);
    last_tilt_ = t;
}

void StateEstimator::dvlVelocity(double t, const Vector3d &velocity_dvl) {
    if (!advanceTo(t))
        return;
    // The DVL measures its own point: v_com + w x r. Using the gyro-fresh rate
    // here is what keeps rotation from being read as translation.
    const Vector3d predicted = x_.segment<3>(7) + x_.tail<3>().cross(mounts_.dvl_position);
    const Vector3d innovation = mounts_.dvl * velocity_dvl - predicted;
    if (fresh(last_dvl_, settings_.dvl_timeout, t))
        learnForce(innovation);
    x_.segment<3>(7) += settings_.dvl_gain * innovation;
    last_dvl_ = t;
}

void StateEstimator::depth(double t, double base_link_z) {
    if (!advanceTo(t))
        return;
    const double k = gain(t - last_depth_, settings_.depth_time_constant);
    x_[2] += k * (base_link_z - model_.baseLinkPosition(x_).z());
    last_depth_ = t;
}

bool StateEstimator::odometry(double t, const Vector3d &p_base, const Quaterniond &q_input, const Vector3d &v_base,
                              const Vector3d &w) {
    const Quaterniond q = q_input.normalized();
    const bool reseed = !advanceTo(t) || (model_.baseLinkPosition(x_) - p_base).norm() > settings_.reset_distance ||
                        orientation().angularDistance(q) > settings_.reset_angle;
    if (reseed) {
        reset(model_.fromBaseLink(p_base, q, v_base, w), initialized_ ? std::max(t, t_) : t);
        last_odom_ = t;
        return true;
    }
    const double elapsed = t - last_odom_;
    last_odom_ = t;

    // Anchors: nothing else measures x/y, and heading must stay in the EKF's frame.
    const double k_xy = gain(elapsed, settings_.horizontal_time_constant);
    x_.head<2>() += k_xy * (p_base - model_.baseLinkPosition(x_)).head<2>();
    const double k_yaw = gain(elapsed, settings_.heading_time_constant);
    setOrientation(Eigen::AngleAxisd(k_yaw * headingError(q, orientation()), Vector3d::UnitZ()) * orientation());

    // Stand-ins for stale sensors.
    const double k = gain(elapsed, settings_.fallback_time_constant);
    const Health h = health(t);
    if (!h.gyro) {
        Vector3d innovation = w - x_.tail<3>();
        if (h.fog)
            innovation -= mounts_.fog_axis * mounts_.fog_axis.dot(innovation);
        x_.tail<3>() += k * innovation;
    } // a fresh IMU gyro already covers the FOG axis when only the FOG is stale
    if (!h.tilt)
        correctTilt(q, k);
    if (!h.dvl)
        x_.segment<3>(7) += k * (v_base - w.cross(model_.baseLinkOffset()) - x_.segment<3>(7));
    if (!h.depth)
        x_[2] += k * (p_base.z() - model_.baseLinkPosition(x_).z());
    return false;
}

StateEstimator::Health StateEstimator::health(double t) const {
    return {fresh(last_gyro_, settings_.gyro_timeout, t), fresh(last_fog_, settings_.fog_timeout, t),
            fresh(last_tilt_, settings_.tilt_timeout, t), fresh(last_dvl_, settings_.dvl_timeout, t),
            fresh(last_depth_, settings_.depth_timeout, t)};
}
} // namespace riptide_mpc
