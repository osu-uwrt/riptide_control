#include "riptide_mpc/fossen_model.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace riptide_mpc {
namespace {
constexpr double kGravity = 9.80665; // c_simulator settings.h GRAVITY

Vector3d vector3(const YAML::Node &node, const char *name) {
    const auto v = node.as<std::vector<double>>();
    if (v.size() < 3)
        throw std::invalid_argument(std::string(name) + ": expected at least three values");
    return Vector3d(v[0], v[1], v[2]);
}

Matrix6d matrix6(const YAML::Node &n, const char *name) {
    Matrix6d m;
    if (n.size() == 36) {
        for (int i = 0; i < 36; ++i)
            m(i / 6, i % 6) = n[i].as<double>();
    } else if (n.size() == 6) {
        for (int r = 0; r < 6; ++r) {
            if (n[r].size() != 6)
                throw std::invalid_argument(std::string(name) + ": expected 6x6 matrix");
            for (int c = 0; c < 6; ++c)
                m(r, c) = n[r][c].as<double>();
        }
    } else
        throw std::invalid_argument(std::string(name) + ": expected flat 36-element or nested 6x6 matrix");
    return m;
}

} // namespace

Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
    return (Eigen::AngleAxisd(yaw, Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch, Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Vector3d::UnitX()))
        .normalized();
}

Quaterniond quaternionExp(const Vector3d &v) {
    const double angle = v.norm();
    if (angle < 1e-12)
        return Quaterniond(1., v.x() / 2, v.y() / 2, v.z() / 2).normalized();
    return Quaterniond(Eigen::AngleAxisd(angle, v / angle));
}

Vector3d quaternionLog(const Quaterniond &input) {
    Quaterniond q = input.normalized();
    if (q.w() < 0) // shortest rotation
        q.coeffs() *= -1;
    const double s = q.vec().norm();
    if (s < 1e-12)
        return 2. * q.vec();
    return 2. * std::atan2(s, q.w()) * q.vec() / s;
}

FossenModel FossenModel::load(const std::string &vehicle_yaml, const std::string &hydrodynamics_yaml) {
    return fromNodes(YAML::LoadFile(vehicle_yaml), YAML::LoadFile(hydrodynamics_yaml));
}

FossenModel FossenModel::fromNodes(const YAML::Node &vehicle, const YAML::Node &hydro) {
    if (hydro["schema_version"].as<int>() != 1)
        throw std::invalid_argument("Unsupported hydrodynamic schema");

    FossenModel model;
    // Robot::storeConfigData, line for line where it affects the dynamics.
    model.mass_ = vehicle["mass"].as<double>();
    const Vector3d com = vector3(vehicle["com"], "com");
    const auto inertia = hydro["rigid_body_inertia3x3"].as<std::vector<double>>();
    if (inertia.size() != 9)
        throw std::invalid_argument("Expected row-major 3x3 rigid inertia");
    Eigen::Matrix3d body_inertia;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            body_inertia(r, c) = inertia[r * 3 + c];
    model.dynamics_.configure(model.mass_, body_inertia, matrix6(hydro["added_mass6x6"], "added_mass6x6"));
    const auto quadratic = hydro["quadratic_damping"].as<std::vector<double>>();
    if (quadratic.size() != 6)
        throw std::invalid_argument("Expected six quadratic damping coefficients");
    model.dynamics_.configureDamping(matrix6(hydro["linear_damping6x6"], "linear_damping6x6"),
                                     Vector6d(Eigen::Map<const Vector6d>(quadratic.data())),
                                     vector3(hydro["damping_center_relative"], "damping_center_relative"));
    model.dynamics_.configureHydrostatics(hydro["water_density"].as<double>(), hydro["displaced_volume"].as<double>(),
                                          vector3(hydro["cob_relative"], "cob_relative"),
                                          vector3(hydro["buoyancy_radii"], "buoyancy_radii"), kGravity,
                                          hydro["water_level"].as<double>(0));
    model.water_current_ = vector3(hydro["current_velocity"], "current_velocity");
    model.base_link_offset_ = vector3(vehicle["base_link"], "base_link") - com;

    const YAML::Node dynamics = hydro["thruster_dynamics"];
    const YAML::Node thrusters = vehicle["thrusters"];
    const auto efficiencies = hydro["thruster_efficiencies"].as<std::vector<double>>();
    if (efficiencies.size() != thrusters.size())
        throw std::invalid_argument("Expected one efficiency per thruster");
    // Optional per-thruster forward/reverse scales (identified per thruster); else thruster_dynamics' shared ones.
    const auto perThruster = [&](const char *key, double shared) {
        std::vector<double> v(thrusters.size(), shared);
        if (hydro[key]) {
            v = hydro[key].as<std::vector<double>>();
            if (v.size() != thrusters.size())
                throw std::invalid_argument(std::string("Expected one ") + key + " entry per thruster");
        }
        return v;
    };
    const auto forward_scales = perThruster("thruster_forward_scales", dynamics["forward_scale"].as<double>(1.0));
    const auto reverse_scales = perThruster("thruster_reverse_scales", dynamics["reverse_scale"].as<double>(1.0));
    model.propeller_radius_ = dynamics["propeller_radius"].as<double>(.05);
    model.command_timeout_ = dynamics["command_timeout"].as<double>(.5);
    const double max_thrust = dynamics["forward_max_force"].as<double>();
    model.thruster_matrix_.resize(6, thrusters.size());
    for (std::size_t i = 0; i < thrusters.size(); ++i) {
        ThrusterParameters p;
        p.delay = dynamics["delay"].as<double>(0.1);
        p.rise = dynamics["rise_time_constant"].as<double>(0.08);
        p.fall = dynamics["fall_time_constant"].as<double>(0.06);
        p.slew = dynamics["slew_rate"].as<double>(0.0);
        p.deadband = dynamics["force_deadband"].as<double>(0.0);
        p.forwardLimit = max_thrust;
        p.reverseLimit = dynamics["reverse_max_force"].as<double>(max_thrust);
        p.forwardScale = forward_scales[i];
        p.reverseScale = reverse_scales[i];
        p.startup = dynamics["startup_time_constant"].as<double>(0.0);
        p.startupForce = dynamics["startup_force"].as<double>(0.0);
        p.efficiency = efficiencies[i];
        model.actuators_.push_back(p);

        const auto pose = thrusters[i]["pose"].as<std::vector<double>>();
        if (pose.size() != 6)
            throw std::invalid_argument("Expected six thruster pose values");
        const Vector3d position = Vector3d(pose[0], pose[1], pose[2]) - com;
        const Vector3d direction = rpyToQuaternion(pose[3], pose[4], pose[5]) * Vector3d::UnitX();
        model.positions_.push_back(position);
        model.directions_.push_back(direction);
        model.thruster_matrix_.col(i) << direction, position.cross(direction);
    }
    model.makeActuator(); // validates the actuator parameters

    if (const YAML::Node hw = hydro["hardware"]) {
        auto curve = [&](const char *key) {
            const auto c = hw[key].as<std::vector<double>>();
            if (c.size() != 4)
                throw std::invalid_argument(std::string("hardware.") + key + ": expected four coefficients");
            return std::array<double, 4>{c[0], c[1], c[2], c[3]};
        };
        auto thrust = [&](const char *key) {
            const auto k = hw[key].as<std::vector<double>>();
            if (k.size() != 2 || !(k[0] > 0) || !std::isfinite(k[1]))
                throw std::invalid_argument(std::string("hardware.") + key + ": expected [k2 > 0, k1]");
            return std::array<double, 2>{k[0], k[1]};
        };
        auto coefficient = [&](const char *key) {
            const auto k = hw[key].as<std::vector<double>>();
            if (k.size() != 2 || !(k[0] > 0) || !(k[1] >= 0) || !std::isfinite(k[0] + k[1]))
                throw std::invalid_argument(std::string("hardware.") + key + ": expected [a > 0, b >= 0]");
            return std::array<double, 2>{k[0], k[1]};
        };
        model.hardware_.present = true;
        model.hardware_.total_thrust_limit = hw["total_thrust_limit"].as<double>(0.);
        const bool propeller = hw["thrust_coefficient_forward"] || hw["thrust_coefficient_reverse"];
        const bool quadratic = hw["thrust_curve_forward"] || hw["thrust_curve_reverse"];
        const bool legacy = hw["force_to_rpm_positive"] || hw["force_to_rpm_negative"];
        if (propeller + quadratic + legacy > 1)
            throw std::invalid_argument(
                "hardware: give one of thrust_coefficient_*, thrust_curve_* or force_to_rpm_*");
        model.hardware_.propeller = propeller;
        model.hardware_.quadratic = quadratic;
        if (propeller) {
            const double diameter = hw["propeller_diameter"].as<double>(0.);
            const double rho = hydro["water_density"].as<double>(0.);
            if (!(diameter > 0) || !(rho > 0))
                throw std::invalid_argument("hardware.thrust_coefficient_*: need propeller_diameter > 0 and "
                                            "water_density > 0");
            model.hardware_.kt_forward = coefficient("thrust_coefficient_forward");
            model.hardware_.kt_reverse = coefficient("thrust_coefficient_reverse");
            model.hardware_.kt_scale = rho * std::pow(diameter, 4) / 3600.0;
        } else if (quadratic) {
            model.hardware_.thrust_forward = thrust("thrust_curve_forward");
            model.hardware_.thrust_reverse = thrust("thrust_curve_reverse");
        } else {
            model.hardware_.rpm_positive = curve("force_to_rpm_positive");
            model.hardware_.rpm_negative = curve("force_to_rpm_negative");
        }
    }
    return model;
}

double HardwareConfig::forceToRpm(double f) const {
    if (f == 0 || !std::isfinite(f))
        return 0; // the fitted curves are meaningless at zero; stop the motor
    if (propeller) {
        // Root of kt_scale (a + b R) R^2 = |F|. Convex and increasing for R > 0, and R0 (b = 0) is at or right
        // of the root, so Newton descends monotonically onto it.
        const auto &[a, b] = f > 0 ? kt_forward : kt_reverse;
        const double target = std::abs(f) / kt_scale;
        double r = std::sqrt(target / a);
        for (int i = 0; i < 50; ++i) {
            const double step = ((a + b * r) * r * r - target) / ((2 * a + 3 * b * r) * r);
            r -= step;
            if (std::abs(step) <= 1e-12 * r)
                break;
        }
        return std::copysign(r, f);
    }
    if (quadratic) {
        // Positive root of k2 R^2 + k1 R = |F|, in the form without cancellation for either sign of k1.
        const auto &[k2, k1] = f > 0 ? thrust_forward : thrust_reverse;
        const double a = std::abs(f), s = std::sqrt(k1 * k1 + 4 * k2 * a);
        return std::copysign(k1 <= 0 ? (s - k1) / (2 * k2) : 2 * a / (k1 + s), f);
    }
    const auto &c = f > 0 ? rpm_positive : rpm_negative;
    const double rpm = c[0] + c[1] * f + c[2] * std::tanh(f) + c[3] * std::pow(std::abs(f), 0.25);
    return rpm * f > 0 ? rpm : 0; // the fitted curve's wrong-sign floor near zero force: stop the motor
}

VectorXd FossenModel::limitTotalThrust(const VectorXd &command) const {
    const double limit = hardware_.total_thrust_limit;
    const double total = command.cwiseAbs().sum();
    return (limit > 0 && total > limit) ? VectorXd(command * (limit / total)) : command;
}

ThrusterDynamics FossenModel::makeActuator() const {
    ThrusterDynamics actuator;
    actuator.configure(actuators_, command_timeout_);
    return actuator;
}

void FossenModel::setActuatorParameters(const std::vector<ThrusterParameters> &parameters) {
    if (static_cast<int>(parameters.size()) != thrusterCount())
        throw std::invalid_argument("Expected one thruster parameter set per thruster");
    ThrusterDynamics check;
    check.configure(parameters, command_timeout_); // throws on invalid values
    actuators_ = parameters;
}

ThrusterDynamics FossenModel::settledActuator(const VectorXd &command) const {
    ThrusterDynamics actuator = makeActuator();
    if (command.size() != thrusterCount() || !command.allFinite())
        return actuator;
    double settle = 0;
    for (const auto &p : actuators_)
        settle = std::max(settle, p.delay + 10 * std::max(p.rise, p.fall));
    // Re-issued every 50 ms, well inside the command watchdog.
    const double step = 0.05;
    for (double t = 0; t <= settle; t += step) {
        actuator.command(command);
        actuator.advance(step);
    }
    actuator.command(command);
    return actuator;
}

VectorXd FossenModel::commandLowerBound() const {
    VectorXd lb(thrusterCount());
    for (int i = 0; i < thrusterCount(); ++i) {
        const auto &p = actuators_[i];
        lb[i] = p.reverseScale > 0 ? -p.reverseLimit / p.reverseScale : 0.;
    }
    return lb;
}

VectorXd FossenModel::commandUpperBound() const {
    VectorXd ub(thrusterCount());
    for (int i = 0; i < thrusterCount(); ++i) {
        const auto &p = actuators_[i];
        ub[i] = p.forwardScale > 0 ? p.forwardLimit / p.forwardScale : 0.;
    }
    return ub;
}

Vector6d FossenModel::propulsionWrench(const State13d &x, const VectorXd &force) const {
    const Quaterniond q = Quaterniond(x[3], x[4], x[5], x[6]).normalized();
    VectorXd thrust = force;
    for (int i = 0; i < thrusterCount(); ++i) {
        const double z = (x.head<3>() + q * positions_[i]).z();
        const double axis_z = (q * directions_[i]).z();
        const double extent = std::max(.001, propeller_radius_ * std::sqrt(std::max(0., 1 - axis_z * axis_z)));
        const double c = std::clamp((dynamics_.waterLevel() - z) / extent, -1., 1.);
        thrust[i] *= (std::acos(-c) + c * std::sqrt(std::max(0., 1 - c * c))) / M_PI;
    }
    return thruster_matrix_ * thrust;
}

State13d FossenModel::derivative(const State13d &x, const VectorXd &force) const {
    Vector6d wrench = propulsionWrench(x, force);
    if (!disturbance_.isZero()) {
        const Quaterniond q = Quaterniond(x[3], x[4], x[5], x[6]).normalized();
        wrench.head<3>() += q.conjugate() * disturbance_.head<3>();
        wrench.tail<3>() += disturbance_.tail<3>();
    }
    return dynamics_.derivative(x, wrench, water_current_);
}

VectorXd FossenModel::commandToTarget(const VectorXd &command, bool smooth) const {
    VectorXd target(thrusterCount());
    for (int i = 0; i < thrusterCount(); ++i) {
        const auto &p = actuators_[i];
        double f = (!smooth && std::abs(command[i]) < p.deadband) ? 0 : command[i];
        f *= f >= 0 ? p.forwardScale : p.reverseScale;
        if (!smooth)
            f = std::clamp(f, -p.reverseLimit, p.forwardLimit);
        target[i] = f * p.efficiency;
    }
    return target;
}

void FossenModel::evolveActuators(VectorXd &force, const VectorXd &target, double dt) const {
    if (dt <= 0)
        return;
    for (int i = 0; i < thrusterCount(); ++i) {
        const auto &p = actuators_[i];
        const bool starting = target[i] * force[i] < 0 ||
                              (std::abs(force[i]) < p.startupForce && std::abs(target[i]) > std::abs(force[i]));
        const double tau = p.startup > 0 && starting ? p.startup
                           : (target[i] * force[i] >= 0 && std::abs(target[i]) > std::abs(force[i])) ? p.rise
                                                                                                    : p.fall;
        double delta = tau > 0 ? (target[i] - force[i]) * (-std::expm1(-dt / tau)) : target[i] - force[i];
        if (p.slew > 0)
            delta = std::clamp(delta, -p.slew * dt, p.slew * dt);
        force[i] += delta;
    }
}

namespace {
// physics_simulator::rungeKutta4 body update (plain vector RK4, then the
// quaternion renormalization done by Robot::setState).
template <typename Derivative> void rk4(State13d &x, double h, const Derivative &f) {
    const State13d k1 = f(x);
    const State13d k2 = f(x + h / 2 * k1);
    const State13d k3 = f(x + h / 2 * k2);
    const State13d k4 = f(x + h * k3);
    x += h / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
    x.segment<4>(3).normalize();
}
} // namespace

void FossenModel::step(State13d &x, VectorXd &force, const VectorXd &target, double h) const {
    evolveActuators(force, target, h / 2);
    rk4(x, h, [&](const State13d &s) { return derivative(s, force); });
    evolveActuators(force, target, h / 2);
}

void FossenModel::step(State13d &x, ThrusterDynamics &actuator, double h) const {
    actuator.advance(h / 2);
    const VectorXd force = actuator.forces();
    rk4(x, h, [&](const State13d &s) { return derivative(s, force); });
    actuator.advance(h / 2);
}

State13d FossenModel::fromBaseLink(const Vector3d &p_base, const Quaterniond &orientation,
                                   const Vector3d &v_base, const Vector3d &w) const {
    const Quaterniond q = orientation.normalized();
    State13d x;
    x.head<3>() = p_base - q * base_link_offset_;
    x.segment<4>(3) << q.w(), q.x(), q.y(), q.z();
    x.segment<3>(7) = v_base - w.cross(base_link_offset_);
    x.tail<3>() = w;
    return x;
}

Vector3d FossenModel::baseLinkPosition(const State13d &x) const {
    return x.head<3>() + Quaterniond(x[3], x[4], x[5], x[6]).normalized() * base_link_offset_;
}

Vector3d FossenModel::baseLinkVelocity(const State13d &x) const {
    return x.segment<3>(7) + x.tail<3>().cross(base_link_offset_);
}
} // namespace riptide_mpc
