#pragma once

#include "riptide_mpc/marine_dynamics.hpp"
#include "riptide_mpc/thruster_dynamics.hpp"

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <array>
#include <string>
#include <vector>

namespace riptide_mpc {
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;

// Optional `hardware:` section of the model file: what the real actuators need.
struct HardwareConfig {
    bool present = false;
    double total_thrust_limit = 0; // N, sum of |force| over thrusters; 0 = none
    std::array<double, 4> rpm_positive{}, rpm_negative{};
    // Load-cell thrust curve |F| = k2 rpm^2 + k1 rpm per direction ([k2, k1], rpm >= 0). When set
    // (thrust_curve_forward/reverse), forceToRpm is its exact inverse instead of the legacy curves.
    bool quadratic = false;
    std::array<double, 2> thrust_forward{}, thrust_reverse{};
    // Propeller law |F| = K_T rho D^4 (rpm/60)^2 with K_T = a + b rpm per direction ([a > 0, b >= 0]).
    // When set (thrust_coefficient_forward/reverse + propeller_diameter; rho is the model's water_density),
    // forceToRpm is its exact inverse.
    bool propeller = false;
    std::array<double, 2> kt_forward{}, kt_reverse{};
    double kt_scale = 0; // rho D^4 / 3600: newtons per (K_T rpm^2)

    // Propeller inverse when `propeller`, quadratic inverse when `quadratic`, otherwise
    // complete_controller's "Force to RPM Transform": c0 + c1 F + c2 tanh(F) + c3 |F|^(1/4).
    double forceToRpm(double force) const;
};

// Prediction model built from the SAME files the simulator loads (vehicle YAML +
// hydrodynamics YAML) and the simulator's own MarineDynamics library. Every
// function here mirrors c_simulator's Robot/physics loop so that, in simulation,
// the model and the plant are the same equations with the same coefficients.
//
// State13d is the simulator state: [COM position (world), q_wxyz (body->world),
// body linear velocity at COM, body angular velocity]. Thruster forces are the
// REALIZED per-thruster forces (after delay/lag), in newtons.
class FossenModel {
  public:
    static FossenModel load(const std::string &vehicle_yaml, const std::string &hydrodynamics_yaml);
    // Same, from already parsed documents (e.g. a model edited in memory).
    static FossenModel fromNodes(const YAML::Node &vehicle, const YAML::Node &hydrodynamics);

    int thrusterCount() const {
        return static_cast<int>(positions_.size());
    }
    const MarineDynamics &dynamics() const {
        return dynamics_;
    }
    // 6xN: body wrench at the COM per newton of each thruster.
    const MatrixXd &thrusterMatrix() const {
        return thruster_matrix_;
    }
    // base_link origin relative to the COM, in body axes.
    const Vector3d &baseLinkOffset() const {
        return base_link_offset_;
    }
    double mass() const {
        return mass_;
    }
    const HardwareConfig &hardware() const {
        return hardware_;
    }
    // Unmodelled wrench learned online (offset-free MPC): [force in world frame,
    // torque in body frame], applied at the COM in every prediction. In a pool it
    // is model error: buoyancy/weight trim, COB/COM offsets, drag, thruster scale.
    void setDisturbance(const Vector6d &d) {
        disturbance_ = d;
    }
    const Vector6d &disturbance() const {
        return disturbance_;
    }
    // Scales a command so the sum of |force| respects the hardware power budget.
    VectorXd limitTotalThrust(const VectorXd &command) const;
    const std::vector<ThrusterParameters> &actuatorParameters() const {
        return actuators_;
    }
    // Replaces the thruster model (one entry per thruster), e.g. live from a tuning
    // sweep. Throws std::invalid_argument and leaves the model unchanged if invalid.
    void setActuatorParameters(const std::vector<ThrusterParameters> &parameters);
    double commandTimeout() const {
        return command_timeout_;
    }
    // Most negative / most positive command that still changes the realized force.
    VectorXd commandLowerBound() const;
    VectorXd commandUpperBound() const;

    // Simulator Robot::propulsionWrench: includes the partial-submergence disk factor.
    Vector6d propulsionWrench(const State13d &x, const VectorXd &thrust) const;
    State13d derivative(const State13d &x, const VectorXd &thrust) const;

    // ThrusterDynamics::command target (deadband/scale/limit/efficiency), and a
    // smooth variant (no deadband or limit) used for linearization.
    VectorXd commandToTarget(const VectorXd &command, bool smooth = false) const;
    // ThrusterDynamics::evolve for a constant target over dt.
    void evolveActuators(VectorXd &force, const VectorXd &target, double dt) const;

    // One physics_simulator step: actuators h/2, RK4 body with held thrust, actuators h/2.
    void step(State13d &x, VectorXd &force, const VectorXd &target, double h) const;
    // Same step, but the actuator is a live ThrusterDynamics (delay queue).
    void step(State13d &x, ThrusterDynamics &actuator, double h) const;

    ThrusterDynamics makeActuator() const;
    // An actuator that has been holding `command` long enough to settle on it: the
    // replica to restart from when the thruster model changes mid-run (the real
    // thrusters keep spinning).
    ThrusterDynamics settledActuator(const VectorXd &command) const;

    // base_link pose/twist (what odometry reports) <-> simulator COM state.
    State13d fromBaseLink(const Vector3d &p_base, const Quaterniond &q, const Vector3d &v_base_body,
                          const Vector3d &w_body) const;
    Vector3d baseLinkPosition(const State13d &x) const;
    Vector3d baseLinkVelocity(const State13d &x) const;

  private:
    MarineDynamics dynamics_;
    MatrixXd thruster_matrix_;
    std::vector<Vector3d> positions_, directions_;
    std::vector<ThrusterParameters> actuators_;
    Vector3d base_link_offset_ = Vector3d::Zero();
    Vector3d water_current_ = Vector3d::Zero();
    Vector6d disturbance_ = Vector6d::Zero();
    HardwareConfig hardware_;
    double mass_ = 1., propeller_radius_ = .05, command_timeout_ = .5;
};

// tf2::Quaternion::setRPY convention, as used by the simulator for every mounting pose.
Quaterniond rpyToQuaternion(double roll, double pitch, double yaw);
Quaterniond quaternionExp(const Vector3d &rotation_vector);
Vector3d quaternionLog(const Quaterniond &q);
} // namespace riptide_mpc
