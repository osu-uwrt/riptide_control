#pragma once

#include "riptide_mpc/fossen_model.hpp"
#include "riptide_mpc/motion_profile.hpp"
#include "riptide_mpc/path_plan.hpp"

#include <cstdint>
#include <vector>

namespace riptide_mpc {
// Same values as riptide_msgs2/ControllerCommand.
enum class Mode : std::uint8_t { DISABLED = 0, FEEDFORWARD = 1, VELOCITY = 2, POSITION = 3 };

struct Reference {
    Mode linear_mode = Mode::DISABLED, angular_mode = Mode::DISABLED;
    Vector3d position = Vector3d::Zero();        // base_link in the odometry frame
    Vector3d linear_velocity = Vector3d::Zero(); // base_link velocity
    bool linear_velocity_in_body = true;         // false: odometry-frame velocity
    Quaterniond orientation = Quaterniond::Identity();
    Vector3d angular_velocity = Vector3d::Zero(); // body rates
    Vector6d feedforward = Vector6d::Zero();      // body wrench used in FEEDFORWARD mode
    // POSITION mode path (both linear and angular): the reference moves along
    // it and comes to rest on its end, which position/orientation should equal.
    // Null: go straight to position/orientation. A new pointer restarts the path.
    std::shared_ptr<const PathPlan> path;
    // Where the path's moving look targets are now (odometry frame), indexed by
    // PathPoint::look_target. The reference turns toward them on top of the
    // planned heading, within the angular limits.
    std::vector<Vector3d> look_targets;
};

struct MpcSettings {
    double dt = 0.05; // stage length == control period
    int horizon = 30;
    double model_step = 0.002;         // nominal rollout step; physics_simulator default
    double linearization_step = 0.025; // step of the finite-difference Jacobian model
    int sqp_iterations = 1;
    int qp_max_iterations = 50;
    bool compensate_delay = true; // roll the known in-flight commands through the delay

    // Setpoint changes are followed along a speed/acceleration-limited profile
    // instead of being chased directly.
    bool profile_motion = true;
    MotionLimits motion;
    double max_reference_lag = 1.0;       // m; re-seed the profile at the vehicle beyond this
    double max_reference_lag_angle = 0.8; // rad
    // Reference governor: when the vehicle falls this far behind the profile
    // (thrust saturation, model error) the profile's cruise speed is scaled down
    // smoothly, reaching a crawl at the "stop" lag, so it never runs away.
    double governor_lag = 0.03, governor_stop_lag = 0.15;     // m
    double governor_angle = 0.05, governor_stop_angle = 0.25; // rad

    // Output weights (diagonal) on [position, attitude, linear velocity, angular velocity].
    Vector3d q_position{400, 400, 400};
    Vector3d q_attitude{3000, 3000, 300}; // roll/pitch dominate: never tilt to gain distance
    Vector3d q_linear_velocity{100, 100, 100};   // VELOCITY-mode tracking
    Vector3d q_angular_velocity{50, 50, 50};     // VELOCITY-mode tracking
    Vector3d q_linear_damping{20, 20, 20};       // zero-velocity weight while holding position
    Vector3d q_angular_damping{10, 10, 10};      // zero-rate weight while holding attitude
    double terminal_factor = 5.;
    double r_thrust = 1e-3;      // on (u - feedforward from inverse dynamics), per N^2
    double r_thrust_rate = 5e-3; // on successive change of (u - feedforward), per N^2
};

struct MpcOutput {
    VectorXd command;               // per-thruster command to publish [N]
    Vector6d wrench = Vector6d::Zero(); // body wrench at COM that command requests
    std::vector<State13d> prediction;   // predicted COM states, after the delay
    double solve_ms = 0;
    int qp_iterations = 0;
    bool converged = true;
    bool active = false; // false when outputting zeros (disabled)
};

class MpcController {
  public:
    MpcController(FossenModel model, MpcSettings settings);

    // Internal replica of the plant's actuator (delay queue + lag). Keep it in
    // lockstep with the real thrusters: advance by elapsed time, issue() every
    // published command, stopActuators() when the vehicle is killed.
    void advance(double dt);
    void issue(const VectorXd &command);
    void stopActuators();
    void resetActuators();
    void clearWarmStart();
    // Learned model error (see StateEstimator::disturbance); used in every prediction.
    void setDisturbance(const Vector6d &d) {
        model_.setDisturbance(d);
    }
    // Live speed/accel/jerk limits for the motion profile (next compute()).
    void setMotionLimits(const MotionLimits &m) {
        settings_.motion = m;
    }
    // Live cost weights (q_*, terminal_factor, r_thrust, r_thrust_rate) from `w`; next compute().
    void setCostWeights(const MpcSettings &w) {
        settings_.q_position = w.q_position;
        settings_.q_attitude = w.q_attitude;
        settings_.q_linear_velocity = w.q_linear_velocity;
        settings_.q_angular_velocity = w.q_angular_velocity;
        settings_.q_linear_damping = w.q_linear_damping;
        settings_.q_angular_damping = w.q_angular_damping;
        settings_.terminal_factor = w.terminal_factor;
        settings_.r_thrust = w.r_thrust;
        settings_.r_thrust_rate = w.r_thrust_rate;
    }
    // Live thruster model (delay, lag, scale, efficiency). The actuator replica
    // restarts settled on the last command. Throws if invalid; nothing changes then.
    void setActuatorParameters(const std::vector<ThrusterParameters> &parameters);
    // Live hydrodynamic model swap (identification): the new model keeps this one's
    // thruster parameters and learned disturbance, so the actuator replica, the
    // profile and the warm start all carry over.
    void setModel(FossenModel model);
    const VectorXd &lastCommand() const {
        return last_command_;
    }
    // Identification (pool_identify): `fixed` holds a thruster's command at a value over the whole horizon
    // (NaN = free; all zero = released, thrusters off); `bias` is added to the feedforward of the free
    // thrusters (a null-space pattern adds no wrench, so tracking costs nothing). Empty vectors clear them.
    // The MPC predicts with both, so it holds the vehicle with the remaining thrusters.
    void setIdentificationInputs(const VectorXd &fixed, const VectorXd &bias);
    const VectorXd &fixedCommands() const {
        return fixed_;
    }

    MpcOutput compute(const State13d &measured, const Reference &reference);

    const FossenModel &model() const {
        return model_;
    }
    const MpcSettings &settings() const {
        return settings_;
    }
    const ThrusterDynamics &actuatorEstimate() const {
        return actuator_;
    }
    // Profiled reference (odometry frame, base_link). It is on the prediction's
    // clock: this is where the vehicle should be referenceLead() seconds from now.
    const MotionProfile &profile() const {
        return live_.pose;
    }
    // Where the profile is along reference.path.
    const PathProgress &pathProgress() const {
        return live_.progress;
    }
    // Yaw the profile has turned away from the path's planned attitude to face
    // its moving look targets (world frame; identity when there are none).
    const Quaterniond &lookOffset() const {
        return live_.look.orientation;
    }
    double referenceLead() const {
        return settings_.compensate_delay ? model_.actuatorParameters().front().delay : 0.;
    }

  private:
    struct Point {
        State13d x;
        VectorXd f;
    };
    using Output = Eigen::Matrix<double, 12, 1>;
    // Reference for the end of one stage.
    struct StageReference {
        MotionProfile pose;
        Vector3d linear_velocity = Vector3d::Zero();  // VELOCITY mode, command frame
        Vector3d angular_velocity = Vector3d::Zero(); // VELOCITY mode, body
        PathProgress progress;                        // along Reference::path
        MotionProfile look; // yaw toward Reference::look_targets, applied on the path's attitude
    };

    Point boxplus(const Point &p, const VectorXd &delta) const;
    VectorXd boxminus(const Point &a, const Point &b) const;
    Point stage(const Point &p, const VectorXd &command, bool fine) const;
    void stepReference(StageReference &s, const Reference &r, double dt) const;
    void pathPose(StageReference &s, const PathPlan &path) const;
    void seedReference(const State13d &measured, const Reference &r);
    Output output(const State13d &x, const Reference &r, const StageReference &s) const;
    VectorXd feedforwardInput(const StageReference &s, const Reference &r, const State13d &x0) const;
    Output weights(const Reference &r) const;
    MatrixXd effectiveThrusterMatrix(const State13d &x) const;
    VectorXd allocate(const State13d &x, const Vector6d &wrench) const;
    void applyBounds();

    FossenModel model_;
    MpcSettings settings_;
    ThrusterDynamics actuator_;
    VectorXd warm_, last_command_, lb_, ub_;
    VectorXd issued_feedforward_, last_deviation_; // for the smoothness term on u - u_ref
    VectorXd fixed_, bias_;                         // identification inputs (empty = none)
    StageReference live_; // profile state "now"; advanced with the actuator replica
    MotionLimits motion_; // settings_.motion with the governor's speed scaling
    Reference target_;    // latest setpoints the profile moves toward
    bool linear_seeded_ = false, angular_seeded_ = false;
    int nu_, nx_;
};
} // namespace riptide_mpc
