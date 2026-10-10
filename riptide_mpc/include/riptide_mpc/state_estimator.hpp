#pragma once

#include "riptide_mpc/fossen_model.hpp"

#include <deque>
#include <limits>
#include <string>

namespace riptide_mpc {
// Sensor mounting, from the vehicle YAML (the same poses the simulator and URDF use).
struct SensorMounts {
    Quaterniond imu = Quaterniond::Identity(); // body <- imu_link rotation
    Quaterniond dvl = Quaterniond::Identity(); // body <- dvl_link rotation
    Vector3d dvl_position = Vector3d::Zero();  // DVL relative to the COM, body axes
    Vector3d fog_axis = Vector3d::UnitZ();     // FOG sensitive axis, body axes

    // Positions relative to `com`: the model's (FossenModel::com()), which may differ from the vehicle config's.
    static SensorMounts load(const std::string &vehicle_yaml, const Vector3d &com);
};

struct EstimatorSettings {
    double model_step = 0.002; // propagation step between measurements

    // Fraction of each innovation applied per sample (1 = trust the sensor outright).
    double gyro_gain = 1.0;
    double fog_gain = 1.0;
    double dvl_gain = 0.7;
    // Complementary-filter time constants [s]: how quickly the propagated state
    // is pulled toward a slow or noisy absolute reference.
    double tilt_time_constant = 1.0;       // IMU gravity reference (roll/pitch)
    double depth_time_constant = 0.3;      // depth sensor
    double heading_time_constant = 2.0;    // EKF yaw (keeps the EKF's frame)
    double horizontal_time_constant = 2.0; // EKF x/y (nothing else observes them)
    double fallback_time_constant = 0.3;   // EKF standing in for a stale sensor

    // A sensor older than this is stale and the EKF takes over its channel [s].
    double gyro_timeout = 0.2, fog_timeout = 0.1, tilt_timeout = 0.5, dvl_timeout = 1.0, depth_timeout = 0.5;
    // Re-initialize from the EKF when the estimate disagrees by more than this
    // (EKF reset, simulator teleport).
    double reset_distance = 1.0, reset_angle = 0.5;
    double max_propagation = 2.0; // s; a longer gap re-initializes instead of propagating

    // Disturbance wrench (offset-free MPC): learned from the velocity innovations
    // the model leaves behind (DVL for force, gyro/FOG for torque).
    bool estimate_disturbance = true;
    double disturbance_force_time_constant = 3.0;  // s
    double disturbance_torque_time_constant = 3.0; // s
    double max_disturbance_force = 20.0;           // N, per the world-frame vector
    double max_disturbance_torque = 5.0;           // N m
    // Learning slows with speed (factor 1 / (1 + (speed / gate)^2)): the wrench is
    // meant to capture static error (buoyancy, trim); speed-dependent error such
    // as drag would otherwise be learned in motion and linger after stopping.
    double disturbance_gate_speed = 0.1; // m/s
    double disturbance_gate_rate = 0.2;  // rad/s
};

// Model-based observer that builds the MPC's feedback state from raw sensors.
//
// The state is propagated with the Fossen model and a replica of the thrusters
// (the commands the MPC sent), then corrected by each measurement:
//   IMU gyro + FOG  -> body rates, directly (no filter lag)
//   IMU orientation -> roll/pitch, slowly (gyro handles the fast motion)
//   DVL             -> velocity, with the lever arm using the current gyro rate
//   depth           -> z
//   EKF odometry    -> x/y and heading anchor, and fallback for stale sensors
// Times are seconds on the ROS clock; measurements may arrive in any order.
class StateEstimator {
  public:
    StateEstimator(FossenModel model, SensorMounts mounts, EstimatorSettings settings);

    bool initialized() const {
        return initialized_;
    }
    double time() const {
        return t_;
    }
    const State13d &state() const {
        return x_;
    }
    const EstimatorSettings &settings() const {
        return settings_;
    }
    // [force (world), torque (body)] the model is missing; see FossenModel::setDisturbance.
    const Vector6d &disturbance() const {
        return model_.disturbance();
    }
    void resetDisturbance() {
        model_.setDisturbance(Vector6d::Zero());
    }

    void reset(const State13d &x, double t);
    void invalidate() {
        initialized_ = false;
    }
    void propagate(double t);

    // Thruster replica: every published command, and kill (queued commands dropped).
    void command(double t, const VectorXd &command);
    void stopActuators(double t);
    // Live thruster model; the replica restarts settled on `last_command`. Throws if invalid.
    void setActuatorParameters(const std::vector<ThrusterParameters> &parameters, const VectorXd &last_command);
    // Live hydrodynamic model swap; keeps the thruster parameters, actuator replica and disturbance.
    void setModel(FossenModel model);
    // Holds the learned disturbance (no learning) while paused, e.g. during an identification release, when
    // the free-floating vehicle would otherwise teach it a wrench the MPC then applies when control resumes.
    void pauseDisturbanceLearning(bool paused) {
        learning_paused_ = paused;
    }

    void imuRate(double t, const Vector3d &rate_imu_frame);
    void imuOrientation(double t, const Quaterniond &orientation_imu_frame);
    void fogRate(double t, double rate);
    void dvlVelocity(double t, const Vector3d &velocity_dvl_frame);
    void depth(double t, double base_link_z);
    // Returns true when the estimate was (re)initialized from this message.
    bool odometry(double t, const Vector3d &p_base, const Quaterniond &q, const Vector3d &v_base_body,
                  const Vector3d &w_body);

    struct Health {
        bool gyro, fog, tilt, dvl, depth;
    };
    Health health(double t) const;

  private:
    static constexpr double kNever = -std::numeric_limits<double>::infinity();
    Quaterniond orientation() const;
    // A velocity innovation e over the time it built up means a missing wrench of
    // M e / dt; move the estimate toward it with the configured time constants.
    void learnForce(const Vector3d &velocity_innovation_body);
    void learnTorque(const Vector3d &rate_innovation_body);
    double gate() const;
    void setOrientation(const Quaterniond &q);
    void correctTilt(const Quaterniond &measured_body, double gain);
    // Brings the estimate to t; false if the measurement is unusable.
    bool advanceTo(double t);
    static double gain(double elapsed, double time_constant);
    bool fresh(double last, double timeout, double t) const {
        return t - last < timeout;
    }
    // Recent COM velocity and body rate, so a measurement stamped in the past (the DVL's
    // velocity is ~0.12 s old) is compared with what the estimate was then, not now.
    void record();
    bool velocityAt(double t, Vector3d &v, Vector3d &w) const;
    struct Past {
        double t;
        Vector3d v, w;
    };
    std::deque<Past> history_;

    FossenModel model_;
    SensorMounts mounts_;
    EstimatorSettings settings_;
    ThrusterDynamics actuator_;
    State13d x_ = State13d::Zero();
    double t_ = 0;
    bool initialized_ = false;
    bool learning_paused_ = false;
    double last_gyro_ = kNever, last_fog_ = kNever, last_tilt_ = kNever, last_dvl_ = kNever, last_depth_ = kNever,
           last_odom_ = kNever;
};
} // namespace riptide_mpc
