#pragma once

// Pool identification of the parameters that cannot be measured on land:
// net buoyancy and centre of buoyancy, drag, and added mass. The thruster
// geometry, force curve, limits and delay are TRUSTED (load-cell calibrated), so
// the realized thrust wrench is known exactly from the commands; everything the
// vehicle does beyond that is attributed to the hydrodynamic terms.
//
//   statics:  holds at several attitudes -> tau_hold = -(B - W) up, torque = up x (B r_cob)
//   drag:     steady speed along one axis  -> tau - tau_hold(up) = D1 v + D2 |v| v
//   inertia:  accel/decel phases          -> integral(tau + g - D v) dt = M dv
//
// The MPC flies the whole sequence in POSITION mode inside a small box, so the
// vehicle is always under closed-loop control.
#include "riptide_mpc/fossen_model.hpp"
#include "riptide_mpc/motion_profile.hpp"
#include "riptide_mpc/state_estimator.hpp"

#include <yaml-cpp/yaml.h>

#include <optional>
#include <string>
#include <vector>

namespace riptide_mpc::ident {

// Axis numbering follows the body twist/wrench: 0 surge, 1 sway, 2 heave, 5 yaw.
struct Segment {
    enum class Kind { Hold, Run };
    int id = 0;
    Kind kind = Kind::Hold;
    int axis = -1;     // Run only
    double speed = 0;  // Run: commanded cruise speed [m/s or rad/s]
    int direction = 0; // Run: +1 / -1 along the axis
    std::string label;
};

struct Sample {
    int segment = 0;
    double t = 0, dt = 0;
    Vector3d v = Vector3d::Zero();         // base_link velocity, body axes (DVL, lever arm removed)
    Vector3d w = Vector3d::Zero();         // body rates (IMU gyro, FOG on its axis)
    Vector3d up = Vector3d::UnitZ();       // world +z in body axes (IMU tilt)
    Vector6d impulse = Vector6d::Zero();   // integral of the thrust wrench (body, at COM) over (t - dt, t]
};

// Realized thrust from the commands, paired with the raw sensors at each DVL sample.
class Recorder {
  public:
    Recorder(FossenModel model, SensorMounts mounts);

    void command(double t, const VectorXd &command);
    void stopThrusters(double t);
    void imuRate(double t, const Vector3d &rate_imu);
    void imuOrientation(double t, const Quaterniond &orientation_imu);
    void fogRate(double t, double rate);
    void dvlVelocity(double t, const Vector3d &velocity_dvl);

    void begin(const Segment &segment, double t);
    void end(double t);
    bool recording() const {
        return active_.has_value();
    }
    const std::vector<Sample> &samples() const {
        return samples_;
    }
    const std::vector<Segment> &segments() const {
        return segments_;
    }

  private:
    void advance(double t);

    FossenModel model_;
    SensorMounts mounts_;
    ThrusterDynamics actuator_;
    bool started_ = false;
    double t_ = 0, since_ = 0, fog_time_ = -1e9;
    Vector6d impulse_ = Vector6d::Zero();
    std::optional<Vector3d> w_, up_;
    double fog_ = 0;
    std::optional<Segment> active_;
    std::vector<Sample> samples_;
    std::vector<Segment> segments_;
};

struct SequenceSettings {
    bool statics = true, surge = true, sway = true, heave = true, yaw = true;
    double lane_length = 3.0;  // m, surge and sway runs (from the start point, +x / +y body)
    double heave_span = 0.8;   // m, downward from the start depth
    double yaw_span = M_PI;    // rad, centred on the start heading
    std::vector<double> linear_speeds{0.2, 0.35, 0.5, 0.65};
    std::vector<double> heave_speeds{0.15, 0.25, 0.35};
    std::vector<double> yaw_rates{0.3, 0.6, 0.9};
    double linear_accel = 0.5, angular_accel = 1.2;
    double tilt = 10 * M_PI / 180; // statics holds at +-tilt in roll and pitch
    double hold_secs = 6, settle_timeout = 25, run_timeout_margin = 12;
    int repeats = 1; // out-and-back pairs per speed (more drag and inertia data in a small pool)
};

struct Step {
    enum class Kind { Hold, Move };
    Kind kind = Kind::Hold;
    Vector3d position = Vector3d::Zero();
    Quaterniond orientation = Quaterniond::Identity();
    MotionLimits limits;
    bool record = true;
    double record_secs = 6; // Hold: how long to record once settled
    Segment segment;
    double timeout = 30;
};

std::vector<Step> buildSequence(const SequenceSettings &s, const Vector3d &start, double start_yaw,
                                const MotionLimits &base);

// Walks the steps: holds settle then record; runs record from the command until
// the MPC reports settled again. `settled` = controller trust says "on target".
class Sequencer {
  public:
    struct Output {
        Vector3d position = Vector3d::Zero();
        Quaterniond orientation = Quaterniond::Identity();
        MotionLimits limits;
        bool new_step = false;        // apply `limits` now
        std::optional<Segment> begin; // start recording this segment
        bool end = false;             // stop recording
        bool done = false;
        std::string message;
    };
    // first_segment_id: distinct per identification iteration, so pooled recordings never share ids.
    explicit Sequencer(std::vector<Step> steps, int first_segment_id = 0)
        : steps_(std::move(steps)), next_segment_id_(first_segment_id) {}
    Output update(double t, bool settled);
    std::size_t index() const {
        return index_;
    }
    std::size_t size() const {
        return steps_.size();
    }

  private:
    std::vector<Step> steps_;
    std::size_t index_ = 0;
    bool entered_ = false, recording_ = false;
    double entered_at_ = 0, recording_since_ = 0;
    int next_segment_id_ = 0;
};

struct StaticsFit {
    bool ok = false;
    int holds = 0;
    double buoyancy = 0, volume = 0;
    Vector3d cob = Vector3d::Zero();
    double force_residual = 0, torque_residual = 0; // rms over holds [N, N m]
    bool cob_z_observed = false;
    std::string note;
};

struct AxisFit {
    int axis = 0;
    bool drag_ok = false, mass_ok = false;
    double d1 = 0, d2 = 0, drag_r2 = 0;
    int steady_samples = 0, speed_levels = 0;
    double mass_total = 0, added_mass = 0, mass_r2 = 0;
    int windows = 0;
    std::string note;
};

struct Result {
    StaticsFit statics;
    std::vector<AxisFit> axes;
};

// `prior` is the MPC model file (hydrodynamics schema); `mass` from the vehicle file.
Result fit(const YAML::Node &prior, double mass, const std::vector<Sample> &samples,
           const std::vector<Segment> &segments);
// Copy of `prior` with every accepted value replaced.
YAML::Node identifiedModel(const YAML::Node &prior, const Result &result, const std::string &provenance);
YAML::Node report(const YAML::Node &prior, const Result &result);

void writeRecording(const std::string &dir, const std::vector<Sample> &samples, const std::vector<Segment> &segments);
void readRecording(const std::string &dir, std::vector<Sample> &samples, std::vector<Segment> &segments);
} // namespace riptide_mpc::ident
