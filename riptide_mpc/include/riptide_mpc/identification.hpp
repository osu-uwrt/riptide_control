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
// Optional blocks (pool_identify v3):
//   thrusters:  one thruster's command ramped (+ and -) while the MPC holds with the other seven, plus
//               zero-wrench (null-space) patterns while holding: the still balances fit each thruster's
//               forward and reverse gain (relative: their forward mean stays the prior's, the load cell's).
//   releases:   all thrusters off from level and tilted holds: the vehicle floats up and rights itself,
//               with no thruster in the loop: heave inertia/drag and roll/pitch inertia/damping.
//
// The MPC flies the whole sequence in POSITION mode inside a small box, so the
// vehicle is always under closed-loop control (except during a release).
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
    enum class Kind { Hold, Run, Release, Ramp, NullSpace };
    int id = 0;
    Kind kind = Kind::Hold;
    int axis = -1;        // Run only
    double speed = 0;     // Run: commanded cruise speed [m/s or rad/s]
    int direction = 0;    // Run: +1 / -1 along the axis
    int thruster = -1;    // Ramp: the ramped thruster; NullSpace: the pattern
    double amplitude = 0; // Ramp: peak command [N]; NullSpace: pattern scale [N]
    std::string label;
};

struct Sample {
    int segment = 0;
    double t = 0, dt = 0;
    Vector3d v = Vector3d::Zero();         // base_link velocity, body axes (DVL, lever arm removed)
    Vector3d w = Vector3d::Zero();         // body rates (IMU gyro, FOG on its axis)
    Vector3d up = Vector3d::UnitZ();       // world +z in body axes (IMU tilt)
    Vector6d impulse = Vector6d::Zero();   // integral of the thrust wrench (body, at COM) over (t - dt, t]
    // Per thruster, integral of its realized force's positive and negative parts over (t - dt, t].
    VectorXd thrust_positive, thrust_negative;
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
    VectorXd positive_, negative_;
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

    // Thruster block: each thruster ramped 0 -> +force -> 0 -> -force -> 0 over ramp_secs while holding.
    bool thruster_ramps = false;
    double ramp_force = 8, ramp_secs = 40;
    // Null-space block: each zero-wrench pattern at +-null_amplitude (largest thruster) for null_secs.
    bool null_space = false;
    double null_amplitude = 4, null_secs = 10, null_settle = 3;
    // Release block: thrusters off for release_secs (or until pool_identify cuts it: too shallow or tilted)
    // from a level hold and, with release_tilted, from +tilt pitch and roll holds; then back to the start.
    bool releases = false, release_tilted = true;
    double release_secs = 30, recover_timeout = 90;
    double release_tilt = 0.35; // rad: tilted releases at +- this in pitch and in roll (roll is barely damped by eye)
};

struct Step {
    // Release/Ramp/NullSpace: settle on the pose like a Hold, then apply identification inputs and record.
    enum class Kind { Hold, Move, Release, Ramp, NullSpace };
    Kind kind = Kind::Hold;
    Vector3d position = Vector3d::Zero();
    Quaterniond orientation = Quaterniond::Identity();
    MotionLimits limits;
    bool record = true;
    double record_secs = 6; // Hold, Release, Ramp, NullSpace: how long to record once settled
    Segment segment;
    double timeout = 30;
    VectorXd bias;        // NullSpace: per-thruster pattern [N]
    double pre_secs = 0;  // NullSpace: inputs applied this long before recording
    int thrusters = 0;    // Release/Ramp: thruster count for the fixed commands
    bool guarded = true;  // false: pool_identify's tilt guard is off (recovering from a release, still tilted)
};

// `thrusters` and `null_space` (thruster count x patterns, e.g. nullSpacePatterns of the model's thruster
// matrix) are only needed for the thruster, null-space and release blocks.
std::vector<Step> buildSequence(const SequenceSettings &s, const Vector3d &start, double start_yaw,
                                const MotionLimits &base, int thrusters = 0,
                                const MatrixXd &null_space = MatrixXd());
// Zero-wrench thrust patterns of `thruster_matrix` (6 x n), each scaled to a largest entry of 1.
MatrixXd nullSpacePatterns(const MatrixXd &thruster_matrix);

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
        // Identification inputs for the MPC this tick (empty = none): fixed commands (NaN = free) and bias.
        VectorXd fixed, bias;
    };
    // first_segment_id: distinct per identification iteration, so pooled recordings never share ids.
    explicit Sequencer(std::vector<Step> steps, int first_segment_id = 0)
        : steps_(std::move(steps)), next_segment_id_(first_segment_id) {}
    // `cut` ends a running release early (pool_identify: too shallow or too tilted).
    Output update(double t, bool settled, bool cut = false);
    std::size_t index() const {
        return index_;
    }
    std::size_t size() const {
        return steps_.size();
    }
    // The current step is a release with the thrusters off (recording).
    bool releasing() const {
        return index_ < steps_.size() && steps_[index_].kind == Step::Kind::Release && recording_;
    }
    // pool_identify's tilt guard applies (not during a release or the recovery after it).
    bool guarded() const {
        return index_ >= steps_.size() || (steps_[index_].guarded && !releasing());
    }

  private:
    std::vector<Step> steps_;
    std::size_t index_ = 0;
    bool entered_ = false, recording_ = false;
    double entered_at_ = 0, recording_since_ = 0, applying_since_ = -1;
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

// Per-thruster gains relative to the prior model: realized = gain x the prior's force, split by direction.
// Holding still only excites the thruster matrix's null space, so a group of thrusters that works on the same
// wrench components (Talos: the four surge thrusters, the four vectored ones) has an overall scale that trades
// exactly with the statics. Each group's forward mean is therefore pinned to the prior's (the load cell's);
// within a group, and forward vs reverse, the gains are fitted.
struct ThrusterFit {
    bool ok = false;
    VectorXd forward, reverse;           // gains; each group's forward mean pinned to 1
    VectorXd forward_time, reverse_time; // seconds of still data with that thruster beyond 1 N each way
    std::vector<bool> forward_ok, reverse_ok;
    std::vector<int> group;              // group of each thruster (see above)
    std::vector<bool> reverse_pinned;    // per group: never at mixed signs, so its reverse mean is pinned too
    int chunks = 0;
    double buoyancy = 0;                 // statics of the joint fit, for comparison
    Vector3d cob = Vector3d::Zero();
    double force_residual = 0, torque_residual = 0;
    std::string note;
};

struct Result {
    StaticsFit statics;
    std::vector<AxisFit> axes;
    ThrusterFit thrusters;
};

// `prior` is the MPC model file (hydrodynamics schema); `mass` from the vehicle file. `thruster_matrix`
// (6 x n, the model's) enables the per-thruster fit; the other fits then use the corrected thrust.
// `vehicle` (the vehicle document) enables the release fit (it simulates the releases with the model).
Result fit(const YAML::Node &prior, double mass, const std::vector<Sample> &samples,
           const std::vector<Segment> &segments, const MatrixXd &thruster_matrix = MatrixXd(),
           const YAML::Node &vehicle = YAML::Node());
// Copy of `prior` with every accepted value replaced. Thruster gains are relative to the model the recording's
// thrust was computed with (the Recorder's): pass it as `thrust_base` when that is not `prior`.
YAML::Node identifiedModel(const YAML::Node &prior, const Result &result, const std::string &provenance,
                           const YAML::Node &thrust_base = YAML::Node());
YAML::Node report(const YAML::Node &prior, const Result &result);

void writeRecording(const std::string &dir, const std::vector<Sample> &samples, const std::vector<Segment> &segments);
void readRecording(const std::string &dir, std::vector<Sample> &samples, std::vector<Segment> &segments);
} // namespace riptide_mpc::ident
