#include "riptide_mpc/identification.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace riptide_mpc::ident {
namespace {
constexpr double kGravity = 9.80665;

Eigen::Matrix3d skew(const Vector3d &u) {
    Eigen::Matrix3d m;
    m << 0, -u.z(), u.y(), u.z(), 0, -u.x(), -u.y(), u.x(), 0;
    return m;
}

// Model dof index (0..5) of a Run axis.
int dof(int axis) {
    return axis;
}

// 6x6 matrices are stored flat (36) or nested (6x6) in the model files.
double getEntry(const YAML::Node &m, int r, int c) {
    return m.size() == 36 ? m[r * 6 + c].as<double>() : m[r][c].as<double>();
}
void setEntry(YAML::Node m, int r, int c, double v) {
    if (m.size() == 36)
        m[r * 6 + c] = v;
    else
        m[r][c] = v;
}

double rigidInertia(const YAML::Node &prior, int dof, double mass) {
    if (dof < 3)
        return mass;
    return matrix3(prior["rigid_body_inertia3x3"], "rigid_body_inertia3x3").diagonal()[dof - 3];
}

// Signed motion along a run's axis: v for linear axes, the rate for yaw.
double along(const Sample &s, int axis) {
    return axis < 3 ? s.v[axis] : s.w[axis - 3];
}

// Wrench needed to hold still at attitude `up`, from the statics fit.
Vector6d holdWrench(const StaticsFit &st, double weight, const Vector3d &up) {
    Vector6d w;
    w.head<3>() = -(st.buoyancy - weight) * up;
    w.tail<3>() = up.cross(st.buoyancy * st.cob);
    return w;
}

// The segment's samples in time order, dropping the first `skip` fraction.
std::vector<const Sample *> segmentSamples(const std::vector<Sample> &samples, int id, double skip = 0) {
    std::vector<const Sample *> out;
    for (const auto &s : samples)
        if (s.segment == id)
            out.push_back(&s);
    out.erase(out.begin(), out.begin() + static_cast<long>(skip * out.size()));
    return out;
}
} // namespace

//=============================== Recorder ===============================//

Recorder::Recorder(FossenModel model, SensorMounts mounts)
    : model_(std::move(model)), mounts_(mounts), actuator_(model_.makeActuator()) {
    mounts_.fog_axis.normalize();
    positive_ = negative_ = VectorXd::Zero(model_.thrusterCount());
}

void Recorder::advance(double t) {
    if (!started_) {
        started_ = true;
        t_ = since_ = t;
        return;
    }
    while (t_ < t - 1e-12) {
        const double h = std::min(0.002, t - t_);
        actuator_.advance(h);
        const VectorXd f = actuator_.forces();
        impulse_ += model_.thrusterMatrix() * f * h;
        positive_ += f.cwiseMax(0.) * h;
        negative_ += f.cwiseMin(0.) * h;
        t_ += h;
    }
}

void Recorder::command(double t, const VectorXd &command) {
    advance(t);
    actuator_.command(command);
}

void Recorder::stopThrusters(double t) {
    advance(t);
    actuator_.stop();
}

void Recorder::imuRate(double t, const Vector3d &rate_imu) {
    Vector3d w = mounts_.imu * rate_imu;
    if (t - fog_time_ < 0.1) // the FOG owns its axis
        w += (fog_ - mounts_.fog_axis.dot(w)) * mounts_.fog_axis;
    w_ = w;
}

void Recorder::fogRate(double t, double rate) {
    fog_ = rate;
    fog_time_ = t;
    if (w_)
        *w_ += (rate - mounts_.fog_axis.dot(*w_)) * mounts_.fog_axis;
}

void Recorder::imuOrientation(double, const Quaterniond &orientation_imu) {
    const Quaterniond body = (orientation_imu.normalized() * mounts_.imu.conjugate()).normalized();
    up_ = body.conjugate() * Vector3d::UnitZ();
}

void Recorder::dvlVelocity(double t, const Vector3d &velocity_dvl) {
    advance(t);
    if (active_ && w_ && up_) {
        Sample s;
        s.segment = active_->id;
        s.t = t;
        s.dt = t - since_;
        s.w = *w_;
        // DVL point -> COM -> base_link, all with the fresh gyro rate.
        const Vector3d v_com = mounts_.dvl * velocity_dvl - s.w.cross(mounts_.dvl_position);
        s.v = v_com + s.w.cross(model_.baseLinkOffset());
        s.up = *up_;
        s.impulse = impulse_;
        s.thrust_positive = positive_;
        s.thrust_negative = negative_;
        if (s.dt > 0)
            samples_.push_back(s);
    }
    impulse_.setZero();
    positive_.setZero();
    negative_.setZero();
    since_ = t;
}

void Recorder::begin(const Segment &segment, double t) {
    advance(t);
    impulse_.setZero();
    positive_.setZero();
    negative_.setZero();
    since_ = t;
    active_ = segment;
    segments_.push_back(segment);
}

void Recorder::end(double t) {
    advance(t);
    active_.reset();
}

//=============================== Sequence ===============================//

MatrixXd nullSpacePatterns(const MatrixXd &T) {
    if (T.cols() <= T.rows())
        return MatrixXd(T.cols(), 0);
    Eigen::JacobiSVD<MatrixXd> svd(T, Eigen::ComputeFullV);
    MatrixXd N = svd.matrixV().rightCols(T.cols() - T.rows());
    for (int j = 0; j < N.cols(); ++j) {
        Eigen::Index k;
        N.col(j).cwiseAbs().maxCoeff(&k);
        N.col(j) /= N(k, j); // largest entry +1
    }
    return N;
}

std::vector<Step> buildSequence(const SequenceSettings &s, const Vector3d &start, double yaw,
                                const MotionLimits &base, int thrusters, const MatrixXd &null_space) {
    std::vector<Step> steps;
    const Quaterniond heading(Eigen::AngleAxisd(yaw, Vector3d::UnitZ()));
    auto hold = [&](const Vector3d &p, const Quaterniond &q, const std::string &label, bool record) {
        Step st;
        st.kind = Step::Kind::Hold;
        st.position = p;
        st.orientation = q;
        st.limits = base;
        st.record = record;
        st.record_secs = s.hold_secs;
        st.segment.kind = Segment::Kind::Hold;
        st.segment.label = label;
        st.timeout = s.settle_timeout;
        steps.push_back(st);
    };
    auto run = [&](int axis, double speed, int direction, const Vector3d &p, const Quaterniond &q, double span,
                   const std::string &label) {
        Step st;
        st.kind = Step::Kind::Move;
        st.position = p;
        st.orientation = q;
        st.limits = base;
        if (axis == 2) { // vertical limits govern heave; never above the MPC's own vertical accel cap
            st.limits.linear_speed_vertical = speed;
            st.limits.linear_accel_vertical = base.linear_accel_vertical > 0
                                                  ? std::min(s.linear_accel, base.linear_accel_vertical)
                                                  : s.linear_accel;
        } else if (axis < 3) {
            st.limits.linear_speed = speed;
            st.limits.linear_accel = s.linear_accel;
        } else {
            st.limits.angular_speed = speed;
            st.limits.angular_accel = s.angular_accel;
        }
        st.segment.kind = Segment::Kind::Run;
        st.segment.axis = axis;
        st.segment.speed = speed;
        st.segment.direction = direction;
        st.segment.label = label;
        st.timeout = span / speed + s.run_timeout_margin;
        steps.push_back(st);
    };

    if (s.statics) {
        const struct {
            const char *label;
            Vector3d axis;
            double angle;
        } attitudes[] = {{"level", Vector3d::UnitX(), 0},        {"pitch +", Vector3d::UnitY(), s.tilt},
                         {"pitch -", Vector3d::UnitY(), -s.tilt}, {"roll +", Vector3d::UnitX(), s.tilt},
                         {"roll -", Vector3d::UnitX(), -s.tilt},  {"level (repeat)", Vector3d::UnitX(), 0}};
        for (const auto &a : attitudes)
            hold(start, heading * Quaterniond(Eigen::AngleAxisd(a.angle, a.axis)), std::string("hold ") + a.label,
                 true);
    }
    // Identification inputs while holding the start pose: settle, then apply and record.
    auto ident = [&](Step::Kind kind, Segment::Kind segment, const Quaterniond &q, const std::string &label,
                     double secs) -> Step & {
        Step st;
        st.kind = kind;
        st.position = start;
        st.orientation = q;
        st.limits = base;
        st.record = true;
        st.record_secs = secs;
        st.segment.kind = segment;
        st.segment.label = label;
        st.timeout = s.settle_timeout;
        st.thrusters = thrusters;
        steps.push_back(st);
        return steps.back();
    };
    if (s.thruster_ramps && thrusters > 0) {
        hold(start, heading, "start thruster ramps", false);
        for (int i = 0; i < thrusters; ++i) {
            Step &st = ident(Step::Kind::Ramp, Segment::Kind::Ramp, heading, "ramp thruster " + std::to_string(i),
                             s.ramp_secs);
            st.segment.thruster = i;
            st.segment.amplitude = s.ramp_force;
        }
    }
    if (s.null_space && thrusters > 0 && null_space.rows() == thrusters)
        for (int j = 0; j < null_space.cols(); ++j)
            for (double sign : {1., -1.}) {
                Step &st = ident(Step::Kind::NullSpace, Segment::Kind::NullSpace, heading,
                                 "null-space pattern " + std::to_string(j) + (sign > 0 ? " +" : " -"), s.null_secs);
                st.segment.thruster = j;
                st.segment.amplitude = sign * s.null_amplitude;
                st.bias = sign * s.null_amplitude * null_space.col(j);
                st.pre_secs = s.null_settle;
            }
    const struct {
        bool enabled;
        int axis;
        Vector3d offset;
        const std::vector<double> *speeds;
        const char *name;
    } linear[] = {{s.surge, 0, heading * Vector3d(s.lane_length, 0, 0), &s.linear_speeds, "surge"},
                  {s.sway, 1, heading * Vector3d(0, s.lane_length, 0), &s.linear_speeds, "sway"},
                  {s.heave, 2, Vector3d(0, 0, -s.heave_span), &s.heave_speeds, "heave"}};
    for (const auto &l : linear) {
        if (!l.enabled)
            continue;
        hold(start, heading, std::string("start ") + l.name, false);
        // Heave runs go down first: +z is up, so "direction" is the sign of the body z velocity.
        const int out_sign = l.axis == 2 ? -1 : 1;
        for (double v : *l.speeds)
            for (int k = 0; k < std::max(1, s.repeats); ++k) {
                run(l.axis, v, out_sign, start + l.offset, heading, l.offset.norm(), std::string(l.name) + " out");
                run(l.axis, v, -out_sign, start, heading, l.offset.norm(), std::string(l.name) + " back");
            }
    }
    if (s.yaw) {
        const Quaterniond a(Eigen::AngleAxisd(yaw - s.yaw_span / 2, Vector3d::UnitZ()));
        const Quaterniond b(Eigen::AngleAxisd(yaw + s.yaw_span / 2, Vector3d::UnitZ()));
        hold(start, a, "start yaw", false);
        for (double r : s.yaw_rates)
            for (int k = 0; k < std::max(1, s.repeats); ++k) {
                run(5, r, 1, start, b, s.yaw_span, "yaw left");
                run(5, r, -1, start, a, s.yaw_span, "yaw right");
            }
    }
    if (s.releases && thrusters > 0) {
        struct Attitude {
            const char *label;
            Vector3d axis;
            double angle;
        };
        std::vector<Attitude> attitudes{{"level", Vector3d::UnitX(), 0}};
        if (s.release_tilted) {
            attitudes.push_back({"pitched +", Vector3d::UnitY(), s.release_tilt});
            attitudes.push_back({"pitched -", Vector3d::UnitY(), -s.release_tilt});
            attitudes.push_back({"rolled +", Vector3d::UnitX(), s.release_tilt});
            attitudes.push_back({"rolled -", Vector3d::UnitX(), -s.release_tilt});
        }
        for (const auto &a : attitudes) {
            Step &st = ident(Step::Kind::Release, Segment::Kind::Release,
                             heading * Quaterniond(Eigen::AngleAxisd(a.angle, a.axis)),
                             std::string("release ") + a.label, s.release_secs);
            st.segment.axis = 2; // it rises: heave data, +z body
            st.segment.direction = 1;
            hold(start, heading, std::string("recover after release ") + a.label, false);
            steps.back().timeout = s.recover_timeout;
            steps.back().guarded = false; // it starts from the free-float attitude
        }
    }
    hold(start, heading, "finish", false);
    return steps;
}

namespace {
// Identification inputs of a step `tau` seconds into applying them.
void stepInputs(const Step &step, double tau, Sequencer::Output &out) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    switch (step.kind) {
    case Step::Kind::Release:
        out.fixed = VectorXd::Zero(step.thrusters);
        break;
    case Step::Kind::Ramp: { // 0 -> +F -> 0 -> -F -> 0, four equal legs
        out.fixed = VectorXd::Constant(step.thrusters, nan);
        const double q = std::clamp(4 * tau / step.record_secs, 0., 4.), f = step.segment.amplitude;
        out.fixed[step.segment.thruster] = q < 1 ? f * q : q < 2 ? f * (2 - q) : q < 3 ? -f * (q - 2) : -f * (4 - q);
        break;
    }
    case Step::Kind::NullSpace:
        out.fixed = VectorXd::Constant(step.bias.size(), nan);
        out.bias = step.bias;
        break;
    default:
        break;
    }
}
} // namespace

Sequencer::Output Sequencer::update(double t, bool settled, bool cut) {
    Output out;
    if (index_ >= steps_.size()) {
        out.done = true;
        return out;
    }
    const Step &step = steps_[index_];
    out.position = step.position;
    out.orientation = step.orientation;
    out.limits = step.limits;
    const auto startRecording = [&]() {
        Segment seg = step.segment;
        seg.id = next_segment_id_++;
        out.begin = seg;
        recording_ = true;
        recording_since_ = t;
    };
    if (!entered_) {
        entered_ = true;
        entered_at_ = t;
        applying_since_ = -1;
        out.new_step = true;
        std::ostringstream m;
        m << "step " << index_ + 1 << "/" << steps_.size() << ": " << step.segment.label;
        out.message = m.str();
        if (step.kind == Step::Kind::Move && step.record)
            startRecording();
        return out;
    }
    const double elapsed = t - entered_at_;
    const bool ident = step.kind == Step::Kind::Release || step.kind == Step::Kind::Ramp ||
                       step.kind == Step::Kind::NullSpace;
    bool finished = false;
    if (step.kind == Step::Kind::Hold || ident) {
        if (!recording_ && applying_since_ < 0) {
            // settle first (trust is stale for a moment after a setpoint change)
            if ((elapsed > 1.0 && settled) || elapsed > step.timeout) {
                if (elapsed > step.timeout)
                    out.message = step.segment.label + ": did not settle, " +
                                  (step.record ? "recording anyway" : "moving on");
                if (!step.record)
                    finished = true;
                else if (step.pre_secs > 0)
                    applying_since_ = t; // inputs on, record once the vehicle has answered them
                else
                    startRecording();
            }
        } else if (!recording_) {
            if (t - applying_since_ >= step.pre_secs)
                startRecording();
        } else {
            finished = t - recording_since_ >= step.record_secs;
            if (step.kind == Step::Kind::Release && cut && !finished) {
                finished = true;
                out.message = step.segment.label + ": cut short (too shallow or tilted)";
            }
        }
    } else {
        finished = (elapsed > 1.0 && settled) || elapsed > step.timeout;
        if (elapsed > step.timeout)
            out.message = step.segment.label + ": timed out";
    }
    if (ident && !finished && (recording_ || applying_since_ >= 0))
        stepInputs(step, t - (applying_since_ >= 0 ? applying_since_ : recording_since_), out);
    if (finished) {
        out.end = recording_;
        recording_ = false;
        entered_ = false;
        applying_since_ = -1;
        ++index_;
        out.done = index_ >= steps_.size();
    }
    return out;
}

//================================ Fit ==================================//

namespace {
// Statics, then per-axis drag and inertia (runs, and releases as heave data), then roll/pitch from releases.
void fitReleases(const YAML::Node &prior, const YAML::Node &vehicle, double mass, const StaticsFit &hold_model,
                 const std::vector<Sample> &samples, const std::vector<Segment> &segments, Result &result);

void fitHydrodynamics(const YAML::Node &prior, double mass, const std::vector<Sample> &samples,
                      const std::vector<Segment> &segments, Result &result, const YAML::Node &vehicle) {
    const double weight = mass * kGravity;
    const double rho = prior["water_density"].as<double>();

    // ---- statics: B and p = B r_cob from every still hold ----
    StaticsFit &st = result.statics;
    std::vector<Vector3d> ups;
    std::vector<Vector6d> holds;
    int wobbling_holds = 0;
    for (const auto &seg : segments) {
        if (seg.kind != Segment::Kind::Hold)
            continue;
        Vector6d impulse = Vector6d::Zero();
        Vector3d up = Vector3d::Zero();
        double time = 0;
        const auto hold = segmentSamples(samples, seg.id, 0.3);
        for (const Sample *s : hold)
            if (s->v.norm() < 0.05 && s->w.norm() < 0.05) {
                impulse += s->impulse;
                up += s->up * s->dt;
                time += s->dt;
            }
        if (time < 1.0) {
            // A vehicle that wobbles about the hold is rarely still. Averaged over
            // the whole hold, the wobble's inertial torque (I times the net rate
            // change over the hold) is small, so use every sample instead.
            impulse.setZero();
            up.setZero();
            time = 0;
            for (const Sample *s : hold) {
                impulse += s->impulse;
                up += s->up * s->dt;
                time += s->dt;
            }
            if (time < 1.0)
                continue;
            ++wobbling_holds;
        }
        ups.push_back(up.normalized());
        holds.push_back(impulse / time);
    }
    st.holds = static_cast<int>(holds.size());
    if (!holds.empty()) {
        double b = 0;
        for (std::size_t k = 0; k < holds.size(); ++k)
            b += weight - ups[k].dot(holds[k].head<3>());
        st.buoyancy = b / holds.size();
        // torque = up x p  ->  [up]x p = tau
        Eigen::MatrixXd A(3 * holds.size(), 3);
        Eigen::VectorXd y(3 * holds.size());
        for (std::size_t k = 0; k < holds.size(); ++k) {
            A.block<3, 3>(3 * k, 0) = skew(ups[k]);
            y.segment<3>(3 * k) = holds[k].tail<3>();
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const auto sv = svd.singularValues();
        Vector3d p;
        st.cob_z_observed = sv(2) > 0.05 * sv(0);
        if (st.cob_z_observed) {
            p = svd.solve(y);
        } else { // no tilted holds: the vertical COB offset is unobservable, keep the prior's
            const auto prior_cob = prior["cob_relative"].as<std::vector<double>>();
            const double pz = st.buoyancy * prior_cob[2];
            const Eigen::VectorXd rhs = y - A.col(2) * pz;
            const Eigen::Vector2d pxy = A.leftCols(2).colPivHouseholderQr().solve(rhs);
            p = Vector3d(pxy[0], pxy[1], pz);
            st.note = "no tilted holds: cob z kept from the prior";
        }
        st.cob = p / st.buoyancy;
        st.volume = st.buoyancy / (rho * kGravity);
        double fr = 0, tr = 0;
        for (std::size_t k = 0; k < holds.size(); ++k) {
            const Vector6d r = holds[k] - holdWrench(st, weight, ups[k]);
            fr += r.head<3>().squaredNorm();
            tr += r.tail<3>().squaredNorm();
        }
        st.force_residual = std::sqrt(fr / holds.size());
        st.torque_residual = std::sqrt(tr / holds.size());
        const double prior_volume = prior["displaced_volume"].as<double>();
        st.ok = std::abs(st.volume / prior_volume - 1) < 0.25 && st.cob.norm() < 0.15;
        if (!st.ok)
            st.note = "implausible (volume off by >25% or cob > 15 cm): kept the prior";
        else if (wobbling_holds > 0)
            st.note = (st.note.empty() ? "" : st.note + "; ") + std::to_string(wobbling_holds) +
                      " hold(s) never still: averaged over the whole hold";
    } else {
        st.note = "no usable holds";
    }
    // Without an accepted statics fit, the prior's statics define "hold".
    StaticsFit hold_model = st;
    if (!st.ok) {
        const auto cob = prior["cob_relative"].as<std::vector<double>>();
        hold_model.buoyancy = rho * kGravity * prior["displaced_volume"].as<double>();
        hold_model.cob = Vector3d(cob[0], cob[1], cob[2]);
    }

    // ---- per axis: drag from cruise, then inertia from accel/decel ----
    std::map<int, std::vector<const Segment *>> runs;
    for (const auto &seg : segments)
        if (seg.kind == Segment::Kind::Run)
            runs[seg.axis].push_back(&seg);
    for (const auto &[axis, list] : runs) {
        AxisFit a;
        a.axis = axis;
        const int d = dof(axis);
        auto excess = [&](const Sample &s) { // thrust beyond what holding still needs, per second
            return s.impulse[d] / s.dt - holdWrench(hold_model, weight, s.up)[d];
        };
        // drag: y = D1 v + D2 |v| v over cruise samples
        std::vector<double> xs, ys;
        std::vector<double> levels;
        for (const Segment *seg : list) {
            const auto ss = segmentSamples(samples, seg->id);
            // Cruise = within 10% of the run's top speed; the 90th percentile, so one DVL
            // spike cannot set a threshold the steady part never reaches.
            std::vector<double> along_run;
            for (const Sample *s : ss)
                along_run.push_back(seg->direction * along(*s, axis));
            if (along_run.empty())
                continue;
            const auto p90 = along_run.begin() + static_cast<long>(0.9 * (along_run.size() - 1));
            std::nth_element(along_run.begin(), p90, along_run.end());
            const double smax = *p90;
            if (smax < 0.02)
                continue;
            std::vector<const Sample *> cruise;
            for (const Sample *s : ss)
                if (seg->direction * along(*s, axis) >= 0.9 * smax)
                    cruise.push_back(s);
            if (cruise.size() < 5)
                continue;
            cruise.erase(cruise.begin());
            cruise.pop_back();
            std::vector<double> speeds;
            for (const Sample *s : cruise)
                speeds.push_back(std::abs(along(*s, axis)));
            std::nth_element(speeds.begin(), speeds.begin() + speeds.size() / 2, speeds.end());
            const double median = speeds[speeds.size() / 2];
            for (const Sample *s : cruise) {
                const double v = along(*s, axis);
                if (std::abs(std::abs(v) - median) > 0.07 * median)
                    continue;
                xs.push_back(v);
                ys.push_back(excess(*s));
            }
            if (std::none_of(levels.begin(), levels.end(), [&](double l) { return std::abs(l - median) < 0.2 * median; }))
                levels.push_back(median);
        }
        a.steady_samples = static_cast<int>(xs.size());
        a.speed_levels = static_cast<int>(levels.size());
        if (a.steady_samples >= 6 && a.speed_levels >= 2) {
            double s11 = 0, s12 = 0, s22 = 0, b1 = 0, b2 = 0;
            for (std::size_t k = 0; k < xs.size(); ++k) {
                const double q1 = xs[k], q2 = std::abs(xs[k]) * xs[k];
                s11 += q1 * q1, s12 += q1 * q2, s22 += q2 * q2, b1 += q1 * ys[k], b2 += q2 * ys[k];
            }
            const double det = s11 * s22 - s12 * s12;
            a.d1 = (b1 * s22 - b2 * s12) / det;
            a.d2 = (s11 * b2 - s12 * b1) / det;
            if (a.d1 < 0) { // non-negative least squares on two terms
                a.d1 = 0;
                a.d2 = b2 / s22;
            }
            if (a.d2 < 0) {
                a.d2 = 0;
                a.d1 = b1 / s11;
            }
            const double mean = std::accumulate(ys.begin(), ys.end(), 0.) / ys.size();
            double res = 0, tot = 0;
            for (std::size_t k = 0; k < xs.size(); ++k) {
                const double f = a.d1 * xs[k] + a.d2 * std::abs(xs[k]) * xs[k];
                res += (ys[k] - f) * (ys[k] - f);
                tot += (ys[k] - mean) * (ys[k] - mean);
            }
            a.drag_r2 = tot > 0 ? 1 - res / tot : 0;
            const double v_ref = *std::max_element(levels.begin(), levels.end());
            const double prior_drag = getEntry(prior["linear_damping6x6"], d, d) * v_ref +
                                      prior["quadratic_damping"][d].as<double>() * v_ref * v_ref;
            const double drag = a.d1 * v_ref + a.d2 * v_ref * v_ref;
            const double ratio = prior_drag > 0 ? drag / prior_drag : 1;
            a.drag_ok = std::isfinite(a.d1) && std::isfinite(a.d2) && a.drag_r2 >= 0.8 && ratio > 0.2 && ratio < 5;
            if (!a.drag_ok)
                a.note = "drag rejected (R2 < 0.8 or > 5x from the prior)";
        } else {
            a.note = "drag: need cruise samples at two or more speeds";
        }
        const double d1 = a.drag_ok ? a.d1 : getEntry(prior["linear_damping6x6"], d, d);
        const double d2 = a.drag_ok ? a.d2 : prior["quadratic_damping"][d].as<double>();

        // inertia: integral(excess - drag) dt = M dv over accel and decel windows
        std::vector<double> dvs, js;
        for (const Segment *seg : list) {
            const auto ss = segmentSamples(samples, seg->id);
            std::vector<double> s(ss.size());
            double smax = 0;
            for (std::size_t k = 0; k < ss.size(); ++k)
                smax = std::max(smax, s[k] = seg->direction * along(*ss[k], axis));
            if (smax < 0.02 || ss.size() < 4)
                continue;
            auto window = [&](std::size_t i, std::size_t j) {
                double y = 0;
                for (std::size_t k = i + 1; k <= j; ++k) {
                    const double v = 0.5 * (along(*ss[k - 1], axis) + along(*ss[k], axis));
                    y += excess(*ss[k]) * ss[k]->dt - (d1 * v + d2 * std::abs(v) * v) * ss[k]->dt;
                }
                dvs.push_back(along(*ss[j], axis) - along(*ss[i], axis));
                js.push_back(y);
            };
            std::size_t i0 = 0;
            while (i0 < s.size() && s[i0] < 0.05 * smax)
                ++i0;
            std::size_t i1 = i0;
            while (i1 < s.size() && s[i1] < 0.9 * smax)
                ++i1;
            if (i0 > 0 && i1 < s.size() && i1 > i0)
                window(i0 - 1, i1);
            std::size_t j0 = s.size() - 1;
            while (j0 > 0 && s[j0] < 0.9 * smax)
                --j0;
            std::size_t j1 = j0;
            while (j1 < s.size() && s[j1] > 0.05 * smax)
                ++j1;
            if (j1 < s.size() && j1 > j0 && j0 > i1)
                window(j0, j1);
        }
        a.windows = static_cast<int>(dvs.size());
        const double rigid = rigidInertia(prior, d, mass);
        if (a.windows >= 2) {
            double num = 0, den = 0;
            for (std::size_t k = 0; k < dvs.size(); ++k)
                num += js[k] * dvs[k], den += dvs[k] * dvs[k];
            a.mass_total = num / den;
            a.added_mass = a.mass_total - rigid;
            const double mean = std::accumulate(js.begin(), js.end(), 0.) / js.size();
            double res = 0, tot = 0;
            for (std::size_t k = 0; k < dvs.size(); ++k) {
                res += std::pow(js[k] - a.mass_total * dvs[k], 2);
                tot += std::pow(js[k] - mean, 2);
            }
            a.mass_r2 = tot > 0 ? 1 - res / tot : 0;
            // Over 3x rigid is rejected: Talos fits 3-4x on surge/sway, but a model with those
            // values flew much worse (2026-10-03), so they are likely sensing/actuation lag
            // folded into the inertia rather than mass.
            const bool fit_ok = std::isfinite(a.mass_total) && a.mass_r2 >= 0.8;
            const bool range_ok = a.added_mass > -0.25 * rigid && a.added_mass < 3 * rigid;
            a.mass_ok = fit_ok && range_ok;
            if (!a.mass_ok)
                a.note += std::string(a.note.empty() ? "" : "; ") +
                          (fit_ok ? "added mass rejected (outside -0.25..3x rigid)" : "added mass rejected (R2 < 0.8)");
            a.added_mass = std::max(0., a.added_mass);
        } else {
            a.note += std::string(a.note.empty() ? "" : "; ") + "added mass: no complete accel/decel windows";
        }
        result.axes.push_back(a);
    }

    // ---- heave, roll and pitch from the releases: no thruster in the loop, so the body alone ----
    if (vehicle && vehicle.IsMap())
        fitReleases(prior, vehicle, mass, hold_model, samples, segments, result);
}

// Output error over the releases: each release is simulated with the full model from the moment it starts,
// driven by its recorded (decaying) thrust, and fitted (Levenberg-Marquardt) to the measured velocity, rates
// and tilt. A free-floating vehicle pitches toward its COB-over-COM attitude while it rises, so the axes
// couple too much for a one-axis regression. Fitted: heave/roll/pitch added mass and linear damping, and the
// horizontal COB offset as a nuisance (the free-float attitude pins it; reported, not written). The statics'
// vertical COB and buoyancy stay the holds' - a release only sees ratios to them (damping/stiffness) - and a
// parameter not determined to 10% of its scale goes back to the prior.
void fitReleases(const YAML::Node &prior, const YAML::Node &vehicle, double mass, const StaticsFit &hold_model,
                 const std::vector<Sample> &samples, const std::vector<Segment> &segments, Result &result) {
    const double rho = prior["water_density"].as<double>();
    YAML::Node base = YAML::Clone(prior);
    base["displaced_volume"] = hold_model.buoyancy / (rho * kGravity);
    base["cob_relative"] = std::vector<double>{hold_model.cob.x(), hold_model.cob.y(), hold_model.cob.z()};
    for (const auto &a : result.axes) {
        if (a.drag_ok) {
            setEntry(base["linear_damping6x6"], a.axis, a.axis, a.d1);
            base["quadratic_damping"][a.axis] = a.d2;
        }
        if (a.mass_ok)
            setEntry(base["added_mass6x6"], a.axis, a.axis, a.added_mass);
    }
    std::vector<std::vector<const Sample *>> shots; // one per release, from its first sample
    for (const auto &seg : segments)
        if (seg.kind == Segment::Kind::Release) {
            const auto ss = segmentSamples(samples, seg.id);
            if (ss.size() >= 8)
                shots.push_back(ss);
        }
    if (shots.empty())
        return;
    // Channel scales: the spread of each measured quantity over the releases.
    Eigen::Array<double, 9, 1> mean = Eigen::Array<double, 9, 1>::Zero(), sq = mean;
    int count = 0;
    for (const auto &shot : shots)
        for (const Sample *x : shot) {
            Eigen::Array<double, 9, 1> m;
            m << x->v.array(), x->w.array(), x->up.array();
            mean += m;
            sq += m * m;
            ++count;
        }
    mean /= count;
    const Eigen::Array<double, 9, 1> scale = (sq / count - mean * mean).max(0.).sqrt().max(0.005);

    // Parameters: 0,1 = cob x, y; then per axis 2 (heave), 3 (roll), 4 (pitch): added mass, linear damping.
    const int params = 8;
    const auto axisOf = [](int j) { return 2 + (j - 2) / 2; };
    const auto isMass = [](int j) { return j >= 2 && j % 2 == 0; };
    Eigen::VectorXd theta(params), ref(params);
    theta[0] = hold_model.cob.x();
    theta[1] = hold_model.cob.y();
    ref[0] = ref[1] = 0.02; // m: 10% of it = 2 mm
    for (int j = 2; j < params; ++j) {
        const int d = axisOf(j);
        theta[j] = getEntry(base[isMass(j) ? "added_mass6x6" : "linear_damping6x6"], d, d);
        ref[j] = isMass(j) ? rigidInertia(prior, d, mass) + theta[j] : std::max(theta[j], 1.);
    }
    // Damping scale (and start, where the prior has next to none): roll/pitch their critical damping
    // 2 sqrt(I K) with the righting stiffness K = B |cob| from the holds; an untuned prior (zero damping)
    // would otherwise demand a 0.1 standard error and start the fit from an undamped, ringing model.
    for (int j = 3; j < params; j += 2) {
        const int d = axisOf(j);
        if (d < 3)
            continue;
        const double inertia = ref[j - 1], stiffness = hold_model.buoyancy * hold_model.cob.norm();
        const double critical = 2 * std::sqrt(std::max(inertia * stiffness, 0.));
        ref[j] = std::max(ref[j], critical);
        if (theta[j] < 0.1 * critical)
            theta[j] = critical;
    }
    const auto residual = [&](const Eigen::VectorXd &th) {
        YAML::Node h = YAML::Clone(base);
        h["cob_relative"] = std::vector<double>{th[0], th[1], hold_model.cob.z()};
        for (int j = 2; j < params; ++j)
            setEntry(h[isMass(j) ? "added_mass6x6" : "linear_damping6x6"], axisOf(j), axisOf(j), std::max(0., th[j]));
        const FossenModel model = FossenModel::fromNodes(vehicle, h);
        const int n = model.thrusterCount();
        std::vector<double> r;
        for (const auto &shot : shots) {
            const Sample &first = *shot.front();
            // Deep (fully submerged; the depth does not matter): z = 0 would be the water surface.
            State13d x = model.fromBaseLink(Vector3d(0, 0, -10), Quaterniond::FromTwoVectors(first.up, Vector3d::UnitZ()),
                                            first.v, first.w);
            double t = first.t;
            for (std::size_t k = 1; k < shot.size(); ++k) {
                const Sample &m = *shot[k];
                // The recorded realized thrust over this interval (it decays to zero after the release).
                VectorXd force = VectorXd::Zero(n);
                if (m.thrust_positive.size() == n && m.dt > 0)
                    force = (m.thrust_positive + m.thrust_negative) / m.dt;
                while (t < m.t - 1e-9) {
                    const double step = std::min(0.01, m.t - t);
                    VectorXd held = force;
                    model.step(x, held, force, step);
                    t += step;
                }
                const Quaterniond qp(x[3], x[4], x[5], x[6]);
                Eigen::Array<double, 9, 1> e;
                e << (model.baseLinkVelocity(x) - m.v).array(), (x.tail<3>() - m.w).array(),
                    (qp.normalized().conjugate() * Vector3d::UnitZ() - m.up).array();
                e /= scale;
                r.insert(r.end(), e.data(), e.data() + 9);
            }
        }
        return Eigen::VectorXd(Eigen::Map<Eigen::VectorXd>(r.data(), r.size()));
    };
    const auto jacobian = [&](const Eigen::VectorXd &th, const Eigen::VectorXd &r0, const std::vector<bool> &free) {
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(r0.size(), params);
        for (int j = 0; j < params; ++j)
            if (free[j]) {
                Eigen::VectorXd t2 = th;
                const double h = 0.02 * ref[j];
                t2[j] += h;
                J.col(j) = (residual(t2) - r0) / h;
            }
        return J;
    };
    const Eigen::VectorXd theta_prior = theta;
    Eigen::VectorXd r = residual(theta);
    const double prior_cost = r.squaredNorm();
    std::vector<bool> free(params, true);
    Eigen::VectorXd relative_error = Eigen::VectorXd::Constant(params, 1e9);
    for (int round = 0; round < 4; ++round) {
        double lambda = 1e-2;
        for (int iteration = 0; iteration < 25; ++iteration) {
            const Eigen::MatrixXd J = jacobian(theta, r, free);
            const Eigen::MatrixXd JtJ = J.transpose() * J;
            const Eigen::VectorXd g = J.transpose() * r;
            bool improved = false;
            for (int tries = 0; tries < 8 && !improved; ++tries) {
                Eigen::MatrixXd H = JtJ;
                for (int j = 0; j < params; ++j)
                    H(j, j) += free[j] ? lambda * std::max(JtJ(j, j), 1e-9) : 1.;
                Eigen::VectorXd next = theta - H.ldlt().solve(g);
                for (int j = 0; j < params; ++j)
                    next[j] = !free[j] ? theta[j] : j >= 2 ? std::max(0., next[j]) : next[j];
                const Eigen::VectorXd rn = residual(next);
                if (rn.squaredNorm() < r.squaredNorm()) {
                    improved = (theta - next).cwiseQuotient(ref).norm() > 1e-5;
                    theta = next;
                    r = rn;
                    lambda = std::max(lambda / 3, 1e-6);
                } else {
                    lambda *= 4;
                }
            }
            if (!improved)
                break;
        }
        // Standard errors at the estimate, against each parameter's scale (not its own value).
        const Eigen::MatrixXd J = jacobian(theta, r, free);
        std::vector<int> idx;
        for (int j = 0; j < params; ++j)
            if (free[j])
                idx.push_back(j);
        if (idx.empty())
            break;
        Eigen::MatrixXd Jf(J.rows(), idx.size());
        for (std::size_t k = 0; k < idx.size(); ++k)
            Jf.col(k) = J.col(idx[k]);
        const double s2 = r.squaredNorm() / std::max<Eigen::Index>(1, r.size() - static_cast<Eigen::Index>(idx.size()));
        const Eigen::MatrixXd cov = s2 * (Jf.transpose() * Jf).ldlt().solve(Eigen::MatrixXd::Identity(idx.size(), idx.size()));
        bool dropped = false;
        for (std::size_t k = 0; k < idx.size(); ++k) {
            const int j = idx[k];
            relative_error[j] = std::sqrt(std::max(0., cov(k, k))) / ref[j];
            if (!(relative_error[j] <= 0.1)) {
                free[j] = false;
                theta[j] = theta_prior[j];
                dropped = true;
            }
        }
        if (!dropped)
            break;
        r = residual(theta);
    }
    double sst = 0;
    for (const auto &shot : shots)
        for (std::size_t k = 1; k < shot.size(); ++k) {
            Eigen::Array<double, 9, 1> m;
            m << shot[k]->v.array(), shot[k]->w.array(), shot[k]->up.array();
            sst += (((m - mean) / scale).square()).sum();
        }
    const double r2 = sst > 0 ? 1 - r.squaredNorm() / sst : 0;
    const bool fit_ok = r2 >= 0.8 && r.squaredNorm() <= prior_cost;
    for (int d : {2, 3, 4}) {
        AxisFit a;
        a.axis = d;
        a.windows = static_cast<int>(shots.size());
        const int jm = 2 + 2 * (d - 2), jd = jm + 1;
        const double rigid = rigidInertia(prior, d, mass);
        a.added_mass = theta[jm];
        a.mass_total = rigid + theta[jm];
        a.d1 = theta[jd];
        a.d2 = base["quadratic_damping"][d].as<double>();
        a.mass_r2 = a.drag_r2 = r2;
        // An implausible inertia from the same fit discredits that axis' damping too.
        const bool plausible = !free[jm] || (a.added_mass > -0.25 * rigid && a.added_mass < 4 * rigid);
        a.mass_ok = fit_ok && free[jm] && plausible;
        a.drag_ok = fit_ok && free[jd] && plausible;
        char m[220];
        std::snprintf(m, sizeof m,
                      "release fit (relative to the hold statics), R2 %.2f: inertia %s, damping %s; free-float cob x %.4f, "
                      "y %.4f%s",
                      r2, !free[jm] ? "not determined (prior)" : plausible ? "fitted" : "implausible",
                      !free[jd] ? "not determined (prior)" : plausible ? "fitted" : "rejected with it", theta[0], theta[1],
                      free[0] && free[1] ? "" : " (not determined)");
        a.note = fit_ok ? m : "release fit rejected (R2 < 0.8 or worse than the prior)";
        const auto runs_fit = std::find_if(result.axes.begin(), result.axes.end(),
                                           [d](const AxisFit &f) { return f.axis == d; });
        if (runs_fit != result.axes.end()) { // heave from the runs wins; the release is a cross-check
            char c[200];
            std::snprintf(c, sizeof c, "release cross-check: inertia %.1f%s, linear drag %.1f%s", a.mass_total,
                          a.mass_ok ? "" : " (not determined)", a.d1, a.drag_ok ? "" : " (not determined)");
            if (runs_fit->mass_ok && runs_fit->drag_ok) {
                runs_fit->note += std::string(runs_fit->note.empty() ? "" : "; ") + c;
                continue;
            }
            a.note = std::string(c) + "; used: the runs did not fit";
            *runs_fit = a;
            continue;
        }
        result.axes.push_back(a);
    }
}

// Still stretches of holds, ramps and null-space patterns, averaged over about a second: the thrust there
// balances the statics alone.
struct Chunk {
    VectorXd positive, negative; // impulse of each thruster's positive / negative force [N s]
    Vector3d up = Vector3d::UnitZ();
    double time = 0;
};

std::vector<Chunk> stillChunks(const std::vector<Sample> &samples, const std::vector<Segment> &segments) {
    std::vector<Chunk> out;
    for (const auto &seg : segments) {
        if (seg.kind != Segment::Kind::Hold && seg.kind != Segment::Kind::Ramp && seg.kind != Segment::Kind::NullSpace)
            continue;
        Chunk c;
        Vector3d up = Vector3d::Zero();
        for (const Sample *s : segmentSamples(samples, seg.id, seg.kind == Segment::Kind::Hold ? 0.3 : 0.)) {
            const bool still = s->v.norm() < 0.05 && s->w.norm() < 0.05 && s->thrust_positive.size() > 0;
            if (!still) { // a moving stretch breaks the chunk
                c = Chunk();
                up.setZero();
                continue;
            }
            if (c.time == 0) {
                c.positive = VectorXd::Zero(s->thrust_positive.size());
                c.negative = c.positive;
            }
            c.positive += s->thrust_positive;
            c.negative += s->thrust_negative;
            up += s->up * s->dt;
            c.time += s->dt;
            if (c.time >= 1.0) {
                c.up = up.normalized();
                out.push_back(c);
                c = Chunk();
                up.setZero();
            }
        }
    }
    return out;
}

// Thrusters that work on the same wrench components (each column's entries above 10% of its largest),
// joined transitively. On Talos that splits the four surge thrusters (Fx, pitch, yaw) from the four vectored
// ones (Fy, Fz, roll) only because the vectored four sit ~1 cm from the COM along x (their pitch/yaw entries
// stay under 10%); a COM several cm off would join all eight into one group, i.e. one shared scale pin.
std::vector<int> thrusterGroups(const MatrixXd &T) {
    const int n = static_cast<int>(T.cols());
    std::vector<int> group(n);
    std::iota(group.begin(), group.end(), 0);
    const auto root = [&](int i) {
        while (group[i] != i)
            i = group[i];
        return i;
    };
    for (int r = 0; r < T.rows(); ++r) {
        int first = -1;
        for (int i = 0; i < n; ++i) {
            if (std::abs(T(r, i)) <= 0.1 * T.col(i).cwiseAbs().maxCoeff())
                continue;
            if (first < 0)
                first = i;
            else
                group[root(i)] = root(first);
        }
    }
    std::map<int, int> label;
    std::vector<int> out(n);
    for (int i = 0; i < n; ++i)
        out[i] = label.emplace(root(i), static_cast<int>(label.size())).first->second;
    return out;
}

// Joint least squares over the still chunks for x = [k+ (n), k- (n), B, p = B r_cob (3)]:
//   sum_i T_i (k+_i P_i + k-_i N_i) / t + [B up; -up x p] = [W up; 0]
// with each thruster group's forward mean pinned to 1 (see ThrusterFit) and a weak pull of every gain
// toward 1 for directions never exercised.
ThrusterFit fitThrusters(double mass, const std::vector<Chunk> &chunks, const MatrixXd &T) {
    ThrusterFit f;
    const int n = static_cast<int>(T.cols());
    f.group = thrusterGroups(T);
    const int groups = *std::max_element(f.group.begin(), f.group.end()) + 1;
    f.forward = f.reverse = VectorXd::Ones(n);
    f.forward_time = f.reverse_time = VectorXd::Zero(n);
    f.forward_ok.assign(n, false);
    f.reverse_ok.assign(n, false);
    f.chunks = static_cast<int>(chunks.size());
    if (chunks.size() < 10) {
        f.note = "too little still data (thruster ramps, null-space patterns, holds)";
        return f;
    }
    const double weight = mass * kGravity, arm = 0.25; // torque rows in N at a 25 cm lever
    const int cols = 2 * n + 4, data = 6 * f.chunks;
    MatrixXd Ad = MatrixXd::Zero(data, cols);
    VectorXd yd = VectorXd::Zero(data);
    for (int c = 0; c < f.chunks; ++c) {
        const Chunk &k = chunks[c];
        for (int i = 0; i < n; ++i) {
            Ad.block<6, 1>(6 * c, i) = T.col(i) * (k.positive[i] / k.time);
            Ad.block<6, 1>(6 * c, n + i) = T.col(i) * (k.negative[i] / k.time);
            if (k.positive[i] / k.time > 1)
                f.forward_time[i] += k.time;
            if (k.negative[i] / k.time < -1)
                f.reverse_time[i] += k.time;
        }
        Ad.block<3, 1>(6 * c, 2 * n) = k.up;
        Ad.block<3, 3>(6 * c + 3, 2 * n + 1) = -skew(k.up);
        yd.segment<3>(6 * c) = weight * k.up;
        Ad.block(6 * c + 3, 0, 3, cols) /= arm;
    }
    // Pins: each group's forward mean = 1, and its reverse mean where reverse_pinned; a weak pull of every
    // gain toward 1 keeps directions that were never exercised at the prior.
    const double pin = 1e3, pull = 0.3;
    MatrixXd A;
    VectorXd y;
    const auto solve = [&]() {
        const int pins = groups + static_cast<int>(std::count(f.reverse_pinned.begin(), f.reverse_pinned.end(), true));
        A = MatrixXd::Zero(data + pins + 2 * n, cols);
        y = VectorXd::Zero(A.rows());
        A.topRows(data) = Ad;
        y.head(data) = yd;
        int row = data;
        for (int g = 0; g < groups; ++g) {
            const double members = static_cast<double>(std::count(f.group.begin(), f.group.end(), g));
            for (int reverse = 0; reverse < 2; ++reverse) {
                if (reverse && !f.reverse_pinned[g])
                    continue;
                for (int i = 0; i < n; ++i)
                    if (f.group[i] == g)
                        A(row, reverse * n + i) = pin / members;
                y[row++] = pin;
            }
        }
        for (int i = 0; i < 2 * n; ++i) {
            A(row + i, i) = pull;
            y[row + i] = pull;
        }
        return VectorXd(A.colPivHouseholderQr().solve(y));
    };
    // A group's reverse mean the data cannot tell from its forward one (Talos' vectored four only ever move
    // together while holding, so never at mixed signs) is pinned too: unless its members ran at mixed signs
    // beyond 2 N for 20 s, or where its standard error exceeds 3%. Mixed signs are what separate the two:
    // its other information is the small net hold force, which model error elsewhere swamps.
    f.reverse_pinned.assign(groups, false);
    for (int g = 0; g < groups; ++g) {
        double mixed = 0;
        for (const Chunk &k : chunks) {
            bool forward = false, reverse = false;
            for (int i = 0; i < n; ++i)
                if (f.group[i] == g) {
                    forward = forward || k.positive[i] / k.time > 2;
                    reverse = reverse || k.negative[i] / k.time < -2;
                }
            mixed += forward && reverse ? k.time : 0;
        }
        f.reverse_pinned[g] = mixed < 20;
    }
    VectorXd x = solve();
    {
        const VectorXd rd = Ad * x - yd;
        const double s2 = rd.squaredNorm() / std::max(1, data - cols);
        const MatrixXd cov = s2 * (A.transpose() * A).ldlt().solve(MatrixXd::Identity(cols, cols));
        bool repin = false;
        for (int g = 0; g < groups; ++g) {
            VectorXd a = VectorXd::Zero(cols);
            const double members = static_cast<double>(std::count(f.group.begin(), f.group.end(), g));
            for (int i = 0; i < n; ++i)
                if (f.group[i] == g)
                    a[n + i] = 1 / members;
            if (std::sqrt(a.dot(cov * a)) > 0.03) {
                f.reverse_pinned[g] = true;
                repin = true;
            }
        }
        if (repin)
            x = solve();
    }
    double fr = 0, tr = 0;
    const VectorXd r = A.topRows(6 * f.chunks) * x - y.head(6 * f.chunks);
    for (int c = 0; c < f.chunks; ++c) {
        fr += r.segment<3>(6 * c).squaredNorm();
        tr += (r.segment<3>(6 * c + 3) * arm).squaredNorm();
    }
    f.force_residual = std::sqrt(fr / f.chunks);
    f.torque_residual = std::sqrt(tr / f.chunks);
    f.buoyancy = x[2 * n];
    f.cob = x.segment<3>(2 * n + 1) / f.buoyancy;
    bool plausible = std::isfinite(f.buoyancy) && x.allFinite();
    for (int i = 0; i < n; ++i) {
        f.forward_ok[i] = f.forward_time[i] >= 5 && x[i] > 0.5 && x[i] < 1.5;
        f.reverse_ok[i] = f.reverse_time[i] >= 5 && x[n + i] > 0.4 && x[n + i] < 1.5;
        f.forward[i] = f.forward_ok[i] ? x[i] : 1.;
        f.reverse[i] = f.reverse_ok[i] ? x[n + i] : 1.;
        plausible = plausible && (f.forward_time[i] < 5 || f.forward_ok[i]) && (f.reverse_time[i] < 5 || f.reverse_ok[i]);
    }
    f.ok = plausible && std::any_of(f.forward_ok.begin(), f.forward_ok.end(), [](bool b) { return b; });
    if (!plausible)
        f.note = "implausible gains (outside 0.5..1.5 forward, 0.4..1.5 reverse) or nonfinite: kept the prior";
    else if (!f.ok)
        f.note = "no thruster exercised enough (5 s beyond 1 N): kept the prior";
    return f;
}

// The recording with every thruster's force scaled by its fitted gain.
std::vector<Sample> correctedSamples(const std::vector<Sample> &samples, const ThrusterFit &f, const MatrixXd &T) {
    std::vector<Sample> out = samples;
    for (auto &s : out)
        if (s.thrust_positive.size() == T.cols())
            s.impulse = T * (f.forward.cwiseProduct(s.thrust_positive) + f.reverse.cwiseProduct(s.thrust_negative));
    return out;
}
} // namespace

Result fit(const YAML::Node &prior, double mass, const std::vector<Sample> &samples,
           const std::vector<Segment> &segments, const MatrixXd &thruster_matrix, const YAML::Node &vehicle) {
    Result result;
    if (thruster_matrix.cols() > 0) {
        result.thrusters = fitThrusters(mass, stillChunks(samples, segments), thruster_matrix);
        if (result.thrusters.ok) {
            fitHydrodynamics(prior, mass, correctedSamples(samples, result.thrusters, thruster_matrix), segments,
                             result, vehicle);
            return result;
        }
    }
    fitHydrodynamics(prior, mass, samples, segments, result, vehicle);
    return result;
}

namespace {
// [a, b, c] for every list of numbers (a matrix: one row per line), like the hand-written model files.
void flowNumberLists(YAML::Node n) {
    if (n.IsMap()) {
        for (auto kv : n)
            flowNumberLists(kv.second);
    } else if (n.IsSequence()) {
        if (std::all_of(n.begin(), n.end(), [](const YAML::Node &e) { return e.IsScalar(); }))
            n.SetStyle(YAML::EmitterStyle::Flow);
        else
            for (auto e : n)
                flowNumberLists(e);
    }
}
} // namespace

YAML::Node withBody(const YAML::Node &model, const YAML::Node &vehicle) {
    if (model["mass"] || model["com"])
        return YAML::Clone(model);
    YAML::Node m;
    m["mass"] = YAML::Clone(vehicle["mass"]);
    m["com"] = YAML::Clone(vehicle["com"]);
    for (const auto &kv : model)
        m[kv.first.as<std::string>()] = YAML::Clone(kv.second);
    return m;
}

YAML::Node identifiedModel(const YAML::Node &prior_in, const Result &r, const YAML::Node &thrust_base_in) {
    YAML::Node m = YAML::Clone(prior_in);
    if (r.statics.ok) {
        m["displaced_volume"] = r.statics.volume;
        m["cob_relative"] = std::vector<double>{r.statics.cob.x(), r.statics.cob.y(), r.statics.cob.z()};
    }
    for (const auto &a : r.axes) {
        const int d = dof(a.axis);
        if (a.drag_ok) {
            setEntry(m["linear_damping6x6"], d, d, a.d1);
            m["quadratic_damping"][d] = a.d2;
        }
        if (a.mass_ok)
            setEntry(m["added_mass6x6"], d, d, a.added_mass);
    }
    if (r.thrusters.ok) {
        // The gains scale the base model's per-direction scales (efficiency, at most 1, stays as it was):
        // realized = command x scale x efficiency.
        const YAML::Node base = thrust_base_in && thrust_base_in.IsMap() ? thrust_base_in : prior_in;
        const std::size_t n = base["thruster_efficiencies"].size();
        const auto scales = [&](const char *list, const char *shared) {
            std::vector<double> v(n, base["thruster_dynamics"][shared].as<double>(1.0));
            if (base[list])
                v = base[list].as<std::vector<double>>();
            return v;
        };
        std::vector<double> forward = scales("thruster_forward_scales", "forward_scale"),
                            reverse = scales("thruster_reverse_scales", "reverse_scale");
        if (static_cast<int>(n) == r.thrusters.forward.size() && forward.size() == n && reverse.size() == n) {
            for (std::size_t i = 0; i < n; ++i) {
                forward[i] *= r.thrusters.forward[i];
                reverse[i] *= r.thrusters.reverse[i];
            }
            m["thruster_efficiencies"] = base["thruster_efficiencies"];
            m["thruster_forward_scales"] = forward;
            m["thruster_reverse_scales"] = reverse;
        }
    }
    for (const char *unused : {"schema_version", "parameter_status", "provenance"})
        m.remove(unused);
    flowNumberLists(m);
    return m;
}

YAML::Node report(const YAML::Node &prior, const Result &r) {
    YAML::Node out;
    const auto &st = r.statics;
    YAML::Node s;
    s["accepted"] = st.ok;
    s["holds"] = st.holds;
    s["displaced_volume"] = st.volume;
    s["displaced_volume_prior"] = prior["displaced_volume"].as<double>();
    s["net_buoyancy_N"] = st.buoyancy;
    s["cob_relative"] = std::vector<double>{st.cob.x(), st.cob.y(), st.cob.z()};
    s["cob_relative_prior"] = prior["cob_relative"];
    s["cob_z_observed"] = st.cob_z_observed;
    s["residual_force_rms_N"] = st.force_residual;
    s["residual_torque_rms_Nm"] = st.torque_residual;
    if (!st.note.empty())
        s["note"] = st.note;
    out["statics"] = s;
    static const char *names[] = {"surge", "sway", "heave", "roll", "pitch", "yaw"};
    for (const auto &a : r.axes) {
        const int d = dof(a.axis);
        YAML::Node n;
        n["drag_accepted"] = a.drag_ok;
        n["linear_damping"] = a.d1;
        n["linear_damping_prior"] = getEntry(prior["linear_damping6x6"], d, d);
        n["quadratic_damping"] = a.d2;
        n["quadratic_damping_prior"] = prior["quadratic_damping"][d].as<double>();
        n["drag_r2"] = a.drag_r2;
        n["cruise_samples"] = a.steady_samples;
        n["speed_levels"] = a.speed_levels;
        n["added_mass_accepted"] = a.mass_ok;
        n["added_mass"] = a.added_mass;
        n["added_mass_prior"] = getEntry(prior["added_mass6x6"], d, d);
        n["total_inertia"] = a.mass_total;
        n["inertia_r2"] = a.mass_r2;
        n["accel_windows"] = a.windows;
        if (!a.note.empty())
            n["note"] = a.note;
        out[names[d]] = n;
    }
    const ThrusterFit &f = r.thrusters;
    if (f.forward.size() > 0) {
        YAML::Node t;
        t["accepted"] = f.ok;
        t["still_chunks"] = f.chunks;
        t["forward_gain"] = std::vector<double>(f.forward.data(), f.forward.data() + f.forward.size());
        t["reverse_gain"] = std::vector<double>(f.reverse.data(), f.reverse.data() + f.reverse.size());
        t["forward_seconds"] = std::vector<double>(f.forward_time.data(), f.forward_time.data() + f.forward_time.size());
        t["reverse_seconds"] = std::vector<double>(f.reverse_time.data(), f.reverse_time.data() + f.reverse_time.size());
        t["forward_accepted"] = f.forward_ok;
        t["reverse_accepted"] = f.reverse_ok;
        t["joint_net_buoyancy_N"] = f.buoyancy;
        t["joint_cob_relative"] = std::vector<double>{f.cob.x(), f.cob.y(), f.cob.z()};
        t["residual_force_rms_N"] = f.force_residual;
        t["residual_torque_rms_Nm"] = f.torque_residual;
        t["group"] = f.group;
        t["reverse_mean_pinned"] = f.reverse_pinned; // per group: never at mixed signs, reverse mean = prior
        t["note"] = f.note.empty() ? "gains relative to the prior model; each group's forward mean pinned to 1 "
                                     "(holding still cannot tell a group's overall scale from the statics)"
                                   : f.note;
        out["thrusters"] = t;
    }
    return out;
}

//================================= IO ===================================//

void writeRecording(const std::string &dir, const std::vector<Sample> &samples, const std::vector<Segment> &segments) {
    std::ofstream seg(dir + "/segments.csv");
    seg << "id,kind,axis,speed,direction,label,thruster,amplitude\n";
    static const char *kinds[] = {"hold", "run", "release", "ramp", "null"};
    for (const auto &s : segments)
        seg << s.id << ',' << kinds[static_cast<int>(s.kind)] << ',' << s.axis << ',' << s.speed << ',' << s.direction
            << ',' << s.label << ',' << s.thruster << ',' << s.amplitude << '\n';
    std::ofstream out(dir + "/recording.csv");
    out.precision(10);
    // Then each thruster's positive-force impulse p0.. and negative-force impulse n0.. [N s].
    const long n = samples.empty() ? 0 : samples.front().thrust_positive.size();
    out << "segment,t,dt,vx,vy,vz,wx,wy,wz,upx,upy,upz,jx,jy,jz,jroll,jpitch,jyaw";
    for (const char *sign : {"p", "n"})
        for (long i = 0; i < n; ++i)
            out << ',' << sign << i;
    out << '\n';
    for (const auto &s : samples) {
        out << s.segment << ',' << s.t << ',' << s.dt;
        for (const Vector3d *v : {&s.v, &s.w, &s.up})
            out << ',' << v->x() << ',' << v->y() << ',' << v->z();
        for (int i = 0; i < 6; ++i)
            out << ',' << s.impulse[i];
        for (const VectorXd *part : {&s.thrust_positive, &s.thrust_negative})
            for (long i = 0; i < n; ++i)
                out << ',' << (i < part->size() ? (*part)[i] : 0.);
        out << '\n';
    }
}

void readRecording(const std::string &dir, std::vector<Sample> &samples, std::vector<Segment> &segments) {
    auto rows = [](const std::string &path) {
        std::ifstream in(path);
        if (!in)
            throw std::runtime_error("cannot read " + path);
        std::vector<std::vector<std::string>> out;
        std::string line;
        std::getline(in, line); // header
        while (std::getline(in, line)) {
            std::vector<std::string> cells;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ','))
                cells.push_back(cell);
            out.push_back(cells);
        }
        return out;
    };
    segments.clear();
    for (const auto &c : rows(dir + "/segments.csv")) {
        Segment s;
        s.id = std::stoi(c.at(0));
        const std::string kind = c.at(1);
        s.kind = kind == "hold"      ? Segment::Kind::Hold
                 : kind == "release" ? Segment::Kind::Release
                 : kind == "ramp"    ? Segment::Kind::Ramp
                 : kind == "null"    ? Segment::Kind::NullSpace
                                     : Segment::Kind::Run;
        s.axis = std::stoi(c.at(2));
        s.speed = std::stod(c.at(3));
        s.direction = std::stoi(c.at(4));
        s.label = c.size() > 5 ? c[5] : "";
        if (c.size() > 7) {
            s.thruster = std::stoi(c[6]);
            s.amplitude = std::stod(c[7]);
        }
        segments.push_back(s);
    }
    samples.clear();
    for (const auto &c : rows(dir + "/recording.csv")) {
        if (c.size() < 18)
            continue;
        Sample s;
        s.segment = std::stoi(c[0]);
        s.t = std::stod(c[1]);
        s.dt = std::stod(c[2]);
        s.v = Vector3d(std::stod(c[3]), std::stod(c[4]), std::stod(c[5]));
        s.w = Vector3d(std::stod(c[6]), std::stod(c[7]), std::stod(c[8]));
        s.up = Vector3d(std::stod(c[9]), std::stod(c[10]), std::stod(c[11]));
        for (int i = 0; i < 6; ++i)
            s.impulse[i] = std::stod(c[12 + i]);
        const long n = (static_cast<long>(c.size()) - 18) / 2;
        if (n > 0) {
            s.thrust_positive.resize(n);
            s.thrust_negative.resize(n);
            for (long i = 0; i < n; ++i) {
                s.thrust_positive[i] = std::stod(c[18 + i]);
                s.thrust_negative[i] = std::stod(c[18 + n + i]);
            }
        }
        samples.push_back(s);
    }
}
} // namespace riptide_mpc::ident
