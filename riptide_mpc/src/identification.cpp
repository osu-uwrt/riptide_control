#include "riptide_mpc/identification.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
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
    const auto I = prior["rigid_body_inertia3x3"].as<std::vector<double>>();
    return I.at((dof - 3) * 4); // diagonal of the row-major 3x3
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
        impulse_ += model_.thrusterMatrix() * actuator_.forces() * h;
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
        if (s.dt > 0)
            samples_.push_back(s);
    }
    impulse_.setZero();
    since_ = t;
}

void Recorder::begin(const Segment &segment, double t) {
    advance(t);
    impulse_.setZero();
    since_ = t;
    active_ = segment;
    segments_.push_back(segment);
}

void Recorder::end(double t) {
    advance(t);
    active_.reset();
}

//=============================== Sequence ===============================//

std::vector<Step> buildSequence(const SequenceSettings &s, const Vector3d &start, double yaw,
                                const MotionLimits &base) {
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
        if (axis < 3) {
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
        for (double v : *l.speeds) {
            run(l.axis, v, out_sign, start + l.offset, heading, l.offset.norm(), std::string(l.name) + " out");
            run(l.axis, v, -out_sign, start, heading, l.offset.norm(), std::string(l.name) + " back");
        }
    }
    if (s.yaw) {
        const Quaterniond a(Eigen::AngleAxisd(yaw - s.yaw_span / 2, Vector3d::UnitZ()));
        const Quaterniond b(Eigen::AngleAxisd(yaw + s.yaw_span / 2, Vector3d::UnitZ()));
        hold(start, a, "start yaw", false);
        for (double r : s.yaw_rates) {
            run(5, r, 1, start, b, s.yaw_span, "yaw left");
            run(5, r, -1, start, a, s.yaw_span, "yaw right");
        }
    }
    hold(start, heading, "finish", false);
    return steps;
}

Sequencer::Output Sequencer::update(double t, bool settled) {
    Output out;
    if (index_ >= steps_.size()) {
        out.done = true;
        return out;
    }
    const Step &step = steps_[index_];
    out.position = step.position;
    out.orientation = step.orientation;
    out.limits = step.limits;
    if (!entered_) {
        entered_ = true;
        entered_at_ = t;
        out.new_step = true;
        std::ostringstream m;
        m << "step " << index_ + 1 << "/" << steps_.size() << ": " << step.segment.label;
        out.message = m.str();
        if (step.kind == Step::Kind::Move && step.record) {
            Segment seg = step.segment;
            seg.id = next_segment_id_++;
            out.begin = seg;
            recording_ = true;
            recording_since_ = t;
        }
        return out;
    }
    const double elapsed = t - entered_at_;
    bool finished = false;
    if (step.kind == Step::Kind::Hold) {
        if (!recording_) {
            // settle first (trust is stale for a moment after a setpoint change)
            if ((elapsed > 1.0 && settled) || elapsed > step.timeout) {
                if (elapsed > step.timeout)
                    out.message = step.segment.label + ": did not settle, recording anyway";
                if (!step.record) {
                    finished = true;
                } else {
                    Segment seg = step.segment;
                    seg.id = next_segment_id_++;
                    out.begin = seg;
                    recording_ = true;
                    recording_since_ = t;
                }
            }
        } else {
            finished = t - recording_since_ >= step.record_secs;
        }
    } else {
        finished = (elapsed > 1.0 && settled) || elapsed > step.timeout;
        if (elapsed > step.timeout)
            out.message = step.segment.label + ": timed out";
    }
    if (finished) {
        out.end = recording_;
        recording_ = false;
        entered_ = false;
        ++index_;
        out.done = index_ >= steps_.size();
    }
    return out;
}

//================================ Fit ==================================//

Result fit(const YAML::Node &prior, double mass, const std::vector<Sample> &samples,
           const std::vector<Segment> &segments) {
    Result result;
    const double weight = mass * kGravity;
    const double rho = prior["water_density"].as<double>();

    // ---- statics: B and p = B r_cob from every still hold ----
    StaticsFit &st = result.statics;
    std::vector<Vector3d> ups;
    std::vector<Vector6d> holds;
    for (const auto &seg : segments) {
        if (seg.kind != Segment::Kind::Hold)
            continue;
        Vector6d impulse = Vector6d::Zero();
        Vector3d up = Vector3d::Zero();
        double time = 0;
        for (const Sample *s : segmentSamples(samples, seg.id, 0.3))
            if (s->v.norm() < 0.05 && s->w.norm() < 0.05) {
                impulse += s->impulse;
                up += s->up * s->dt;
                time += s->dt;
            }
        if (time < 1.0)
            continue;
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
            double smax = 0;
            for (const Sample *s : ss)
                smax = std::max(smax, seg->direction * along(*s, axis));
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
            a.mass_ok = std::isfinite(a.mass_total) && a.mass_r2 >= 0.8 && a.added_mass > -0.25 * rigid &&
                        a.added_mass < 3 * rigid;
            if (!a.mass_ok)
                a.note += std::string(a.note.empty() ? "" : "; ") + "added mass rejected (R2 < 0.8 or implausible)";
            a.added_mass = std::max(0., a.added_mass);
        } else {
            a.note += std::string(a.note.empty() ? "" : "; ") + "added mass: no complete accel/decel windows";
        }
        result.axes.push_back(a);
    }
    return result;
}

YAML::Node identifiedModel(const YAML::Node &prior_in, const Result &r, const std::string &provenance) {
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
    m["parameter_status"] = "pool_identified";
    m["provenance"] = provenance;
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
    return out;
}

//================================= IO ===================================//

void writeRecording(const std::string &dir, const std::vector<Sample> &samples, const std::vector<Segment> &segments) {
    std::ofstream seg(dir + "/segments.csv");
    seg << "id,kind,axis,speed,direction,label\n";
    for (const auto &s : segments)
        seg << s.id << ',' << (s.kind == Segment::Kind::Hold ? "hold" : "run") << ',' << s.axis << ',' << s.speed
            << ',' << s.direction << ',' << s.label << '\n';
    std::ofstream out(dir + "/recording.csv");
    out.precision(10);
    out << "segment,t,dt,vx,vy,vz,wx,wy,wz,upx,upy,upz,jx,jy,jz,jroll,jpitch,jyaw\n";
    for (const auto &s : samples) {
        out << s.segment << ',' << s.t << ',' << s.dt;
        for (const Vector3d *v : {&s.v, &s.w, &s.up})
            out << ',' << v->x() << ',' << v->y() << ',' << v->z();
        for (int i = 0; i < 6; ++i)
            out << ',' << s.impulse[i];
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
        s.kind = c.at(1) == "hold" ? Segment::Kind::Hold : Segment::Kind::Run;
        s.axis = std::stoi(c.at(2));
        s.speed = std::stod(c.at(3));
        s.direction = std::stoi(c.at(4));
        s.label = c.size() > 5 ? c[5] : "";
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
        samples.push_back(s);
    }
}
} // namespace riptide_mpc::ident
