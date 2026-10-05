#include "riptide_mpc/path_plan.hpp"
#include "riptide_mpc/fossen_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace riptide_mpc {
namespace {
using Eigen::Vector2d;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr int kTable = 256;          // arc-length table intervals per curved piece
constexpr double kSampleStep = 0.02; // m; speed-limit samples
constexpr double kYawStep = 0.01;    // m; heading-mode yaw samples
constexpr double kMinBlendAngle = 0.035, kMaxBlendAngle = 2.6; // rad; joins rounded off between these
constexpr double kMaxSamples = 2e5;

double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

Quaterniond yawRotation(double yaw) {
    return Quaterniond(Eigen::AngleAxisd(yaw, Vector3d::UnitZ()));
}

// q = yawRotation(twist) * tilt, tilt about a horizontal axis.
double twist(const Quaterniond &q) {
    return 2 * std::atan2(q.z(), q.w());
}

Quaterniond tilt(const Quaterniond &q) {
    return (yawRotation(-twist(q)) * q).normalized();
}

double catmullRom(const std::vector<double> &y, double u) {
    const int n = static_cast<int>(y.size()) - 1;
    if (n < 1)
        return y.empty() ? 0. : y[0];
    const double x = std::clamp(u, 0., 1.) * n;
    const int i = std::min(static_cast<int>(x), n - 1);
    const double f = x - i;
    const double p0 = y[std::max(i - 1, 0)], p1 = y[i], p2 = y[i + 1], p3 = y[std::min(i + 2, n)];
    return p1 + 0.5 * f * (p2 - p0 + f * (2 * p0 - 5 * p1 + 4 * p2 - p3 + f * (3 * (p1 - p2) + p3 - p0)));
}
} // namespace

void PathPlan::Piece::derivatives(double t, Vector3d &p, Vector3d &d1, Vector3d &d2) const {
    switch (kind) {
    case LINE:
        p = a + t * (b - a);
        d1 = b - a;
        d2.setZero();
        return;
    case ARC: {
        const double phi = phi0 + sweep * t, r = r0 + (r1 - r0) * t, dr = r1 - r0, cs = std::cos(phi),
                     sn = std::sin(phi);
        p = Vector3d(cx + r * cs, cy + r * sn, z0 + (z1 - z0) * t);
        d1 = Vector3d(dr * cs - r * sweep * sn, dr * sn + r * sweep * cs, z1 - z0);
        d2 = Vector3d(-2 * dr * sweep * sn - r * sweep * sweep * cs, 2 * dr * sweep * cs - r * sweep * sweep * sn, 0);
        return;
    }
    case BEZIER:
        p = (1 - t) * (1 - t) * a + 2 * t * (1 - t) * b + t * t * c;
        d1 = 2 * (1 - t) * (b - a) + 2 * t * (c - b);
        d2 = 2 * (a - 2 * b + c);
        return;
    case TURN:
        p = a;
        d1.setZero();
        d2.setZero();
        return;
    }
}

Vector3d PathPlan::Piece::at(double t) const {
    Vector3d p, d1, d2;
    derivatives(t, p, d1, d2);
    return p;
}

void PathPlan::Piece::finalize() {
    table.clear();
    if (kind == TURN)
        return; // length is set from the rotation
    Vector3d p, d1, d2;
    const bool uniform = kind == LINE || (kind == ARC && r0 == r1);
    if (uniform) {
        derivatives(t0, p, d1, d2);
        length = d1.norm() * (t1 - t0);
        return;
    }
    table.resize(kTable + 1);
    table[0] = 0;
    const double dt = (t1 - t0) / kTable;
    derivatives(t0, p, d1, d2);
    double previous = d1.norm();
    for (int k = 1; k <= kTable; ++k) { // Simpson on each interval
        Vector3d dm;
        derivatives(t0 + (k - 0.5) * dt, p, dm, d2);
        derivatives(t0 + k * dt, p, d1, d2);
        const double next = d1.norm();
        table[k] = table[k - 1] + dt * (previous + 4 * dm.norm() + next) / 6;
        previous = next;
    }
    length = table.back();
}

double PathPlan::Piece::tAt(double local) const {
    if (length <= 0)
        return t0;
    local = std::clamp(local, 0., length);
    if (table.empty())
        return t0 + (t1 - t0) * local / length;
    const auto it = std::upper_bound(table.begin(), table.end(), local);
    const int k = std::clamp(static_cast<int>(it - table.begin()) - 1, 0, kTable - 1);
    const double span = table[k + 1] - table[k];
    const double f = span > 0 ? (local - table[k]) / span : 0;
    return t0 + (t1 - t0) * (k + f) / kTable;
}

std::shared_ptr<const PathPlan> PathPlan::build(const Waypoint &start, const std::vector<PathPoint> &points,
                                                const MotionLimits &m, const PathOptions &o) {
    if (points.empty())
        throw std::invalid_argument("path has no points");
    if (!start.position.allFinite() || !start.orientation.coeffs().allFinite())
        throw std::invalid_argument("nonfinite path start");
    auto plan = std::make_shared<PathPlan>();
    const double rho = m.linear_speed / m.angular_speed; // metres of s per radian turned in place

    // One raw piece per point.
    std::vector<Piece> raw(points.size());
    Vector3d from = start.position;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const PathPoint &q = points[i];
        if (!q.position.allFinite() || !q.orientation.coeffs().allFinite() || !q.center.allFinite() ||
            !q.look_at.allFinite() || !std::isfinite(q.sweep) || !std::isfinite(q.yaw_offset) || !std::isfinite(q.spin) ||
            !std::isfinite(q.spin_rate))
            throw std::invalid_argument("nonfinite path point " + std::to_string(i));
        Piece &p = raw[i];
        p.segment = i;
        const Vector2d c = q.center.head<2>();
        const double r0 = (from.head<2>() - c).norm(), r1 = (q.position.head<2>() - c).norm();
        if (q.shape == PathShape::ARC && std::abs(q.sweep) > 1e-9 && std::max(r0, r1) > 1e-6) {
            p.kind = Piece::ARC;
            p.cx = c.x();
            p.cy = c.y();
            p.sweep = q.sweep;
            p.r0 = r0;
            p.r1 = r1;
            p.phi0 = r0 > 1e-6 ? std::atan2(from.y() - c.y(), from.x() - c.x())
                               : std::atan2(q.position.y() - c.y(), q.position.x() - c.x()) - q.sweep;
            p.z0 = from.z();
            p.z1 = q.position.z();
        } else if ((q.position - from).norm() > 1e-6) {
            p.kind = Piece::LINE;
            p.a = from;
            p.b = q.position;
        } else {
            p.kind = Piece::TURN;
            p.a = from;
        }
        p.finalize();
        from = p.at(1); // an arc ends where its sweep takes it
    }

    // Round off sharp joins between moving pieces with a quadratic blend from
    // `d` before the corner to `d` after it (split in half: one per segment).
    std::vector<double> original(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i)
        original[i] = raw[i].length;
    std::vector<std::vector<Piece>> blends(raw.size());
    for (std::size_t i = 0; i + 1 < raw.size(); ++i) {
        Piece &A = raw[i], &B = raw[i + 1];
        if (A.kind == Piece::TURN || B.kind == Piece::TURN)
            continue;
        Vector3d p, ta, tb, d2;
        A.derivatives(A.t1, p, ta, d2);
        B.derivatives(B.t0, p, tb, d2);
        const double angle = std::acos(std::clamp(ta.normalized().dot(tb.normalized()), -1., 1.));
        const double d = std::min({o.corner_radius, 0.45 * original[i], 0.45 * original[i + 1]});
        if (angle < kMinBlendAngle || angle > kMaxBlendAngle || d < 0.01)
            continue;
        const double ta_trim = A.tAt(A.length - d), tb_trim = B.tAt(d);
        const Vector3d P0 = A.at(ta_trim), J = A.at(A.t1), P2 = B.at(tb_trim), mid = (P0 + 2 * J + P2) / 4;
        A.t1 = ta_trim;
        A.finalize();
        B.t0 = tb_trim;
        B.finalize();
        Piece h1, h2;
        h1.kind = h2.kind = Piece::BEZIER;
        h1.a = P0, h1.b = (P0 + J) / 2, h1.c = mid, h1.segment = i;
        h2.a = mid, h2.b = (J + P2) / 2, h2.c = P2, h2.segment = i + 1;
        h1.finalize();
        h2.finalize();
        blends[i] = {h1, h2};
    }
    for (std::size_t i = 0; i < raw.size(); ++i) {
        if (i > 0 && !blends[i - 1].empty())
            plan->pieces_.push_back(blends[i - 1][1]);
        plan->pieces_.push_back(raw[i]);
        if (!blends[i].empty())
            plan->pieces_.push_back(blends[i][0]);
    }

    // Attitude per segment, in order: each starts where the previous ended.
    double s = 0, yaw_before = twist(start.orientation);
    Quaterniond tilt_before = tilt(start.orientation);
    std::size_t k = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        const PathPoint &q = points[i];
        Segment g;
        g.first = k;
        while (k < plan->pieces_.size() && plan->pieces_[k].segment == i)
            ++k;
        g.last = k;
        g.start_yaw = yaw_before;
        g.spin = q.spin;
        if (q.heading == PathHeading::LOOK_AT && q.look_target >= 0) {
            g.look_target = q.look_target;
            g.look_at = q.look_at;
            plan->follows_targets_ = true;
        }
        g.tilt0 = tilt_before;
        g.tilt1 = tilt(q.orientation);
        const bool in_place = g.last - g.first == 1 && plan->pieces_[g.first].kind == Piece::TURN;
        const Vector3d here = plan->pieces_[g.first].kind == Piece::TURN ? plan->pieces_[g.first].a : Vector3d::Zero();
        if (q.heading == PathHeading::WAYPOINT) {
            g.turn = wrap(twist(q.orientation) - yaw_before);
        } else if (in_place) { // nothing to face along; looking at a point still turns to it
            const Vector2d to = q.look_at.head<2>() - here.head<2>();
            g.turn = q.heading == PathHeading::LOOK_AT && to.norm() > 0.05
                         ? wrap(std::atan2(to.y(), to.x()) + q.yaw_offset - yaw_before)
                         : 0.;
        } else {
            g.linear = false;
        }
        g.s0 = s;
        if (in_place) {
            Piece &p = plan->pieces_[g.first];
            p.length = rho * (std::abs(g.turn) + std::abs(g.spin) + g.tilt0.angularDistance(g.tilt1));
            p.s0 = s;
            s += p.length;
        } else {
            for (std::size_t j = g.first; j < g.last; ++j) {
                plan->pieces_[j].s0 = s;
                s += plan->pieces_[j].length;
            }
        }
        g.s1 = s;
        if (!g.linear) {
            const double len = g.s1 - g.s0;
            const int n = std::max(8, static_cast<int>(std::ceil(len / kYawStep)));
            if (n > kMaxSamples)
                throw std::invalid_argument("path too long");
            g.mode_yaw.resize(n + 1);
            double held = yaw_before - q.yaw_offset;
            for (int j = 0; j <= n; ++j) {
                Vector3d p, t, curvature;
                plan->geometry(g.s0 + len * j / n, g.first, g.last, p, t, curvature);
                const Vector2d to = q.heading == PathHeading::PATH ? Vector2d(t.head<2>())
                                                                   : Vector2d(q.look_at.head<2>() - p.head<2>());
                const bool valid = q.heading == PathHeading::PATH ? to.norm() > 0.3 : to.norm() > 0.05;
                const double raw_yaw = valid ? std::atan2(to.y(), to.x()) : held;
                held = raw_yaw;
                const double y = raw_yaw + q.yaw_offset;
                g.mode_yaw[j] = j == 0 ? y : g.mode_yaw[j - 1] + wrap(y - g.mode_yaw[j - 1]);
            }
            const double shift = yaw_before + wrap(g.mode_yaw[0] - yaw_before) - g.mode_yaw[0];
            for (double &y : g.mode_yaw)
                y += shift;
            g.delta = yaw_before - g.mode_yaw[0];
            g.blend = std::clamp(1.5 * std::abs(g.delta) * rho, 0.02, std::max(len, 0.02));
        }
        plan->segments_.push_back(g);
        yaw_before = plan->yaw(plan->segments_.back(), g.s1);
        tilt_before = g.tilt1;
    }
    plan->length_ = s;
    if (s <= 0)
        throw std::invalid_argument("path does not move or turn");
    // Steady spin, scaled to a whole number of turns (at least one) over the path.
    double total = 0;
    for (std::size_t i = 0; i < points.size(); ++i)
        total += points[i].spin_rate * (plan->segments_[i].s1 - plan->segments_[i].s0);
    if (std::abs(total) > 1e-9) {
        const double turns = std::copysign(std::max(1., std::round(std::abs(total) / (2 * M_PI))), total);
        const double scale = 2 * M_PI * turns / total;
        double phase = 0;
        for (std::size_t i = 0; i < points.size(); ++i) {
            Segment &g = plan->segments_[i];
            g.phase0 = phase;
            g.spin_rate = points[i].spin_rate * scale;
            phase += g.spin_rate * (g.s1 - g.s0);
        }
    }

    // Speed limits sampled along s (both ends of every piece included, so a
    // join appears twice: once for each side).
    auto &samples = plan->samples_;
    std::vector<std::size_t> first_sample(plan->pieces_.size());
    for (std::size_t j = 0; j < plan->pieces_.size(); ++j) {
        const Piece &p = plan->pieces_[j];
        first_sample[j] = samples.size();
        const int n = std::max(1, static_cast<int>(std::ceil(p.length / kSampleStep)));
        if (samples.size() + n > kMaxSamples)
            throw std::invalid_argument("path too long");
        for (int i = 0; i <= n; ++i) {
            Sample x;
            x.s = p.s0 + p.length * i / n;
            Vector3d t, curvature;
            plan->geometry(x.s, j, j + 1, x.position, t, curvature);
            if (p.kind == Piece::TURN) {
                x.linear_cap = kInf;
                x.accel = m.linear_accel;
                x.jerk = m.linear_jerk;
            } else {
                const AxisLimits l = linearLimitsAlong(m, t);
                x.linear_cap = l.speed;
                x.accel = l.accel;
                x.jerk = l.jerk;
            }
            const double h = 0.005, sp = std::min(s, x.s + h), sm = std::max(0., x.s - h);
            const double density = plan->orientation(sp).angularDistance(plan->orientation(sm)) / (sp - sm);
            x.angular_cap = density > 1e-6 ? m.angular_speed / density : kInf;
            const double kappa = curvature.norm();
            x.envelope = std::min({x.linear_cap, x.angular_cap, kappa > 1e-9 ? std::sqrt(o.lateral_accel / kappa) : kInf});
            samples.push_back(x);
        }
    }
    // Joins: a velocity jump of 2 v sin(turn/2) is held to the kink speed.
    for (std::size_t j = 0; j + 1 < plan->pieces_.size(); ++j) {
        const Piece &A = plan->pieces_[j], &B = plan->pieces_[j + 1];
        double cap = kInf;
        if ((A.kind == Piece::TURN) != (B.kind == Piece::TURN)) {
            cap = o.kink_speed;
        } else if (A.kind != Piece::TURN) {
            Vector3d p, ta, tb, d2;
            A.derivatives(A.t1, p, ta, d2);
            B.derivatives(B.t0, p, tb, d2);
            const double half = std::acos(std::clamp(ta.normalized().dot(tb.normalized()), -1., 1.)) / 2;
            if (std::sin(half) > 1e-9)
                cap = o.kink_speed / (2 * std::sin(half));
        }
        for (std::size_t i : {first_sample[j + 1] - 1, first_sample[j + 1]})
            samples[i].envelope = std::min(samples[i].envelope, cap);
    }
    // Brake for what is ahead, at half the acceleration limit (room for the jerk ramps).
    for (std::size_t j = samples.size() - 1; j-- > 0;)
        samples[j].envelope = std::min(samples[j].envelope,
                                       std::sqrt(samples[j + 1].envelope * samples[j + 1].envelope +
                                                 samples[j].accel * (samples[j + 1].s - samples[j].s)));
    return plan;
}

std::size_t PathPlan::pieceAt(double s, std::size_t begin, std::size_t end) const {
    // Last piece in [begin, end) starting at or before s.
    std::size_t lo = begin, hi = end;
    while (hi - lo > 1) {
        const std::size_t mid = (lo + hi) / 2;
        (pieces_[mid].s0 <= s ? lo : hi) = mid;
    }
    return lo;
}

void PathPlan::geometry(double s, std::size_t begin, std::size_t end, Vector3d &p, Vector3d &t, Vector3d &k) const {
    const Piece &piece = pieces_[pieceAt(s, begin, end)];
    Vector3d d1, d2;
    piece.derivatives(piece.tAt(s - piece.s0), p, d1, d2);
    const double speed = d1.norm();
    if (piece.kind == Piece::TURN || speed < 1e-12) {
        t.setZero();
        k.setZero();
        return;
    }
    t = d1 / speed;
    k = (d2 - d2.dot(t) * t) / (speed * speed);
}

double PathPlan::yaw(const Segment &g, double s) const {
    const double len = g.s1 - g.s0, u = len > 1e-12 ? std::clamp((s - g.s0) / len, 0., 1.) : 1.;
    if (g.linear)
        return g.start_yaw + (g.turn + g.spin) * u;
    const double x = std::clamp((s - g.s0) / g.blend, 0., 1.);
    return catmullRom(g.mode_yaw, u) + g.delta * (1 - x * x * (3 - 2 * x)) + g.spin * u;
}

std::size_t PathPlan::segmentAt(double s) const {
    for (std::size_t i = 0; i < segments_.size(); ++i)
        if (s < segments_[i].s1)
            return i;
    return segments_.size() - 1;
}

double PathPlan::lookYaw(double s, const std::vector<Vector3d> &targets) const {
    s = std::clamp(s, 0., length_);
    const Segment &g = segments_[segmentAt(s)];
    if (g.look_target < 0 || static_cast<std::size_t>(g.look_target) >= targets.size() ||
        !targets[g.look_target].allFinite())
        return 0;
    const Vector3d p = position(s);
    const Vector2d live = targets[g.look_target].head<2>() - p.head<2>(), planned = g.look_at.head<2>() - p.head<2>();
    if (live.norm() < 0.05 || planned.norm() < 0.05) // on top of it: no direction to face
        return 0;
    return wrap(std::atan2(live.y(), live.x()) - std::atan2(planned.y(), planned.x()));
}

Vector3d PathPlan::position(double s) const {
    Vector3d p, t, k;
    geometry(std::clamp(s, 0., length_), 0, pieces_.size(), p, t, k);
    return p;
}

Vector3d PathPlan::tangent(double s) const {
    Vector3d p, t, k;
    geometry(std::clamp(s, 0., length_), 0, pieces_.size(), p, t, k);
    return t;
}

Quaterniond PathPlan::orientation(double s) const {
    s = std::clamp(s, 0., length_);
    const Segment &g = segments_[segmentAt(s)];
    const double len = g.s1 - g.s0, u = len > 1e-12 ? std::clamp((s - g.s0) / len, 0., 1.) : 1.;
    const double spin = g.phase0 + g.spin_rate * (s - g.s0);
    return (yawRotation(yaw(g, s) + spin) * g.tilt0.slerp(u, g.tilt1)).normalized();
}

std::size_t PathPlan::sampleAt(double s) const {
    const auto it = std::upper_bound(samples_.begin(), samples_.end(), s,
                                     [](double value, const Sample &x) { return value < x.s; });
    return it == samples_.begin() ? 0 : static_cast<std::size_t>(it - samples_.begin()) - 1;
}

void PathPlan::step(PathProgress &p, double linear_scale, double angular_scale, double dt) const {
    using namespace scalar;
    const int steps = static_cast<int>(std::ceil(dt / kSubstep - 1e-9));
    const double h = dt / std::max(steps, 1);
    for (int i = 0; i < steps; ++i) {
        const double remaining = length_ - p.s;
        if (remaining < 2e-4 && std::abs(p.v) < 2e-3 && std::abs(p.a) < 2e-2) {
            p = {length_, 0, 0};
            return;
        }
        const std::size_t here = sampleAt(p.s);
        const AxisLimits l{samples_[here].linear_cap, samples_[here].accel, samples_[here].jerk};
        double j;
        bool braking = false;
        if (remaining < 2e-3 && std::abs(p.v) < 2e-2 && std::abs(p.a) < 1e-1) { // capture onto the end
            const double c = kCapturePole;
            j = std::clamp(c * c * c * remaining - 3 * c * c * p.v - 3 * c * p.a, -l.jerk, l.jerk);
        } else {
            // Cruise at what the path allows a little ahead (the accel ramp's worth).
            const double look = p.s + std::max(p.v, 0.) * (l.accel / l.jerk + h);
            double target = 1e3;
            for (std::size_t k = here; k < samples_.size(); ++k) {
                const Sample &x = samples_[k];
                target = std::min({target, x.envelope, linear_scale * x.linear_cap, angular_scale * x.angular_cap});
                if (x.s > look)
                    break;
            }
            j = velocityJerk(p.v, p.a, target, l, h);
            double vc = p.v, ac = p.a;
            const double dc = advance(vc, ac, j, h);
            if (vc > 0 && dc + stoppingDistance(vc, std::clamp(ac, -l.accel, l.accel), l) > remaining) {
                j = brakingJerk(p.v, p.a, l, h);
                braking = true;
            }
        }
        double d = advance(p.v, p.a, j, h);
        if (p.v < 0 && (braking || remaining > 2e-3)) { // never back up along the path
            p.v = p.a = 0;
            d = std::max(d, 0.);
        }
        p.s = std::clamp(p.s + d, 0., length_);
    }
}

void PathPlan::pose(const PathProgress &p, MotionProfile &out) const {
    const double s = std::clamp(p.s, 0., length_);
    Vector3d t, k;
    geometry(s, 0, pieces_.size(), out.position, t, k);
    out.velocity = t * p.v;
    out.acceleration = t * p.a + k * p.v * p.v;
    out.orientation = orientation(s);
    // Attitude rate per metre of s (world frame), and its change, by differences.
    const auto rate = [this](double at) {
        const double h = 0.005, sp = std::min(length_, at + h), sm = std::max(0., at - h);
        return sp > sm ? Vector3d(quaternionLog(orientation(sp) * orientation(sm).conjugate()) / (sp - sm))
                       : Vector3d::Zero();
    };
    const Vector3d w = rate(s);
    const double h = 0.02, sp = std::min(length_, s + h), sm = std::max(0., s - h);
    const Vector3d w_dot = sp > sm ? Vector3d((rate(sp) - rate(sm)) / (sp - sm)) : Vector3d::Zero();
    out.angular_velocity = w * p.v;
    out.angular_acceleration = w_dot * p.v * p.v + w * p.a;
}

double PathPlan::project(const Vector3d &p, double near, double window) const {
    double best = near, best_distance = kInf;
    for (std::size_t k = sampleAt(near - window); k < samples_.size() && samples_[k].s <= near + window; ++k) {
        const double distance = (samples_[k].position - p).norm();
        if (distance < best_distance) {
            best_distance = distance;
            best = samples_[k].s;
        }
    }
    return best;
}
} // namespace riptide_mpc
