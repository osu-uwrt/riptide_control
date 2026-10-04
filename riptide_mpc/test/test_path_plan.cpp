#include "riptide_mpc/fossen_model.hpp"
#include "riptide_mpc/path_plan.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace riptide_mpc;

namespace {
MotionLimits limits() { // as in config/mpc.yaml
    MotionLimits m;
    m.linear_speed = 0.5;
    m.linear_accel = 0.3;
    m.linear_jerk = 2.0;
    m.angular_speed = 1.0;
    m.angular_accel = 1.2;
    m.angular_jerk = 6.0;
    return m;
}

double yawOf(const Quaterniond &q) {
    const Vector3d x = q * Vector3d::UnitX();
    return std::atan2(x.y(), x.x());
}

double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}

Quaterniond yaw(double a) {
    return quaternionExp(Vector3d(0, 0, a));
}

PathPoint line(const Vector3d &to, double heading_yaw = 0) {
    PathPoint p;
    p.position = to;
    p.orientation = yaw(heading_yaw);
    return p;
}

PathPoint arc(const Vector3d &center, double sweep, const Vector3d &end, PathHeading heading = PathHeading::WAYPOINT) {
    PathPoint p;
    p.shape = PathShape::ARC;
    p.center = center;
    p.sweep = sweep;
    p.position = end;
    p.heading = heading;
    p.look_at = center;
    return p;
}

std::shared_ptr<const PathPlan> plan(const Vector3d &from, double from_yaw, const std::vector<PathPoint> &points,
                                     const PathOptions &options = {}) {
    return PathPlan::build({from, yaw(from_yaw)}, points, limits(), options);
}

// Flies the reference along a plan and records what it did.
struct Flight {
    double time = 0, max_speed = 0, max_lateral = 0, max_rate = 0;
    bool ended = false;
};

Flight fly(const PathPlan &p, double timeout = 200) {
    Flight f;
    PathProgress progress;
    MotionProfile pose;
    const double dt = 0.01;
    for (; f.time < timeout && !p.atEnd(progress); f.time += dt) {
        p.step(progress, 1, 1, dt);
        p.pose(progress, pose);
        const double speed = pose.velocity.norm();
        f.max_speed = std::max(f.max_speed, speed);
        if (speed > 1e-6) {
            const Vector3d t = pose.velocity / speed;
            f.max_lateral = std::max(f.max_lateral, (pose.acceleration - pose.acceleration.dot(t) * t).norm());
        }
        f.max_rate = std::max(f.max_rate, pose.angular_velocity.norm());
    }
    f.ended = p.atEnd(progress);
    return f;
}
} // namespace

TEST(PathPlan, ArcIsACircleOfTheRightLength) {
    const auto p = plan(Vector3d(3, 0, -1), 0, {arc(Vector3d::Zero(), M_PI / 2, Vector3d(0, 3, -1))});
    EXPECT_NEAR(p->length(), 3 * M_PI / 2, 1e-3);
    for (double s = 0; s <= p->length(); s += 0.05) {
        EXPECT_NEAR(p->position(s).head<2>().norm(), 3, 1e-6);
        EXPECT_NEAR(p->position(s).z(), -1, 1e-9);
    }
    EXPECT_LT((p->end().position - Vector3d(0, 3, -1)).norm(), 1e-9);
    EXPECT_LT((p->tangent(0) - Vector3d(0, 1, 0)).norm(), 1e-6); // counterclockwise
}

// The end point sets radius and depth; the sweep sets where round the arc ends.
TEST(PathPlan, ArcSweepMakesSpiralsAndHelices) {
    const double sweep = 2.5 * M_PI, end_radius = std::hypot(-4., 0.5);
    const auto p = plan(Vector3d(2, 0, -1), 0, {arc(Vector3d::Zero(), sweep, Vector3d(-4, 0.5, -2))});
    EXPECT_LT((p->end().position - Vector3d(0, end_radius, -2)).norm(), 1e-9);
    double last_radius = 0, last_z = 0;
    for (double s = 0; s <= p->length(); s += 0.1) {
        const Vector3d x = p->position(s);
        EXPECT_GE(x.head<2>().norm(), last_radius - 1e-9);
        if (s > 0) {
            EXPECT_LE(x.z(), last_z + 1e-9);
        }
        last_radius = x.head<2>().norm();
        last_z = x.z();
    }
    const auto cw = plan(Vector3d(2, 0, -1), 0, {arc(Vector3d::Zero(), -M_PI / 2, Vector3d(0, -2, -1))});
    EXPECT_LT((cw->end().position - Vector3d(0, -2, -1)).norm(), 1e-9);
    EXPECT_LT((cw->tangent(0) - Vector3d(0, -1, 0)).norm(), 1e-6); // clockwise
}

TEST(PathPlan, SharpCornersAreRoundedSmoothly) {
    PathOptions o;
    const Vector3d corner(2, 0, 0);
    const auto p = plan(Vector3d::Zero(), 0, {line(corner), line(Vector3d(2, 2, 0))}, o);
    EXPECT_LT(p->length(), 4);
    double closest = 1e9, worst_turn = 0;
    Vector3d last_tangent = p->tangent(0);
    for (double s = 0; s <= p->length(); s += 0.001) {
        const Vector3d x = p->position(s);
        closest = std::min(closest, (x - corner).norm());
        EXPECT_LT(std::min(std::abs(x.y()), std::abs(x.x() - 2)), o.corner_radius); // near the polyline
        const Vector3d t = p->tangent(s);
        worst_turn = std::max(worst_turn, std::acos(std::clamp(t.dot(last_tangent), -1., 1.)));
        last_tangent = t;
    }
    std::printf("corner cut by %.3f m, worst tangent change %.4f rad per mm\n", closest, worst_turn);
    EXPECT_GT(closest, 0.05);
    EXPECT_LT(worst_turn, 0.02);
}

TEST(PathPlan, ProfileKeepsLateralAccelerationAndStopsOnTheEnd) {
    PathOptions o;
    const auto p = plan(Vector3d(0.5, 0, 0), 0, {arc(Vector3d::Zero(), 2 * M_PI, Vector3d(0.5, 0, 0))}, o);
    const Flight f = fly(*p);
    std::printf("circle r=0.5: %.1f s, peak speed %.3f (sqrt(a/k) %.3f), peak lateral %.3f m/s^2\n", f.time,
                f.max_speed, std::sqrt(o.lateral_accel * 0.5), f.max_lateral);
    EXPECT_TRUE(f.ended);
    EXPECT_LT(f.max_lateral, o.lateral_accel * 1.1);
    EXPECT_GT(f.max_speed, 0.9 * std::sqrt(o.lateral_accel * 0.5));
    PathProgress end{p->length(), 0, 0};
    MotionProfile pose;
    p->pose(end, pose);
    EXPECT_LT((pose.position - Vector3d(0.5, 0, 0)).norm(), 1e-9);
}

TEST(PathPlan, StraightPathCruisesAtTheSpeedLimit) {
    const auto p = plan(Vector3d::Zero(), 0, {line(Vector3d(6, 0, 0))});
    const Flight f = fly(*p);
    EXPECT_TRUE(f.ended);
    EXPECT_NEAR(f.max_speed, limits().linear_speed, 1e-3);
}

TEST(PathPlan, LookAtFacesTheCenterAroundAnArc) {
    // Starts facing away from the center, so it first has to turn round.
    const auto p = plan(Vector3d(3, 0, 0), 0,
                        {arc(Vector3d::Zero(), M_PI, Vector3d(-3, 0, 0), PathHeading::LOOK_AT)});
    double worst = 0;
    for (double s = 0; s <= p->length(); s += 0.01) {
        const Vector3d to = -p->position(s);
        const double error = std::abs(wrap(yawOf(p->orientation(s)) - std::atan2(to.y(), to.x())));
        if (s > 1.5 * M_PI * limits().linear_speed / limits().angular_speed + 1e-6) // past the turn round
            worst = std::max(worst, error);
    }
    const Flight f = fly(*p);
    std::printf("look-at heading error after aligning %.2e rad, peak turn rate %.2f rad/s\n", worst, f.max_rate);
    EXPECT_LT(worst, 1e-3);
    EXPECT_LT(f.max_rate, limits().angular_speed * 1.1);
    EXPECT_NEAR(wrap(yawOf(p->end().orientation)), 0, 1e-6); // at (-3, 0) facing +x
}

TEST(PathPlan, PathHeadingFacesTheDirectionOfTravel) {
    PathPoint forward = arc(Vector3d::Zero(), M_PI, Vector3d(-3, 0, 0), PathHeading::PATH);
    PathPoint sideways = forward;
    sideways.yaw_offset = M_PI / 2;
    for (const PathPoint &point : {forward, sideways}) {
        const auto p = plan(Vector3d(3, 0, 0), M_PI / 2, {point}); // already facing along the arc
        double worst = 0;
        for (double s = 0; s <= p->length(); s += 0.01) {
            const Vector3d t = p->tangent(s);
            worst = std::max(worst, std::abs(wrap(yawOf(p->orientation(s)) - std::atan2(t.y(), t.x()) -
                                                  point.yaw_offset)));
        }
        if (point.yaw_offset == 0)
            EXPECT_LT(worst, 1e-3);
        else // turns to sideways first
            EXPECT_LT(std::abs(wrap(yawOf(p->end().orientation) - (-M_PI / 2 + M_PI / 2))), 1e-3);
    }
}

TEST(PathPlan, SpinAddsWholeTurns) {
    PathPoint p = line(Vector3d(4, 0, 0));
    p.spin = 2 * M_PI;
    const auto path = plan(Vector3d::Zero(), 0, {p});
    double turned = 0, last = 0;
    for (double s = 0; s <= path->length() + 1e-9; s += 0.01) {
        const double y = yawOf(path->orientation(s));
        turned += wrap(y - last);
        last = y;
    }
    const Flight f = fly(*path);
    std::printf("spun %.4f rad over 4 m, peak rate %.2f rad/s\n", turned, f.max_rate);
    EXPECT_NEAR(turned, 2 * M_PI, 1e-3);
    EXPECT_LT(path->end().orientation.angularDistance(yaw(0)), 1e-9);
    EXPECT_LT(f.max_rate, limits().angular_speed * 1.1);
}

TEST(PathPlan, TurningInPlaceIsProgress) {
    const double rho = limits().linear_speed / limits().angular_speed;
    const auto p = plan(Vector3d::Zero(), 0, {line(Vector3d::Zero(), M_PI / 2)});
    EXPECT_NEAR(p->length(), rho * M_PI / 2, 1e-9);
    PathPoint look = line(Vector3d::Zero());
    look.heading = PathHeading::LOOK_AT;
    look.look_at = Vector3d(0, 5, 0);
    const auto q = plan(Vector3d::Zero(), 0, {look});
    EXPECT_NEAR(yawOf(q->end().orientation), M_PI / 2, 1e-9);
    EXPECT_TRUE(fly(*q).ended);
}

TEST(PathPlan, ReversalSlowsToTheKinkSpeed) {
    PathOptions o;
    const auto p = plan(Vector3d::Zero(), 0, {line(Vector3d(2, 0, 0)), line(Vector3d::Zero())}, o);
    PathProgress progress;
    MotionProfile pose;
    double at_turn = 1e9;
    for (double t = 0; t < 60 && !p->atEnd(progress); t += 0.01) {
        p->step(progress, 1, 1, 0.01);
        p->pose(progress, pose);
        if (std::abs(pose.position.x() - 2) < 0.01)
            at_turn = std::min(at_turn, pose.velocity.norm());
    }
    EXPECT_TRUE(p->atEnd(progress));
    EXPECT_LT(at_turn, 2 * o.kink_speed);
}

// A steady spin carries on through corners at one rate per metre, rounded to whole
// turns so the path still ends facing where it would without the spin.
TEST(PathPlan, SteadySpinIsConstantAcrossSegments) {
    PathPoint a = line(Vector3d(4, 0, 0)), b = line(Vector3d(4, 3, 0));
    a.spin_rate = b.spin_rate = 1.5;
    const auto p = plan(Vector3d::Zero(), 0, {a, b});
    const double turns = std::round(1.5 * p->length() / (2 * M_PI)), rate = 2 * M_PI * turns / p->length();
    const double h = 0.05;
    double worst = 0;
    for (double s = 0; s + h <= p->length(); s += h)
        worst = std::max(worst, std::abs(wrap(yawOf(p->orientation(s + h)) - yawOf(p->orientation(s))) / h - rate));
    const Flight f = fly(*p);
    std::printf("steady spin %.3f rad/m (%.0f turns over %.2f m), worst deviation %.2e rad/m, peak rate %.2f rad/s\n",
                rate, turns, p->length(), worst, f.max_rate);
    EXPECT_LT(worst, 1e-6);
    EXPECT_LT(p->end().orientation.angularDistance(yaw(0)), 1e-9);
    EXPECT_LT(f.max_rate, limits().angular_speed * 1.1);
}
