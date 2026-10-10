#include "riptide_mpc/box_qp.hpp"
#include "riptide_mpc/mpc.hpp"
#include "riptide_mpc/settle_trust.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace riptide_mpc;

namespace {
FossenModel talos() {
    return FossenModel::load(TALOS_VEHICLE, TALOS_HYDRO);
}

struct ClosedLoop {
    FossenModel plant_model = talos();
    ThrusterDynamics plant_actuator = plant_model.makeActuator();
    MpcController controller;
    State13d x;
    double max_solve_ms = 0, max_speed = 0, max_rate = 0, max_tilt = 0;

    explicit ClosedLoop(const MpcSettings &s = MpcSettings()) : controller(talos(), s) {
        x = plant_model.fromBaseLink(Vector3d(0, 0, -2), Quaterniond::Identity(), Vector3d::Zero(), Vector3d::Zero());
    }
    // Plant is stepped exactly like physics_simulator (2 ms, split actuator/body).
    void run(const Reference &r, double seconds) {
        const double plant_step = 0.002, period = controller.settings().dt;
        const int per_tick = static_cast<int>(std::lround(period / plant_step));
        const int ticks = static_cast<int>(std::lround(seconds / period));
        for (int t = 0; t < ticks; ++t) {
            const MpcOutput out = controller.compute(x, r);
            max_solve_ms = std::max(max_solve_ms, out.solve_ms);
            plant_actuator.command(out.command);
            controller.issue(out.command);
            for (int i = 0; i < per_tick; ++i) {
                plant_model.step(x, plant_actuator, plant_step);
                max_speed = std::max(max_speed, plant_model.baseLinkVelocity(x).norm());
                max_rate = std::max(max_rate, x.tail<3>().norm());
                max_tilt = std::max(max_tilt, std::acos(std::clamp((orientation() * Vector3d::UnitZ()).z(), -1., 1.)));
            }
            controller.advance(period);
        }
    }
    Vector3d position() const {
        return plant_model.baseLinkPosition(x);
    }
    Quaterniond orientation() const {
        return Quaterniond(x[3], x[4], x[5], x[6]).normalized();
    }
};
} // namespace

TEST(BoxQp, MatchesKktConditions) {
    const int n = 12;
    Eigen::MatrixXd R = Eigen::MatrixXd::Random(n, n);
    const Eigen::MatrixXd H = R.transpose() * R + 0.1 * Eigen::MatrixXd::Identity(n, n);
    const Eigen::VectorXd g = 5 * Eigen::VectorXd::Random(n);
    const Eigen::VectorXd lb = -Eigen::VectorXd::Ones(n), ub = Eigen::VectorXd::Ones(n);
    Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
    ASSERT_TRUE(solveBoxQp(H, g, lb, ub, x).converged);
    const Eigen::VectorXd grad = H * x + g;
    for (int i = 0; i < n; ++i) {
        if (x[i] > lb[i] + 1e-9 && x[i] < ub[i] - 1e-9)
            EXPECT_NEAR(grad[i], 0, 1e-6);
        else if (x[i] <= lb[i] + 1e-9)
            EXPECT_GE(grad[i], -1e-6);
        else
            EXPECT_LE(grad[i], 1e-6);
    }
}

TEST(FossenModel, LoadsSimulatorConfiguration) {
    const FossenModel m = talos();
    EXPECT_EQ(m.thrusterCount(), 8);
    EXPECT_NEAR(m.mass(), YAML::LoadFile(TALOS_VEHICLE)["mass"].as<double>(), 1e-12);
    Eigen::FullPivLU<MatrixXd> lu(m.thrusterMatrix());
    EXPECT_EQ(lu.rank(), 6);
    // Round trip between odometry (base_link) and simulator (COM) state.
    const Quaterniond q = quaternionExp(Vector3d(0.1, -0.2, 0.3));
    const State13d x = m.fromBaseLink(Vector3d(1, 2, -3), q, Vector3d(0.1, 0.2, 0.3), Vector3d(0.3, -0.2, 0.1));
    EXPECT_TRUE((m.baseLinkPosition(x) - Vector3d(1, 2, -3)).norm() < 1e-12);
    EXPECT_TRUE((m.baseLinkVelocity(x) - Vector3d(0.1, 0.2, 0.3)).norm() < 1e-12);
}

// A model file with its own mass and com flies that body: weight, rigid mass, and every position taken from
// the vehicle config's poses (thrusters, base_link) re-centered on that COM.
TEST(FossenModel, ModelFileBodyReplacesTheVehicleConfigs) {
    const YAML::Node vehicle = YAML::LoadFile(TALOS_VEHICLE);
    YAML::Node hydro = YAML::Clone(YAML::LoadFile(TALOS_HYDRO));
    hydro.remove("schema_version"); // optional
    const FossenModel nominal = FossenModel::fromNodes(vehicle, hydro);
    const Vector3d shift(0.01, -0.02, 0.005), com = nominal.com() + shift;
    hydro["mass"] = nominal.mass() + 1.5;
    hydro["com"] = std::vector<double>{com.x(), com.y(), com.z()};
    const FossenModel m = FossenModel::fromNodes(vehicle, hydro);
    EXPECT_DOUBLE_EQ(m.mass(), nominal.mass() + 1.5);
    EXPECT_NEAR(m.dynamics().mass()(0, 0) - nominal.dynamics().mass()(0, 0), 1.5, 1e-12);
    EXPECT_LT((m.com() - com).norm(), 1e-15);
    EXPECT_LT((m.baseLinkOffset() - (nominal.baseLinkOffset() - shift)).norm(), 1e-12);
    for (int i = 0; i < m.thrusterCount(); ++i) {
        const Vector3d f = nominal.thrusterMatrix().col(i).head<3>();
        EXPECT_LT((m.thrusterMatrix().col(i).head<3>() - f).norm(), 1e-12);
        const Vector3d torque = nominal.thrusterMatrix().col(i).tail<3>() - shift.cross(f);
        EXPECT_LT((m.thrusterMatrix().col(i).tail<3>() - torque).norm(), 1e-12) << "thruster " << i;
    }
    // At rest, level and submerged, the net heave force is buoyancy minus the model's weight.
    const auto restingHeaveForce = [](const FossenModel &model) {
        const State13d rest =
            model.fromBaseLink(Vector3d(0, 0, -2), Quaterniond::Identity(), Vector3d::Zero(), Vector3d::Zero());
        const Vector6d nu_dot = model.derivative(rest, VectorXd::Zero(model.thrusterCount())).segment<6>(7);
        return (model.dynamics().mass() * nu_dot)[2];
    };
    EXPECT_NEAR(restingHeaveForce(m) - restingHeaveForce(nominal), -1.5 * 9.80665, 1e-9);

    YAML::Node half = YAML::Clone(hydro);
    half.remove("com");
    EXPECT_THROW(FossenModel::fromNodes(vehicle, half), std::invalid_argument);
}

TEST(FossenModel, ReadsTheRigidInertiaAsRowsOrFlat) {
    const YAML::Node vehicle = YAML::LoadFile(TALOS_VEHICLE);
    YAML::Node hydro = YAML::Clone(YAML::LoadFile(TALOS_HYDRO));
    const Eigen::Matrix3d I = matrix3(hydro["rigid_body_inertia3x3"], "rigid_body_inertia3x3");
    YAML::Node rows, flat_list;
    for (int r = 0; r < 3; ++r) {
        rows.push_back(std::vector<double>{I(r, 0), I(r, 1), I(r, 2)});
        for (int c = 0; c < 3; ++c)
            flat_list.push_back(I(r, c));
    }
    hydro["rigid_body_inertia3x3"] = rows;
    const FossenModel from_rows = FossenModel::fromNodes(vehicle, hydro);
    hydro["rigid_body_inertia3x3"] = flat_list;
    const FossenModel from_flat = FossenModel::fromNodes(vehicle, hydro);
    EXPECT_EQ(from_rows.dynamics().mass(), talos().dynamics().mass());
    EXPECT_EQ(from_flat.dynamics().mass(), talos().dynamics().mass());
}

TEST(FossenModel, OwnActuatorStepMatchesSimulatorActuator) {
    const FossenModel m = talos();
    auto actuator = m.makeActuator();
    VectorXd command(8);
    command << 10, -5, 20, -28, 3, 0, 7, -12;
    // A command issued `delay` ago is exactly what the horizon stages assume.
    actuator.command(command);
    State13d a = m.fromBaseLink(Vector3d(0, 0, -2), Quaterniond::Identity(), Vector3d::Zero(), Vector3d::Zero());
    for (int i = 0; i < 50; ++i) // the 0.1 s delay
        m.step(a, actuator, 0.002);
    State13d b = a;
    VectorXd force = actuator.forces();
    const VectorXd target = m.commandToTarget(command);
    for (int i = 0; i < 150; ++i) { // stay inside the 0.5 s command watchdog
        m.step(a, actuator, 0.002);
        m.step(b, force, target, 0.002);
    }
    EXPECT_LT((a - b).norm(), 1e-9);
    EXPECT_LT((actuator.forces() - force).norm(), 1e-9);
}

TEST(Mpc, PositionStepConvergesOnExactPlant) {
    ClosedLoop loop;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = Vector3d(1.0, -0.5, -2.5);
    r.orientation = quaternionExp(Vector3d(0, 0, M_PI / 4));
    loop.run(r, 10.0);
    const double position_error = (loop.position() - r.position).norm();
    const double angle_error = quaternionLog(r.orientation.conjugate() * loop.orientation()).norm();
    std::printf("position error %.2e m, attitude error %.2e rad, max solve %.1f ms\n", position_error, angle_error,
                loop.max_solve_ms);
    EXPECT_LT(position_error, 1e-3);
    EXPECT_LT(angle_error, 1e-3);
    EXPECT_LT(loop.x.tail<6>().norm(), 1e-3);
}

TEST(Mpc, MovesBetweenSetpointsWithinMotionLimits) {
    auto transit = [](bool profile) {
        MpcSettings s;
        s.profile_motion = profile;
        ClosedLoop loop(s);
        Reference r;
        r.linear_mode = r.angular_mode = Mode::POSITION;
        r.position = loop.position();
        loop.run(r, 3.0); // settle first: the cold start (thrust delay) is not the move
        loop.max_speed = loop.max_rate = loop.max_tilt = 0;
        r.position += Vector3d(1.0, 0.5, -0.5);
        r.orientation = quaternionExp(Vector3d(0, 0, M_PI / 4));
        loop.run(r, 12.0);
        std::printf("profile %-3s: peak speed %.2f m/s, peak rate %.2f rad/s, peak tilt %.1f deg, final error %.1e m\n",
                    profile ? "on" : "off", loop.max_speed, loop.max_rate, loop.max_tilt * 180 / M_PI,
                    (loop.position() - r.position).norm());
        return std::make_pair(loop, r);
    };
    transit(false); // reference: the unprofiled lunge
    const auto [loop, r] = transit(true);
    const MotionLimits limits;
    EXPECT_LT(loop.max_speed, 1.1 * limits.linear_speed);
    EXPECT_LT(loop.max_rate, 1.2 * limits.angular_speed);
    EXPECT_LT(loop.max_tilt, 2.0 * M_PI / 180);
    EXPECT_LT((loop.position() - r.position).norm(), 1e-3);
    EXPECT_LT(quaternionLog(r.orientation.conjugate() * loop.orientation()).norm(), 1e-3);
}

// Moves along each axis and diagonally; reports how far the vehicle trails the
// reference in transit and how far it passes the setpoint after the profile stops.
TEST(Mpc, TracksProfileWithoutOvershoot) {
    const Vector3d moves[] = {Vector3d(2, 0, 0), Vector3d(0, 2, 0), Vector3d(0, 0, -1.2), Vector3d(1.2, -1.2, 0.6)};
    double worst_lag = 0, worst_overshoot = 0;
    for (const Vector3d &move : moves) {
        ClosedLoop loop;
        Reference r;
        r.linear_mode = r.angular_mode = Mode::POSITION;
        r.position = loop.position();
        loop.run(r, 2.0);
        r.position += move;
        const Vector3d direction = move.normalized();
        double lag = 0, overshoot = 0;
        // The profile leads by the actuator delay; compare with its value from then.
        const int lead = static_cast<int>(std::lround(loop.controller.referenceLead() / loop.controller.settings().dt));
        std::vector<Vector3d> history;
        for (int k = 0; k < 200; ++k) {
            history.push_back(loop.controller.profile().position);
            loop.run(r, loop.controller.settings().dt);
            if (lead > 0 && static_cast<int>(history.size()) >= lead) // reference for this instant
                lag = std::max(lag, (loop.position() - history[history.size() - lead]).norm());
            overshoot = std::max(overshoot, (loop.position() - r.position).dot(direction));
        }
        std::printf("move [%+.1f %+.1f %+.1f]: max lag %.1f mm, overshoot %.2f mm, final %.1e m\n", move.x(),
                    move.y(), move.z(), lag * 1e3, overshoot * 1e3, (loop.position() - r.position).norm());
        worst_lag = std::max(worst_lag, lag);
        worst_overshoot = std::max(worst_overshoot, overshoot);
        EXPECT_LT((loop.position() - r.position).norm(), 1e-3);
    }
    EXPECT_LT(worst_lag, 0.005);
    EXPECT_LT(worst_overshoot, 0.001);
}

TEST(SettleTrust, ResetsOnNewSetpointAndSettlesSoonAfterArrival) {
    ClosedLoop loop;
    SettleTrust trust;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = loop.position();
    auto step = [&] {
        loop.run(r, loop.controller.settings().dt);
        Vector6d twist;
        twist << loop.plant_model.baseLinkVelocity(loop.x), loop.x.tail<3>();
        return trust.update(loop.controller.settings().dt, r, loop.controller.profile(), loop.position(),
                            loop.orientation(), twist);
    };
    for (int k = 0; k < 60; ++k)
        step();
    EXPECT_GT(trust.trust(), 0.99);
    r.position += Vector3d(1.5, 0, -0.5);
    r.orientation = quaternionExp(Vector3d(0, 0, 0.6));
    EXPECT_EQ(step(), 0.);
    double arrived = -1, t = 0;
    while (t < 20 && trust.trust() < 0.95) {
        step();
        t += loop.controller.settings().dt;
        if (arrived < 0 && (loop.controller.profile().position - r.position).norm() < 1e-6 &&
            loop.controller.profile().orientation.angularDistance(r.orientation) < 1e-6)
            arrived = t;
        if (arrived < 0) {
            EXPECT_EQ(trust.trust(), 0.) << "trust must not grow before the profile arrives";
        }
    }
    std::printf("profile arrived at %.2f s, trust > 0.95 at %.2f s\n", arrived, t);
    EXPECT_GT(arrived, 0);
    EXPECT_LT(t - arrived, 1.5);
}

TEST(MotionProfile, JerkLimitedAndStopsExactlyOnTarget) {
    MotionProfile p;
    const MotionLimits m;
    const double dt = 0.05;
    const Quaterniond q_target(Eigen::AngleAxisd(2.5, Vector3d(0.2, 0.1, 1).normalized()));
    Vector3d target(2.0, -1.0, 0.5);
    double peak_speed = 0, peak_accel = 0, peak_jerk = 0, peak_rate = 0, peak_ang_accel = 0, peak_ang_jerk = 0;
    for (int i = 0; i < 400; ++i) {
        if (i == 40) // retarget mid-move, sideways
            target = Vector3d(0.5, 1.5, -0.5);
        const Vector3d a0 = p.acceleration, alpha0 = p.angular_acceleration;
        p.stepLinear(target, m, dt);
        p.stepAngular(q_target, m, dt);
        peak_speed = std::max(peak_speed, p.velocity.norm());
        peak_accel = std::max(peak_accel, p.acceleration.norm());
        peak_jerk = std::max(peak_jerk, (p.acceleration - a0).norm() / dt);
        peak_rate = std::max(peak_rate, p.angular_velocity.norm());
        peak_ang_accel = std::max(peak_ang_accel, p.angular_acceleration.norm());
        peak_ang_jerk = std::max(peak_ang_jerk, (p.angular_acceleration - alpha0).norm() / dt);
    }
    std::printf("profile peaks: %.3f m/s, %.3f m/s^2, %.2f m/s^3; %.3f rad/s, %.3f rad/s^2, %.2f rad/s^3\n",
                peak_speed, peak_accel, peak_jerk, peak_rate, peak_ang_accel, peak_ang_jerk);
    EXPECT_EQ((p.position - target).norm(), 0);
    EXPECT_LT(p.orientation.angularDistance(q_target), 1e-12);
    EXPECT_EQ(p.velocity.norm() + p.acceleration.norm(), 0);
    // Per-axis limits; sideways braking during the retarget can add to the along-line terms.
    EXPECT_LE(peak_speed, std::sqrt(2) * m.linear_speed * 1.001);
    EXPECT_LE(peak_accel, std::sqrt(2) * m.linear_accel + 1e-9);
    EXPECT_LE(peak_jerk, std::sqrt(3) * m.linear_jerk * 1.01);
    EXPECT_LE(peak_rate, m.angular_speed * 1.001); // 2 ms decision step
    EXPECT_LE(peak_ang_accel, m.angular_accel + 1e-9);
    EXPECT_LE(peak_ang_jerk, std::sqrt(3) * m.angular_jerk * 1.01);
}

TEST(MotionProfile, SettlesFromSmallAwkwardStates) {
    // Seeds like the ones a cold start produces: millimetres off, drifting the wrong way.
    const MotionLimits m;
    for (double sign : {1., -1.}) {
        MotionProfile p;
        p.position = Vector3d(1.5e-3, 0, 0);
        p.velocity = Vector3d(sign * 1.2e-2, 3e-3, 0);
        p.acceleration = Vector3d(0.1, 0, 0);
        p.orientation = Quaterniond(Eigen::AngleAxisd(0.03, Vector3d::UnitY()));
        p.angular_velocity = Vector3d(0, sign * 0.25, 0);
        double t = 0;
        while (t < 5 && (p.position.norm() > 0 || p.orientation.angularDistance(Quaterniond::Identity()) > 0)) {
            p.stepLinear(Vector3d::Zero(), m, 0.05);
            p.stepAngular(Quaterniond::Identity(), m, 0.05);
            t += 0.05;
        }
        std::printf("awkward seed (%+.0f): at rest after %.2f s\n", sign, t);
        EXPECT_LT(t, 2.5);
        EXPECT_EQ(p.velocity.norm() + p.angular_velocity.norm(), 0);
    }
}

TEST(MotionProfile, StraightMoveTakesNearMinimumTime) {
    // Rest-to-rest over 3 m with a cruise phase: S-curve time is d/v + v/a + a/j.
    MotionProfile p;
    const MotionLimits m;
    const Vector3d target(3, 0, 0);
    double t = 0, peak_accel = 0, peak_speed = 0;
    while (t < 20 && (p.position != target || p.velocity.norm() > 0)) {
        p.stepLinear(target, m, 0.002);
        t += 0.002;
        peak_accel = std::max(peak_accel, p.acceleration.norm());
        peak_speed = std::max(peak_speed, p.velocity.norm());
    }
    const double ideal = 3 / m.linear_speed + m.linear_speed / m.linear_accel + m.linear_accel / m.linear_jerk;
    std::printf("3 m move: %.3f s (ideal %.3f s), peak %.3f m/s, %.3f m/s^2\n", t, ideal, peak_speed, peak_accel);
    EXPECT_LT(t, ideal + 0.15);
    EXPECT_NEAR(peak_speed, m.linear_speed, 1e-3);
    EXPECT_NEAR(peak_accel, m.linear_accel, 1e-3);
}

TEST(Mpc, HoldsLevelWithNoSteadyStateError) {
    ClosedLoop loop;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = loop.position();
    loop.run(r, 5.0);
    std::printf("hold drift %.2e m\n", (loop.position() - r.position).norm());
    EXPECT_LT((loop.position() - r.position).norm(), 1e-4);
    EXPECT_LT(quaternionLog(loop.orientation()).norm(), 1e-4);
}

// thruster_sweep swaps the thruster model mid-hold: the replica restarts settled on
// the last command, so the hold barely moves; an invalid model changes nothing.
TEST(Mpc, ThrusterModelSwapMidHoldDoesNotKick) {
    ClosedLoop loop;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = loop.position();
    loop.run(r, 3.0);
    // Same model: the restart alone must be invisible.
    std::vector<ThrusterParameters> p = loop.controller.model().actuatorParameters();
    loop.controller.setActuatorParameters(p);
    EXPECT_LT((loop.controller.actuatorEstimate().forces() - loop.plant_actuator.forces()).norm(), 1e-3);
    loop.max_rate = 0;
    loop.run(r, 1.0);
    EXPECT_LT(loop.max_rate, 1e-3);
    // A modest mismatch (slower actuators than the plant) still holds.
    for (auto &a : p)
        a.delay = 0.15;
    loop.controller.setActuatorParameters(p);
    EXPECT_DOUBLE_EQ(loop.controller.model().actuatorParameters().front().delay, 0.15);
    EXPECT_NEAR(loop.controller.actuatorEstimate().forces().norm(), loop.plant_actuator.forces().norm(), 0.5);
    loop.max_rate = 0;
    loop.run(r, 3.0);
    std::printf("after swap: drift %.2e m, max rate %.2e rad/s\n", (loop.position() - r.position).norm(),
                loop.max_rate);
    EXPECT_LT((loop.position() - r.position).norm(), 0.01);
    EXPECT_LT(loop.max_rate, 0.02);

    auto bad = p;
    bad.front().efficiency = 1.5;
    EXPECT_THROW(loop.controller.setActuatorParameters(bad), std::invalid_argument);
    EXPECT_DOUBLE_EQ(loop.controller.model().actuatorParameters().front().efficiency, p.front().efficiency);
}

TEST(Mpc, TracksBodyVelocity) {
    ClosedLoop loop;
    Reference r;
    r.linear_mode = Mode::VELOCITY;
    r.angular_mode = Mode::POSITION;
    r.linear_velocity = Vector3d(0.4, 0, 0);
    loop.run(r, 8.0);
    const Vector3d v = loop.plant_model.baseLinkVelocity(loop.x);
    std::printf("velocity %.4f %.4f %.4f\n", v.x(), v.y(), v.z());
    EXPECT_LT((v - r.linear_velocity).norm(), 5e-3);
}

namespace {
double yawOf(const Quaterniond &q) {
    const Vector3d x = q * Vector3d::UnitX();
    return std::atan2(x.y(), x.x());
}
double wrap(double a) {
    return std::atan2(std::sin(a), std::cos(a));
}
PathPoint point(const Vector3d &position, const Quaterniond &orientation = Quaterniond::Identity()) {
    PathPoint p;
    p.position = position;
    p.orientation = orientation;
    return p;
}
// Starts reference r on a path from where its profile is now.
void startPath(Reference &r, const MpcController &c, const std::vector<PathPoint> &points,
               const PathOptions &options = {}) {
    r.path = PathPlan::build({c.profile().position, c.profile().orientation}, points, c.settings().motion, options);
    r.position = r.path->end().position;
    r.orientation = r.path->end().orientation;
}
} // namespace

// A path of quarter turns in place spins through every one of them (the
// waypoints all share a position) at a steady rate, without pausing at each.
TEST(Mpc, PathOfQuarterTurnsSpinsInPlace) {
    ClosedLoop loop;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = loop.position();
    loop.run(r, 3.0);
    const int turns = 2;
    std::vector<PathPoint> points;
    for (int k = 1; k <= 4 * turns; ++k)
        points.push_back(point(r.position, quaternionExp(Vector3d(0, 0, k * M_PI / 2))));
    startPath(r, loop.controller, points);
    double travelled = 0, last = yawOf(loop.orientation()), slowest_mid = 1e9;
    const double dt = loop.controller.settings().dt;
    const MotionLimits limits;
    for (int k = 0; k < static_cast<int>(40 / dt); ++k) {
        loop.run(r, dt);
        const double yaw = yawOf(loop.orientation());
        travelled += wrap(yaw - last);
        last = yaw;
        if (travelled > 1.0 && travelled < 4 * turns * M_PI / 2 - 1.0) // cruising, away from start and stop
            slowest_mid = std::min(slowest_mid, std::abs(loop.x[12]));
    }
    std::printf("spun %.2f rad of %.2f, slowest mid-spin rate %.2f rad/s, drift %.1f mm\n", travelled,
                turns * 2 * M_PI, slowest_mid, (loop.position() - r.position).norm() * 1e3);
    EXPECT_NEAR(travelled, turns * 2 * M_PI, 0.02);
    EXPECT_GT(slowest_mid, 0.8 * limits.angular_speed);
    EXPECT_LT((loop.position() - r.position).norm(), 0.01);
    EXPECT_TRUE(r.path->atEnd(loop.controller.pathProgress()));
}

// A heading change on a long leg is spread over the move rather than done
// first at full rate.
TEST(Mpc, PathSpreadsTurnOverTheLeg) {
    ClosedLoop loop;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = loop.position();
    loop.run(r, 3.0);
    const Vector3d start = r.position, goal = start + Vector3d(4, 0, 0);
    startPath(r, loop.controller, {point(goal, quaternionExp(Vector3d(0, 0, 2.5)))});
    const double dt = loop.controller.settings().dt;
    double yaw_at_quarter = 0, yaw_at_half = 0;
    for (int k = 0; k < static_cast<int>(20 / dt); ++k) {
        loop.run(r, dt);
        const double covered = (loop.position() - start).x() / 4;
        if (!yaw_at_quarter && covered > 0.25)
            yaw_at_quarter = yawOf(loop.orientation());
        if (!yaw_at_half && covered > 0.5)
            yaw_at_half = yawOf(loop.orientation());
    }
    std::printf("yaw at 1/4 of the leg %.2f, at 1/2 %.2f, final %.2f (goal 2.5)\n", yaw_at_quarter, yaw_at_half,
                yawOf(loop.orientation()));
    EXPECT_NEAR(yaw_at_quarter, 2.5 * 0.25, 0.4);
    EXPECT_NEAR(yaw_at_half, 2.5 * 0.5, 0.4);
    EXPECT_LT((loop.position() - goal).norm(), 1e-3);
    EXPECT_NEAR(yawOf(loop.orientation()), 2.5, 1e-3);
}

// A path whose last leg turns sharply (a 5.6 m run, then 0.5 m straight up):
// the reference must come to rest on the goal instead of circling it.
TEST(Mpc, PathSettlesAfterSharpFinalCorner) {
    ClosedLoop loop;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = loop.position();
    r.orientation = quaternionExp(Vector3d(0, 0, M_PI));
    loop.run(r, 6.0);
    const Vector3d a = r.position + Vector3d(0.4, 5.6, 0.07), b = a + Vector3d(0, 0, 0.5);
    startPath(r, loop.controller, {point(a, r.orientation), point(b, r.orientation)});
    const double dt = loop.controller.settings().dt;
    double rested = -1;
    for (double t = 0; t < 30; t += dt) {
        loop.run(r, dt);
        const MotionProfile &p = loop.controller.profile();
        if (rested < 0 && (p.position - b).norm() < 1e-4 && p.velocity.norm() < 1e-3)
            rested = t;
    }
    std::printf("reference at rest on the goal %.1f s into the path; vehicle %.1f mm off\n", rested,
                (loop.position() - b).norm() * 1e3);
    EXPECT_GT(rested, 0);
    EXPECT_LT(rested, 18);
    EXPECT_LT((loop.controller.profile().position - b).norm(), 1e-6);
    EXPECT_LT((loop.position() - b).norm(), 0.01);
}

// Half an orbit about a point 1.5 m ahead while looking at it (the pole in
// prequal): the vehicle tracks the arc and keeps facing the point.
TEST(Mpc, OrbitsAPointLookingAtIt) {
    ClosedLoop loop;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = loop.position();
    loop.run(r, 3.0);
    const Vector3d center = r.position + Vector3d(1.5, 0, 0);
    PathPoint orbit = point(center + Vector3d(1.5, 0, 0));
    orbit.shape = PathShape::ARC;
    orbit.center = center;
    orbit.sweep = M_PI;
    orbit.heading = PathHeading::LOOK_AT;
    orbit.look_at = center;
    startPath(r, loop.controller, {orbit});
    const double dt = loop.controller.settings().dt;
    double worst_position = 0, worst_heading = 0, t = 0;
    for (; t < 40 && !r.path->atEnd(loop.controller.pathProgress()); t += dt) {
        loop.run(r, dt);
        worst_position = std::max(worst_position, std::abs((loop.position() - center).head<2>().norm() - 1.5));
        const Vector3d to = center - loop.position();
        worst_heading = std::max(worst_heading, std::abs(wrap(yawOf(loop.orientation()) - std::atan2(to.y(), to.x()))));
    }
    loop.run(r, 3.0);
    std::printf("orbit: %.1f s, worst radius error %.1f mm, worst heading error %.2f deg, end %.1f mm off\n", t,
                worst_position * 1e3, worst_heading * 180 / M_PI, (loop.position() - r.position).norm() * 1e3);
    EXPECT_LT(worst_position, 0.03);
    EXPECT_LT(worst_heading, 0.05);
    EXPECT_LT((loop.position() - r.position).norm(), 0.01);
}

// The same orbit while the point drifts sideways (a pole re-estimated by mapping):
// the vehicle keeps facing where it is now, and still faces it after the path ends.
TEST(Mpc, OrbitKeepsFacingAMovingLookTarget) {
    ClosedLoop loop;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = loop.position();
    loop.run(r, 3.0);
    const Vector3d center = r.position + Vector3d(1.5, 0, 0), drift(0.02, 0.04, 0); // m/s
    PathPoint orbit = point(center + Vector3d(1.5, 0, 0));
    orbit.shape = PathShape::ARC;
    orbit.center = center;
    orbit.sweep = M_PI;
    orbit.heading = PathHeading::LOOK_AT;
    orbit.look_at = center;
    orbit.look_target = 0;
    startPath(r, loop.controller, {orbit});
    r.look_targets = {center};
    const double dt = loop.controller.settings().dt;
    double worst_heading = 0, t = 0;
    for (; t < 40 && !r.path->atEnd(loop.controller.pathProgress()); t += dt) {
        r.look_targets[0] = center + drift * t;
        loop.run(r, dt);
        const Vector3d to = r.look_targets[0] - loop.position();
        worst_heading = std::max(worst_heading, std::abs(wrap(yawOf(loop.orientation()) - std::atan2(to.y(), to.x()))));
    }
    loop.run(r, 3.0);
    const Vector3d to = r.look_targets[0] - loop.position();
    const double end_heading = std::abs(wrap(yawOf(loop.orientation()) - std::atan2(to.y(), to.x())));
    std::printf("orbit of a target drifting %.0f mm/s: %.1f s, worst heading error %.2f deg, at the end %.2f deg\n",
                drift.norm() * 1e3, t, worst_heading * 180 / M_PI, end_heading * 180 / M_PI);
    EXPECT_LT((r.look_targets[0] - center).norm(), 0.5); // it really moved
    EXPECT_GT((r.look_targets[0] - center).norm(), 0.2);
    EXPECT_LT(worst_heading, 0.06);
    EXPECT_LT(end_heading, 0.02);
    // The planned end faces the original point; the profile faces the moved one.
    EXPECT_GT(loop.controller.profile().orientation.angularDistance(r.orientation), 0.05);
    EXPECT_LT(loop.controller.profile().orientation.angularDistance(loop.controller.lookOffset() * r.orientation), 1e-6);
}

// Identification: one thruster's command is given (pool_identify ramps it) and the MPC holds the pose with
// the other seven; then every thruster fixed at zero (a release) turns them all off.
TEST(Mpc, HoldsWithOneThrusterCommandedThenReleases) {
    ClosedLoop loop;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = loop.position();
    loop.run(r, 3.0);
    VectorXd fixed = VectorXd::Constant(8, std::numeric_limits<double>::quiet_NaN());
    fixed[3] = 6.0;
    loop.controller.setIdentificationInputs(fixed, VectorXd());
    double worst = 0;
    for (int k = 0; k < 120; ++k) {
        loop.run(r, 0.05);
        worst = std::max(worst, (loop.position() - r.position).norm());
    }
    std::printf("thruster 3 held at %.2f N; worst position error %.1f mm\n", loop.controller.lastCommand()[3],
                worst * 1e3);
    EXPECT_NEAR(loop.controller.lastCommand()[3], 6.0, 1e-9);
    EXPECT_LT(worst, 0.02);
    loop.controller.setIdentificationInputs(VectorXd::Zero(8), VectorXd());
    loop.run(r, 0.1);
    EXPECT_EQ(loop.controller.lastCommand().cwiseAbs().maxCoeff(), 0.0);
    loop.controller.setIdentificationInputs(VectorXd(), VectorXd()); // back to normal control
    loop.run(r, 0.1);
    EXPECT_GT(loop.controller.lastCommand().cwiseAbs().maxCoeff(), 0.0);
}

// A zero-wrench bias is flown on top of the hold at no cost: the commands carry it, the pose stays.
TEST(Mpc, NullSpaceBiasDoesNotMoveTheVehicle) {
    ClosedLoop loop;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = loop.position();
    loop.run(r, 3.0);
    const VectorXd before = loop.controller.lastCommand();
    const MatrixXd T = loop.controller.model().thrusterMatrix();
    Eigen::JacobiSVD<MatrixXd> svd(T, Eigen::ComputeFullV);
    VectorXd bias = svd.matrixV().col(7);
    bias *= 4.0 / bias.cwiseAbs().maxCoeff();
    loop.controller.setIdentificationInputs(VectorXd::Constant(8, std::numeric_limits<double>::quiet_NaN()), bias);
    double worst = 0;
    for (int k = 0; k < 100; ++k) {
        loop.run(r, 0.05);
        worst = std::max(worst, (loop.position() - r.position).norm());
    }
    const double carried = (loop.controller.lastCommand() - before).dot(bias) / bias.squaredNorm();
    std::printf("bias carried %.2f of the pattern; worst position error %.1f mm\n", carried, worst * 1e3);
    EXPECT_GT(carried, 0.8);
    EXPECT_LT(worst, 0.005);
}

// Separate vertical limits: flat moves cruise at the horizontal limit, straight
// climbs at the vertical one, and a 45 degree climb at the ellipse between.
TEST(MotionProfile, VerticalLimitsSlowClimbsNotFlatMoves) {
    MotionLimits m;
    m.linear_speed = 0.8;
    m.linear_accel = 0.7;
    m.linear_speed_vertical = 0.3;
    m.linear_accel_vertical = 0.3;
    const auto peak = [&m](const Vector3d &target) {
        MotionProfile p;
        double t = 0, speed = 0;
        while (t < 60 && (p.position != target || p.velocity.norm() > 0)) {
            p.stepLinear(target, m, 0.002);
            t += 0.002;
            speed = std::max(speed, p.velocity.norm());
        }
        EXPECT_EQ(p.position, target);
        return speed;
    };
    const double flat = peak(Vector3d(4, 0, 0)), climb = peak(Vector3d(0, 0, 3)),
                 diagonal = peak(Vector3d(3, 0, -3)), expected = 1 / std::hypot(M_SQRT1_2 / 0.8, M_SQRT1_2 / 0.3);
    std::printf("peak speed: flat %.3f, climb %.3f, 45 deg dive %.3f (ellipse %.3f) m/s\n", flat, climb, diagonal,
                expected);
    EXPECT_NEAR(flat, 0.8, 1e-3);
    EXPECT_NEAR(climb, 0.3, 1e-3);
    EXPECT_NEAR(diagonal, expected, 1e-3);
}
