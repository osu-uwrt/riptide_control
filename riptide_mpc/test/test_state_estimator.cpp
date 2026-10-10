#include "riptide_mpc/mpc.hpp"
#include "riptide_mpc/sim_rig.hpp"
#include "riptide_mpc/state_estimator.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <cstdio>
#include <random>

using namespace riptide_mpc;

namespace {
FossenModel talos() {
    return FossenModel::load(TALOS_VEHICLE, TALOS_HYDRO);
}

Quaterniond mountRotation(const char *sensor) {
    const auto p = YAML::LoadFile(TALOS_VEHICLE)[sensor]["pose"].as<std::vector<double>>();
    return rpyToQuaternion(p[3], p[4], p[5]);
}

double angleBetween(const Quaterniond &a, const Quaterniond &b) {
    return a.angularDistance(b);
}

// Plant + sensors synthesized the way physics_simulator does (formulas, rates,
// noise), and an EKF stand-in whose body rates lag truth by a 4 s time constant
// (the talos_ekf.yaml rate problem). Position/orientation of the EKF are truth.
struct Rig {
    FossenModel model = talos();
    ThrusterDynamics plant_actuator = model.makeActuator();
    State13d x;
    double t = 0;
    const Quaterniond q_imu = mountRotation("imu"), q_dvl = mountRotation("dvl");
    Vector3d r_dvl; // DVL relative to COM
    Vector3d ekf_rate = Vector3d::Zero(), ekf_velocity = Vector3d::Zero();
    bool dvl_enabled = true;
    std::mt19937 rng{7};
    std::normal_distribution<double> unit{0., 1.};

    Rig() {
        const YAML::Node v = YAML::LoadFile(TALOS_VEHICLE);
        const auto dvl = v["dvl"]["pose"].as<std::vector<double>>();
        const auto com = v["com"].as<std::vector<double>>();
        r_dvl = Vector3d(dvl[0] - com[0], dvl[1] - com[1], dvl[2] - com[2]);
        x = model.fromBaseLink(Vector3d(0, 0, -2), Quaterniond::Identity(), Vector3d::Zero(), Vector3d::Zero());
    }
    Quaterniond q() const {
        return Quaterniond(x[3], x[4], x[5], x[6]).normalized();
    }
    Vector3d noise(double sigma) {
        return sigma * Vector3d(unit(rng), unit(rng), unit(rng));
    }
    static bool due(int step, double rate_hz) {
        return step % static_cast<int>(std::lround(500. / rate_hz)) == 0;
    }

    // Advances the plant one 2 ms step and delivers the sensors due at that step.
    void step(StateEstimator &est, int k) {
        model.step(x, plant_actuator, 0.002);
        t += 0.002;
        const Vector3d w = x.tail<3>();
        if (due(k, 500)) // FOG, body z
            est.fogRate(t, w.z() + 0.01 * M_PI / 180 * unit(rng));
        if (due(k, 50)) { // IMU: sigma_omega 0.01 deg/s, sigma_angle 0.5 deg
            est.imuRate(t, q_imu.conjugate() * w + noise(0.01 * M_PI / 180));
            const Vector3d axis = noise(1.).normalized();
            const Quaterniond jitter(Eigen::AngleAxisd(0.5 * M_PI / 180 * unit(rng), axis));
            est.imuOrientation(t, jitter * q() * q_imu);
        }
        if (dvl_enabled && due(k, 8)) // sim dvl_noise_stddev 0.001
            est.dvlVelocity(t, q_dvl.conjugate() * (x.segment<3>(7) + w.cross(r_dvl)) + noise(0.001));
        if (due(k, 20)) // depth sigma 0.01
            est.depth(t, model.baseLinkPosition(x).z() + 0.01 * unit(rng));
        const double k_lag = 1 - std::exp(-0.002 / 4.0);
        ekf_rate += k_lag * (w - ekf_rate);
        ekf_velocity = model.baseLinkVelocity(x);
        if (due(k, 30))
            est.odometry(t, model.baseLinkPosition(x), q(), ekf_velocity, ekf_rate);
    }
};

struct Errors {
    double rate = 0, velocity = 0, tilt = 0, position = 0;
    void add(const Rig &rig, const StateEstimator &est) {
        const State13d &e = est.state();
        rate = std::max(rate, (e.tail<3>() - rig.x.tail<3>()).norm());
        velocity = std::max(velocity, (rig.model.baseLinkVelocity(e) - rig.model.baseLinkVelocity(rig.x)).norm());
        tilt = std::max(tilt, angleBetween(Quaterniond(e[3], e[4], e[5], e[6]), rig.q()));
        position = std::max(position, (rig.model.baseLinkPosition(e) - rig.model.baseLinkPosition(rig.x)).norm());
    }
};

// Runs the MPC closed loop on estimator feedback; returns the worst estimate errors.
Errors runClosedLoop(Rig &rig, StateEstimator &est, MpcController &mpc, const Reference &r, double seconds,
                     double measure_after = 0) {
    Errors err;
    const int ticks = static_cast<int>(std::lround(seconds / mpc.settings().dt));
    int k = 0;
    for (int tick = 0; tick < ticks; ++tick) {
        est.propagate(rig.t);
        const MpcOutput out = mpc.compute(est.state(), r);
        rig.plant_actuator.command(out.command);
        mpc.issue(out.command);
        est.command(rig.t, out.command);
        for (int i = 0; i < 25; ++i)
            rig.step(est, ++k);
        mpc.advance(mpc.settings().dt);
        if (rig.t >= measure_after)
            err.add(rig, est);
    }
    return err;
}

Reference poseStep(const Rig &rig) {
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = rig.model.baseLinkPosition(rig.x) + Vector3d(1.0, 0.5, -0.5);
    r.orientation = Quaterniond(Eigen::AngleAxisd(0.8, Vector3d::UnitZ()));
    return r;
}
} // namespace

TEST(SensorMounts, MatchSimulatorConvention) {
    const SensorMounts m = SensorMounts::load(TALOS_VEHICLE, talos().com());
    EXPECT_LT(angleBetween(m.imu, mountRotation("imu")), 1e-12);
    EXPECT_LT(angleBetween(m.dvl, mountRotation("dvl")), 1e-12);
    EXPECT_NEAR((m.fog_axis - Vector3d::UnitZ()).norm(), 0, 1e-12);
    Rig rig;
    EXPECT_NEAR((m.dvl_position - rig.r_dvl).norm(), 0, 1e-12);
}

TEST(StateEstimator, TiltCorrectionKeepsHeading) {
    StateEstimator est(talos(), SensorMounts(), EstimatorSettings());
    State13d x = State13d::Zero();
    const Quaterniond start(Eigen::AngleAxisd(1.0, Vector3d::UnitZ()));
    x.segment<4>(3) << start.w(), start.x(), start.y(), start.z();
    est.reset(x, 0);
    const Quaterniond truth = Eigen::AngleAxisd(1.0, Vector3d::UnitZ()) * Eigen::AngleAxisd(0.2, Vector3d::UnitX());
    for (int i = 1; i <= 500; ++i) { // 10 s of 50 Hz IMU, gravity reference only
        State13d frozen = est.state();
        frozen.tail<6>().setZero(); // isolate the correction from dynamics
        frozen.head<3>().setZero();
        est.reset(frozen, i * 0.02); // no propagation: test the correction alone
        est.imuOrientation(i * 0.02, truth);
    }
    const Quaterniond q(est.state()[3], est.state()[4], est.state()[5], est.state()[6]);
    EXPECT_LT(angleBetween(q, truth), 1e-3);
}

TEST(StateEstimator, TracksTruthDespiteLaggingEkfRates) {
    Rig rig;
    StateEstimator est(talos(), SensorMounts::load(TALOS_VEHICLE, talos().com()), EstimatorSettings());
    MpcController mpc(talos(), MpcSettings());
    rig.step(est, 0); // first EKF message seeds the estimate
    ASSERT_TRUE(est.initialized());
    const Reference r = poseStep(rig);
    const Errors err = runClosedLoop(rig, est, mpc, r, 8.0, 0.5);
    const Vector3d p_err = rig.model.baseLinkPosition(rig.x) - r.position;
    std::printf("max estimate error: rate %.4f rad/s, velocity %.4f m/s, tilt %.3f deg, position %.4f m\n", err.rate,
                err.velocity, err.tilt * 180 / M_PI, err.position);
    std::printf("final pose error %.4f m, %.3f deg; EKF rate error %.3f rad/s\n", p_err.norm(),
                angleBetween(rig.q(), r.orientation) * 180 / M_PI, (rig.ekf_rate - rig.x.tail<3>()).norm());
    EXPECT_LT(err.rate, 0.01);
    EXPECT_LT(err.velocity, 0.01);
    EXPECT_LT(err.tilt, 0.5 * M_PI / 180);
    EXPECT_LT(p_err.norm(), 0.01);
    EXPECT_LT(angleBetween(rig.q(), r.orientation), 0.5 * M_PI / 180);
}

TEST(StateEstimator, FallsBackToEkfVelocityWhenDvlDrops) {
    Rig rig;
    StateEstimator est(talos(), SensorMounts::load(TALOS_VEHICLE, talos().com()), EstimatorSettings());
    MpcController mpc(talos(), MpcSettings());
    rig.step(est, 0);
    rig.dvl_enabled = false; // lost bottom lock for the whole run
    const Reference r = poseStep(rig);
    runClosedLoop(rig, est, mpc, r, 3.0);
    EXPECT_FALSE(est.health(rig.t).dvl);
    const Errors late = runClosedLoop(rig, est, mpc, r, 5.0, rig.t + 1.0);
    std::printf("DVL out: max velocity error %.4f m/s\n", late.velocity);
    EXPECT_LT(late.velocity, 0.02);
    EXPECT_LT((rig.model.baseLinkPosition(rig.x) - r.position).norm(), 0.02);
}

TEST(StateEstimator, ReseedsWhenEkfJumps) {
    StateEstimator est(talos(), SensorMounts(), EstimatorSettings());
    const Quaterniond I = Quaterniond::Identity();
    EXPECT_TRUE(est.odometry(0, Vector3d(0, 0, -1), I, Vector3d::Zero(), Vector3d::Zero()));
    EXPECT_FALSE(est.odometry(0.03, Vector3d(0, 0, -1), I, Vector3d::Zero(), Vector3d::Zero()));
    EXPECT_TRUE(est.odometry(0.06, Vector3d(5, 0, -1), I, Vector3d::Zero(), Vector3d::Zero()));
    EXPECT_NEAR(talos().baseLinkPosition(est.state()).x(), 5, 1e-9);
}

namespace {
// The simulator's plant with pool-plausible errors the MPC's model does not know:
// +2% displaced volume (~6 N lift), COB 1 cm forward (~3 N m pitch), drag x1.3,
// added mass x1.2, thrusters 10% weak.
std::string mismatchedPlant() {
    YAML::Node d = YAML::LoadFile(TALOS_MODEL_SIM);
    d["displaced_volume"] = d["displaced_volume"].as<double>() * 1.02;
    d["cob_relative"][0] = d["cob_relative"][0].as<double>() + 0.01;
    auto scale = [](YAML::Node n, double k) {
        for (std::size_t i = 0; i < n.size(); ++i) {
            if (n[i].IsSequence())
                for (std::size_t j = 0; j < n[i].size(); ++j)
                    n[i][j] = n[i][j].as<double>() * k;
            else
                n[i] = n[i].as<double>() * k;
        }
    };
    scale(d["quadratic_damping"], 1.3);
    scale(d["linear_damping6x6"], 1.3);
    scale(d["added_mass6x6"], 1.2);
    for (std::size_t i = 0; i < d["thruster_efficiencies"].size(); ++i)
        d["thruster_efficiencies"][i] = 0.9;
    const std::string path = testing::TempDir() + "/mismatched_plant.yaml";
    std::ofstream(path) << d;
    return path;
}

double holdError(SimRig &rig, Reference &r, double seconds, double *attitude = nullptr) {
    double err = 0, att = 0;
    int n = 0;
    rig.run(r, seconds, [&](const SimRig &s) {
        if (s.t > seconds - 3) {
            err += (s.position() - r.position).norm();
            att += s.orientation().angularDistance(r.orientation);
            ++n;
        }
    });
    if (attitude)
        *attitude = att / std::max(n, 1);
    return err / std::max(n, 1);
}
} // namespace

TEST(OffsetFree, RemovesSteadyStateErrorUnderModelMismatch) {
    const std::string plant = mismatchedPlant();
    for (bool learn : {false, true}) {
        EstimatorSettings e;
        e.estimate_disturbance = learn;
        SimRig rig(TALOS_VEHICLE, plant, TALOS_MODEL_SIM, MpcSettings(), e);
        rig.feed_disturbance = learn;
        Reference r;
        r.linear_mode = r.angular_mode = Mode::POSITION;
        r.position = rig.position();
        double attitude = 0;
        const double err = holdError(rig, r, 20, &attitude);
        const Vector6d d = rig.estimator.disturbance();
        std::printf("disturbance %-3s: hold error %.1f mm, %.2f deg; learned F [%.2f %.2f %.2f] N, M [%.2f %.2f %.2f] N m\n",
                    learn ? "on" : "off", err * 1e3, attitude * 180 / M_PI, d[0], d[1], d[2], d[3], d[4], d[5]);
        if (learn) {
            EXPECT_LT(err, 0.005);
            EXPECT_LT(attitude, 0.3 * M_PI / 180);
            // COB 1 cm forward of the +2% buoyancy, plus the 10% weak thrusters holding the
            // model's own COB moment.
            const YAML::Node sim = YAML::LoadFile(TALOS_MODEL_SIM);
            const double buoyancy = sim["water_density"].as<double>() * sim["displaced_volume"].as<double>() * 9.80665;
            const double cob_x = sim["cob_relative"][0].as<double>();
            const double pitch = -(1.02 * buoyancy * (cob_x + 0.01) - buoyancy * cob_x) - 0.1 * buoyancy * cob_x;
            EXPECT_NEAR(d[4], pitch, 1.0);
            EXPECT_GT(d[2], 5.0);         // ~6 N of extra lift
        } else {
            EXPECT_GT(attitude, 5 * M_PI / 180); // the model alone cannot know
        }
    }
}

TEST(OffsetFree, LearnsNothingWhenTheModelIsRight) {
    SimRig rig(TALOS_VEHICLE, TALOS_MODEL_SIM, TALOS_MODEL_SIM);
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = rig.position();
    EXPECT_LT(holdError(rig, r, 15), 0.005);
    EXPECT_LT(rig.estimator.disturbance().head<3>().norm(), 0.3);
    EXPECT_LT(rig.estimator.disturbance().tail<3>().norm(), 0.1);
}

namespace {
// The simulator's model with the vehicle's hardware section: complete_controller's force->RPM
// curves and the 73 N power budget (the tests never read the robot's own model file).
std::string simModelWithHardware() {
    YAML::Node d = YAML::LoadFile(TALOS_MODEL_SIM);
    d["hardware"]["total_thrust_limit"] = 73.0;
    d["hardware"]["force_to_rpm_positive"] = std::vector<double>{-475.886186, 32.327429, -324.744664, 1443.832105};
    d["hardware"]["force_to_rpm_negative"] = std::vector<double>{257.460623, 42.948545, -24.402056, -986.338071};
    const std::string path = testing::TempDir() + "/talos_sim_hardware.yaml";
    std::ofstream(path) << d;
    return path;
}
} // namespace

TEST(Hardware, ForceToRpmMatchesCompleteController) {
    const FossenModel model = FossenModel::load(TALOS_VEHICLE, simModelWithHardware());
    const HardwareConfig &hw = model.hardware();
    ASSERT_TRUE(hw.present);
    // complete_controller's transform: c0 + c1 F + c2 tanh F + c3 |F|^0.25, split by sign.
    const double f = 10.0;
    EXPECT_NEAR(hw.forceToRpm(f), -475.886186 + 32.327429 * f - 324.744664 * std::tanh(f) + 1443.832105 * std::pow(f, 0.25),
                1e-9);
    EXPECT_NEAR(hw.forceToRpm(-f), 257.460623 - 42.948545 * f + 24.402056 * std::tanh(f) - 986.338071 * std::pow(f, 0.25),
                1e-9);
    EXPECT_EQ(hw.forceToRpm(0.0), 0.0);
    // Very near zero the fitted curves give the wrong sign (below ~0.012 N forward, ~0.005 N reverse):
    // a stop, not a reversal.
    EXPECT_EQ(hw.forceToRpm(0.005), 0.0);  // curve: about -93 rpm
    EXPECT_EQ(hw.forceToRpm(-0.001), 0.0); // curve: about +82 rpm
    EXPECT_LT(hw.forceToRpm(-0.01), 0.0);  // past the floor the curve applies
    EXPECT_GT(hw.forceToRpm(24.0), hw.forceToRpm(12.0)); // monotonic over the working range
    EXPECT_LT(hw.forceToRpm(-24.0), hw.forceToRpm(-12.0));
}

TEST(Hardware, QuadraticThrustCurveInverts) {
    YAML::Node d = YAML::LoadFile(simModelWithHardware());
    d["hardware"].remove("force_to_rpm_positive");
    d["hardware"].remove("force_to_rpm_negative");
    const std::array<double, 2> fwd{3.5e-6, -1.1e-3}, rev{2.8e-6, 4e-4}; // k1 of both signs
    d["hardware"]["thrust_curve_forward"] = std::vector<double>{fwd[0], fwd[1]};
    d["hardware"]["thrust_curve_reverse"] = std::vector<double>{rev[0], rev[1]};
    const std::string path = testing::TempDir() + "/talos_sim_quadratic.yaml";
    std::ofstream(path) << d;
    const HardwareConfig hw = FossenModel::load(TALOS_VEHICLE, path).hardware();
    ASSERT_TRUE(hw.quadratic);
    EXPECT_EQ(hw.forceToRpm(0.0), 0.0);
    for (double f : {0.01, 1.0, 12.0, 24.0}) {
        const double r = hw.forceToRpm(f), q = hw.forceToRpm(-f);
        EXPECT_GT(r, 0);
        EXPECT_LT(q, 0);
        EXPECT_NEAR(fwd[0] * r * r + fwd[1] * r, f, 1e-9); // exact inverse of the thrust curve
        EXPECT_NEAR(rev[0] * q * q - rev[1] * q, f, 1e-9);
    }
    EXPECT_GT(hw.forceToRpm(24.0), hw.forceToRpm(12.0));

    d["hardware"]["force_to_rpm_positive"] = std::vector<double>{0, 0, 0, 1};
    std::ofstream(path) << d;
    EXPECT_THROW(FossenModel::load(TALOS_VEHICLE, path), std::invalid_argument); // both forms given
}

TEST(Hardware, PropellerThrustCoefficientInverts) {
    YAML::Node d = YAML::LoadFile(simModelWithHardware());
    d["hardware"].remove("force_to_rpm_positive");
    d["hardware"].remove("force_to_rpm_negative");
    const std::array<double, 2> fwd{0.3964, 1.48e-5}, rev{0.3114, 1.25e-5};
    const double diameter = 0.076, rho = d["water_density"].as<double>();
    d["hardware"]["propeller_diameter"] = diameter;
    d["hardware"]["thrust_coefficient_forward"] = std::vector<double>{fwd[0], fwd[1]};
    d["hardware"]["thrust_coefficient_reverse"] = std::vector<double>{rev[0], rev[1]};
    const std::string path = testing::TempDir() + "/talos_sim_propeller.yaml";
    std::ofstream(path) << d;
    const HardwareConfig hw = FossenModel::load(TALOS_VEHICLE, path).hardware();
    ASSERT_TRUE(hw.propeller);
    const auto thrust = [&](const std::array<double, 2> &k, double rpm) {
        return (k[0] + k[1] * rpm) * rho * std::pow(diameter, 4) * (rpm / 60) * (rpm / 60);
    };
    EXPECT_EQ(hw.forceToRpm(0.0), 0.0);
    for (double f : {0.001, 0.5, 1.0, 12.0, 24.0, 40.0}) {
        const double r = hw.forceToRpm(f), q = hw.forceToRpm(-f);
        EXPECT_GT(r, 0);
        EXPECT_LT(q, 0);
        EXPECT_NEAR(thrust(fwd, r), f, 1e-9 * f); // exact inverse of the propeller law
        EXPECT_NEAR(thrust(rev, -q), f, 1e-9 * f);
    }
    // Blue Robotics T200 data at 16 V: 1994.08 rpm forward gives 1.5876 kgf.
    EXPECT_NEAR(hw.forceToRpm(1.5876 * 9.80665), 1994.08, 30.0);
    EXPECT_GT(hw.forceToRpm(24.0), hw.forceToRpm(12.0));

    d["hardware"]["thrust_curve_forward"] = std::vector<double>{3.5e-6, -1.1e-3};
    d["hardware"]["thrust_curve_reverse"] = std::vector<double>{2.8e-6, 4e-4};
    std::ofstream(path) << d;
    EXPECT_THROW(FossenModel::load(TALOS_VEHICLE, path), std::invalid_argument); // two forms given
    d["hardware"].remove("thrust_curve_forward");
    d["hardware"].remove("thrust_curve_reverse");
    d["hardware"]["thrust_coefficient_reverse"] = std::vector<double>{0.3114, -1e-5};
    std::ofstream(path) << d;
    EXPECT_THROW(FossenModel::load(TALOS_VEHICLE, path), std::invalid_argument); // b < 0
}

TEST(Hardware, TotalThrustBudgetScalesCommands) {
    const FossenModel model = FossenModel::load(TALOS_VEHICLE, simModelWithHardware());
    const double max_force = YAML::LoadFile(TALOS_MODEL_SIM)["thruster_dynamics"]["forward_max_force"].as<double>();
    Eigen::VectorXd u(8);
    u << 20, -20, 20, -20, 10, 10, -5, 5;
    const Eigen::VectorXd limited = model.limitTotalThrust(u);
    EXPECT_NEAR(limited.cwiseAbs().sum(), 73.0, 1e-9);
    EXPECT_NEAR((limited.normalized() - u.normalized()).norm(), 0, 1e-12); // same direction
    EXPECT_LT(model.commandUpperBound().maxCoeff(), max_force + 1e-9);
}
