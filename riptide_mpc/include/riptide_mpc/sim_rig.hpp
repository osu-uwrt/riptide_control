#pragma once

// Offline closed loop for tests and model-mismatch studies: a PLANT model (the
// simulator's equations, stepped at 2 ms exactly like physics_simulator) with
// sensors synthesized the way the simulator does (IMU 50 Hz with 0.5 deg tilt
// noise, FOG 500 Hz, DVL 8 Hz, depth 20 Hz, EKF 30 Hz with lagging rates),
// driving the StateEstimator + MPC built from a separate MODEL file.
#include "riptide_mpc/mpc.hpp"
#include "riptide_mpc/state_estimator.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <functional>
#include <random>

namespace riptide_mpc {
struct SimRig {
    FossenModel plant;
    ThrusterDynamics plant_actuator;
    SensorMounts mounts;
    StateEstimator estimator;
    MpcController mpc;
    bool feed_disturbance = true; // pass the learned disturbance to the MPC
    // Optional taps on the same command and sensor stream the estimator sees.
    struct Hooks {
        std::function<void(double, const VectorXd &)> command;
        std::function<void(double, double)> fog;
        std::function<void(double, const Vector3d &)> imu_rate;
        std::function<void(double, const Quaterniond &)> imu_orientation;
        std::function<void(double, const Vector3d &)> dvl;
    } hooks;
    State13d x;
    double t = 0;
    Vector3d ekf_rate = Vector3d::Zero();
    std::mt19937 rng{7};
    std::normal_distribution<double> unit{0., 1.};
    int step_count = 0;

    SimRig(const std::string &vehicle_yaml, const std::string &plant_hydro, const std::string &model_hydro,
           const MpcSettings &mpc_settings = MpcSettings(), const EstimatorSettings &estimator_settings = {},
           const Vector3d &start = Vector3d(0, 0, -2))
        : SimRig(vehicle_yaml, FossenModel::load(vehicle_yaml, plant_hydro), FossenModel::load(vehicle_yaml, model_hydro),
                 mpc_settings, estimator_settings, start) {}
    // `mounts` are the plant's (sensor synthesis); the estimator takes them about its own model's COM.
    SimRig(const std::string &vehicle_yaml, const FossenModel &plant_model, const FossenModel &model,
           const MpcSettings &mpc_settings = MpcSettings(), const EstimatorSettings &estimator_settings = {},
           const Vector3d &start = Vector3d(0, 0, -2))
        : plant(plant_model), plant_actuator(plant.makeActuator()), mounts(SensorMounts::load(vehicle_yaml, plant.com())),
          estimator(model, SensorMounts::load(vehicle_yaml, model.com()), estimator_settings), mpc(model, mpc_settings) {
        x = plant.fromBaseLink(start, Quaterniond::Identity(), Vector3d::Zero(), Vector3d::Zero());
        stepPlant(); // first EKF message seeds the estimate
    }

    Quaterniond orientation() const {
        return Quaterniond(x[3], x[4], x[5], x[6]).normalized();
    }
    Vector3d position() const {
        return plant.baseLinkPosition(x);
    }

    // One 2 ms physics step plus the sensor messages due at it.
    void stepPlant() {
        const int k = step_count++;
        plant.step(x, plant_actuator, 0.002);
        t += 0.002;
        const Vector3d w = x.tail<3>();
        const auto due = [k](double hz) { return k % static_cast<int>(std::lround(500. / hz)) == 0; };
        if (due(500)) {
            const double fog = mounts.fog_axis.dot(w) + 0.01 * M_PI / 180 * unit(rng);
            estimator.fogRate(t, fog);
            if (hooks.fog)
                hooks.fog(t, fog);
        }
        if (due(50)) {
            const Vector3d rate = mounts.imu.conjugate() * w + 0.01 * M_PI / 180 * noise3();
            estimator.imuRate(t, rate);
            const Quaterniond jitter(Eigen::AngleAxisd(0.5 * M_PI / 180 * unit(rng), noise3().normalized()));
            const Quaterniond q_imu = jitter * orientation() * mounts.imu;
            estimator.imuOrientation(t, q_imu);
            if (hooks.imu_rate)
                hooks.imu_rate(t, rate);
            if (hooks.imu_orientation)
                hooks.imu_orientation(t, q_imu);
        }
        if (due(8)) {
            const Vector3d v = mounts.dvl.conjugate() * (x.segment<3>(7) + w.cross(mounts.dvl_position)) + 0.001 * noise3();
            estimator.dvlVelocity(t, v);
            if (hooks.dvl)
                hooks.dvl(t, v);
        }
        if (due(20))
            estimator.depth(t, position().z() + 0.01 * unit(rng));
        ekf_rate += (1 - std::exp(-0.002 / 4.0)) * (w - ekf_rate);
        if (due(30))
            estimator.odometry(t, position(), orientation(), plant.baseLinkVelocity(x), ekf_rate);
    }

    // Runs the loop for `seconds`; `each_tick` sees the rig after every control period.
    void run(const Reference &r, double seconds, const std::function<void(const SimRig &)> &each_tick = {}) {
        const double dt = mpc.settings().dt;
        const int ticks = static_cast<int>(std::lround(seconds / dt));
        const int per_tick = static_cast<int>(std::lround(dt / 0.002));
        for (int i = 0; i < ticks; ++i) {
            estimator.propagate(t);
            mpc.setDisturbance(feed_disturbance ? estimator.disturbance() : Vector6d::Zero());
            const MpcOutput out = mpc.compute(estimator.state(), r);
            plant_actuator.command(out.command);
            if (hooks.command)
                hooks.command(t, out.command);
            mpc.issue(out.command);
            estimator.command(t, out.command);
            for (int j = 0; j < per_tick; ++j)
                stepPlant();
            mpc.advance(dt);
            if (each_tick)
                each_tick(*this);
        }
    }

  private:
    Vector3d noise3() {
        return Vector3d(unit(rng), unit(rng), unit(rng));
    }
};
} // namespace riptide_mpc
