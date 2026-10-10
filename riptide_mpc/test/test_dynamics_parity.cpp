// riptide_mpc carries a copy of the simulator's dynamics so it does not depend on
// c_simulator. Built only when c_simulator is installed: the copy must match it.
#include "riptide_mpc/fossen_model.hpp"

#include <c_simulator/MarineDynamics.h>
#include <c_simulator/ThrusterDynamics.h>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <random>

using namespace riptide_mpc;

TEST(DynamicsParity, MarineDynamicsMatchesTheSimulator) {
    const FossenModel model = FossenModel::load(TALOS_VEHICLE, TALOS_MODEL_SIM);
    const MarineDynamics &mine = model.dynamics();
    c_simulator::MarineDynamics sim;
    const YAML::Node h = YAML::LoadFile(TALOS_MODEL_SIM);
    auto m6 = [](const YAML::Node &n) {
        c_simulator::Matrix6d m;
        for (int r = 0; r < 6; ++r)
            for (int c = 0; c < 6; ++c)
                m(r, c) = n.size() == 36 ? n[r * 6 + c].as<double>() : n[r][c].as<double>();
        return m;
    };
    auto v3 = [](const YAML::Node &n) { return Eigen::Vector3d(n[0].as<double>(), n[1].as<double>(), n[2].as<double>()); };
    sim.configure(model.mass(), matrix3(h["rigid_body_inertia3x3"], "rigid_body_inertia3x3"), m6(h["added_mass6x6"]));
    const auto q = h["quadratic_damping"].as<std::vector<double>>();
    sim.configureDamping(m6(h["linear_damping6x6"]), c_simulator::Vector6d(Eigen::Map<const c_simulator::Vector6d>(q.data())), v3(h["damping_center_relative"]));
    sim.configureHydrostatics(h["water_density"].as<double>(), h["displaced_volume"].as<double>(), v3(h["cob_relative"]),
                              v3(h["buoyancy_radii"]), 9.80665, h["water_level"].as<double>(0));

    std::mt19937 rng(3);
    std::uniform_real_distribution<double> u(-1, 1);
    for (int k = 0; k < 200; ++k) {
        State13d x;
        x << 3 * u(rng), 3 * u(rng), -0.3 + 1.5 * u(rng), u(rng), u(rng), u(rng), u(rng), u(rng), u(rng), u(rng),
            u(rng), u(rng), u(rng);
        x.segment<4>(3).normalize();
        Vector6d tau;
        for (int i = 0; i < 6; ++i)
            tau[i] = 30 * u(rng);
        const Eigen::Vector3d current(0.1 * u(rng), 0.1 * u(rng), 0);
        const State13d a = mine.derivative(x, tau, current);
        const c_simulator::State13d b = sim.derivative(x, tau, current);
        ASSERT_LT((a - b).norm(), 1e-12 * (1 + b.norm())) << "state " << k;
    }
}

TEST(DynamicsParity, ThrusterDynamicsMatchesTheSimulator) {
    const FossenModel model = FossenModel::load(TALOS_VEHICLE, TALOS_MODEL_SIM);
    ThrusterDynamics mine = model.makeActuator();
    std::vector<c_simulator::ThrusterParameters> params;
    for (const auto &p : model.actuatorParameters()) {
        c_simulator::ThrusterParameters s;
        s.delay = p.delay, s.rise = p.rise, s.fall = p.fall, s.slew = p.slew, s.deadband = p.deadband;
        s.forwardLimit = p.forwardLimit, s.reverseLimit = p.reverseLimit, s.forwardScale = p.forwardScale;
        s.reverseScale = p.reverseScale, s.efficiency = p.efficiency;
        params.push_back(s);
    }
    c_simulator::ThrusterDynamics sim;
    sim.configure(params, model.commandTimeout());
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> u(-35, 35);
    for (int k = 0; k < 400; ++k) {
        if (k % 25 == 0) {
            Eigen::VectorXd cmd(model.thrusterCount());
            for (int i = 0; i < cmd.size(); ++i)
                cmd[i] = u(rng);
            mine.command(cmd);
            sim.command(cmd);
        }
        mine.advance(0.002);
        sim.advance(0.002);
        ASSERT_LT((mine.forces() - sim.forces()).norm(), 1e-12) << "step " << k;
    }
}
