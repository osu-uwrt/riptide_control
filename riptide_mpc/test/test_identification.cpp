#include "riptide_mpc/identification.hpp"
#include "riptide_mpc/settle_trust.hpp"
#include "riptide_mpc/sim_rig.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>

using namespace riptide_mpc;

namespace {
double entry(const YAML::Node &m, int r, int c) {
    return m.size() == 36 ? m[r * 6 + c].as<double>() : m[r][c].as<double>();
}

// The MPC's model with pool-plausible errors it does not know about. Thrust is
// left exact: the identification assumes load-cell calibrated thrusters.
YAML::Node truthPlant() {
    YAML::Node d = YAML::Clone(YAML::LoadFile(TALOS_MODEL_SIM));
    d["displaced_volume"] = d["displaced_volume"].as<double>() * 1.02;
    d["cob_relative"][0] = d["cob_relative"][0].as<double>() + 0.01;
    d["cob_relative"][2] = d["cob_relative"][2].as<double>() - 0.005;
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
    return d;
}
} // namespace

// Flies the default sequence with the MPC on `model_path` against `plant_path`, then fits.
ident::Result flyAndFit(const std::string &plant_path, const std::string &model_path) {
    SimRig rig(TALOS_VEHICLE, plant_path, model_path);
    ident::Recorder recorder(FossenModel::load(TALOS_VEHICLE, model_path), SensorMounts::load(TALOS_VEHICLE));
    rig.hooks.command = [&](double t, const VectorXd &u) { recorder.command(t, u); };
    rig.hooks.fog = [&](double t, double r) { recorder.fogRate(t, r); };
    rig.hooks.imu_rate = [&](double t, const Vector3d &w) { recorder.imuRate(t, w); };
    rig.hooks.imu_orientation = [&](double t, const Quaterniond &q) { recorder.imuOrientation(t, q); };
    rig.hooks.dvl = [&](double t, const Vector3d &v) { recorder.dvlVelocity(t, v); };

    const MotionLimits base = rig.mpc.settings().motion;
    ident::Sequencer sequencer(ident::buildSequence(ident::SequenceSettings(), rig.position(), 0.0, base));
    SettleTrust trust;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = rig.position();
    const double dt = rig.mpc.settings().dt;
    bool done = false;
    for (int tick = 0; tick < 20 * 900 && !done; ++tick) {
        const auto out = sequencer.update(rig.t, trust.trust() > 0.95);
        if (!out.message.empty())
            std::printf("[%6.1f s] %s\n", rig.t, out.message.c_str());
        if (out.new_step)
            rig.mpc.setMotionLimits(out.limits);
        if (out.end)
            recorder.end(rig.t);
        if (out.begin)
            recorder.begin(*out.begin, rig.t);
        done = out.done;
        r.position = out.position;
        r.orientation = out.orientation;
        rig.run(r, dt, [&](const SimRig &s) {
            Vector6d twist;
            twist << s.plant.baseLinkVelocity(s.x), s.x.tail<3>();
            trust.update(dt, r, s.mpc.profile(), s.position(), s.orientation(), twist);
        });
    }
    EXPECT_TRUE(done) << "sequence did not finish";
    std::printf("sequence: %.0f s simulated, %zu samples, %zu segments\n", rig.t, recorder.samples().size(),
                recorder.segments().size());

    // Round trip through the CSV files the pool tool writes.
    std::vector<ident::Sample> samples;
    std::vector<ident::Segment> segments;
    ident::writeRecording(testing::TempDir(), recorder.samples(), recorder.segments());
    ident::readRecording(testing::TempDir(), samples, segments);
    EXPECT_EQ(samples.size(), recorder.samples().size());

    const YAML::Node prior = YAML::LoadFile(model_path);
    const double mass = YAML::LoadFile(TALOS_VEHICLE)["mass"].as<double>();
    const ident::Result result = ident::fit(prior, mass, samples, segments);
    std::printf("%s\n", YAML::Dump(ident::report(prior, result)).c_str());
    return result;
}

// The fitted values must match `truth` (a hydrodynamics file) and load in the MPC.
void expectRecovered(const ident::Result &result, const YAML::Node &truth, const std::string &prior_path) {
    const YAML::Node prior = YAML::LoadFile(prior_path);
    const double mass = YAML::LoadFile(TALOS_VEHICLE)["mass"].as<double>();
    const auto &st = result.statics;
    ASSERT_TRUE(st.ok);
    const double v_true = truth["displaced_volume"].as<double>();
    EXPECT_NEAR(st.volume / v_true, 1.0, 0.002); // 0.2% of ~320 N = 0.6 N
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(st.cob[i], truth["cob_relative"][i].as<double>(), 0.002) << "cob " << i;

    ASSERT_EQ(result.axes.size(), 4u);
    for (const auto &a : result.axes) {
        const int d = a.axis;
        EXPECT_TRUE(a.drag_ok) << "axis " << d << ": " << a.note;
        EXPECT_TRUE(a.mass_ok) << "axis " << d << ": " << a.note;
        // Compare drag where it was measured: at the fastest commanded speed.
        const double v = d == 5 ? 0.9 : d == 2 ? 0.35 : 0.65;
        const double true_drag = entry(truth["linear_damping6x6"], d, d) * v +
                                 truth["quadratic_damping"][d].as<double>() * v * v;
        EXPECT_NEAR((a.d1 * v + a.d2 * v * v) / true_drag, 1.0, 0.1) << "drag axis " << d;
        const double rigid = d < 3 ? mass : prior["rigid_body_inertia3x3"][8].as<double>();
        const double true_total = rigid + entry(truth["added_mass6x6"], d, d);
        EXPECT_NEAR(a.mass_total / true_total, 1.0, 0.15) << "inertia axis " << d;
    }

    const YAML::Node model = ident::identifiedModel(prior, result, "test");
    for (int i = 0; i < 3; ++i) // the identified model carries the fitted statics
        EXPECT_NEAR(model["cob_relative"][i].as<double>(), truth["cob_relative"][i].as<double>(), 0.002);
    const std::string model_path = testing::TempDir() + "/identified.yaml";
    std::ofstream(model_path) << model;
    EXPECT_NO_THROW(FossenModel::load(TALOS_VEHICLE, model_path)); // loadable by the MPC
}

TEST(Identification, RecoversThePoolParametersFromTheSequence) {
    const YAML::Node truth = truthPlant();
    const std::string plant_path = testing::TempDir() + "/identification_truth.yaml";
    std::ofstream(plant_path) << truth;
    expectRecovered(flyAndFit(plant_path, TALOS_MODEL_SIM), truth, TALOS_MODEL_SIM);
}

// From a prior with zero drag/added mass and wrong statics (config/models/talos_untuned.yaml),
// flying against the simulator's own plant.
TEST(Identification, LearnsFromAnUntunedPrior) {
    expectRecovered(flyAndFit(TALOS_MODEL_SIM, TALOS_MODEL_UNTUNED), YAML::LoadFile(TALOS_MODEL_SIM),
                    TALOS_MODEL_UNTUNED);
}
