#include "riptide_mpc/identification.hpp"
#include "riptide_mpc/settle_trust.hpp"
#include "riptide_mpc/sim_rig.hpp"

#include <gtest/gtest.h>

#include <algorithm>
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

// Flies the sequence with the MPC on `model_path` against `plant_path`, then fits (per thruster too when
// the settings fly thruster ramps or null-space patterns). Releases are cut 1 m below the surface.
ident::Result flyAndFit(const std::string &plant_path, const std::string &model_path,
                        const ident::SequenceSettings &settings = ident::SequenceSettings(),
                        const Vector3d &start = Vector3d(0, 0, -2)) {
    SimRig rig(TALOS_VEHICLE, plant_path, model_path, MpcSettings(), EstimatorSettings(), start);
    ident::Recorder recorder(FossenModel::load(TALOS_VEHICLE, model_path), SensorMounts::load(TALOS_VEHICLE));
    rig.hooks.command = [&](double t, const VectorXd &u) { recorder.command(t, u); };
    rig.hooks.fog = [&](double t, double r) { recorder.fogRate(t, r); };
    rig.hooks.imu_rate = [&](double t, const Vector3d &w) { recorder.imuRate(t, w); };
    rig.hooks.imu_orientation = [&](double t, const Quaterniond &q) { recorder.imuOrientation(t, q); };
    rig.hooks.dvl = [&](double t, const Vector3d &v) { recorder.dvlVelocity(t, v); };

    const MotionLimits base = rig.mpc.settings().motion;
    const MatrixXd T = rig.mpc.model().thrusterMatrix();
    ident::Sequencer sequencer(ident::buildSequence(settings, rig.position(), 0.0, base, rig.mpc.model().thrusterCount(),
                                                    ident::nullSpacePatterns(T)));
    SettleTrust trust;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = rig.position();
    const double dt = rig.mpc.settings().dt;
    bool done = false;
    for (int tick = 0; tick < 20 * 1500 && !done; ++tick) {
        const auto out = sequencer.update(rig.t, trust.trust() > 0.95, sequencer.releasing() && rig.position().z() > -1.0);
        rig.mpc.setIdentificationInputs(out.fixed, out.bias);
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
    const bool per_thruster = settings.thruster_ramps || settings.null_space;
    const ident::Result result = ident::fit(prior, mass, samples, segments, per_thruster ? T : MatrixXd(),
                                            settings.releases ? YAML::LoadFile(TALOS_VEHICLE) : YAML::Node());
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

// The full identification with the v3 blocks, in deep water, against a plant whose thrusters differ from the
// model one by one and forward vs reverse, with wrong statics and wrong heave/roll/pitch dynamics: thruster
// ramps and null-space patterns while holding give each thruster's gains, the runs the drag and inertia, and
// the releases roll/pitch damping and a heave cross-check.
TEST(Identification, IdentifiesEachThrusterAndTheReleases) {
    // Each group's forward mean is 1 (surge 0,1,6,7; vectored 2..5): the load-cell scale the fit assumes.
    YAML::Node truth = YAML::Clone(YAML::LoadFile(TALOS_MODEL_SIM));
    const std::vector<double> efficiency{1.05, 0.95, 1.03, 0.92, 1.05, 1.0, 1.04, 0.96};
    const std::vector<double> reverse{0.75, 0.8, 0.7, 0.75, 0.85, 0.75, 0.7, 0.8};
    truth["thruster_forward_scales"] = efficiency;
    truth["thruster_reverse_scales"] = reverse;
    truth["displaced_volume"] = truth["displaced_volume"].as<double>() * 1.01;
    // A COB like the real Talos' (floats ~20 deg nose up): the simulator copy's (x = z) floats ~50 deg, where
    // rolling about the body x axis is mostly turning about the vertical and nothing rights it.
    truth["cob_relative"] = std::vector<double>{0.006, 0.0, 0.016};
    const auto set = [](YAML::Node m, int r, double v) {
        if (m.size() == 36)
            m[r * 7] = v;
        else
            m[r][r] = v;
    };
    set(truth["added_mass6x6"], 2, entry(truth["added_mass6x6"], 2, 2) * 1.3);
    set(truth["linear_damping6x6"], 2, entry(truth["linear_damping6x6"], 2, 2) * 1.5);
    set(truth["added_mass6x6"], 3, 0.4);
    set(truth["added_mass6x6"], 4, 0.3);
    set(truth["linear_damping6x6"], 3, entry(truth["linear_damping6x6"], 3, 3) * 1.3);
    set(truth["linear_damping6x6"], 4, entry(truth["linear_damping6x6"], 4, 4) * 1.3);
    const std::string plant_path = testing::TempDir() + "/thruster_truth.yaml";
    std::ofstream(plant_path) << truth;

    ident::SequenceSettings s; // the full identification, as flown in the pool, plus the v3 blocks
    s.thruster_ramps = s.null_space = s.releases = true;
    const ident::Result result = flyAndFit(plant_path, TALOS_MODEL_SIM, s, Vector3d(0, 0, -4.5)); // the dive well

    // The statics feed the release fit (it only sees ratios to them), so they must be right too.
    ASSERT_TRUE(result.statics.ok) << result.statics.note;
    EXPECT_NEAR(result.statics.volume / truth["displaced_volume"].as<double>(), 1.0, 0.003);
    for (int i = 0; i < 3; ++i)
        EXPECT_NEAR(result.statics.cob[i], truth["cob_relative"][i].as<double>(), 0.002) << "cob " << i;

    // Gains are relative to the model's thrust with each group's forward mean pinned: truth / mean(group).
    const auto &f = result.thrusters;
    ASSERT_TRUE(f.ok) << f.note;
    EXPECT_EQ(f.group, (std::vector<int>{0, 0, 1, 1, 1, 1, 0, 0})); // surge thrusters, vectored thrusters
    // The vectored four only ever move together while holding (their null-space pattern), so their reverse
    // mean is pinned as well; the surge four mix signs, so theirs is measured against their forward mean.
    EXPECT_EQ(f.reverse_pinned, (std::vector<bool>{false, true}));
    std::vector<double> mean_reverse(2, 0.);
    for (int i = 0; i < 8; ++i)
        mean_reverse[f.group[i]] += reverse[i] / 4;
    const auto expected_forward = [&](int i) { return efficiency[i]; };
    const auto expected_reverse = [&](int i) {
        return f.reverse_pinned[f.group[i]] ? reverse[i] / mean_reverse[f.group[i]] : reverse[i];
    };
    for (int i = 0; i < 8; ++i) {
        EXPECT_TRUE(f.forward_ok[i] && f.reverse_ok[i]) << "thruster " << i;
        EXPECT_NEAR(f.forward[i], expected_forward(i), 0.03) << "forward gain " << i;
        EXPECT_NEAR(f.reverse[i], expected_reverse(i), 0.04) << "reverse gain " << i;
    }
    const double mass = YAML::LoadFile(TALOS_VEHICLE)["mass"].as<double>();
    const YAML::Node prior = YAML::LoadFile(TALOS_MODEL_SIM);
    for (const auto &a : result.axes) {
        if (a.axis < 2)
            continue;
        const int d = a.axis;
        const double rigid = d < 3 ? mass : prior["rigid_body_inertia3x3"][(d - 3) * 4].as<double>();
        EXPECT_TRUE(a.drag_ok) << "axis " << d << ": " << a.note;
        EXPECT_NEAR(a.d1 / entry(truth["linear_damping6x6"], d, d), 1.0, 0.2) << "linear damping axis " << d;
        // Inertia only where the releases determine it: a release barely accelerates in heave, and righting
        // from a tilt is overdamped, so the fit keeps the prior's unless it is determined to 10%.
        std::printf("axis %d: %s\n", d, a.note.c_str());
        if (a.mass_ok)
            EXPECT_NEAR(a.mass_total / (rigid + entry(truth["added_mass6x6"], d, d)), 1.0, 0.15) << "inertia axis " << d;
    }
    EXPECT_EQ(std::count_if(result.axes.begin(), result.axes.end(),
                            [](const ident::AxisFit &a) { return a.axis >= 2 && a.axis <= 4; }),
              3); // heave (runs, release cross-check), roll and pitch (releases)

    // The identified model flies the plant's thrusters (relative to the pinned mean) and loads in the MPC.
    const YAML::Node model = ident::identifiedModel(prior, result, "test");
    const std::string model_path = testing::TempDir() + "/identified_thrusters.yaml";
    std::ofstream(model_path) << model;
    const FossenModel identified = FossenModel::load(TALOS_VEHICLE, model_path);
    for (int i = 0; i < 8; ++i) {
        const ThrusterParameters &p = identified.actuatorParameters()[i];
        EXPECT_NEAR(p.efficiency * p.forwardScale, expected_forward(i), 0.03);
        EXPECT_NEAR(p.efficiency * p.reverseScale, expected_reverse(i), 0.04);
        EXPECT_LE(p.efficiency, 1.0);
    }
}
