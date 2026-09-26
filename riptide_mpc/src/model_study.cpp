// How much does model error hurt the controller? Runs the MPC offline against a
// plant built from one hydrodynamics file while the MPC/estimator use another,
// with and without disturbance (offset-free) estimation.
//
//   ros2 run riptide_mpc model_study --plant <sim hydro yaml> --model <mpc hydro yaml> [--vehicle <vehicle yaml>]
//
// Defaults: plant = config/models/talos_sim.yaml (the simulator's Talos plant), model = riptide_mpc
// config/models/talos.yaml, vehicle = riptide_descriptions talos.yaml,
// settings = riptide_mpc config/mpc.yaml (weights, motion limits, estimator).
#include "riptide_mpc/settings_file.hpp"
#include "riptide_mpc/sim_rig.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdio>
#include <cstring>
#include <string>

using namespace riptide_mpc;

namespace {
struct Metrics {
    double overshoot = 0, settle = -1, final_error = 0, final_attitude = 0, peak_tilt = 0, peak_speed = 0;
};

// One move from the current setpoint; the vehicle starts wherever the last one left it.
Metrics move(SimRig &rig, Reference &r, const Vector3d &offset, double yaw, double seconds) {
    r.position += offset;
    r.orientation = Quaterniond(Eigen::AngleAxisd(yaw, Vector3d::UnitZ())) * r.orientation;
    const Vector3d direction = offset.norm() > 0 ? Vector3d(offset.normalized()) : Vector3d::Zero();
    Metrics m;
    double elapsed = 0, err_sum = 0, att_sum = 0;
    int tail = 0;
    rig.run(r, seconds, [&](const SimRig &s) {
        elapsed += s.mpc.settings().dt;
        const Vector3d e = s.position() - r.position;
        const double att = s.orientation().angularDistance(r.orientation);
        m.overshoot = std::max(m.overshoot, e.dot(direction));
        m.peak_tilt = std::max(m.peak_tilt, std::acos(std::clamp((s.orientation() * Vector3d::UnitZ()).z(), -1., 1.)));
        m.peak_speed = std::max(m.peak_speed, s.plant.baseLinkVelocity(s.x).norm());
        if (e.norm() > 0.02 || att > 0.035)
            m.settle = -1;
        else if (m.settle < 0)
            m.settle = elapsed;
        if (elapsed > seconds - 3) { // average over the last 3 s
            err_sum += e.norm();
            att_sum += att;
            ++tail;
        }
    });
    m.final_error = err_sum / std::max(tail, 1);
    m.final_attitude = att_sum / std::max(tail, 1);
    return m;
}

void study(const std::string &vehicle, const std::string &plant, const std::string &model,
           const std::string &settings, bool disturbance) {
    MpcSettings s;
    EstimatorSettings e;
    loadSettingsFile(settings, s, e);
    e.estimate_disturbance = disturbance;
    SimRig rig(vehicle, plant, model, s, e);
    rig.feed_disturbance = disturbance;
    Reference r;
    r.linear_mode = r.angular_mode = Mode::POSITION;
    r.position = rig.position();

    std::printf("\n== disturbance estimation %s\n", disturbance ? "ON" : "OFF");
    std::printf("%-26s %10s %10s %10s %10s %9s %9s\n", "scenario", "overshoot", "settle", "final err", "final att",
                "peak tilt", "peak v");
    const struct {
        const char *name;
        Vector3d offset;
        double yaw, seconds;
    } scenarios[] = {{"hold (from rest, 20 s)", Vector3d::Zero(), 0, 20},
                     {"surge +1.5 m", Vector3d(1.5, 0, 0), 0, 12},
                     {"sway +1.0 m, yaw +90 deg", Vector3d(0, 1.0, 0), M_PI / 2, 12},
                     {"heave -0.5 m", Vector3d(0, 0, -0.5), 0, 10},
                     {"diagonal back to start", Vector3d(-1.5, -1.0, 0.5), -M_PI / 2, 14}};
    for (const auto &s : scenarios) {
        const Metrics m = move(rig, r, s.offset, s.yaw, s.seconds);
        char settle[32];
        if (m.settle < 0)
            std::snprintf(settle, sizeof settle, "never");
        else
            std::snprintf(settle, sizeof settle, "%.1f s", m.settle);
        std::printf("%-26s %7.1f mm %10s %7.1f mm %6.2f deg %5.1f deg %5.2f m/s\n", s.name, m.overshoot * 1e3,
                    settle, m.final_error * 1e3, m.final_attitude * 180 / M_PI, m.peak_tilt * 180 / M_PI,
                    m.peak_speed);
    }
    const Vector6d d = rig.estimator.disturbance();
    if (disturbance)
        std::printf("learned disturbance: force [%.2f %.2f %.2f] N (world), torque [%.2f %.2f %.2f] N m (body)\n",
                    d[0], d[1], d[2], d[3], d[4], d[5]);
}
} // namespace

int main(int argc, char **argv) {
    std::string vehicle = ament_index_cpp::get_package_share_directory("riptide_descriptions2") + "/config/talos.yaml";
    std::string plant = ament_index_cpp::get_package_share_directory("riptide_mpc") +
                        "/config/models/talos_sim.yaml"; // the simulator's plant
    std::string model = ament_index_cpp::get_package_share_directory("riptide_mpc") + "/config/models/talos.yaml";
    std::string settings = ament_index_cpp::get_package_share_directory("riptide_mpc") + "/config/mpc.yaml";
    for (int i = 1; i + 1 < argc; i += 2) {
        if (!std::strcmp(argv[i], "--vehicle"))
            vehicle = argv[i + 1];
        else if (!std::strcmp(argv[i], "--plant"))
            plant = argv[i + 1];
        else if (!std::strcmp(argv[i], "--model"))
            model = argv[i + 1];
        else if (!std::strcmp(argv[i], "--settings"))
            settings = argv[i + 1];
        else {
            std::fprintf(stderr, "usage: model_study [--plant yaml] [--model yaml] [--vehicle yaml] [--settings mpc.yaml]\n");
            return 2;
        }
    }
    std::printf("vehicle: %s\nplant (truth): %s\nMPC model: %s\nsettings: %s\n", vehicle.c_str(), plant.c_str(),
                model.c_str(), settings.c_str());
    study(vehicle, plant, model, settings, false);
    study(vehicle, plant, model, settings, true);
    return 0;
}
