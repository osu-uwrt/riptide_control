// How well does the MPC's prediction model match the real vehicle? Replays a bag:
// from many start times, the measured state (odometry) is rolled forward through
// the model with the thruster commands that were actually sent (thruster_forces,
// through the same delay/lag queue the MPC uses) and compared with where the
// vehicle actually went. The controller that produced the commands does not
// matter (complete_controller bags work); the model sees only commands and state.
//
//   ros2 run riptide_mpc bag_prediction --bag <bag dir> [--model <hydro yaml>] [--vehicle <vehicle yaml>]
//       [--horizons 0.25,0.5,1.0,1.6] [--stride 0.5] [--step 0.002] [--csv <file>]
//       [--odom-topic /talos/odometry/filtered] [--thrust-topic /talos/thruster_forces]
//       [--thrust-scale 1.0]  (multiplies every recorded command: is the real thrust weaker?)
//
// Prints, per horizon, the RMS and mean (bias) error of the model and of a
// constant-velocity guess (the "no model" baseline). The MPC's online disturbance
// estimate absorbs a constant bias; the scatter around it is what it cannot fix.
// The model starts with zero disturbance here, so this is the raw model.
#include "riptide_mpc/fossen_model.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace riptide_mpc;

namespace {
struct Odom {
    double t;
    Vector3d p;
    Quaterniond q;
    Vector3d v, w; // base_link body frame
};
struct Thrust {
    double t;
    VectorXd f;
};

constexpr double kMaxOdomGap = 0.2;   // s; longer gaps invalidate a window
constexpr double kMaxThrustGap = 0.3; // s
constexpr double kWarmup = 1.0;       // s of command history replayed before each start (> delay + lag)
constexpr int kChannels = 12;
const char *kNames[kChannels] = {"x",    "y",     "z",   "roll", "pitch", "yaw",
                                 "surge", "sway", "heave", "p",    "q",     "r"};
const char *kUnits[4] = {"m", "deg", "m/s", "deg/s"};

// Sample at t (linear / slerp between neighbours); false outside the data or across a gap.
bool interpolate(const std::vector<Odom> &odom, double t, Odom &out) {
    auto it = std::lower_bound(odom.begin(), odom.end(), t, [](const Odom &o, double t) { return o.t < t; });
    if (it == odom.end() || it == odom.begin())
        return false;
    const Odom &b = *it, &a = *(it - 1);
    if (b.t - a.t > kMaxOdomGap)
        return false;
    const double s = (t - a.t) / (b.t - a.t);
    out = {t, a.p + s * (b.p - a.p), a.q.slerp(s, b.q), a.v + s * (b.v - a.v), a.w + s * (b.w - a.w)};
    return true;
}

// Prediction minus measurement: world position, attitude as a body rotation vector
// (roll/pitch/yaw for small errors), body linear and angular velocity.
std::array<double, kChannels> error(const Vector3d &p, const Quaterniond &q, const Vector3d &v, const Vector3d &w,
                                    const Odom &m) {
    const Vector3d dp = p - m.p, da = quaternionLog(m.q.conjugate() * q) * 180 / M_PI, dv = v - m.v,
                   dw = (w - m.w) * 180 / M_PI;
    return {dp.x(), dp.y(), dp.z(), da.x(), da.y(), da.z(), dv.x(), dv.y(), dv.z(), dw.x(), dw.y(), dw.z()};
}

struct Stats {
    std::array<double, kChannels> sum{}, sum_sq{};
    int n = 0;
    void add(const std::array<double, kChannels> &e) {
        for (int i = 0; i < kChannels; ++i) {
            sum[i] += e[i];
            sum_sq[i] += e[i] * e[i];
        }
        ++n;
    }
    double rms(int i) const {
        return n ? std::sqrt(sum_sq[i] / n) : 0;
    }
    double mean(int i) const {
        return n ? sum[i] / n : 0;
    }
};

std::vector<double> parseList(const std::string &s) {
    std::vector<double> out;
    std::stringstream ss(s);
    for (std::string item; std::getline(ss, item, ',');)
        out.push_back(std::stod(item));
    return out;
}

void readBag(const std::string &bag, const std::string &odom_topic, const std::string &thrust_topic,
             std::vector<Odom> &odom, std::vector<Thrust> &thrust) {
    rosbag2_cpp::Reader reader;
    reader.open(bag);
    rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serializer;
    rclcpp::Serialization<std_msgs::msg::Float32MultiArray> thrust_serializer;
    while (reader.has_next()) {
        const auto msg = reader.read_next();
        if (msg->topic_name != odom_topic && msg->topic_name != thrust_topic)
            continue;
        // Receive time for both: thruster_forces has no header.
        const double t = msg->time_stamp * 1e-9;
        const rclcpp::SerializedMessage serialized(*msg->serialized_data);
        if (msg->topic_name == odom_topic) {
            nav_msgs::msg::Odometry o;
            odom_serializer.deserialize_message(&serialized, &o);
            const auto &p = o.pose.pose.position;
            const auto &q = o.pose.pose.orientation;
            const auto &v = o.twist.twist.linear;
            const auto &w = o.twist.twist.angular;
            odom.push_back({t, Vector3d(p.x, p.y, p.z), Quaterniond(q.w, q.x, q.y, q.z).normalized(),
                            Vector3d(v.x, v.y, v.z), Vector3d(w.x, w.y, w.z)});
        } else {
            std_msgs::msg::Float32MultiArray f;
            thrust_serializer.deserialize_message(&serialized, &f);
            VectorXd v(f.data.size());
            for (size_t i = 0; i < f.data.size(); ++i)
                v[i] = f.data[i];
            thrust.push_back({t, v});
        }
    }
    const auto by_time = [](const auto &a, const auto &b) { return a.t < b.t; };
    std::stable_sort(odom.begin(), odom.end(), by_time);
    std::stable_sort(thrust.begin(), thrust.end(), by_time);
}
} // namespace

int main(int argc, char **argv) {
    std::string bag, csv, odom_topic = "/talos/odometry/filtered", thrust_topic = "/talos/thruster_forces";
    std::string model_file = ament_index_cpp::get_package_share_directory("riptide_mpc") + "/config/models/talos.yaml";
    std::string vehicle = ament_index_cpp::get_package_share_directory("riptide_descriptions2") + "/config/talos.yaml";
    std::vector<double> horizons{0.25, 0.5, 1.0, 1.6}; // 1.6 s = MPC horizon (30 x 0.05 s) + thruster delay
    double stride = 0.5, h = 0.002, thrust_scale = 1;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string a = argv[i], v = argv[i + 1];
        if (a == "--bag") bag = v;
        else if (a == "--model") model_file = v;
        else if (a == "--vehicle") vehicle = v;
        else if (a == "--horizons") horizons = parseList(v);
        else if (a == "--stride") stride = std::stod(v);
        else if (a == "--step") h = std::stod(v);
        else if (a == "--csv") csv = v;
        else if (a == "--odom-topic") odom_topic = v;
        else if (a == "--thrust-topic") thrust_topic = v;
        else if (a == "--thrust-scale") thrust_scale = std::stod(v);
        else {
            std::fprintf(stderr, "unknown argument %s\n", a.c_str());
            return 2;
        }
    }
    if (bag.empty() || horizons.empty() || !(stride > 0) || !(h > 0)) {
        std::fprintf(stderr, "usage: bag_prediction --bag <dir> [--model yaml] [--vehicle yaml] [--horizons a,b,..] "
                             "[--stride s] [--step s] [--csv file] [--odom-topic t] [--thrust-topic t] [--thrust-scale k]\n");
        return 2;
    }
    std::sort(horizons.begin(), horizons.end());

    const FossenModel model = FossenModel::load(vehicle, model_file);
    std::vector<Odom> odom;
    std::vector<Thrust> thrust;
    readBag(bag, odom_topic, thrust_topic, odom, thrust);
    if (odom.size() < 2 || thrust.empty()) {
        std::fprintf(stderr, "bag has %zu %s and %zu %s messages\n", odom.size(), odom_topic.c_str(), thrust.size(),
                     thrust_topic.c_str());
        return 1;
    }
    for (auto &c : thrust) {
        c.f *= thrust_scale;
        if (c.f.size() != model.thrusterCount()) {
            std::fprintf(stderr, "%s has %ld thrusters, the model %d\n", thrust_topic.c_str(), c.f.size(),
                         model.thrusterCount());
            return 1;
        }
    }
    std::printf("model %s\nbag %s: %zu odometry, %zu thrust messages over %.0f s\n", model_file.c_str(), bag.c_str(),
                odom.size(), thrust.size(), odom.back().t - odom.front().t);

    std::ofstream out;
    if (!csv.empty()) {
        out.open(csv);
        out << "t0,horizon";
        for (const char *who : {"model", "const_vel"})
            for (const char *n : kNames)
                out << ',' << who << '_' << n;
        out << '\n';
    }

    std::vector<Stats> model_stats(horizons.size()), baseline_stats(horizons.size());
    int skipped = 0;
    const double t_begin = std::max(odom.front().t, thrust.front().t) + kWarmup;
    for (double t0 = t_begin; t0 + horizons.back() <= odom.back().t; t0 += stride) {
        // Every sample the window needs must exist, without gaps.
        std::vector<Odom> truth(horizons.size());
        Odom start;
        bool ok = interpolate(odom, t0, start);
        for (size_t k = 0; ok && k < horizons.size(); ++k)
            ok = interpolate(odom, t0 + horizons[k], truth[k]);
        auto c = std::lower_bound(thrust.begin(), thrust.end(), t0 - kWarmup,
                                  [](const Thrust &a, double t) { return a.t < t; });
        for (auto it = c; ok && it != thrust.end() && it->t <= t0 + horizons.back(); ++it)
            ok = (it + 1 == thrust.end() || (it + 1)->t - it->t <= kMaxThrustGap);
        if (!ok) {
            ++skipped;
            continue;
        }

        // Actuator: replay the command history up to t0 so the in-flight (delayed,
        // lagging) thrust at t0 is what the real thrusters were doing.
        ThrusterDynamics actuator = model.makeActuator();
        double t = t0 - kWarmup;
        for (; c != thrust.end() && c->t <= t0; ++c) {
            if (c->t > t)
                actuator.advance(c->t - t);
            t = std::max(t, c->t);
            actuator.command(c->f);
        }
        if (t0 > t)
            actuator.advance(t0 - t);
        t = t0;

        State13d x = model.fromBaseLink(start.p, start.q, start.v, start.w);
        for (size_t k = 0; k < horizons.size(); ++k) {
            const double end = t0 + horizons[k];
            while (t < end - 1e-9) {
                for (; c != thrust.end() && c->t <= t; ++c)
                    actuator.command(c->f);
                const double dt = std::min(h, end - t);
                model.step(x, actuator, dt);
                t += dt;
            }
            const Quaterniond q = Quaterniond(x[3], x[4], x[5], x[6]).normalized();
            const auto e_model = error(model.baseLinkPosition(x), q, model.baseLinkVelocity(x), x.tail<3>(), truth[k]);

            // Constant velocity: keep the body twist measured at t0.
            const double H = horizons[k];
            const auto e_base = error(start.p + start.q * (start.v * H), start.q * quaternionExp(start.w * H), start.v,
                                      start.w, truth[k]);
            model_stats[k].add(e_model);
            baseline_stats[k].add(e_base);
            if (out.is_open()) {
                out << t0 - odom.front().t << ',' << H;
                for (double v : e_model)
                    out << ',' << v;
                for (double v : e_base)
                    out << ',' << v;
                out << '\n';
            }
        }
    }

    std::printf("%d windows every %.2f s (%d skipped for gaps)\n\n", model_stats.front().n, stride, skipped);
    std::printf("error = prediction - measured. rms: model / const-velocity baseline, then model bias (mean)\n");
    for (size_t k = 0; k < horizons.size(); ++k) {
        std::printf("\nhorizon %.2f s\n  %-6s %-6s %9s %9s %9s %7s\n", horizons[k], "", "", "model", "const-v", "bias",
                    "skill");
        for (int i = 0; i < kChannels; ++i) {
            const double m = model_stats[k].rms(i), b = baseline_stats[k].rms(i);
            // skill: fraction of the baseline error the model removes (negative = worse than no model)
            std::printf("  %-6s %-6s %9.3f %9.3f %+9.3f %6.0f%%\n", kNames[i], kUnits[i / 3], m, b,
                        model_stats[k].mean(i), b > 0 ? 100 * (1 - m / b) : 0.);
        }
    }
    if (out.is_open())
        std::printf("\nper-window errors: %s\n", csv.c_str());
    return 0;
}
