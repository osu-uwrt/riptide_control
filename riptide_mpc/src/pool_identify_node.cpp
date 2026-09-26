// Pool identification session: flies the identification sequence through the
// running MPC (POSITION setpoints on controller/linear|angular, speed limits via
// the MPC's live motion.* parameters), records the realized thrust and raw
// sensors, fits buoyancy/COB, drag and added mass, and writes a new model file.
//
// The operator arms the vehicle as usual and keeps the physical kill in hand. The
// session starts from wherever the vehicle is (it must be at depth), stays inside
// a box around that point, and ends holding the start pose. Killing the vehicle
// ends the session; whatever was recorded is saved and fitted.
#include "riptide_mpc/identification.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <riptide_msgs2/msg/controller_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <optional>

using namespace std::chrono_literals;
using riptide_msgs2::msg::ControllerCommand;

namespace riptide_mpc {
class PoolIdentifyNode : public rclcpp::Node {
  public:
    PoolIdentifyNode() : Node("pool_identify") {
        const std::string robot = robotName();
        const std::string share = ament_index_cpp::get_package_share_directory("riptide_mpc");
        vehicle_ = declare_parameter<std::string>(
            "vehicle_config",
            ament_index_cpp::get_package_share_directory("riptide_descriptions2") + "/config/" + robot + ".yaml");
        model_path_ = declare_parameter<std::string>("hydrodynamics_config", share + "/config/models/" + robot + ".yaml");
        const char *home = std::getenv("HOME");
        output_root_ = declare_parameter<std::string>("output_dir", std::string(home ? home : ".") + "/osu-uwrt/mpc_identification");
        mpc_node_ = declare_parameter<std::string>("mpc_node", "mpc_controller");

        auto &s = settings_;
        s.statics = declare_parameter("statics", s.statics);
        s.surge = declare_parameter("surge", s.surge);
        s.sway = declare_parameter("sway", s.sway);
        s.heave = declare_parameter("heave", s.heave);
        s.yaw = declare_parameter("yaw", s.yaw);
        s.lane_length = declare_parameter("lane_length", s.lane_length);
        s.heave_span = declare_parameter("heave_span", s.heave_span);
        s.yaw_span = declare_parameter("yaw_span", s.yaw_span);
        s.linear_speeds = declare_parameter("linear_speeds", s.linear_speeds);
        s.heave_speeds = declare_parameter("heave_speeds", s.heave_speeds);
        s.yaw_rates = declare_parameter("yaw_rates", s.yaw_rates);
        s.linear_accel = declare_parameter("linear_accel", s.linear_accel);
        s.angular_accel = declare_parameter("angular_accel", s.angular_accel);
        s.tilt = declare_parameter("tilt", s.tilt);
        s.hold_secs = declare_parameter("hold_secs", s.hold_secs);
        s.settle_timeout = declare_parameter("settle_timeout", s.settle_timeout);
        min_start_depth_ = declare_parameter("min_start_depth", 0.8);
        min_depth_ = declare_parameter("min_depth", 0.4);
        abort_margin_ = declare_parameter("abort_margin", 1.5);
        trust_threshold_ = declare_parameter("trust_threshold", 0.95);

        const FossenModel model = FossenModel::load(vehicle_, model_path_);
        recorder_.emplace(model, SensorMounts::load(vehicle_));
        RCLCPP_INFO(get_logger(), "Identification model prior: %s", model_path_.c_str());

        const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(50);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = m; });
        trust_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "controller/scale/trust", 10, [this](const geometry_msgs::msg::Twist &m) {
                trust_ = std::min({m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z});
            });
        kill_sub_ = create_subscription<std_msgs::msg::Bool>("state/kill", 10, [this](const std_msgs::msg::Bool &m) {
            if (m.data && killed_ == false)
                recorder_->stopThrusters(now_s());
            killed_ = m.data;
        });
        thrust_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "thruster_forces", 10, [this](const std_msgs::msg::Float32MultiArray &m) {
                Eigen::VectorXd u(m.data.size());
                for (std::size_t i = 0; i < m.data.size(); ++i)
                    u[i] = m.data[i];
                recorder_->command(now_s(), u);
            });
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "vectornav/imu", sensor_qos, [this](const sensor_msgs::msg::Imu &m) {
                const double t = now_s();
                recorder_->imuRate(t, Vector3d(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z));
                const auto &o = m.orientation;
                if (m.orientation_covariance[0] >= 0)
                    recorder_->imuOrientation(t, Quaterniond(o.w, o.x, o.y, o.z));
                have_imu_ = true;
            });
        fog_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "gyro/twist", sensor_qos, [this](const geometry_msgs::msg::TwistWithCovarianceStamped &m) {
                recorder_->fogRate(now_s(), m.twist.twist.angular.z);
            });
        dvl_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "dvl_twist", sensor_qos, [this](const geometry_msgs::msg::TwistWithCovarianceStamped &m) {
                const auto &v = m.twist.twist.linear;
                recorder_->dvlVelocity(now_s(), Vector3d(v.x, v.y, v.z));
                last_dvl_ = now_s();
            });
        lin_pub_ = create_publisher<ControllerCommand>("controller/linear", 10);
        ang_pub_ = create_publisher<ControllerCommand>("controller/angular", 10);
        params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, mpc_node_);
        timer_ = rclcpp::create_timer(this, get_clock(), 50ms, [this] { tick(); });
        RCLCPP_WARN(get_logger(), "Pool identification: arm the vehicle at depth with the kill switch in hand. "
                                  "Do not run an autonomy tree at the same time.");
    }

  private:
    double now_s() {
        return get_clock()->now().seconds();
    }
    std::string robotName() const {
        std::string ns = get_namespace();
        return ns.substr(ns.rfind('/') + 1).empty() ? "talos" : ns.substr(ns.rfind('/') + 1);
    }
    Vector3d position() const {
        const auto &p = odom_->pose.pose.position;
        return Vector3d(p.x, p.y, p.z);
    }

    void publish(const Vector3d &p, const Quaterniond &q) {
        ControllerCommand lin, ang;
        lin.mode = ang.mode = ControllerCommand::POSITION;
        lin.setpoint_vect.x = p.x();
        lin.setpoint_vect.y = p.y();
        lin.setpoint_vect.z = p.z();
        ang.setpoint_quat.w = q.w();
        ang.setpoint_quat.x = q.x();
        ang.setpoint_quat.y = q.y();
        ang.setpoint_quat.z = q.z();
        lin_pub_->publish(lin);
        ang_pub_->publish(ang);
    }

    std::vector<rclcpp::Parameter> motionParameters(const MotionLimits &m) const {
        return {rclcpp::Parameter("motion.linear_speed", m.linear_speed),
                rclcpp::Parameter("motion.linear_accel", m.linear_accel),
                rclcpp::Parameter("motion.angular_speed", m.angular_speed),
                rclcpp::Parameter("motion.angular_accel", m.angular_accel)};
    }

    void sendLimits(const MotionLimits &m) {
        params_pending_ = true;
        params_->set_parameters(motionParameters(m),
                                [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
                                    for (const auto &r : f.get())
                                        if (!r.successful)
                                            RCLCPP_ERROR(get_logger(), "MPC rejected motion limits: %s", r.reason.c_str());
                                    params_pending_ = false;
                                });
    }

    // Waits for armed, at depth, all sensors and the MPC; then captures the start.
    bool ready() {
        const double t = now_s();
        std::string missing;
        if (!odom_)
            missing += " odometry";
        if (trust_ < 0)
            missing += " controller-trust";
        if (!have_imu_)
            missing += " imu";
        if (t - last_dvl_ > 1.0)
            missing += " dvl(bottom lock)";
        if (!params_->service_is_ready())
            missing += " " + mpc_node_ + "-parameters";
        if (killed_.value_or(true))
            missing += " armed";
        if (odom_ && position().z() > -min_start_depth_)
            missing += " depth(>" + std::to_string(min_start_depth_) + " m)";
        if (!missing.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for:%s", missing.c_str());
            return false;
        }
        // Remember the MPC's own limits to restore at the end (async: never block the executor).
        if (!base_ready_) {
            if (!base_requested_) {
                base_requested_ = true;
                params_->get_parameters({"motion.linear_speed", "motion.linear_accel", "motion.angular_speed",
                                         "motion.angular_accel"},
                                        [this](std::shared_future<std::vector<rclcpp::Parameter>> f) {
                                            const auto v = f.get();
                                            base_.linear_speed = v.at(0).as_double();
                                            base_.linear_accel = v.at(1).as_double();
                                            base_.angular_speed = v.at(2).as_double();
                                            base_.angular_accel = v.at(3).as_double();
                                            base_ready_ = true;
                                        });
            }
            return false;
        }

        start_ = position();
        const auto &o = odom_->pose.pose.orientation;
        const Quaterniond q(o.w, o.x, o.y, o.z);
        const Eigen::Matrix3d R = q.normalized().toRotationMatrix();
        const double yaw = std::atan2(R(1, 0), R(0, 0));
        start_q_ = Quaterniond(Eigen::AngleAxisd(yaw, Vector3d::UnitZ()));
        sequencer_.emplace(ident::buildSequence(settings_, start_, yaw, base_));
        abort_radius_ = std::max(settings_.lane_length, settings_.heave_span) + abort_margin_;
        RCLCPP_WARN(get_logger(), "Starting identification from [%.2f %.2f %.2f], heading %.0f deg: %zu steps",
                    start_.x(), start_.y(), start_.z(), yaw * 180 / M_PI, sequencer_->size());
        return true;
    }

    void tick() {
        if (finished_)
            return;
        const double t = now_s();
        if (!sequencer_) {
            if (!ready())
                return;
        }
        if (killed_.value_or(false)) {
            finish("vehicle killed: session ended early");
            return;
        }
        const Vector3d p = position();
        if ((p - start_).norm() > abort_radius_ || p.z() > -min_depth_) {
            publish(start_, start_q_);
            finish("left the safe box (or too shallow): holding the start pose");
            return;
        }
        if (params_pending_) { // do not advance until the new limits are in the MPC
            publish(setpoint_, setpoint_q_);
            return;
        }
        const auto out = sequencer_->update(t, trust_ > trust_threshold_);
        if (!out.message.empty())
            RCLCPP_INFO(get_logger(), "%s", out.message.c_str());
        if (out.end)
            recorder_->end(t);
        if (out.begin)
            recorder_->begin(*out.begin, t);
        if (out.done) {
            finish("sequence complete");
            return;
        }
        setpoint_ = out.position;
        setpoint_q_ = out.orientation;
        if (out.new_step)
            sendLimits(out.limits);
        else
            publish(setpoint_, setpoint_q_);
    }

    void finish(const std::string &why) {
        finished_ = true;
        const double t = now_s();
        if (recorder_->recording())
            recorder_->end(t);
        RCLCPP_WARN(get_logger(), "Identification finished: %s", why.c_str());
        if (sequencer_) {
            params_->set_parameters(motionParameters(base_));
            if (!killed_.value_or(false))
                publish(start_, start_q_);
        }
        if (recorder_->samples().empty()) {
            RCLCPP_ERROR(get_logger(), "Nothing was recorded");
            rclcpp::shutdown();
            return;
        }
        char stamp[32];
        const std::time_t now = std::time(nullptr);
        std::strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", std::localtime(&now));
        const std::string dir = output_root_ + "/" + robotName() + "_" + stamp;
        std::filesystem::create_directories(dir);
        ident::writeRecording(dir, recorder_->samples(), recorder_->segments());
        const YAML::Node prior = YAML::LoadFile(model_path_);
        const double mass = YAML::LoadFile(vehicle_)["mass"].as<double>();
        const auto result = ident::fit(prior, mass, recorder_->samples(), recorder_->segments());
        YAML::Node report = ident::report(prior, result);
        report["session"] = why;
        report["prior_model"] = model_path_;
        std::ofstream(dir + "/report.yaml") << report << "\n";
        std::ofstream(dir + "/model_identified.yaml")
            << ident::identifiedModel(prior, result, std::string("pool identification ") + stamp + " from " + model_path_)
            << "\n";
        RCLCPP_WARN(get_logger(), "Saved %zu samples to %s\n%s", recorder_->samples().size(), dir.c_str(),
                    YAML::Dump(report).c_str());
        RCLCPP_WARN(get_logger(), "Review report.yaml, then use the model with mpc_model:=%s/model_identified.yaml "
                                  "(or copy it over riptide_mpc config/models/%s.yaml)",
                    dir.c_str(), robotName().c_str());
        rclcpp::shutdown();
    }

    std::string vehicle_, model_path_, output_root_, mpc_node_;
    ident::SequenceSettings settings_;
    double min_start_depth_, min_depth_, abort_margin_, trust_threshold_, abort_radius_ = 0;
    std::optional<ident::Recorder> recorder_;
    std::optional<ident::Sequencer> sequencer_;
    MotionLimits base_;
    Vector3d start_ = Vector3d::Zero(), setpoint_ = Vector3d::Zero();
    Quaterniond start_q_ = Quaterniond::Identity(), setpoint_q_ = Quaterniond::Identity();
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    double trust_ = -1, last_dvl_ = -1e9;
    std::optional<bool> killed_;
    bool have_imu_ = false, params_pending_ = false, finished_ = false, base_requested_ = false, base_ready_ = false;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr trust_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr thrust_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr fog_sub_, dvl_sub_;
    rclcpp::Publisher<ControllerCommand>::SharedPtr lin_pub_, ang_pub_;
    std::shared_ptr<rclcpp::AsyncParametersClient> params_;
    rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace riptide_mpc

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<riptide_mpc::PoolIdentifyNode>();
    rclcpp::spin(node);
    if (rclcpp::ok())
        rclcpp::shutdown();
    return 0;
}
