// Pool identification session: flies the identification sequence through the
// running MPC (POSITION setpoints on controller/linear|angular, speed limits via
// the MPC's live motion.* parameters), records the realized thrust and raw
// sensors, fits buoyancy/COB, drag and added mass, and writes a new model file.
// Optional blocks drive the MPC's identification inputs (controller/identification/
// thrusters, allowed through its identification.allow parameter for the session):
// per-thruster ramps and null-space patterns while it holds (per-thruster gains),
// and releases with every thruster off (heave and roll/pitch dynamics).
//
// The operator arms the vehicle as usual and keeps the physical kill in hand. The
// session starts from wherever the vehicle is (it must be at depth), stays inside
// a box around that point, and ends holding the start pose. Killing the vehicle
// ends the session; whatever was recorded is saved and fitted. With record_bag, a
// `ros2 bag record` of bag_topics runs for the whole session into <session>/bag.
#include "riptide_mpc/identification.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <riptide_msgs2/msg/controller_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <thread>

#include <fcntl.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace std::chrono_literals;
using riptide_msgs2::msg::ControllerCommand;

namespace riptide_mpc {
class PoolIdentifyNode : public rclcpp::Node {
  public:
    PoolIdentifyNode() : Node("pool_identify") {
        // Empty (default): the files the running MPC uses (its vehicle_config / hydrodynamics_config), read
        // from it before the session starts, so the prior is always the model being flown.
        vehicle_ = declare_parameter<std::string>("vehicle_config", "");
        model_path_ = declare_parameter<std::string>("hydrodynamics_config", "");
        const char *home = std::getenv("HOME");
        output_root_ = declare_parameter<std::string>("output_dir", std::string(home ? home : ".") + "/osu-uwrt/mpc_identification");
        mpc_node_ = declare_parameter<std::string>("mpc_node", "mpc_controller");
        record_bag_ = declare_parameter("record_bag", true);
        bag_topics_ = declare_parameter<std::vector<std::string>>(
            "bag_topics", {"command/requested_rpm", "thruster_forces", "state/thrusters/telemetry",
                           "state/thrusters/rpm", "state/kill", "odometry/filtered", "controller/mpc/state",
                           "controller/mpc/reference", "controller/mpc/disturbance", "controller/mpc/solve_time_ms",
                           "controller_debug_wrench", "controller/linear", "controller/angular",
                           "controller/motion_enabled", "controller/scale/trust", "controller/identification/thrusters",
                           "vectornav/imu", "gyro/twist", "dvl_twist", "depth/pose", "pool_identify/done", "/tf",
                           "/tf_static"});

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
        s.repeats = static_cast<int>(declare_parameter("repeats", static_cast<int64_t>(s.repeats)));
        s.settle_timeout = declare_parameter("settle_timeout", s.settle_timeout);
        s.thruster_ramps = declare_parameter("thruster_ramps", s.thruster_ramps);
        s.ramp_force = declare_parameter("ramp_force", s.ramp_force);
        s.ramp_secs = declare_parameter("ramp_secs", s.ramp_secs);
        s.null_space = declare_parameter("null_space", s.null_space);
        s.null_amplitude = declare_parameter("null_amplitude", s.null_amplitude);
        s.null_secs = declare_parameter("null_secs", s.null_secs);
        s.releases = declare_parameter("releases", s.releases);
        s.release_tilted = declare_parameter("release_tilted", s.release_tilted);
        s.release_secs = declare_parameter("release_secs", s.release_secs);
        s.recover_timeout = declare_parameter("recover_timeout", s.recover_timeout);
        s.release_tilt = declare_parameter("release_tilt", s.release_tilt);
        release_min_depth_ = declare_parameter("release_min_depth", 1.0); // m; a release ends shallower than this
        release_max_tilt_ = declare_parameter("release_max_tilt", 0.8);   // rad; or tilted more than this
        max_depth_margin_ = declare_parameter("max_depth_margin", 1.0);   // m below the deepest planned point
        min_start_depth_ = declare_parameter("min_start_depth", 0.8);
        min_depth_ = declare_parameter("min_depth", 0.4);
        abort_margin_ = declare_parameter("abort_margin", 1.5);
        trust_threshold_ = declare_parameter("trust_threshold", 0.95);
        // Iterations: fly, fit everything recorded so far, load the fitted model into the
        // running MPC, fly again; stop when two successive fits agree (or max_iterations).
        max_iterations_ = static_cast<int>(declare_parameter("max_iterations", static_cast<int64_t>(1)));
        tolerance_ = declare_parameter("convergence_tolerance", 0.10); // relative: drag, inertia, volume
        cob_tolerance_ = declare_parameter("cob_tolerance", 0.002);    // m
        gain_tolerance_ = declare_parameter("gain_tolerance", 0.03);   // thruster gains, absolute
        guard_tilt_ = declare_parameter("guard_tilt_error", 0.45);     // rad off the commanded tilt, for 1 s
        guard_rate_ = declare_parameter("guard_tilt_rate", 0.8);       // rad/s roll/pitch rate RMS over 1 s
        current_model_ = model_path_;
        // Trigger mode (autonomy tree, untethered): start only on the pool_identify/start service,
        // report pool_identify/done (2 Hz), and stay up after finishing so the tree can read it.
        wait_for_trigger_ = declare_parameter("wait_for_trigger", false);

        if (!vehicle_.empty() && !model_path_.empty())
            loadModel();
        else
            RCLCPP_INFO(get_logger(), "Identification model prior: the running MPC's (read from %s)", mpc_node_.c_str());

        const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(50);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) { odom_ = m; });
        // The MPC's profiled reference: where the attitude should be right now, mid-move included.
        reference_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "controller/mpc/reference", 10, [this](const geometry_msgs::msg::PoseStamped &m) {
                const auto &o = m.pose.orientation;
                reference_q_ = Quaterniond(o.w, o.x, o.y, o.z).normalized();
                reference_time_ = now_s();
            });
        trust_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "controller/scale/trust", 10, [this](const geometry_msgs::msg::Twist &m) {
                trust_ = std::min({m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z});
            });
        kill_sub_ = create_subscription<std_msgs::msg::Bool>("state/kill", 10, [this](const std_msgs::msg::Bool &m) {
            if (m.data && killed_ == false && recorder_)
                recorder_->stopThrusters(now_s());
            killed_ = m.data;
        });
        thrust_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
            "thruster_forces", 10, [this](const std_msgs::msg::Float32MultiArray &m) {
                if (!recorder_)
                    return;
                Eigen::VectorXd u(m.data.size());
                for (std::size_t i = 0; i < m.data.size(); ++i)
                    u[i] = m.data[i];
                recorder_->command(now_s(), u);
            });
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "vectornav/imu", sensor_qos, [this](const sensor_msgs::msg::Imu &m) {
                const double t = now_s();
                have_imu_ = true;
                if (!recorder_)
                    return;
                recorder_->imuRate(t, Vector3d(m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z));
                const auto &o = m.orientation;
                if (m.orientation_covariance[0] >= 0)
                    recorder_->imuOrientation(t, Quaterniond(o.w, o.x, o.y, o.z));
            });
        fog_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "gyro/twist", sensor_qos, [this](const geometry_msgs::msg::TwistWithCovarianceStamped &m) {
                if (recorder_)
                    recorder_->fogRate(now_s(), m.twist.twist.angular.z);
            });
        dvl_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "dvl_twist", sensor_qos, [this](const geometry_msgs::msg::TwistWithCovarianceStamped &m) {
                const auto &v = m.twist.twist.linear;
                last_dvl_ = now_s();
                if (recorder_)
                    recorder_->dvlVelocity(now_s(), Vector3d(v.x, v.y, v.z));
            });
        ident_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("controller/identification/thrusters", 10);
        lin_pub_ = create_publisher<ControllerCommand>("controller/linear", 10);
        ang_pub_ = create_publisher<ControllerCommand>("controller/angular", 10);
        params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, mpc_node_);
        done_pub_ = create_publisher<std_msgs::msg::Bool>("pool_identify/done", 10);
        done_timer_ = rclcpp::create_timer(this, get_clock(), 500ms, [this] {
            std_msgs::msg::Bool m;
            m.data = finished_;
            done_pub_->publish(m);
        });
        if (wait_for_trigger_)
            start_srv_ = create_service<std_srvs::srv::Trigger>(
                "pool_identify/start", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                              std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
                    res->success = !finished_;
                    res->message = finished_ ? "already finished" : triggered_ ? "already started" : "starting";
                    if (!finished_ && !triggered_)
                        RCLCPP_WARN(get_logger(), "Start requested (pool_identify/start)");
                    triggered_ = true;
                });
        timer_ = rclcpp::create_timer(this, get_clock(), 50ms, [this] { tick(); });
        RCLCPP_WARN(get_logger(), "Pool identification: arm the vehicle at depth with the kill switch in hand. "
                                  "Do not run an autonomy tree at the same time.");
    }

    ~PoolIdentifyNode() override {
        stopBag();
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
                rclcpp::Parameter("motion.angular_accel", m.angular_accel),
                // Heave runs set their speed here; the MPC's vertical cap would clip them otherwise.
                rclcpp::Parameter("motion.linear_speed_vertical", m.linear_speed_vertical),
                rclcpp::Parameter("motion.linear_accel_vertical", m.linear_accel_vertical)};
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
        if (wait_for_trigger_ && !triggered_)
            missing += " start-trigger(pool_identify/start)";
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
                                         "motion.angular_accel", "motion.linear_speed_vertical",
                                         "motion.linear_accel_vertical"},
                                        [this](std::shared_future<std::vector<rclcpp::Parameter>> f) {
                                            const auto v = f.get();
                                            base_.linear_speed = v.at(0).as_double();
                                            base_.linear_accel = v.at(1).as_double();
                                            base_.angular_speed = v.at(2).as_double();
                                            base_.angular_accel = v.at(3).as_double();
                                            base_.linear_speed_vertical = v.at(4).as_double();
                                            base_.linear_accel_vertical = v.at(5).as_double();
                                            base_ready_ = true;
                                        });
            }
            return false;
        }

        // The prior is the model the MPC is flying (unless given): read its files once.
        if (!recorder_) {
            if (!model_requested_) {
                model_requested_ = true;
                params_->get_parameters({"vehicle_config", "hydrodynamics_config"},
                                        [this](std::shared_future<std::vector<rclcpp::Parameter>> f) {
                                            try {
                                                const auto v = f.get();
                                                if (vehicle_.empty())
                                                    vehicle_ = v.at(0).as_string();
                                                if (model_path_.empty())
                                                    model_path_ = v.at(1).as_string();
                                                loadModel();
                                            } catch (const std::exception &e) {
                                                RCLCPP_ERROR(get_logger(), "Cannot use the MPC's model (%s); give "
                                                             "model:=<file> (and vehicle_config)", e.what());
                                                model_requested_ = false;
                                            }
                                        });
            }
            return false;
        }
        // The identification blocks drive the MPC's identification inputs: allow them for this session.
        if (usesInputs() && !allow_ok_) {
            if (!allow_requested_) {
                allow_requested_ = true;
                params_->set_parameters({rclcpp::Parameter("identification.allow", true)},
                                        [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
                                            const auto r = f.get();
                                            if (r.empty() || !r.front().successful) {
                                                RCLCPP_ERROR(get_logger(), "MPC refused identification.allow (%s): "
                                                             "thruster/null-space/release blocks off",
                                                             r.empty() ? "no reply" : r.front().reason.c_str());
                                                settings_.thruster_ramps = settings_.null_space = settings_.releases = false;
                                            }
                                            allow_ok_ = true;
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
        start_yaw_ = yaw;
        char stamp[32];
        const std::time_t now = std::time(nullptr);
        std::strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", std::localtime(&now));
        stamp_ = stamp;
        dir_ = output_root_ + "/" + robotName() + "_" + stamp_;
        std::filesystem::create_directories(dir_);
        startBag();
        sequencer_.emplace(ident::buildSequence(settings_, start_, yaw, base_, thrusters_, null_space_));
        abort_radius_ = std::max(settings_.lane_length, settings_.heave_span) + abort_margin_;
        RCLCPP_WARN(get_logger(), "Starting identification from [%.2f %.2f %.2f], heading %.0f deg: %zu steps, "
                    "up to %d iteration(s); session %s",
                    start_.x(), start_.y(), start_.z(), yaw * 180 / M_PI, sequencer_->size(), max_iterations_,
                    dir_.c_str());
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
        // Horizontal box around the start, and depth limits: a release rises freely toward release_min_depth.
        const double deepest = start_.z() - (settings_.heave ? settings_.heave_span : 0.) - max_depth_margin_;
        if ((p - start_).head<2>().norm() > abort_radius_ || p.z() > -min_depth_ || p.z() < deepest) {
            publishInputs(ident::Sequencer::Output());
            publish(start_, start_q_);
            finish("left the safe box (or too shallow/deep): holding the start pose");
            return;
        }
        if (const std::string why = sequencer_->guarded() ? unstable(t) : ""; !why.empty()) {
            if (iteration_ > 0) { // our model did this: back to the one that flew
                RCLCPP_ERROR(get_logger(), "%s with %s: reverting the MPC to %s", why.c_str(), current_model_.c_str(),
                             previous_model_.c_str());
                params_->set_parameters({rclcpp::Parameter("hydrodynamics_config", previous_model_)});
                current_model_ = previous_model_;
            }
            publish(start_, start_q_);
            finish(why + ": session ended");
            return;
        }
        if (params_pending_) { // do not advance until the new limits are in the MPC
            publish(setpoint_, setpoint_q_);
            return;
        }
        bool cut = false;
        if (sequencer_->releasing()) {
            const auto &o = odom_->pose.pose.orientation;
            const Vector3d up = Quaterniond(o.w, o.x, o.y, o.z).normalized().conjugate() * Vector3d::UnitZ();
            cut = p.z() > -release_min_depth_ || std::acos(std::clamp(up.z(), -1., 1.)) > release_max_tilt_;
        }
        const auto out = sequencer_->update(t, trust_ > trust_threshold_, cut);
        publishInputs(out);
        if (!out.message.empty())
            RCLCPP_INFO(get_logger(), "%s", out.message.c_str());
        if (out.end)
            recorder_->end(t);
        if (out.begin)
            recorder_->begin(*out.begin, t);
        if (out.done) {
            iterationComplete();
            return;
        }
        setpoint_ = out.position;
        setpoint_q_ = out.orientation;
        if (out.new_step)
            sendLimits(out.limits);
        else
            publish(setpoint_, setpoint_q_);
    }

    // Divergence guard: tilt far off the MPC's reference tilt (its profile, which moves between holds at the
    // angular limits; the step's attitude if the reference is not coming), or fast roll/pitch, for about a second.
    std::string unstable(double t) {
        const auto &o = odom_->pose.pose.orientation;
        const Quaterniond q = Quaterniond(o.w, o.x, o.y, o.z).normalized();
        const Quaterniond commanded = reference_q_ && t - reference_time_ < 0.5 ? *reference_q_ : setpoint_q_;
        const Vector3d up = q.conjugate() * Vector3d::UnitZ(), up_cmd = commanded.conjugate() * Vector3d::UnitZ();
        const double tilt_error = std::acos(std::clamp(up.dot(up_cmd), -1., 1.));
        const auto &w = odom_->twist.twist.angular;
        rates_.push_back({t, w.x * w.x + w.y * w.y});
        while (!rates_.empty() && t - rates_.front().first > 1.0)
            rates_.pop_front();
        double sum = 0;
        for (const auto &r : rates_)
            sum += r.second;
        const double rate_rms = std::sqrt(sum / rates_.size());
        if (tilt_error > guard_tilt_ || (rates_.size() > 10 && rate_rms > guard_rate_)) {
            if (!bad_since_)
                bad_since_ = t;
            if (t - *bad_since_ > 1.0) {
                char m[160];
                std::snprintf(m, sizeof m, "unstable (tilt error %.0f deg, roll/pitch rate rms %.2f rad/s)",
                              tilt_error * 180 / M_PI, rate_rms);
                return m;
            }
        } else {
            bad_since_.reset();
        }
        return "";
    }

    // Successive pooled fits agree: same acceptance, and every accepted value within tolerance.
    bool converged(const ident::Result &a, const ident::Result &b, YAML::Node &changes) const {
        bool ok = a.statics.ok == b.statics.ok;
        const auto rel = [](double x, double y) { return std::abs(x - y) / std::max(std::abs(y), 1e-9); };
        if (a.statics.ok && b.statics.ok) {
            const double dv = rel(b.statics.volume, a.statics.volume);
            const double dc = (b.statics.cob - a.statics.cob).cwiseAbs().maxCoeff();
            changes["statics"]["volume_rel"] = dv;
            changes["statics"]["cob_m"] = dc;
            ok = ok && dv <= tolerance_ && dc <= cob_tolerance_;
        }
        for (const auto &fb : b.axes) {
            const auto it = std::find_if(a.axes.begin(), a.axes.end(), [&](const ident::AxisFit &f) { return f.axis == fb.axis; });
            if (it == a.axes.end())
                return false;
            const ident::AxisFit &fa = *it;
            static const char *names[] = {"surge", "sway", "heave", "roll", "pitch", "yaw"};
            const std::string name = names[fb.axis];
            ok = ok && fa.drag_ok == fb.drag_ok && fa.mass_ok == fb.mass_ok;
            if (fa.drag_ok && fb.drag_ok) { // compare drag where it is flown; its D1/D2 split is ill-conditioned
                double worst = 0;
                for (double v : fb.axis == 5 ? std::vector<double>{0.3, 0.6, 0.9} : std::vector<double>{0.15, 0.3, 0.5})
                    worst = std::max(worst, rel(fb.d1 * v + fb.d2 * v * v, fa.d1 * v + fa.d2 * v * v));
                changes[name]["drag_rel"] = worst;
                ok = ok && worst <= tolerance_;
            }
            if (fa.mass_ok && fb.mass_ok) {
                const double dm = rel(fb.mass_total, fa.mass_total);
                changes[name]["inertia_rel"] = dm;
                ok = ok && dm <= tolerance_;
            }
        }
        // Thruster gains are all relative to the same (the recorder's) model, so they compare directly.
        if (a.thrusters.ok && b.thrusters.ok && a.thrusters.forward.size() == b.thrusters.forward.size()) {
            const double df = (b.thrusters.forward - a.thrusters.forward).cwiseAbs().maxCoeff();
            const double dr = (b.thrusters.reverse - a.thrusters.reverse).cwiseAbs().maxCoeff();
            changes["thrusters"]["forward_gain"] = df;
            changes["thrusters"]["reverse_gain"] = dr;
            ok = ok && df <= gain_tolerance_ && dr <= gain_tolerance_;
        } else if (a.thrusters.ok != b.thrusters.ok) {
            ok = false;
        }
        return ok;
    }

    // The prior (model_path_ with vehicle_): the recorder's thrust model, the fit's starting point.
    void loadModel() {
        const FossenModel model = FossenModel::load(vehicle_, model_path_);
        recorder_.emplace(model, SensorMounts::load(vehicle_));
        thruster_matrix_ = model.thrusterMatrix();
        null_space_ = ident::nullSpacePatterns(thruster_matrix_);
        thrusters_ = model.thrusterCount();
        current_model_ = model_path_;
        RCLCPP_INFO(get_logger(), "Identification model prior: %s (vehicle %s)", model_path_.c_str(), vehicle_.c_str());
    }

    bool usesInputs() const {
        return settings_.thruster_ramps || settings_.null_space || settings_.releases;
    }

    // The MPC's identification inputs: the step's (fixed commands and bias) or none, every tick of the session
    // (the MPC drops them 0.5 s after the last message anyway).
    void publishInputs(const ident::Sequencer::Output &out) {
        if (!allow_requested_)
            return;
        std_msgs::msg::Float64MultiArray m;
        m.data.assign(2 * thrusters_, 0.);
        for (int i = 0; i < thrusters_; ++i) {
            m.data[i] = i < out.fixed.size() ? out.fixed[i] : std::numeric_limits<double>::quiet_NaN();
            m.data[thrusters_ + i] = i < out.bias.size() ? out.bias[i] : 0.;
        }
        ident_pub_->publish(m);
    }

    // One pass of the sequence is done: fit all data so far, write the model, then either
    // stop (agreed / last iteration) or load it into the MPC and fly again with it.
    void iterationComplete() {
        const double t = now_s();
        if (recorder_->recording())
            recorder_->end(t);
        const YAML::Node prior = YAML::LoadFile(current_model_);
        const double mass = YAML::LoadFile(vehicle_)["mass"].as<double>();
        const auto result = ident::fit(prior, mass, recorder_->samples(), recorder_->segments(), thruster_matrix_,
                                       YAML::LoadFile(vehicle_));
        const std::string path = dir_ + "/model_iter" + std::to_string(iteration_ + 1) + ".yaml";
        std::ofstream(path) << ident::identifiedModel(prior, result,
                                                      "pool identification " + stamp_ + " iteration " +
                                                          std::to_string(iteration_ + 1) + " from " + current_model_,
                                                      YAML::LoadFile(model_path_))
                            << "\n";
        YAML::Node entry;
        entry["iteration"] = iteration_ + 1;
        entry["flown_with"] = current_model_;
        entry["fit"] = ident::report(prior, result);
        entry["model"] = path;
        bool agreed = false;
        if (previous_result_) {
            YAML::Node changes;
            agreed = converged(*previous_result_, result, changes);
            entry["changes_from_previous"] = changes;
            entry["converged"] = agreed;
            RCLCPP_WARN(get_logger(), "Iteration %d vs %d:\n%s", iteration_ + 1, iteration_, YAML::Dump(changes).c_str());
        }
        iterations_.push_back(entry);
        std::ofstream(dir_ + "/iterations.yaml") << iterations_ << "\n";
        previous_result_ = result;

        const bool last = iteration_ + 1 >= max_iterations_;
        // Every iteration's fit flies next, including the last, so the session ends on the best model.
        RCLCPP_WARN(get_logger(), "Iteration %d done%s: loading %s into the MPC", iteration_ + 1,
                    agreed ? " (converged)" : last ? " (max_iterations)" : "", path.c_str());
        previous_model_ = current_model_;
        current_model_ = path;
        params_pending_ = true;
        params_->set_parameters(
            {rclcpp::Parameter("hydrodynamics_config", path)},
            [this, agreed, last](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
                const auto r = f.get();
                params_pending_ = false;
                if (r.empty() || !r.front().successful) {
                    RCLCPP_ERROR(get_logger(), "MPC rejected the model: %s",
                                 r.empty() ? "no reply" : r.front().reason.c_str());
                    current_model_ = previous_model_;
                    finish("MPC rejected the fitted model");
                    return;
                }
                if (agreed || last) {
                    std::filesystem::copy_file(current_model_, dir_ + "/model_final.yaml",
                                               std::filesystem::copy_options::overwrite_existing);
                    finish(agreed ? "converged" : "max_iterations reached");
                    return;
                }
                ++iteration_;
                sequencer_.emplace(ident::buildSequence(settings_, start_, start_yaw_, base_, thrusters_, null_space_),
                                   1000 * iteration_);
                bad_since_.reset();
                RCLCPP_WARN(get_logger(), "Iteration %d of up to %d: flying with %s", iteration_ + 1,
                            max_iterations_, current_model_.c_str());
            });
    }

    // Standalone: exit. Trigger mode: stay up publishing done=true for the tree.
    void end() {
        stopBag();
        if (!wait_for_trigger_)
            rclcpp::shutdown();
    }

    // Runs `ros2 bag record` into <session>/bag. Forked from the executor (main) thread: PR_SET_PDEATHSIG
    // fires when the forking THREAD exits, which for that thread means the process, so the recorder stops
    // with the node. Everything the child needs is built before fork() (the process has middleware threads).
    void startBag() {
        if (!record_bag_ || bag_pid_ > 0)
            return;
        std::vector<std::string> args{"ros2", "bag", "record", "-o", dir_ + "/bag"};
        if (get_parameter("use_sim_time").as_bool())
            args.push_back("--use-sim-time");
        for (const auto &t : bag_topics_)
            args.push_back(get_node_topics_interface()->resolve_topic_name(t));
        std::vector<char *> argv;
        for (auto &a : args)
            argv.push_back(a.data());
        argv.push_back(nullptr);
        const std::string log = dir_ + "/bag.log";
        const pid_t parent = getpid();
        const pid_t pid = fork();
        if (pid == 0) {
            prctl(PR_SET_PDEATHSIG, SIGINT);
            if (getppid() != parent)
                _exit(0);
            const int fd = open(log.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
            if (fd >= 0) {
                dup2(fd, STDOUT_FILENO);
                dup2(fd, STDERR_FILENO);
                close(fd);
            }
            execvp(argv[0], argv.data());
            _exit(127);
        }
        if (pid < 0) {
            RCLCPP_ERROR(get_logger(), "Could not start the session bag (fork failed)");
            return;
        }
        bag_pid_ = pid;
        RCLCPP_WARN(get_logger(), "Recording %zu topics to %s/bag (recorder log: bag.log)", bag_topics_.size(),
                    dir_.c_str());
    }

    // SIGINT lets the recorder close the bag cleanly; SIGKILL only if it hangs.
    void stopBag() {
        if (bag_pid_ <= 0)
            return;
        kill(bag_pid_, SIGINT);
        int status = 0;
        pid_t r = 0;
        for (int i = 0; i < 100 && (r = waitpid(bag_pid_, &status, WNOHANG)) == 0; ++i)
            std::this_thread::sleep_for(100ms);
        if (r == 0) {
            kill(bag_pid_, SIGKILL);
            waitpid(bag_pid_, &status, 0);
            RCLCPP_ERROR(get_logger(), "Session bag recorder did not stop in 10 s: killed (bag may be incomplete)");
        } else if (r == bag_pid_ && WIFEXITED(status) && WEXITSTATUS(status) == 127) {
            RCLCPP_ERROR(get_logger(), "Session bag was not recorded: could not run ros2 (see bag.log)");
        } else {
            RCLCPP_WARN(get_logger(), "Session bag saved to %s/bag", dir_.c_str());
        }
        bag_pid_ = -1;
    }

    void finish(const std::string &why) {
        finished_ = true;
        const double t = now_s();
        if (!recorder_) {
            RCLCPP_ERROR(get_logger(), "Identification finished before it started: %s", why.c_str());
            end();
            return;
        }
        if (recorder_->recording())
            recorder_->end(t);
        RCLCPP_WARN(get_logger(), "Identification finished: %s", why.c_str());
        if (allow_requested_) { // back to normal control, and no more inputs accepted
            publishInputs(ident::Sequencer::Output());
            params_->set_parameters({rclcpp::Parameter("identification.allow", false)});
        }
        if (sequencer_) {
            params_->set_parameters(motionParameters(base_));
            if (!killed_.value_or(false))
                publish(start_, start_q_);
        }
        if (recorder_->samples().empty()) {
            RCLCPP_ERROR(get_logger(), "Nothing was recorded");
            end();
            return;
        }
        const std::string dir = dir_;
        ident::writeRecording(dir, recorder_->samples(), recorder_->segments());
        const YAML::Node prior = YAML::LoadFile(current_model_);
        const double mass = YAML::LoadFile(vehicle_)["mass"].as<double>();
        const auto result = ident::fit(prior, mass, recorder_->samples(), recorder_->segments(), thruster_matrix_,
                                       YAML::LoadFile(vehicle_));
        YAML::Node report = ident::report(prior, result);
        report["session"] = why;
        report["iterations_fitted"] = static_cast<int>(iterations_.size());
        report["prior_model"] = current_model_;
        report["mpc_model_loaded"] = current_model_;
        std::ofstream(dir + "/report.yaml") << report << "\n";
        std::ofstream(dir + "/model_identified.yaml")
            << ident::identifiedModel(prior, result, "pool identification " + stamp_ + " from " + current_model_,
                                      YAML::LoadFile(model_path_))
            << "\n";
        std::ofstream(dir + "/iterations.yaml") << iterations_ << "\n";
        RCLCPP_WARN(get_logger(), "Saved %zu samples to %s\n%s", recorder_->samples().size(), dir.c_str(),
                    YAML::Dump(report).c_str());
        RCLCPP_WARN(get_logger(), "The MPC is flying %s until it restarts. To keep it, copy it over riptide_mpc "
                                  "config/models/%s.yaml (or launch with mpc_model:=<that file>).",
                    current_model_.c_str(), robotName().c_str());
        end();
    }

    std::string vehicle_, model_path_, output_root_, mpc_node_;
    bool record_bag_ = true;
    std::vector<std::string> bag_topics_;
    pid_t bag_pid_ = -1;
    ident::SequenceSettings settings_;
    double min_start_depth_, min_depth_, abort_margin_, trust_threshold_, abort_radius_ = 0;
    int max_iterations_ = 1, iteration_ = 0;
    bool wait_for_trigger_ = false, triggered_ = false;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;
    rclcpp::TimerBase::SharedPtr done_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    double tolerance_ = 0.1, cob_tolerance_ = 0.002, guard_tilt_ = 0.45, guard_rate_ = 0.8, start_yaw_ = 0;
    double gain_tolerance_ = 0.03, release_min_depth_ = 1.0, release_max_tilt_ = 0.8, max_depth_margin_ = 1.0;
    bool allow_requested_ = false, allow_ok_ = false, model_requested_ = false;
    std::optional<Quaterniond> reference_q_;
    double reference_time_ = -1e9;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr reference_sub_;
    int thrusters_ = 0;
    Eigen::MatrixXd thruster_matrix_, null_space_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ident_pub_;
    std::string current_model_, previous_model_, dir_, stamp_;
    std::optional<ident::Result> previous_result_;
    YAML::Node iterations_;
    std::deque<std::pair<double, double>> rates_;
    std::optional<double> bad_since_;
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
