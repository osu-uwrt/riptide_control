// Drop-in replacement for the Simulink complete_controller, driven by an MPC
// whose prediction model is the simulator's Fossen model. Same interface:
//   in:  odometry/filtered, controller/linear, controller/angular (ControllerCommand),
//        controller/FF_body_force, controller/motion_enabled, state/kill,
//        follow_path (FollowPath action: pass through waypoints, settle on the last)
//   out: thruster_forces (Float32MultiArray, newtons), controller_debug_wrench,
//        command/requested_rpm (DshotCommand) and, on the vehicle, the same RPM
//        commands straight to the ESCs over CAN (as complete_controller does)
// With state_source "sensors" the feedback state comes from StateEstimator
// (IMU, FOG, DVL, depth; EKF only as x/y/heading anchor and fallback).
#include "riptide_mpc/mpc.hpp"
#include "riptide_mpc/settle_trust.hpp"
#include "riptide_mpc/state_estimator.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <riptide_msgs2/msg/dshot_command.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <riptide_msgs2/action/follow_path.hpp>
#include <riptide_msgs2/msg/controller_command.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <optional>

extern "C" int send_thruster_cmd_canbus(int16_t cmds[8]); // riptide_controllers send_thruster_cmd_canbus.c

using namespace std::chrono_literals;
using riptide_msgs2::msg::ControllerCommand;
using FollowPath = riptide_msgs2::action::FollowPath;
using PathGoal = rclcpp_action::ServerGoalHandle<FollowPath>;

namespace riptide_mpc {
class MpcControllerNode : public rclcpp::Node {
  public:
    MpcControllerNode() : Node("mpc_controller") {
        const auto vehicle = declare_parameter<std::string>("vehicle_config", "");
        const auto hydro = declare_parameter<std::string>("hydrodynamics_config", "");
        if (vehicle.empty() || hydro.empty())
            throw std::invalid_argument("vehicle_config and hydrodynamics_config are required");
        RCLCPP_INFO(get_logger(), "MPC model: vehicle %s, hydrodynamics %s", vehicle.c_str(), hydro.c_str());

        MpcSettings s;
        s.dt = declare_parameter("control_period", s.dt);
        s.horizon = declare_parameter("horizon_steps", s.horizon);
        s.model_step = declare_parameter("model_step", s.model_step);
        s.linearization_step = declare_parameter("linearization_step", s.linearization_step);
        s.sqp_iterations = declare_parameter("sqp_iterations", s.sqp_iterations);
        s.qp_max_iterations = declare_parameter("qp_max_iterations", s.qp_max_iterations);
        s.compensate_delay = declare_parameter("compensate_delay", s.compensate_delay);
        s.q_position = vec3("weights.position", s.q_position);
        s.q_attitude = vec3("weights.attitude", s.q_attitude);
        s.q_linear_velocity = vec3("weights.linear_velocity", s.q_linear_velocity);
        s.q_angular_velocity = vec3("weights.angular_velocity", s.q_angular_velocity);
        s.q_linear_damping = vec3("weights.linear_damping", s.q_linear_damping);
        s.q_angular_damping = vec3("weights.angular_damping", s.q_angular_damping);
        s.terminal_factor = declare_parameter("weights.terminal_factor", s.terminal_factor);
        s.r_thrust = declare_parameter("weights.thrust", s.r_thrust);
        s.r_thrust_rate = declare_parameter("weights.thrust_rate", s.r_thrust_rate);
        s.profile_motion = declare_parameter("motion.profile", s.profile_motion);
        s.motion.linear_speed = declare_parameter("motion.linear_speed", s.motion.linear_speed);
        s.motion.linear_accel = declare_parameter("motion.linear_accel", s.motion.linear_accel);
        s.motion.linear_jerk = declare_parameter("motion.linear_jerk", s.motion.linear_jerk);
        s.motion.linear_speed_vertical = declare_parameter("motion.linear_speed_vertical", s.motion.linear_speed_vertical);
        s.motion.linear_accel_vertical = declare_parameter("motion.linear_accel_vertical", s.motion.linear_accel_vertical);
        s.motion.linear_jerk_vertical = declare_parameter("motion.linear_jerk_vertical", s.motion.linear_jerk_vertical);
        s.motion.angular_speed = declare_parameter("motion.angular_speed", s.motion.angular_speed);
        s.motion.angular_accel = declare_parameter("motion.angular_accel", s.motion.angular_accel);
        s.motion.angular_jerk = declare_parameter("motion.angular_jerk", s.motion.angular_jerk);
        s.max_reference_lag = declare_parameter("motion.max_reference_lag", s.max_reference_lag);
        s.max_reference_lag_angle = declare_parameter("motion.max_reference_lag_angle", s.max_reference_lag_angle);
        s.governor_lag = declare_parameter("motion.governor_lag", s.governor_lag);
        s.governor_stop_lag = declare_parameter("motion.governor_stop_lag", s.governor_stop_lag);
        s.governor_angle = declare_parameter("motion.governor_angle", s.governor_angle);
        s.governor_stop_angle = declare_parameter("motion.governor_stop_angle", s.governor_stop_angle);
        reference_.linear_velocity_in_body = declare_parameter<std::string>("linear_velocity_frame", "body") == "body";
        respect_motion_enabled_ = declare_parameter("respect_motion_enabled", true);
        odom_timeout_ = declare_parameter("odom_timeout", 0.5);

        const auto source = declare_parameter<std::string>("state_source", "sensors");
        if (source != "sensors" && source != "odometry")
            throw std::invalid_argument("state_source must be 'sensors' or 'odometry'");
        use_sensors_ = source == "sensors";

        const FossenModel model = FossenModel::load(vehicle, hydro);
        controller_.emplace(model, s);
        // Motion limits can be changed live (e.g. by the identification sequence).
        param_callback_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
            MotionLimits m = controller_->settings().motion;
            for (const auto &p : params) {
                const std::string &n = p.get_name();
                if (n.rfind("motion.", 0) != 0 || p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
                    continue;
                const double v = p.as_double();
                if (n == "motion.linear_speed_vertical" || n == "motion.linear_accel_vertical" ||
                    n == "motion.linear_jerk_vertical") { // non-positive: same as horizontal
                    (n == "motion.linear_speed_vertical"   ? m.linear_speed_vertical
                     : n == "motion.linear_accel_vertical" ? m.linear_accel_vertical
                                                           : m.linear_jerk_vertical) = v;
                    continue;
                }
                if (!(v > 0)) {
                    rcl_interfaces::msg::SetParametersResult r;
                    r.successful = false;
                    r.reason = n + " must be positive";
                    return r;
                }
                if (n == "motion.linear_speed") m.linear_speed = v;
                else if (n == "motion.linear_accel") m.linear_accel = v;
                else if (n == "motion.linear_jerk") m.linear_jerk = v;
                else if (n == "motion.angular_speed") m.angular_speed = v;
                else if (n == "motion.angular_accel") m.angular_accel = v;
                else if (n == "motion.angular_jerk") m.angular_jerk = v;
            }
            controller_->setMotionLimits(m);
            rcl_interfaces::msg::SetParametersResult r;
            r.successful = true;
            return r;
        });
        SettleTrustSettings trust;
        trust.growth_tau = declare_parameter("trust.growth_tau", trust.growth_tau);
        trust.decay_tau = declare_parameter("trust.decay_tau", trust.decay_tau);
        trust.position_threshold = declare_parameter("trust.position_threshold", trust.position_threshold);
        trust.attitude_threshold = declare_parameter("trust.attitude_threshold", trust.attitude_threshold);
        trust.linear_velocity_threshold =
            declare_parameter("trust.linear_velocity_threshold", trust.linear_velocity_threshold);
        trust.angular_velocity_threshold =
            declare_parameter("trust.angular_velocity_threshold", trust.angular_velocity_threshold);
        settle_trust_ = SettleTrust(trust);
        if (use_sensors_) {
            EstimatorSettings e;
            e.model_step = s.model_step;
            e.gyro_gain = declare_parameter("estimator.gyro_gain", e.gyro_gain);
            e.fog_gain = declare_parameter("estimator.fog_gain", e.fog_gain);
            e.dvl_gain = declare_parameter("estimator.dvl_gain", e.dvl_gain);
            e.tilt_time_constant = declare_parameter("estimator.tilt_time_constant", e.tilt_time_constant);
            e.depth_time_constant = declare_parameter("estimator.depth_time_constant", e.depth_time_constant);
            e.heading_time_constant = declare_parameter("estimator.heading_time_constant", e.heading_time_constant);
            e.horizontal_time_constant =
                declare_parameter("estimator.horizontal_time_constant", e.horizontal_time_constant);
            e.fallback_time_constant = declare_parameter("estimator.fallback_time_constant", e.fallback_time_constant);
            e.gyro_timeout = declare_parameter("estimator.gyro_timeout", e.gyro_timeout);
            e.fog_timeout = declare_parameter("estimator.fog_timeout", e.fog_timeout);
            e.tilt_timeout = declare_parameter("estimator.tilt_timeout", e.tilt_timeout);
            e.dvl_timeout = declare_parameter("estimator.dvl_timeout", e.dvl_timeout);
            e.depth_timeout = declare_parameter("estimator.depth_timeout", e.depth_timeout);
            e.reset_distance = declare_parameter("estimator.reset_distance", e.reset_distance);
            e.reset_angle = declare_parameter("estimator.reset_angle", e.reset_angle);
            e.estimate_disturbance = declare_parameter("estimator.estimate_disturbance", e.estimate_disturbance);
            e.disturbance_force_time_constant =
                declare_parameter("estimator.disturbance_force_time_constant", e.disturbance_force_time_constant);
            e.disturbance_torque_time_constant =
                declare_parameter("estimator.disturbance_torque_time_constant", e.disturbance_torque_time_constant);
            e.max_disturbance_force = declare_parameter("estimator.max_disturbance_force", e.max_disturbance_force);
            e.max_disturbance_torque = declare_parameter("estimator.max_disturbance_torque", e.max_disturbance_torque);
            e.disturbance_gate_speed = declare_parameter("estimator.disturbance_gate_speed", e.disturbance_gate_speed);
            e.disturbance_gate_rate = declare_parameter("estimator.disturbance_gate_rate", e.disturbance_gate_rate);
            mounts_ = SensorMounts::load(vehicle);
            estimator_.emplace(model, mounts_, e);
        }
        RCLCPP_INFO(get_logger(), "State feedback: %s",
                    use_sensors_ ? "sensors (IMU, FOG, DVL, depth; EKF anchors x/y + heading)" : "odometry");
        RCLCPP_INFO(get_logger(), "MPC ready: %d thrusters, %.3f s x %d stages, actuator delay %.3f s",
                    controller_->model().thrusterCount(), s.dt, s.horizon,
                    controller_->model().actuatorParameters().front().delay);

        const auto odom_topic = declare_parameter<std::string>("odom_topic", "odometry/filtered");
        thruster_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("thruster_forces", 10);
        wrench_pub_ = create_publisher<geometry_msgs::msg::Twist>("controller_debug_wrench", 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>("controller/mpc/predicted_path", 10);
        solve_pub_ = create_publisher<std_msgs::msg::Float64>("controller/mpc/solve_time_ms", 10);
        reference_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("controller/mpc/reference", 10);
        trust_pub_ = create_publisher<geometry_msgs::msg::Twist>("controller/scale/trust", 10);
        disturbance_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("controller/mpc/disturbance", 10);
        rpm_pub_ = create_publisher<riptide_msgs2::msg::DshotCommand>("command/requested_rpm", 10);
        configureHardwareOutput(declare_parameter<std::string>("hardware_output", "auto"));

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                odom_ = m;
                if (!estimator_)
                    return;
                const auto &p = m->pose.pose.position;
                const auto &o = m->pose.pose.orientation;
                const auto &t = m->twist.twist;
                queue(m->header.stamp, [this, p = Vector3d(p.x, p.y, p.z), q = Quaterniond(o.w, o.x, o.y, o.z),
                                        v = Vector3d(t.linear.x, t.linear.y, t.linear.z),
                                        w = Vector3d(t.angular.x, t.angular.y, t.angular.z)](double stamp) {
                    if (estimator_->odometry(stamp, p, q, v, w))
                        RCLCPP_INFO(get_logger(), "State estimate (re)initialized from %s", odom_topic_.c_str());
                });
            });
        odom_topic_ = odom_topic;
        if (use_sensors_)
            subscribeSensors();
        linear_sub_ = create_subscription<ControllerCommand>(
            "controller/linear", 10, [this](const ControllerCommand &m) { linearCommand(m); });
        angular_sub_ = create_subscription<ControllerCommand>(
            "controller/angular", 10, [this](const ControllerCommand &m) { angularCommand(m); });
        ff_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "controller/FF_body_force", 10, [this](const geometry_msgs::msg::Twist &m) {
                reference_.feedforward << m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z;
            });
        motion_sub_ = create_subscription<std_msgs::msg::Bool>(
            "controller/motion_enabled", 10, [this](const std_msgs::msg::Bool &m) { motion_enabled_ = m.data; });
        kill_sub_ = create_subscription<std_msgs::msg::Bool>("state/kill", 10, [this](const std_msgs::msg::Bool &m) {
            if (m.data && !killed_) { // mirror the plant: queued commands dropped, thrust decays
                controller_->stopActuators();
                if (estimator_)
                    estimator_->stopActuators(get_clock()->now().seconds());
            }
            killed_ = m.data;
        });

        corner_radius_ = declare_parameter("path.corner_radius", 0.3);
        path_success_trust_ = declare_parameter("path.success_trust", 0.8);
        path_progress_timeout_ = declare_parameter("path.progress_timeout", 10.0);
        corner_angle_ = declare_parameter("path.corner_angle", 0.8);
        // Also done once the reference has arrived and the vehicle is this close
        // and slow, without waiting for full settle trust.
        finish_position_ = declare_parameter("path.finish_position", 0.10);
        finish_angle_ = declare_parameter("path.finish_angle", 0.10);
        finish_speed_ = declare_parameter("path.finish_speed", 0.10);
        finish_rate_ = declare_parameter("path.finish_rate", 0.15);
        if (!tf_buffer_) {
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }
        path_server_ = rclcpp_action::create_server<FollowPath>(
            this, "follow_path",
            [](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowPath::Goal>) {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](std::shared_ptr<PathGoal>) { return rclcpp_action::CancelResponse::ACCEPT; },
            [this](std::shared_ptr<PathGoal> goal) { startPath(goal); });

        timer_ = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(s.dt), [this] { tick(); });
    }

  private:
    // Sensor messages are applied in stamp order at the start of each tick.
    void queue(const builtin_interfaces::msg::Time &stamp, std::function<void(double)> apply) {
        pending_.push_back({rclcpp::Time(stamp, get_clock()->get_clock_type()).seconds(), std::move(apply)});
    }

    void subscribeSensors() {
        const auto qos = rclcpp::SensorDataQoS().keep_last(50);
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            declare_parameter<std::string>("imu_topic", "vectornav/imu"), qos, [this](const sensor_msgs::msg::Imu &m) {
                const auto &w = m.angular_velocity;
                const auto &o = m.orientation;
                const bool has_orientation = m.orientation_covariance[0] >= 0; // -1 means "not provided"
                queue(m.header.stamp, [this, w = Vector3d(w.x, w.y, w.z), q = Quaterniond(o.w, o.x, o.y, o.z),
                                       has_orientation](double t) {
                    estimator_->imuRate(t, w);
                    if (has_orientation && q.norm() > 0.5)
                        estimator_->imuOrientation(t, q);
                });
            });
        fog_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            declare_parameter<std::string>("fog_topic", "gyro/twist"), qos,
            [this](const geometry_msgs::msg::TwistWithCovarianceStamped &m) {
                queue(m.header.stamp, [this, r = m.twist.twist.angular.z](double t) { estimator_->fogRate(t, r); });
            });
        dvl_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            declare_parameter<std::string>("dvl_topic", "dvl_twist"), qos,
            [this](const geometry_msgs::msg::TwistWithCovarianceStamped &m) {
                const auto &v = m.twist.twist.linear;
                queue(m.header.stamp,
                      [this, v = Vector3d(v.x, v.y, v.z)](double t) { estimator_->dvlVelocity(t, v); });
            });
        depth_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            declare_parameter<std::string>("depth_topic", "depth/pose"), qos,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped &m) {
                const double z = m.pose.pose.position.z + depthFrameOffset(m.header.frame_id);
                queue(m.header.stamp, [this, z](double t) { estimator_->depth(t, z); });
            });
        state_pub_ = create_publisher<nav_msgs::msg::Odometry>("controller/mpc/state", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        mount_check_timer_ = create_wall_timer(2s, [this] { checkMountsAgainstTf(); });
    }

    // depth/pose is base_link z in its own frame (sim: map, vehicle: odom); shift
    // it into the odometry frame the EKF and the setpoints use.
    double depthFrameOffset(const std::string &frame) {
        if (!odom_ || frame.empty() || frame == odom_->header.frame_id)
            return 0;
        try {
            return tf_buffer_->lookupTransform(odom_->header.frame_id, frame, tf2::TimePointZero)
                .transform.translation.z;
        } catch (const tf2::TransformException &e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "depth frame %s -> %s unavailable: %s",
                                 frame.c_str(), odom_->header.frame_id.c_str(), e.what());
            return 0;
        }
    }

    // The estimator takes mounts from the vehicle YAML; the EKF uses TF. Warn once if they disagree.
    void checkMountsAgainstTf() {
        if (!odom_)
            return;
        const std::string base = odom_->child_frame_id.empty() ? robotName() + "/base_link" : odom_->child_frame_id;
        const std::string prefix = base.substr(0, base.rfind('/') + 1);
        const Vector3d r_base = controller_->model().baseLinkOffset();
        struct Check {
            std::string frame;
            Quaterniond rotation;
            std::optional<Vector3d> position; // relative to base_link
        };
        const std::vector<Check> checks{{prefix + "imu_link", mounts_.imu, std::nullopt},
                                        {prefix + "dvl_link", mounts_.dvl, mounts_.dvl_position - r_base}};
        for (const auto &c : checks) {
            geometry_msgs::msg::TransformStamped tf;
            try {
                tf = tf_buffer_->lookupTransform(base, c.frame, tf2::TimePointZero);
            } catch (const tf2::TransformException &) {
                return; // TF not up yet; retry
            }
            const auto &r = tf.transform.rotation;
            const auto &p = tf.transform.translation;
            const double angle = c.rotation.angularDistance(Quaterniond(r.w, r.x, r.y, r.z));
            const double offset = c.position ? (*c.position - Vector3d(p.x, p.y, p.z)).norm() : 0.;
            if (angle > 0.02 || offset > 0.01)
                RCLCPP_WARN(get_logger(), "%s mount in the vehicle YAML differs from TF by %.1f deg / %.1f cm",
                            c.frame.c_str(), angle * 180 / M_PI, offset * 100);
            else
                RCLCPP_INFO(get_logger(), "%s mount matches TF", c.frame.c_str());
        }
        mount_check_timer_->cancel();
    }

    std::string robotName() const {
        std::string ns = get_namespace();
        return ns.substr(ns.rfind('/') + 1);
    }

    void updateEstimate(const rclcpp::Time &now) {
        const double t_now = now.seconds();
        std::stable_sort(pending_.begin(), pending_.end(),
                         [](const Pending &a, const Pending &b) { return a.stamp < b.stamp; });
        for (auto &m : pending_) {
            // Unstamped or future-stamped messages count as now; very old ones are dropped.
            const double t = (m.stamp <= 0 || m.stamp > t_now) ? t_now : m.stamp;
            if (t_now - t < 1.0)
                m.apply(t);
        }
        pending_.clear();
        estimator_->propagate(t_now);

        const auto h = estimator_->health(t_now);
        const std::string stale = std::string(h.gyro ? "" : " gyro") + (h.fog ? "" : " fog") +
                                  (h.tilt ? "" : " imu-orientation") + (h.dvl ? "" : " dvl") +
                                  (h.depth ? "" : " depth");
        if (estimator_->initialized() && !stale.empty())
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Stale sensors, using EKF instead:%s",
                                 stale.c_str());
    }

    // CAN only on the real vehicle: the model must describe the hardware, the node
    // must be on wall time (a simulator on the vehicle computer never drives the
    // real thrusters), and the CAN interface must exist.
    void configureHardwareOutput(const std::string &mode) {
        const auto &hw = controller_->model().hardware();
        if (mode != "auto" && mode != "on" && mode != "off")
            throw std::invalid_argument("hardware_output must be auto, on or off");
        std::string why;
        if (mode == "off")
            why = "hardware_output is off";
        else if (!hw.present)
            why = "the model has no hardware section";
        else if (mode == "auto" && get_parameter("use_sim_time").as_bool())
            why = "running on simulation time";
        else if (!std::filesystem::exists("/sys/class/net/can0"))
            why = "can0 does not exist";
        can_enabled_ = why.empty();
        if (can_enabled_)
            RCLCPP_WARN(get_logger(), "Hardware output ENABLED: thruster RPM commands go to the ESCs over can0");
        else
            RCLCPP_INFO(get_logger(), "Hardware output disabled (%s); publishing thruster_forces only", why.c_str());
    }

    void sendRpm(const VectorXd &force) {
        const auto &hw = controller_->model().hardware();
        if (!hw.present || force.size() != 8)
            return;
        riptide_msgs2::msg::DshotCommand rpm;
        int16_t can[8];
        for (int i = 0; i < 8; ++i) {
            const double r = std::clamp(std::round(hw.forceToRpm(force[i])), -32768., 32767.);
            rpm.values[i] = can[i] = static_cast<int16_t>(r);
        }
        rpm_pub_->publish(rpm);
        if (can_enabled_ && send_thruster_cmd_canbus(can) != 0)
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "CAN thruster command failed");
    }

    void publishDisturbance(const rclcpp::Time &now) {
        if (!estimator_)
            return;
        const Vector6d d = estimator_->disturbance();
        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp = now;
        msg.header.frame_id = odom_ ? odom_->header.frame_id : "odom"; // force frame; torque is body frame
        msg.wrench.force.x = d[0];
        msg.wrench.force.y = d[1];
        msg.wrench.force.z = d[2];
        msg.wrench.torque.x = d[3];
        msg.wrench.torque.y = d[4];
        msg.wrench.torque.z = d[5];
        disturbance_pub_->publish(msg);
    }

    void publishTrust(const rclcpp::Time &now) {
        const double dt = last_trust_ ? (now - *last_trust_).seconds() : 0.;
        last_trust_ = now;
        double trust = 0;
        const bool odom_fresh = odom_ && (now - rclcpp::Time(odom_->header.stamp)).seconds() < odom_timeout_;
        if (killed_ || !odom_fresh || (estimator_ && !estimator_->initialized())) {
            settle_trust_.reset();
        } else {
            State13d x;
            if (estimator_) {
                x = estimator_->state();
            } else {
                const auto &p = odom_->pose.pose.position;
                const auto &t = odom_->twist.twist;
                x = controller_->model().fromBaseLink(Vector3d(p.x, p.y, p.z), *currentOrientation(),
                                                      Vector3d(t.linear.x, t.linear.y, t.linear.z),
                                                      Vector3d(t.angular.x, t.angular.y, t.angular.z));
            }
            Vector6d twist;
            twist << controller_->model().baseLinkVelocity(x), x.tail<3>();
            vehicle_ = VehicleSample{now, controller_->model().baseLinkPosition(x),
                                     Quaterniond(x[3], x[4], x[5], x[6]).normalized(), twist};
            trust = settle_trust_.update(std::max(dt, 0.), reference_, controller_->profile(), vehicle_->position,
                                         vehicle_->orientation, twist);
        }
        geometry_msgs::msg::Twist msg;
        msg.linear.x = msg.linear.y = msg.linear.z = trust;
        msg.angular.x = msg.angular.y = msg.angular.z = trust;
        trust_pub_->publish(msg);
    }

    void publishEstimate(const rclcpp::Time &now) {
        if (!estimator_->initialized() || state_pub_->get_subscription_count() == 0)
            return;
        const State13d &x = estimator_->state();
        const auto &model = controller_->model();
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = now;
        msg.header.frame_id = odom_ ? odom_->header.frame_id : "odom";
        msg.child_frame_id = odom_ ? odom_->child_frame_id : robotName() + "/base_link";
        const Vector3d p = model.baseLinkPosition(x), v = model.baseLinkVelocity(x);
        msg.pose.pose.position.x = p.x();
        msg.pose.pose.position.y = p.y();
        msg.pose.pose.position.z = p.z();
        msg.pose.pose.orientation.w = x[3];
        msg.pose.pose.orientation.x = x[4];
        msg.pose.pose.orientation.y = x[5];
        msg.pose.pose.orientation.z = x[6];
        msg.twist.twist.linear.x = v.x();
        msg.twist.twist.linear.y = v.y();
        msg.twist.twist.linear.z = v.z();
        msg.twist.twist.angular.x = x[10];
        msg.twist.twist.angular.y = x[11];
        msg.twist.twist.angular.z = x[12];
        state_pub_->publish(msg);
    }

    Vector3d vec3(const std::string &name, const Vector3d &fallback) {
        const auto v = declare_parameter<std::vector<double>>(name, {fallback.x(), fallback.y(), fallback.z()});
        if (v.size() != 3)
            throw std::invalid_argument(name + " needs three values");
        return Vector3d(v[0], v[1], v[2]);
    }

    std::optional<Quaterniond> currentOrientation() const {
        if (estimator_ && estimator_->initialized()) {
            const State13d &x = estimator_->state();
            return Quaterniond(x[3], x[4], x[5], x[6]).normalized();
        }
        if (!odom_)
            return std::nullopt;
        const auto &o = odom_->pose.pose.orientation;
        return Quaterniond(o.w, o.x, o.y, o.z).normalized();
    }

    // Waypoints (any TF frame) become a POSITION path in the odometry frame. The
    // result arrives once the vehicle has settled on the last one (trust).
    void startPath(const std::shared_ptr<PathGoal> &goal) {
        const auto reject = [&](uint8_t code, const std::string &why) {
            auto result = std::make_shared<FollowPath::Result>();
            result->error_code = code;
            result->error_msg = why;
            goal->abort(result);
            RCLCPP_WARN(get_logger(), "Path rejected: %s", why.c_str());
        };
        const auto &points = goal->get_goal()->path_points;
        const auto current = currentOrientation();
        if (points.empty())
            return reject(FollowPath::Result::BAD_WYPTS, "no waypoints");
        if (!odom_ || !current)
            return reject(FollowPath::Result::BAD_WYPTS, "no odometry yet");
        const std::string &frame = odom_->header.frame_id;
        std::vector<Waypoint> path;
        for (const auto &point : points) {
            if (point.header.frame_id.empty())
                return reject(FollowPath::Result::MISSING_FRAME_ID, "waypoint without a frame_id");
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            if (point.header.frame_id != frame) {
                try {
                    const auto t = tf_buffer_->lookupTransform(frame, point.header.frame_id, tf2::TimePointZero).transform;
                    transform.translate(Vector3d(t.translation.x, t.translation.y, t.translation.z));
                    transform.rotate(Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z).normalized());
                } catch (const tf2::TransformException &e) {
                    return reject(FollowPath::Result::BAD_WYPTS,
                                  "cannot transform " + point.header.frame_id + " to " + frame + ": " + e.what());
                }
            }
            const auto &p = point.pose.position;
            const auto &o = point.pose.orientation;
            const Quaterniond q(o.w, o.x, o.y, o.z);
            Waypoint w;
            w.position = transform * Vector3d(p.x, p.y, p.z);
            // An empty quaternion keeps the previous waypoint's attitude (the current one for the first).
            w.orientation = q.norm() > 1e-6 ? Quaterniond(Quaterniond(transform.rotation()) * q.normalized())
                            : path.empty()  ? *current
                                            : path.back().orientation;
            if (!w.position.allFinite() || !w.orientation.coeffs().allFinite())
                return reject(FollowPath::Result::BAD_WYPTS, "nonfinite waypoint");
            path.push_back(w);
        }
        finishPath(false, FollowPath::Result::NO_ERROR, "preempted by a new path");

        path_cumulative_.assign(1, 0.);
        path_cumulative_angle_.assign(1, 0.);
        Waypoint start{controller_->profile().position, controller_->profile().orientation};
        if (reference_.linear_mode != Mode::POSITION) { // the profile is seeded at the vehicle
            const auto &p = odom_->pose.pose.position;
            start.position = Vector3d(p.x, p.y, p.z);
        }
        if (reference_.angular_mode != Mode::POSITION)
            start.orientation = *current;
        Waypoint from = start;
        for (const Waypoint &w : path) {
            path_cumulative_.push_back(path_cumulative_.back() + (w.position - from.position).norm());
            path_cumulative_angle_.push_back(path_cumulative_angle_.back() +
                                             w.orientation.angularDistance(from.orientation));
            from = w;
        }
        path_cumulative_.erase(path_cumulative_.begin()); // length from the start to each waypoint
        path_cumulative_angle_.erase(path_cumulative_angle_.begin()); // rotation, likewise
        reference_.linear_mode = reference_.angular_mode = Mode::POSITION;
        reference_.path = path;
        reference_.position = path.back().position;
        reference_.orientation = path.back().orientation;
        reference_.corner_radius = corner_radius_;
        reference_.corner_angle = corner_angle_;
        reference_.path_start = start;
        path_goal_ = goal;
        settle_trust_.reset(); // "settled" must mean on the new path's end
        path_best_progress_ = path_best_progress_angle_ = 0;
        path_progress_time_ = get_clock()->now();
        RCLCPP_INFO(get_logger(), "Following a %zu-waypoint path (%.2f m) in %s", path.size(),
                    path_cumulative_.back(), frame.c_str());
    }

    // Ends the active path goal: success, abort, or cancel (holding where the
    // reference can stop). The last waypoint stays the setpoint otherwise.
    void finishPath(bool success, uint8_t code, const std::string &message, bool canceled = false) {
        reference_.path.clear();
        if (!path_goal_)
            return;
        auto result = std::make_shared<FollowPath::Result>();
        result->error_code = code;
        result->error_msg = message;
        if (path_goal_->is_active()) {
            if (canceled) {
                reference_.position = controller_->profile().stopPoint(controller_->settings().motion);
                reference_.orientation = controller_->profile().orientation;
                path_goal_->canceled(result);
            } else if (success) {
                path_goal_->succeed(result);
            } else {
                path_goal_->abort(result);
            }
        }
        if (success)
            RCLCPP_INFO(get_logger(), "Path complete");
        else
            RCLCPP_WARN(get_logger(), "Path %s: %s", canceled ? "canceled" : "aborted", message.c_str());
        path_goal_.reset();
    }

    void updatePath(const rclcpp::Time &now) {
        if (!path_goal_)
            return;
        if (!path_goal_->is_active()) {
            finishPath(false, FollowPath::Result::NO_ERROR, "goal no longer active");
            return;
        }
        if (path_goal_->is_canceling()) {
            finishPath(false, FollowPath::Result::NO_ERROR, "canceled", true);
            return;
        }
        // Progress of the reference along the path; the governor stalls it when
        // the vehicle cannot keep up.
        const std::size_t k = std::min(controller_->pathIndex(), path_cumulative_.size() - 1);
        const MotionProfile &profile = controller_->profile();
        const double progress =
            std::max(0., path_cumulative_[k] - (reference_.path[k].position - profile.position).norm());
        const double progress_angle = std::max(
            0., path_cumulative_angle_[k] - reference_.path[k].orientation.angularDistance(profile.orientation));
        auto feedback = std::make_shared<FollowPath::Feedback>();
        feedback->has_generated = true;
        feedback->current_prog = static_cast<float>(progress);
        feedback->expected_prog = static_cast<float>(path_cumulative_.back());
        path_goal_->publish_feedback(feedback);

        if (k + 1 == reference_.path.size()) {
            // The profile's last fraction of a millimetre can take seconds; near and
            // slow is arrived enough to judge the vehicle.
            const bool arrived = (profile.position - reference_.position).norm() < 0.01 &&
                                 profile.orientation.angularDistance(reference_.orientation) < 0.02 &&
                                 profile.velocity.norm() < 0.02 && profile.angular_velocity.norm() < 0.05;
            const bool close = vehicle_ && (now - vehicle_->stamp).seconds() < 0.5 &&
                               (vehicle_->position - reference_.position).norm() < finish_position_ &&
                               vehicle_->orientation.angularDistance(reference_.orientation) < finish_angle_ &&
                               vehicle_->twist.head<3>().norm() < finish_speed_ &&
                               vehicle_->twist.tail<3>().norm() < finish_rate_;
            if (settle_trust_.trust() >= path_success_trust_ || (arrived && close)) {
                finishPath(true, FollowPath::Result::NO_ERROR, "");
                return;
            }
            if (arrived) { // settling, not stuck; the caller's timeout bounds this
                path_progress_time_ = now;
                return;
            }
        }
        if (progress > path_best_progress_ + 0.05 || progress_angle > path_best_progress_angle_ + 0.05) {
            path_best_progress_ = std::max(progress, path_best_progress_);
            path_best_progress_angle_ = std::max(progress_angle, path_best_progress_angle_);
            path_progress_time_ = now;
        } else if ((now - *path_progress_time_).seconds() > path_progress_timeout_) {
            finishPath(false, FollowPath::Result::PROGRESS_FAIL, "no progress along the path");
        }
    }

    void linearCommand(const ControllerCommand &m) {
        finishPath(false, FollowPath::Result::NO_ERROR, "preempted by a controller/linear setpoint");
        reference_.linear_mode = static_cast<Mode>(m.mode);
        const Vector3d v(m.setpoint_vect.x, m.setpoint_vect.y, m.setpoint_vect.z);
        if (m.mode == ControllerCommand::POSITION)
            reference_.position = v;
        else if (m.mode == ControllerCommand::VELOCITY)
            reference_.linear_velocity = v;
    }

    void angularCommand(const ControllerCommand &m) {
        finishPath(false, FollowPath::Result::NO_ERROR, "preempted by a controller/angular setpoint");
        reference_.angular_mode = static_cast<Mode>(m.mode);
        if (m.mode == ControllerCommand::POSITION) {
            const Quaterniond q(m.setpoint_quat.w, m.setpoint_quat.x, m.setpoint_quat.y, m.setpoint_quat.z);
            if (q.norm() > 1e-6)
                reference_.orientation = q.normalized();
            else if (auto current = currentOrientation()) // empty quaternion: hold attitude
                reference_.orientation = *current;
        } else if (m.mode == ControllerCommand::VELOCITY) {
            reference_.angular_velocity = Vector3d(m.setpoint_vect.x, m.setpoint_vect.y, m.setpoint_vect.z);
        }
    }

    void publish(const VectorXd &command) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(command.data(), command.data() + command.size());
        thruster_pub_->publish(msg);
        sendRpm(command);
        controller_->issue(command);
        if (estimator_)
            estimator_->command(get_clock()->now().seconds(), command);
    }

    void tick() {
        const rclcpp::Time now = get_clock()->now();
        if (last_tick_) {
            const double elapsed = (now - *last_tick_).seconds();
            if (elapsed < 0) // clock went backwards (simulator restart): start clean
                controller_->resetActuators();
            else
                controller_->advance(elapsed);
        }
        last_tick_ = now;
        if (estimator_) {
            try {
                updateEstimate(now);
            } catch (const std::exception &e) {
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "State estimate invalid, re-seeding: %s",
                                      e.what());
                estimator_->invalidate();
            }
            publishEstimate(now);
            publishDisturbance(now);
        }

        publishTrust(now);
        const VectorXd zero = VectorXd::Zero(controller_->model().thrusterCount());
        const bool odom_fresh = odom_ && (now - rclcpp::Time(odom_->header.stamp)).seconds() < odom_timeout_;
        const bool state_ready = !estimator_ || estimator_->initialized();
        if (killed_ || (respect_motion_enabled_ && !motion_enabled_) || !odom_fresh || !state_ready) {
            if (!odom_fresh && !killed_)
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No fresh odometry; thrusters off");
            // The reference would keep running along the path with nobody following it.
            if (path_goal_)
                finishPath(false, FollowPath::Result::PROGRESS_FAIL,
                           killed_ ? "vehicle killed" : !odom_fresh ? "no fresh odometry" : "motion disabled");
            controller_->clearWarmStart();
            publish(zero);
            return;
        }

        State13d x;
        if (estimator_) {
            x = estimator_->state();
            controller_->setDisturbance(estimator_->disturbance());
        } else {
            const auto &p = odom_->pose.pose.position;
            const auto &t = odom_->twist.twist;
            x = controller_->model().fromBaseLink(Vector3d(p.x, p.y, p.z), *currentOrientation(),
                                                  Vector3d(t.linear.x, t.linear.y, t.linear.z),
                                                  Vector3d(t.angular.x, t.angular.y, t.angular.z));
        }
        MpcOutput out;
        try {
            out = controller_->compute(x, reference_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "MPC failed, thrusters off: %s", e.what());
            controller_->clearWarmStart();
            publish(zero);
            return;
        }
        if (!out.command.allFinite()) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "MPC produced a nonfinite command");
            controller_->clearWarmStart();
            publish(zero);
            return;
        }
        publish(out.command);
        updatePath(now);
        if (out.solve_ms > controller_->settings().dt * 1000)
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "MPC solve %.1f ms exceeds the control period",
                                 out.solve_ms);

        geometry_msgs::msg::Twist wrench;
        wrench.linear.x = out.wrench[0];
        wrench.linear.y = out.wrench[1];
        wrench.linear.z = out.wrench[2];
        wrench.angular.x = out.wrench[3];
        wrench.angular.y = out.wrench[4];
        wrench.angular.z = out.wrench[5];
        wrench_pub_->publish(wrench);
        std_msgs::msg::Float64 solve;
        solve.data = out.solve_ms;
        solve_pub_->publish(solve);

        geometry_msgs::msg::PoseStamped reference;
        reference.header.stamp = now;
        reference.header.frame_id = odom_->header.frame_id;
        const MotionProfile &profile = controller_->profile();
        reference.pose.position.x = profile.position.x();
        reference.pose.position.y = profile.position.y();
        reference.pose.position.z = profile.position.z();
        reference.pose.orientation.w = profile.orientation.w();
        reference.pose.orientation.x = profile.orientation.x();
        reference.pose.orientation.y = profile.orientation.y();
        reference.pose.orientation.z = profile.orientation.z();
        reference_pub_->publish(reference);

        if (path_pub_->get_subscription_count() > 0 && !out.prediction.empty()) {
            nav_msgs::msg::Path path;
            path.header.stamp = now;
            path.header.frame_id = odom_->header.frame_id;
            for (const State13d &s : out.prediction) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path.header;
                const Vector3d b = controller_->model().baseLinkPosition(s);
                pose.pose.position.x = b.x();
                pose.pose.position.y = b.y();
                pose.pose.position.z = b.z();
                pose.pose.orientation.w = s[3];
                pose.pose.orientation.x = s[4];
                pose.pose.orientation.y = s[5];
                pose.pose.orientation.z = s[6];
                path.poses.push_back(pose);
            }
            path_pub_->publish(path);
        }
    }

    struct Pending {
        double stamp;
        std::function<void(double)> apply;
    };

  public:
    ~MpcControllerNode() override {
        if (can_enabled_) { // leave the thrusters stopped
            int16_t zero[8] = {0};
            send_thruster_cmd_canbus(zero);
        }
    }

  private:
    bool can_enabled_ = false;
    std::optional<MpcController> controller_;
    std::optional<StateEstimator> estimator_;
    SensorMounts mounts_;
    std::vector<Pending> pending_;
    bool use_sensors_ = true;
    std::string odom_topic_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    Reference reference_;
    rclcpp_action::Server<FollowPath>::SharedPtr path_server_;
    std::shared_ptr<PathGoal> path_goal_;
    std::vector<double> path_cumulative_; // path length from the start to each waypoint
    double path_best_progress_ = 0, path_best_progress_angle_ = 0;
    std::vector<double> path_cumulative_angle_;
    double corner_angle_ = 0.8, finish_position_ = 0.10, finish_angle_ = 0.10, finish_speed_ = 0.10, finish_rate_ = 0.15;
    struct VehicleSample { // latest base_link state, for the path finish check
        rclcpp::Time stamp;
        Vector3d position;
        Quaterniond orientation;
        Vector6d twist;
    };
    std::optional<VehicleSample> vehicle_;
    std::optional<rclcpp::Time> path_progress_time_;
    double corner_radius_ = 0.3, path_success_trust_ = 0.8, path_progress_timeout_ = 10.;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    std::optional<rclcpp::Time> last_tick_, last_trust_;
    SettleTrust settle_trust_;
    bool killed_ = false, motion_enabled_ = true, respect_motion_enabled_ = true;
    double odom_timeout_ = 0.5;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thruster_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wrench_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr solve_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr reference_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr trust_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr disturbance_pub_;
    rclcpp::Publisher<riptide_msgs2::msg::DshotCommand>::SharedPtr rpm_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr fog_sub_, dvl_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr depth_sub_;
    rclcpp::TimerBase::SharedPtr mount_check_timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ControllerCommand>::SharedPtr linear_sub_, angular_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ff_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_sub_, kill_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace riptide_mpc

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<riptide_mpc::MpcControllerNode>());
    rclcpp::shutdown();
    return 0;
}
