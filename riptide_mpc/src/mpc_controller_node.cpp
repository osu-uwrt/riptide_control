// Drop-in replacement for the Simulink complete_controller, driven by an MPC
// whose prediction model is the simulator's Fossen model. Same interface:
//   in:  odometry/filtered, controller/linear, controller/angular (ControllerCommand),
//        controller/FF_body_force, controller/motion_enabled, state/kill,
//        follow_path (FollowPath action: lines and arcs with a heading mode, settle on the last)
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
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

extern "C" int send_thruster_cmd_canbus(int16_t cmds[8]); // riptide_controllers send_thruster_cmd_canbus.c

using namespace std::chrono_literals;
using riptide_msgs2::msg::ControllerCommand;
using FollowPath = riptide_msgs2::action::FollowPath;
using PathSegment = riptide_msgs2::msg::PathSegment;
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
        table_mode_ = declare_parameter("table_mode", false);
        // The ESC RPM loops settle ~100-190 rpm short of every target (2026-10-03 telemetry: slope ~1,
        // offset ~-140), so real thrust fell to ~0.7 of commanded at holding RPMs. Added to every
        // nonzero request in its direction of rotation.
        rpm_offset_ = declare_parameter("hardware.rpm_offset", 0.0);
        if (table_mode_)
            RCLCPP_WARN(get_logger(), "Table mode ON: thruster RPM capped at %.0f", kTableModeMaxRpm);

        const auto source = declare_parameter<std::string>("state_source", "sensors");
        if (source != "sensors" && source != "odometry")
            throw std::invalid_argument("state_source must be 'sensors' or 'odometry'");
        use_sensors_ = source == "sensors";

        FossenModel model = FossenModel::load(vehicle, hydro);
        vehicle_path_ = vehicle;
        // Thruster model (thruster_dynamics + thruster_efficiencies of the model file),
        // live-settable for thruster_sweep. Defaults are the file's values.
        thruster_file_ = model.actuatorParameters();
        {
            const ThrusterParameters &p = thruster_file_.front();
            std::vector<double> efficiencies;
            for (const auto &a : thruster_file_)
                efficiencies.push_back(a.efficiency);
            ThrusterModel &t = thruster_model_;
            t.delay = declare_parameter("thruster_model.delay", p.delay);
            t.rise = declare_parameter("thruster_model.rise_time_constant", p.rise);
            t.fall = declare_parameter("thruster_model.fall_time_constant", p.fall);
            t.slew = declare_parameter("thruster_model.slew_rate", p.slew);
            t.deadband = declare_parameter("thruster_model.force_deadband", p.deadband);
            // Multipliers on the model file's (possibly per-thruster) scales; 1 = the file's values.
            t.forward_scale = declare_parameter("thruster_model.forward_scale", 1.0);
            t.reverse_scale = declare_parameter("thruster_model.reverse_scale", 1.0);
            t.efficiencies = declare_parameter("thruster_model.efficiencies", efficiencies);
            t.startup = declare_parameter("thruster_model.startup_time_constant", p.startup);
            t.startup_force = declare_parameter("thruster_model.startup_force", p.startupForce);
            model.setActuatorParameters(thrusterParameters(t));
        }
        controller_.emplace(model, s);
        // Spin bias: a thrust pattern with zero net wrench (null space of the thruster matrix) that keeps
        // every thruster turning, because these ESCs take ~0.8 s to start a motor from ~0 rpm (2026-10-03).
        spin_pattern_ = spinPattern(controller_->model().thrusterMatrix());
        spin_bias_ = declare_parameter("hardware.spin_bias", 0.0);
        {
            std::ostringstream o;
            for (int i = 0; i < spin_pattern_.size(); ++i)
                o << (i ? " " : "") << std::fixed << std::setprecision(2) << spin_pattern_[i];
            RCLCPP_INFO(get_logger(), "Spin bias pattern [%s], bias %.1f N", o.str().c_str(), spin_bias_);
        }
        // Identification inputs (pool_identify): off unless identification.allow is set (pool_identify
        // sets it for its session only). See identificationInputs().
        ident_allow_ = declare_parameter("identification.allow", false);
        ident_timeout_ = declare_parameter("identification.timeout", 0.5);
        ident_max_bias_ = declare_parameter("identification.max_bias", 8.0);
        ident_max_fixed_ = declare_parameter("identification.max_fixed", 12.0); // N, largest commanded thruster
        ident_auto_off_ = declare_parameter("identification.auto_off", 10.0);   // s without inputs -> allow off
        if (ident_allow_)
            ident_allowed_at_ = get_clock()->now();
        {
            const MatrixXd T = controller_->model().thrusterMatrix();
            Eigen::JacobiSVD<MatrixXd> svd(T, Eigen::ComputeFullV);
            null_space_ = svd.matrixV().rightCols(std::max<Eigen::Index>(0, T.cols() - T.rows()));
        }
        // Motion limits, table mode and the thruster model can be changed live (e.g. by
        // the identification sequence or thruster_sweep).
        param_callback_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &params) {
            MotionLimits m = controller_->settings().motion;
            bool table_mode = table_mode_;
            ThrusterModel thrusters = thruster_model_;
            bool thrusters_changed = false;
            MpcSettings weights = controller_->settings();
            bool weights_changed = false;
            std::optional<FossenModel> new_model;
            std::optional<double> new_rpm_offset, new_spin_bias;
            std::optional<bool> new_ident_allow;
            std::string new_model_path;
            std::string weights_error;
            for (const auto &p : params) {
                const std::string &n = p.get_name();
                if (n == "hydrodynamics_config") { // live model swap (pool_identify iterations)
                    try {
                        new_model.emplace(FossenModel::load(vehicle_path_, p.as_string()));
                        new_model_path = p.as_string();
                    } catch (const std::exception &e) {
                        rcl_interfaces::msg::SetParametersResult r;
                        r.successful = false;
                        r.reason = "hydrodynamics_config " + p.as_string() + ": " + e.what();
                        return r;
                    }
                    continue;
                }
                if (n == "identification.allow" && p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                    new_ident_allow = p.as_bool();
                    continue;
                }
                if (n == "hardware.spin_bias" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    if (!(p.as_double() >= 0 && p.as_double() <= 6)) {
                        rcl_interfaces::msg::SetParametersResult r;
                        r.successful = false;
                        r.reason = "hardware.spin_bias must be 0..6 N";
                        return r;
                    }
                    new_spin_bias = p.as_double();
                    continue;
                }
                if (n == "hardware.rpm_offset" && p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    if (!(p.as_double() >= 0 && p.as_double() <= 500)) {
                        rcl_interfaces::msg::SetParametersResult r;
                        r.successful = false;
                        r.reason = "hardware.rpm_offset must be 0..500";
                        return r;
                    }
                    new_rpm_offset = p.as_double();
                    continue;
                }
                if (n == "table_mode" && p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                    table_mode = p.as_bool();
                    continue;
                }
                if (n.rfind("thruster_model.", 0) == 0) {
                    thrusters_changed = true;
                    if (n == "thruster_model.efficiencies") thrusters.efficiencies = p.as_double_array();
                    else if (n == "thruster_model.delay") thrusters.delay = p.as_double();
                    else if (n == "thruster_model.rise_time_constant") thrusters.rise = p.as_double();
                    else if (n == "thruster_model.fall_time_constant") thrusters.fall = p.as_double();
                    else if (n == "thruster_model.slew_rate") thrusters.slew = p.as_double();
                    else if (n == "thruster_model.force_deadband") thrusters.deadband = p.as_double();
                    else if (n == "thruster_model.forward_scale") thrusters.forward_scale = p.as_double();
                    else if (n == "thruster_model.reverse_scale") thrusters.reverse_scale = p.as_double();
                    else if (n == "thruster_model.startup_time_constant") thrusters.startup = p.as_double();
                    else if (n == "thruster_model.startup_force") thrusters.startup_force = p.as_double();
                    continue;
                }
                if (n.rfind("weights.", 0) == 0) {
                    weights_changed = true;
                    Vector3d *v3 = n == "weights.position"           ? &weights.q_position
                                   : n == "weights.attitude"         ? &weights.q_attitude
                                   : n == "weights.linear_velocity"  ? &weights.q_linear_velocity
                                   : n == "weights.angular_velocity" ? &weights.q_angular_velocity
                                   : n == "weights.linear_damping"   ? &weights.q_linear_damping
                                   : n == "weights.angular_damping"  ? &weights.q_angular_damping
                                                                     : nullptr;
                    if (v3) {
                        const auto a = p.as_double_array();
                        if (a.size() != 3 || !(a[0] >= 0 && a[1] >= 0 && a[2] >= 0))
                            weights_error = n + " needs three nonnegative values";
                        else
                            *v3 = Vector3d(a[0], a[1], a[2]);
                        continue;
                    }
                    double *scalar = n == "weights.terminal_factor" ? &weights.terminal_factor
                                     : n == "weights.thrust"        ? &weights.r_thrust
                                     : n == "weights.thrust_rate"   ? &weights.r_thrust_rate
                                                                    : nullptr;
                    if (scalar) {
                        const double v = p.as_double();
                        // r_thrust keeps the QP strictly convex.
                        if (!std::isfinite(v) || v < 0 || (scalar == &weights.r_thrust && !(v > 0)))
                            weights_error = n + (scalar == &weights.r_thrust ? " must be positive" : " must be nonnegative");
                        else
                            *scalar = v;
                    }
                    continue;
                }
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
            rcl_interfaces::msg::SetParametersResult r;
            // Validate the whole request before applying any of it.
            if (!weights_error.empty()) {
                r.successful = false;
                r.reason = weights_error;
                return r;
            }
            std::vector<ThrusterParameters> thruster_parameters;
            if (thrusters_changed) {
                try {
                    thruster_parameters = thrusterParameters(thrusters);
                    FossenModel check = controller_->model();
                    check.setActuatorParameters(thruster_parameters);
                } catch (const std::exception &e) {
                    r.successful = false;
                    r.reason = std::string("thruster_model: ") + e.what();
                    return r;
                }
            }
            controller_->setMotionLimits(m);
            if (new_ident_allow) {
                ident_allow_ = *new_ident_allow;
                ident_stamp_.reset();
                ident_allowed_at_ = get_clock()->now();
                RCLCPP_WARN(get_logger(), "Identification inputs %s", ident_allow_ ? "ALLOWED" : "off");
            }
            if (new_spin_bias) {
                spin_bias_ = *new_spin_bias;
                RCLCPP_WARN(get_logger(), "Thruster spin bias: %.1f N", spin_bias_);
            }
            if (new_rpm_offset) {
                rpm_offset_ = *new_rpm_offset;
                RCLCPP_WARN(get_logger(), "ESC rpm offset: %.0f rpm", rpm_offset_);
            }
            if (new_model) {
                // The new file's per-thruster thrust model (scales, efficiencies: e.g. identified per thruster)
                // comes with it; the live thruster_model.* multipliers and timings stay on top.
                const std::vector<ThrusterParameters> old_file = thruster_file_;
                ThrusterModel swapped = thrusters;
                swapped.efficiencies.clear();
                for (const auto &a : new_model->actuatorParameters())
                    swapped.efficiencies.push_back(a.efficiency);
                try {
                    thruster_file_ = new_model->actuatorParameters();
                    const auto parameters = thrusterParameters(swapped);
                    controller_->setModel(*new_model);
                    controller_->setActuatorParameters(parameters);
                    if (estimator_) {
                        estimator_->setModel(*new_model);
                        estimator_->setActuatorParameters(parameters, controller_->lastCommand());
                    }
                } catch (const std::exception &e) {
                    thruster_file_ = old_file;
                    r.successful = false;
                    r.reason = std::string("hydrodynamics_config: ") + e.what();
                    return r;
                }
                thruster_model_ = swapped;
                thrusters_changed = false; // applied with the new file
                RCLCPP_WARN(get_logger(), "MPC model swapped live: %s (thrusters: %s)", new_model_path.c_str(),
                            describe(swapped).c_str());
            }
            if (weights_changed) {
                controller_->setCostWeights(weights);
                const auto v = [](const Vector3d &x) {
                    std::ostringstream o;
                    o << "[" << x.x() << " " << x.y() << " " << x.z() << "]";
                    return o.str();
                };
                RCLCPP_WARN(get_logger(), "MPC weights: attitude %s angular_damping %s position %s linear_damping %s "
                            "thrust %g thrust_rate %g terminal %g", v(weights.q_attitude).c_str(),
                            v(weights.q_angular_damping).c_str(), v(weights.q_position).c_str(),
                            v(weights.q_linear_damping).c_str(), weights.r_thrust, weights.r_thrust_rate,
                            weights.terminal_factor);
            }
            if (table_mode != table_mode_) {
                table_mode_ = table_mode;
                RCLCPP_WARN(get_logger(), "Table mode %s", table_mode_ ? "ON: thruster RPM capped" : "OFF");
            }
            if (thrusters_changed) {
                controller_->setActuatorParameters(thruster_parameters);
                if (estimator_)
                    estimator_->setActuatorParameters(thruster_parameters, controller_->lastCommand());
                thruster_model_ = thrusters;
                RCLCPP_WARN(get_logger(), "Thruster model: %s", describe(thrusters).c_str());
            }
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
            // How long before its stamp a DVL velocity was measured; the estimate compares it with its
            // own state from that long ago. nortek_dvl already back-dates by the DVL's dt2 (~0.12 s).
            dvl_latency_ = declare_parameter("estimator.dvl_latency", 0.0);
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
        // The whole follow_path plan, latched for late viewers; empty once the path ends.
        planned_path_pub_ =
            create_publisher<nav_msgs::msg::Path>("controller/mpc/planned_path", rclcpp::QoS(1).transient_local());
        solve_pub_ = create_publisher<std_msgs::msg::Float64>("controller/mpc/solve_time_ms", 10);
        reference_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("controller/mpc/reference", 10);
        trust_pub_ = create_publisher<geometry_msgs::msg::Twist>("controller/scale/trust", 10);
        disturbance_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>("controller/mpc/disturbance", 10);
        rpm_pub_ = create_publisher<riptide_msgs2::msg::DshotCommand>("command/requested_rpm", 10);
        configureHardwareOutput(declare_parameter<std::string>("hardware_output", "auto"));

        // [8 fixed commands (NaN = free)] or [8 fixed, 8 bias], newtons, thruster order; see identificationInputs().
        ident_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "controller/identification/thrusters", 10, [this](const std_msgs::msg::Float64MultiArray &m) {
                const int n = controller_->model().thrusterCount();
                if (!ident_allow_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "Ignoring identification inputs: identification.allow is false");
                    return;
                }
                if (static_cast<int>(m.data.size()) != n && static_cast<int>(m.data.size()) != 2 * n) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                         "Identification inputs need %d or %d values, got %zu", n, 2 * n,
                                         m.data.size());
                    return;
                }
                ident_fixed_ = Eigen::Map<const VectorXd>(m.data.data(), n);
                for (int i = 0; i < n; ++i) // finite = commanded: never beyond max_fixed
                    if (std::isfinite(ident_fixed_[i]))
                        ident_fixed_[i] = std::clamp(ident_fixed_[i], -ident_max_fixed_, ident_max_fixed_);
                ident_bias_ = static_cast<int>(m.data.size()) == 2 * n ? VectorXd(Eigen::Map<const VectorXd>(m.data.data() + n, n))
                                                                       : VectorXd::Zero(n);
                if (!ident_bias_.allFinite())
                    ident_bias_.setZero();
                // Only the zero-wrench part of the bias, at most max_bias on any thruster.
                ident_bias_ = null_space_ * (null_space_.transpose() * ident_bias_);
                const double peak = ident_bias_.cwiseAbs().maxCoeff();
                if (peak > ident_max_bias_)
                    ident_bias_ *= ident_max_bias_ / peak;
                ident_stamp_ = get_clock()->now();
            });
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

        path_options_.corner_radius = declare_parameter("path.corner_radius", path_options_.corner_radius);
        path_options_.lateral_accel = declare_parameter("path.lateral_accel", path_options_.lateral_accel);
        path_options_.kink_speed = declare_parameter("path.kink_speed", path_options_.kink_speed);
        path_options_.turn_drift = declare_parameter("path.turn_drift", path_options_.turn_drift);
        path_options_.heading_blend = declare_parameter("path.heading_blend", path_options_.heading_blend);
        path_success_trust_ = declare_parameter("path.success_trust", 0.8);
        path_progress_timeout_ = declare_parameter("path.progress_timeout", 10.0);
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
    // age: how long before its stamp the message was measured (the DVL stamps on arrival).
    void queue(const builtin_interfaces::msg::Time &stamp, std::function<void(double)> apply, double age = 0) {
        pending_.push_back({rclcpp::Time(stamp, get_clock()->get_clock_type()).seconds() - age, std::move(apply)});
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
                      [this, v = Vector3d(v.x, v.y, v.z)](double t) { estimator_->dvlVelocity(t, v); },
                      dvl_latency_);
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

    // Table mode (complete_controller's thruster_solver_table_mode): out of the water, a soft
    // curve that saturates at kTableModeMaxRpm replaces the fitted force-to-RPM curve.
    // The thruster_model.* parameters, shared by every thruster but the efficiencies.
    struct ThrusterModel {
        double delay = .1, rise = .08, fall = .06, slew = 0, deadband = 0, forward_scale = 1, reverse_scale = 1;
        double startup = 0, startup_force = 0;
        std::vector<double> efficiencies;
    };

    // Per-thruster actuator parameters for `t`. A scale keeps the model file's
    // command limit (limit / scale): modelling a thruster as weaker lowers the
    // force it can realize, never raises the command the hardware is sent.
    std::vector<ThrusterParameters> thrusterParameters(const ThrusterModel &t) const {
        if (t.efficiencies.size() != thruster_file_.size())
            throw std::invalid_argument("efficiencies needs one value per thruster (" +
                                        std::to_string(thruster_file_.size()) + ")");
        if (!(t.forward_scale > 0) || !(t.reverse_scale > 0))
            throw std::invalid_argument("forward_scale and reverse_scale must be positive");
        std::vector<ThrusterParameters> out;
        for (std::size_t i = 0; i < thruster_file_.size(); ++i) {
            const ThrusterParameters &file = thruster_file_[i];
            ThrusterParameters p = file;
            p.delay = t.delay;
            p.rise = t.rise;
            p.fall = t.fall;
            p.slew = t.slew;
            p.deadband = t.deadband;
            p.forwardScale = file.forwardScale * t.forward_scale;
            p.reverseScale = file.reverseScale * t.reverse_scale;
            p.startup = t.startup;
            p.startupForce = t.startup_force;
            p.forwardLimit = file.forwardLimit * t.forward_scale;
            p.reverseLimit = file.reverseLimit * t.reverse_scale;
            p.efficiency = t.efficiencies[i];
            out.push_back(p);
        }
        return out;
    }

    static std::string describe(const ThrusterModel &t) {
        std::ostringstream s;
        s << "delay " << t.delay << " s, rise " << t.rise << " s, fall " << t.fall << " s, slew " << t.slew
          << " N/s, deadband " << t.deadband << " N, forward x" << t.forward_scale << ", reverse x" << t.reverse_scale
          << ", startup " << t.startup << " s below " << t.startup_force << " N"
          << ", efficiencies [";
        for (std::size_t i = 0; i < t.efficiencies.size(); ++i)
            s << (i ? " " : "") << t.efficiencies[i];
        s << "]";
        return s.str();
    }

    static double tableModeRpm(double force) {
        if (!std::isfinite(force))
            return 0;
        return std::copysign(kTableModeMaxRpm * (1 - std::exp(-0.5 * std::abs(force))), force);
    }

    void sendRpm(const VectorXd &force) {
        const auto &hw = controller_->model().hardware();
        if (!hw.present || force.size() != 8)
            return;
        riptide_msgs2::msg::DshotCommand rpm;
        int16_t can[8];
        for (int i = 0; i < 8; ++i) {
            const double f = force[i];
            double target = table_mode_ ? tableModeRpm(f) : hw.forceToRpm(f);
            if (!table_mode_ && target != 0)
                target += std::copysign(rpm_offset_, target);
            const double r = std::clamp(std::round(target), -32768., 32767.);
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
            Reference settle = reference_;
            settle.orientation = endOrientation();
            trust = settle_trust_.update(std::max(dt, 0.), settle, controller_->profile(), vehicle_->position,
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

    // `frame` -> odometry frame, latest; throws tf2::TransformException.
    Eigen::Isometry3d toOdom(const std::string &frame) const {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        if (frame != odom_->header.frame_id) {
            const auto t = tf_buffer_->lookupTransform(odom_->header.frame_id, frame, tf2::TimePointZero).transform;
            transform.translate(Vector3d(t.translation.x, t.translation.y, t.translation.z));
            transform.rotate(Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z).normalized());
        }
        return transform;
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
        const auto &segments = goal->get_goal()->segments;
        const auto current = currentOrientation();
        if (points.empty())
            return reject(FollowPath::Result::BAD_WYPTS, "no waypoints");
        if (!segments.empty() && segments.size() != points.size())
            return reject(FollowPath::Result::BAD_WYPTS, "segments must be empty or one per path point");
        if (!odom_ || !current)
            return reject(FollowPath::Result::BAD_WYPTS, "no odometry yet");
        const std::string &frame = odom_->header.frame_id;
        std::vector<PathPoint> path;
        std::vector<LookTarget> look_targets;
        std::vector<Vector3d> look_positions;
        for (std::size_t i = 0; i < points.size(); ++i) {
            const auto &point = points[i];
            if (point.header.frame_id.empty())
                return reject(FollowPath::Result::MISSING_FRAME_ID, "waypoint without a frame_id");
            Eigen::Isometry3d transform;
            try {
                transform = toOdom(point.header.frame_id);
            } catch (const tf2::TransformException &e) {
                return reject(FollowPath::Result::BAD_WYPTS,
                              "cannot transform " + point.header.frame_id + " to " + frame + ": " + e.what());
            }
            const auto &p = point.pose.position;
            const auto &o = point.pose.orientation;
            const Quaterniond q(o.w, o.x, o.y, o.z);
            PathPoint w;
            w.position = transform * Vector3d(p.x, p.y, p.z);
            // An empty quaternion keeps the previous waypoint's attitude (the current one for the first).
            w.orientation = q.norm() > 1e-6 ? Quaterniond(Quaterniond(transform.rotation()) * q.normalized())
                            : path.empty()  ? *current
                                            : path.back().orientation;
            if (!segments.empty()) {
                const auto &g = segments[i];
                if (g.shape > PathSegment::ARC || g.heading > PathSegment::HEADING_LOOK_AT)
                    return reject(FollowPath::Result::BAD_WYPTS, "unknown segment shape or heading");
                // Turning about the frame's +z: the opposite way round if that points down here.
                const double handed = (transform.rotation() * Vector3d::UnitZ()).z() < 0 ? -1. : 1.;
                w.shape = static_cast<PathShape>(g.shape);
                w.center = transform * Vector3d(g.center.x, g.center.y, g.center.z);
                w.sweep = handed * g.sweep;
                w.heading = static_cast<PathHeading>(g.heading);
                w.look_at = transform * Vector3d(g.look_at.x, g.look_at.y, g.look_at.z);
                if (!g.look_at_frame.empty()) { // followed live; planned from where it is now
                    if (g.heading != PathSegment::HEADING_LOOK_AT)
                        return reject(FollowPath::Result::BAD_WYPTS, "look_at_frame needs HEADING_LOOK_AT");
                    const LookTarget target{g.look_at_frame, Vector3d(g.look_at.x, g.look_at.y, g.look_at.z)};
                    try {
                        w.look_at = toOdom(target.frame) * target.point;
                    } catch (const tf2::TransformException &e) {
                        return reject(FollowPath::Result::BAD_WYPTS,
                                      "cannot transform look_at_frame " + target.frame + " to " + frame + ": " + e.what());
                    }
                    w.look_target = static_cast<int>(look_targets.size());
                    look_targets.push_back(target);
                    look_positions.push_back(w.look_at);
                }
                w.yaw_offset = g.yaw_offset;
                w.spin = g.spin;
                w.spin_rate = g.spin_rate;
            }
            path.push_back(w);
        }

        Waypoint start{controller_->profile().position, controller_->profile().orientation};
        if (reference_.linear_mode != Mode::POSITION) { // the profile is seeded at the vehicle
            const auto &p = odom_->pose.pose.position;
            start.position = Vector3d(p.x, p.y, p.z);
        }
        if (reference_.angular_mode != Mode::POSITION)
            start.orientation = *current;
        std::shared_ptr<const PathPlan> plan;
        try {
            plan = PathPlan::build(start, path, controller_->settings().motion, path_options_);
        } catch (const std::exception &e) {
            return reject(FollowPath::Result::BAD_WYPTS, e.what());
        }
        finishPath(false, FollowPath::Result::NO_ERROR, "preempted by a new path");

        const Waypoint end = plan->end();
        reference_.linear_mode = reference_.angular_mode = Mode::POSITION;
        reference_.path = plan;
        reference_.look_targets = look_positions;
        look_targets_ = look_targets;
        reference_.position = end.position;
        reference_.orientation = end.orientation;
        path_goal_ = goal;
        settle_trust_.reset(); // "settled" must mean on the new path's end
        path_best_progress_ = 0;
        path_progress_time_ = get_clock()->now();
        RCLCPP_INFO(get_logger(), "Following a %zu-point path (%.2f m) in %s%s", path.size(), plan->length(),
                    frame.c_str(), look_targets.empty() ? "" : ", following look_at frames");
        publishPlannedPath(plan.get(), frame);
    }

    // The plan sampled every 10 cm (turns in place included, so their headings show).
    void publishPlannedPath(const PathPlan *plan, const std::string &frame) {
        nav_msgs::msg::Path msg;
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = frame;
        if (plan) {
            const int n = std::max(1, static_cast<int>(std::ceil(plan->length() / 0.1)));
            for (int i = 0; i <= n; ++i) {
                const double s = plan->length() * i / n;
                const Vector3d p = plan->position(s);
                const Quaterniond q = plan->orientation(s);
                geometry_msgs::msg::PoseStamped pose;
                pose.header = msg.header;
                pose.pose.position.x = p.x();
                pose.pose.position.y = p.y();
                pose.pose.position.z = p.z();
                pose.pose.orientation.w = q.w();
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
                msg.poses.push_back(pose);
            }
        }
        planned_path_pub_->publish(msg);
    }

    // Ends the active path goal: success, abort, or cancel (holding where the
    // reference can stop). The last waypoint stays the setpoint otherwise.
    void finishPath(bool success, uint8_t code, const std::string &message, bool canceled = false) {
        reference_.orientation = endOrientation(); // keep facing the look targets' last direction
        reference_.path.reset();
        reference_.look_targets.clear();
        look_targets_.clear();
        if (!path_goal_)
            return;
        publishPlannedPath(nullptr, odom_ ? odom_->header.frame_id : "");
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
        // Progress of the reference along the path (turning in place counts); the
        // governor stalls it when the vehicle cannot keep up.
        const PathProgress &progress = controller_->pathProgress();
        const MotionProfile &profile = controller_->profile();
        auto feedback = std::make_shared<FollowPath::Feedback>();
        feedback->has_generated = true;
        feedback->current_prog = static_cast<float>(progress.s);
        feedback->expected_prog = static_cast<float>(reference_.path->length());
        path_goal_->publish_feedback(feedback);

        if (progress.s >= reference_.path->length() - 1e-3) {
            // The profile's last fraction of a millimetre can take seconds; near and
            // slow is arrived enough to judge the vehicle.
            const Quaterniond end = endOrientation();
            const bool arrived = (profile.position - reference_.position).norm() < 0.01 &&
                                 profile.orientation.angularDistance(end) < 0.02 &&
                                 profile.velocity.norm() < 0.02 && profile.angular_velocity.norm() < 0.05;
            const bool close = vehicle_ && (now - vehicle_->stamp).seconds() < 0.5 &&
                               (vehicle_->position - reference_.position).norm() < finish_position_ &&
                               vehicle_->orientation.angularDistance(end) < finish_angle_ &&
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
        if (progress.s > path_best_progress_ + 0.05) {
            path_best_progress_ = progress.s;
            path_progress_time_ = now;
        } else if ((now - *path_progress_time_).seconds() > path_progress_timeout_) {
            finishPath(false, FollowPath::Result::PROGRESS_FAIL, "no progress along the path");
        }
    }

    // The setpoint attitude: a path's end, turned toward its moving look targets.
    Quaterniond endOrientation() const {
        return reference_.path ? Quaterniond(controller_->lookOffset() * reference_.orientation)
                               : reference_.orientation;
    }

    // Moves the path's look targets to where their frames are now; a frame that
    // stops resolving keeps its last position.
    void updateLookTargets() {
        if (!reference_.path)
            return;
        for (std::size_t i = 0; i < look_targets_.size(); ++i) {
            try {
                reference_.look_targets[i] = toOdom(look_targets_[i].frame) * look_targets_[i].point;
            } catch (const tf2::TransformException &e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "look_at_frame %s unavailable, holding: %s",
                                     look_targets_[i].frame.c_str(), e.what());
            }
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

    // Zero-wrench thrust pattern with every entry as large as possible (largest = 1); empty if none.
    static VectorXd spinPattern(const MatrixXd &T) {
        if (T.cols() - T.rows() < 1)
            return VectorXd();
        Eigen::JacobiSVD<MatrixXd> svd(T, Eigen::ComputeFullV);
        const MatrixXd N = svd.matrixV().rightCols(T.cols() - T.rows());
        VectorXd best;
        double best_min = 0;
        for (int k = 0; k < 3600; ++k) { // a line through the null space (dim 2 on Talos)
            const double a = M_PI * k / 3600;
            VectorXd n = N.col(0) * std::cos(a);
            if (N.cols() > 1)
                n += N.col(1) * std::sin(a);
            n /= n.cwiseAbs().maxCoeff();
            if (n.cwiseAbs().minCoeff() > best_min) {
                best_min = n.cwiseAbs().minCoeff();
                best = n;
            }
        }
        return best_min > 0.3 ? best : VectorXd();
    }

    // `command` is what the controller and estimator know; the thrusters additionally get the spin bias,
    // which adds no wrench, so their models stay valid without it.
    void publish(const VectorXd &command, bool active = false) {
        VectorXd hw = command;
        // No spin bias under identification inputs: a release must be thrusters off, a ramp exactly its command.
        if (active && spin_bias_ > 0 && spin_pattern_.size() == command.size() &&
            controller_->fixedCommands().size() == 0)
            hw = controller_->model().limitTotalThrust(command + spin_bias_ * spin_pattern_);
        std_msgs::msg::Float32MultiArray msg;
        msg.data.assign(hw.data(), hw.data() + hw.size());
        thruster_pub_->publish(msg);
        sendRpm(hw);
        controller_->issue(command);
        if (estimator_)
            estimator_->command(get_clock()->now().seconds(), command);
    }

    // Applies (or clears) the identification inputs: only while allowed, fresh (identification.timeout) and
    // the vehicle is enabled, so a crashed or stopped pool_identify leaves the MPC in normal control.
    void identificationInputs(const rclcpp::Time &now, bool enabled) {
        // Allowed but nobody sending (pool_identify crashed or forgot): turn the permission off again.
        if (ident_allow_ && ident_auto_off_ > 0 && ident_allowed_at_ &&
            (now - (ident_stamp_ && *ident_stamp_ > *ident_allowed_at_ ? *ident_stamp_ : *ident_allowed_at_)).seconds() >
                ident_auto_off_) {
            RCLCPP_WARN(get_logger(), "No identification inputs for %.0f s: identification.allow off", ident_auto_off_);
            ident_allow_ = false;
            ident_auto_off_pending_ = true; // the parameter itself is reset from the timer below
        }
        const bool active = enabled && ident_allow_ && ident_stamp_ && (now - *ident_stamp_).seconds() <= ident_timeout_;
        std::string state = "normal control";
        if (active) {
            int fixed = 0, zero = 0, which = -1;
            for (int i = 0; i < ident_fixed_.size(); ++i)
                if (std::isfinite(ident_fixed_[i])) {
                    ++fixed;
                    zero += ident_fixed_[i] == 0;
                    which = i;
                }
            if (fixed == ident_fixed_.size() && zero == fixed)
                state = "released (all thrusters off)";
            else if (fixed == 1)
                state = "thruster " + std::to_string(which) + " commanded by identification";
            else if (fixed > 0)
                state = std::to_string(fixed) + " thrusters commanded by identification";
            else if (ident_bias_.norm() > 0)
                state = "null-space bias";
            else
                state = "identification (no inputs)";
            controller_->setIdentificationInputs(ident_fixed_, ident_bias_);
        } else if (controller_->fixedCommands().size() != 0 || ident_state_ != state) {
            controller_->setIdentificationInputs(VectorXd(), VectorXd());
        }
        // A release (every thruster commanded to zero): the free float must not be learned as a disturbance.
        const bool released = active && ident_fixed_.size() > 0 && ident_fixed_.allFinite() && ident_fixed_.isZero();
        if (estimator_)
            estimator_->pauseDisturbanceLearning(released);
        if (state != ident_state_) {
            RCLCPP_WARN(get_logger(), "Identification: %s", state.c_str());
            ident_state_ = state;
        }
        if (ident_auto_off_pending_) {
            ident_auto_off_pending_ = false;
            set_parameter(rclcpp::Parameter("identification.allow", false));
        }
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
            identificationInputs(now, false);
            publish(zero);
            return;
        }
        identificationInputs(now, true);

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
        updateLookTargets();
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
        publish(out.command, out.active);
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
    static constexpr double kTableModeMaxRpm = 500;
    bool can_enabled_ = false;
    bool table_mode_ = false;
    std::string vehicle_path_;
    double dvl_latency_ = 0.0;
    double rpm_offset_ = 0, spin_bias_ = 0;
    bool ident_allow_ = false;
    double ident_timeout_ = 0.5, ident_max_bias_ = 8.0, ident_max_fixed_ = 12.0, ident_auto_off_ = 10.0;
    bool ident_auto_off_pending_ = false;
    std::optional<rclcpp::Time> ident_stamp_, ident_allowed_at_;
    VectorXd ident_fixed_, ident_bias_;
    std::string ident_state_ = "normal control";
    MatrixXd null_space_; // orthonormal basis of the thruster matrix's null space
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ident_sub_;
    VectorXd spin_pattern_;
    std::vector<ThrusterParameters> thruster_file_; // the model file's thruster model
    ThrusterModel thruster_model_;                  // live thruster_model.* values
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
    struct LookTarget { // a PathSegment look_at in its look_at_frame
        std::string frame;
        Vector3d point;
    };
    std::vector<LookTarget> look_targets_; // Reference::look_targets' sources
    double path_best_progress_ = 0;
    PathOptions path_options_;
    double finish_position_ = 0.10, finish_angle_ = 0.10, finish_speed_ = 0.10, finish_rate_ = 0.15;
    struct VehicleSample { // latest base_link state, for the path finish check
        rclcpp::Time stamp;
        Vector3d position;
        Quaterniond orientation;
        Vector6d twist;
    };
    std::optional<VehicleSample> vehicle_;
    std::optional<rclcpp::Time> path_progress_time_;
    double path_success_trust_ = 0.8, path_progress_timeout_ = 10.;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    std::optional<rclcpp::Time> last_tick_, last_trust_;
    SettleTrust settle_trust_;
    bool killed_ = false, motion_enabled_ = true, respect_motion_enabled_ = true;
    double odom_timeout_ = 0.5;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thruster_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr wrench_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_, planned_path_pub_;
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
