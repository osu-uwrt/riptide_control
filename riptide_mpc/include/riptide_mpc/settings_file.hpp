#pragma once

// Reads the MPC and estimator settings from the node's ROS parameter file
// (config/mpc.yaml), so offline tools run with exactly what the node would.
#include "riptide_mpc/mpc.hpp"
#include "riptide_mpc/state_estimator.hpp"

#include <yaml-cpp/yaml.h>

#include <string>

namespace riptide_mpc {
inline void loadSettingsFile(const std::string &path, MpcSettings &s, EstimatorSettings &e) {
    YAML::Node p;
    for (const auto &node : YAML::LoadFile(path))
        if (node.second["ros__parameters"])
            p = node.second["ros__parameters"];
    if (!p)
        return;
    auto num = [](const YAML::Node &n, double &v) {
        if (n)
            v = n.as<double>();
    };
    auto vec3 = [](const YAML::Node &n, Vector3d &v) {
        if (n)
            v = Vector3d(n[0].as<double>(), n[1].as<double>(), n[2].as<double>());
    };
    num(p["control_period"], s.dt);
    if (p["horizon_steps"])
        s.horizon = p["horizon_steps"].as<int>();
    num(p["model_step"], s.model_step);
    num(p["linearization_step"], s.linearization_step);
    if (const auto m = p["motion"]) {
        if (m["profile"])
            s.profile_motion = m["profile"].as<bool>();
        num(m["linear_speed"], s.motion.linear_speed);
        num(m["linear_accel"], s.motion.linear_accel);
        num(m["linear_jerk"], s.motion.linear_jerk);
        num(m["linear_speed_vertical"], s.motion.linear_speed_vertical);
        num(m["linear_accel_vertical"], s.motion.linear_accel_vertical);
        num(m["linear_jerk_vertical"], s.motion.linear_jerk_vertical);
        num(m["angular_speed"], s.motion.angular_speed);
        num(m["angular_accel"], s.motion.angular_accel);
        num(m["angular_jerk"], s.motion.angular_jerk);
        num(m["max_reference_lag"], s.max_reference_lag);
        num(m["max_reference_lag_angle"], s.max_reference_lag_angle);
        num(m["governor_lag"], s.governor_lag);
        num(m["governor_stop_lag"], s.governor_stop_lag);
        num(m["governor_angle"], s.governor_angle);
        num(m["governor_stop_angle"], s.governor_stop_angle);
    }
    if (const auto w = p["weights"]) {
        vec3(w["position"], s.q_position);
        vec3(w["attitude"], s.q_attitude);
        vec3(w["linear_velocity"], s.q_linear_velocity);
        vec3(w["angular_velocity"], s.q_angular_velocity);
        vec3(w["linear_damping"], s.q_linear_damping);
        vec3(w["angular_damping"], s.q_angular_damping);
        num(w["terminal_factor"], s.terminal_factor);
        num(w["thrust"], s.r_thrust);
        num(w["thrust_rate"], s.r_thrust_rate);
    }
    if (const auto x = p["estimator"]) {
        num(x["gyro_gain"], e.gyro_gain);
        num(x["fog_gain"], e.fog_gain);
        num(x["dvl_gain"], e.dvl_gain);
        num(x["tilt_time_constant"], e.tilt_time_constant);
        num(x["depth_time_constant"], e.depth_time_constant);
        num(x["heading_time_constant"], e.heading_time_constant);
        num(x["horizontal_time_constant"], e.horizontal_time_constant);
        if (x["estimate_disturbance"])
            e.estimate_disturbance = x["estimate_disturbance"].as<bool>();
        num(x["disturbance_force_time_constant"], e.disturbance_force_time_constant);
        num(x["disturbance_torque_time_constant"], e.disturbance_torque_time_constant);
        num(x["max_disturbance_force"], e.max_disturbance_force);
        num(x["max_disturbance_torque"], e.max_disturbance_torque);
        num(x["disturbance_gate_speed"], e.disturbance_gate_speed);
        num(x["disturbance_gate_rate"], e.disturbance_gate_rate);
    }
    e.model_step = s.model_step;
}
} // namespace riptide_mpc
