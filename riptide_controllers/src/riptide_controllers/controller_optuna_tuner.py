#! /usr/bin/env python3
#
# Optuna controller-gain tuner for the Riptide controller (PID and/or SMC), all axes at once.
#
# Each trial applies a candidate set of gains to the running complete_controller (the x1e6
# integer-array encoding the overseer uses) and flies a BOX TRAJECTORY around the start pose --
# a loop of waypoints that exercises all 6 DOF (+/-x, +/-y, +/-z corners plus roll/pitch/yaw
# offsets). The objective is the total 6-DOF tracking cost over the loop (normalized ITAE +
# oscillation/target-crossings + late residual), summed over the tuned axes. Optuna minimizes it.
#
# Which axis is tuned in which controller is configurable:
#   pid_axes / smc_axes     -> axis indices [x,y,z,roll,pitch,yaw] = 0..5 (strings: "0,1,2,3,4,5")
#   pid_params / smc_params -> which gains to search (PID: p,i,d ; SMC: eta0,eta1,lambda)
# Default tunes ALL axes in SMC (complete_controller is SMC-based); add pid_axes to also tune PID.
#
# Safety: recovery ALWAYS uses the original (known-stable) gains; the candidate flies only the
# trajectory, so an unstable candidate is penalized (abort), not catastrophic. A hard failure asks
# the RViz ControlPanel to kill via <ns>/ff_tuner/halt.
#
# Run:  ros2 launch riptide_controllers2 controller_optuna_tuner.launch.py robot:=talos

import os
import math
import time
import copy
import threading

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_system_default

from ament_index_python import get_package_share_directory
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Quaternion
from riptide_msgs2.msg import ControllerCommand
from std_msgs.msg import Empty as EmptyMsg

from gain_apply import GainApplier

from queue import Queue, Empty

import optuna

MODE_DISABLED = ControllerCommand.DISABLED
MODE_FEEDFORWARD = ControllerCommand.FEEDFORWARD
MODE_POSITION = ControllerCommand.POSITION

AXIS_NAMES = ["x", "y", "z", "roll", "pitch", "yaw"]

GAIN_PARAMS = {
    "pid": {
        "p": ("controller__PID__p_gains", ["PID", "p_gains"]),
        "i": ("controller__PID__i_gains", ["PID", "i_gains"]),
        "d": ("controller__PID__d_gains", ["PID", "d_gains"]),
    },
    "smc": {
        "eta0": ("controller__SMC__SMC_params__eta_order_0", ["SMC", "SMC_params", "eta_order_0"]),
        "eta1": ("controller__SMC__SMC_params__eta_order_1", ["SMC", "SMC_params", "eta_order_1"]),
        "lambda": ("controller__SMC__SMC_params__lambda", ["SMC", "SMC_params", "lambda"]),
    },
}

ABORT_PENALTY = 50.0     # per-remaining-waypoint cost when a candidate trips a safety limit


class HardHaltException(Exception):
    pass


class ControllerOptunaTuner(Node):

    def __init__(self):
        super().__init__("controller_optuna_tuner")
        cb = ReentrantCallbackGroup()

        # ---- what to tune (strings so launch/-p overrides are painless) -----
        self.robot = self.declare_parameter("robot", "talos").value
        self.vehicle_config = self.declare_parameter("vehicle_config", "").value
        # Per active_force_mask [2,2,2,1,1,1]: SMC = x,y,z (value 2); PID = roll,pitch,yaw (value 1).
        self.pid_axes = self.parse_ints(self.declare_parameter("pid_axes", "3,4,5").value)
        self.smc_axes = self.parse_ints(self.declare_parameter("smc_axes", "0,1,2").value)
        self.pid_params = self.parse_strs(self.declare_parameter("pid_params", "p,i,d").value)
        self.smc_params = self.parse_strs(self.declare_parameter("smc_params", "eta0,eta1,lambda").value)

        # search bounds: multiplicative around current for nonzero gains (log), absolute for zeros
        self.gain_lo_mult = self.declare_parameter("gain_lo_mult", 0.3).value
        self.gain_hi_mult = self.declare_parameter("gain_hi_mult", 3.0).value
        self.zero_gain_max = self.declare_parameter("zero_gain_max", 1.0).value

        # ---- optimizer -----------------------------------------------------
        self.sampler_name = self.declare_parameter("sampler", "cmaes").value   # cmaes | tpe
        self.n_trials = self.declare_parameter("n_trials", 120).value
        self.early_stop_patience = self.declare_parameter("early_stop_patience", 30).value

        # ---- box trajectory ------------------------------------------------
        self.box_xy = self.declare_parameter("box_xy", 0.5).value        # m
        self.box_z = self.declare_parameter("box_z", 0.4).value          # m
        self.max_z = self.declare_parameter("max_z", -0.5).value         # never command shallower
        # roll/pitch are NEVER commanded (kept level); these are the error-normalization scale for
        # their regulation cost -- i.e. "a roll/pitch deviation of this many rad counts as 1 unit"
        self.roll_amp = self.declare_parameter("roll_amp", 0.15).value   # rad (regulation scale)
        self.pitch_amp = self.declare_parameter("pitch_amp", 0.15).value # rad (regulation scale)
        self.yaw_amp = self.declare_parameter("yaw_amp", 1.5708).value   # rad
        # Each segment's time budget is TIED TO THE DISTANCE it must travel, so far waypoints get
        # proportionally more time. This stops the optimizer from winning just by being fast (which
        # trades stability for speed): with enough time to reach, the cost differentiates on quality.
        #   budget = max(lin_dist / cruise_speed, ang_dist / yaw_rate) + settle_margin  (clamped)
        self.cruise_speed = self.declare_parameter("cruise_speed", 0.3).value    # m/s nominal
        self.yaw_rate = self.declare_parameter("yaw_rate", 0.5).value            # rad/s nominal
        self.settle_margin = self.declare_parameter("settle_margin", 2.0).value  # s to settle after
        self.min_segment_time = self.declare_parameter("min_segment_time", 2.0).value
        self.max_segment_time = self.declare_parameter("max_segment_time", 10.0).value
        # advance early once REACHED (a fast-but-stable controller still advances early -> less time)
        self.reach_frac = self.declare_parameter("reach_frac", 0.15).value  # within this*ref = reached
        self.reach_hold = self.declare_parameter("reach_hold", 0.3).value   # s held to count reached

        # ---- objective weights ---------------------------------------------
        self.w_itae = self.declare_parameter("w_itae", 1.0).value
        self.w_osc = self.declare_parameter("w_osc", 0.1).value          # per error target-crossing
        self.w_settle = self.declare_parameter("w_settle", 2.0).value    # late-window residual
        self.settle_frac = self.declare_parameter("settle_frac", 0.05).value   # band (fraction of ref)
        self.late_frac = self.declare_parameter("late_frac", 0.3).value

        # ---- settle / recovery / safety ------------------------------------
        self.settle_pos = self.declare_parameter("settle_pos", 0.15).value
        self.settle_ang = self.declare_parameter("settle_ang", 0.15).value
        self.settle_lin_vel = self.declare_parameter("settle_lin_vel", 0.12).value
        self.settle_ang_vel = self.declare_parameter("settle_ang_vel", 0.25).value
        self.settle_hold = self.declare_parameter("settle_hold", 1.0).value
        self.recovery_timeout = self.declare_parameter("recovery_timeout", 30.0).value
        self.lin_margin = self.declare_parameter("lin_margin", 1.0).value   # m past target -> abort
        self.ang_margin = self.declare_parameter("ang_margin", 0.6).value   # rad past target -> abort
        self.abort_tilt = self.declare_parameter("abort_tilt", 1.2).value   # rad hard tilt cap
        self.odom_timeout = self.declare_parameter("odom_timeout", 2.5).value
        self.hard_radius = self.declare_parameter("hard_radius", 3.0).value
        self.publish_rate = self.declare_parameter("publish_rate", 30.0).value

        self.study_name = self.declare_parameter("study_name", "controller_tuning").value
        storage = self.declare_parameter("storage", "").value
        self.storage = storage if storage else "sqlite:///" + os.path.expanduser("~/controller_optuna.db")
        self.results_path = self.declare_parameter(
            "results_path", os.path.expanduser("~/osu-uwrt/controller_gains.yaml")).value
        # persist the study's best gains into the vehicle yaml at the end, but only if they beat
        # the anchor trial (current gains) by at least this relative margin -- stability first
        self.persist_min_improvement = self.declare_parameter("persist_min_improvement", 0.02).value

        # ---- config: trusted FF + current gains ----------------------------
        self.config_path, tree = self.load_config()
        self.base_wrench = [float(v) for v in tree["controller"]["feed_forward"]["base_wrench"]]
        self.orig_gains = {}
        for _ctrl, params in GAIN_PARAMS.items():
            for _key, (pname, cfgpath) in params.items():
                node_cfg = tree["controller"]
                for part in cfgpath:
                    node_cfg = node_cfg[part]
                self.orig_gains[pname] = [float(v) for v in node_cfg]

        self.tune_dims = self.build_tune_dims()
        self.tuned_axes = sorted(set(self.pid_axes) | set(self.smc_axes))
        # per-axis reference scale (so 1 m and 90 deg contribute comparably)
        self.ref_scale = np.array([self.box_xy, self.box_xy, self.box_z,
                                   self.roll_amp, self.pitch_amp, self.yaw_amp])
        self.abort_margin = np.array([self.lin_margin] * 3 + [self.ang_margin] * 3)
        self.waypoints = self.build_box()

        self.get_logger().info(f"Tuning {len(self.tune_dims)} gains across axes "
                               f"{[AXIS_NAMES[a] for a in self.tuned_axes]} "
                               f"({self.sampler_name}, {len(self.waypoints)}-waypoint box):")
        for dn, _p, _a, cur in self.tune_dims:
            self.get_logger().info(f"   {dn}  (current={cur})")

        # ---- ROS I/O --------------------------------------------------------
        self.odom_queue = Queue(1)
        self.create_subscription(Odometry, "odometry/filtered", self.odom_cb,
                                 qos_profile_system_default, callback_group=cb)
        self.ff_pub = self.create_publisher(Twist, "controller/FF_body_force", qos_profile_system_default)
        self.ctrl_lin_pub = self.create_publisher(ControllerCommand, "controller/linear",
                                                  qos_profile_system_default)
        self.ctrl_ang_pub = self.create_publisher(ControllerCommand, "controller/angular",
                                                  qos_profile_system_default)
        self.halt_pub = self.create_publisher(EmptyMsg, "ff_tuner/halt", qos_profile_system_default)
        # official apply path (yaml patch + overseer reload Trigger + readback verify);
        # a direct set_parameters on complete_controller does NOT reach the running model
        self.applier = GainApplier(self, self.config_path, callback_group=cb)
        self.study = None

    @staticmethod
    def parse_ints(s):
        return [int(x) for x in str(s).strip("[] ").split(",") if x.strip() != ""]

    @staticmethod
    def parse_strs(s):
        return [x.strip() for x in str(s).strip("[] ").split(",") if x.strip() != ""]

    # ========================================================================
    # config
    # ========================================================================
    def load_config(self):
        path = self.vehicle_config
        if not path:
            try:
                share = get_package_share_directory("riptide_descriptions2")
                path = self.prefer_source_config(share, os.path.join(share, "config", self.robot + ".yaml"))
            except Exception:
                path = ""
        with open(path, "r") as f:
            return path, yaml.safe_load(f)

    def prefer_source_config(self, share_dir, fallback):
        parts = os.path.normpath(share_dir).split(os.sep)
        if "install" not in parts:
            return fallback
        root = os.sep.join(parts[:parts.index("install")])
        sub = os.path.join("config", self.robot + ".yaml")
        for cand in (os.path.join(root, "src", "riptide_core", "riptide_descriptions", sub),
                     os.path.join(root, "src", "riptide_descriptions", sub)):
            if os.path.exists(cand):
                return cand
        return fallback

    def build_tune_dims(self):
        dims = []
        for ctrl, axes, keys in (("pid", self.pid_axes, self.pid_params),
                                 ("smc", self.smc_axes, self.smc_params)):
            for key in keys:
                pname, _ = GAIN_PARAMS[ctrl][key]
                for axis in axes:
                    dims.append((f"{ctrl}_{key}_{AXIS_NAMES[axis]}", pname, axis, self.orig_gains[pname][axis]))
        return dims

    def build_box(self):
        """Waypoint offsets from home (dx,dy,dz,droll,dpitch,dyaw). NEVER commands roll/pitch --
        the vehicle is unstable in pitch, so roll/pitch stay 0 (level) and their PID is tuned as
        REGULATION: how well it holds level against the disturbances from translating and yawing."""
        b, z, y = self.box_xy, self.box_z, self.yaw_amp
        return [
            [b, 0, 0, 0, 0, 0],        # +x
            [b, b, 0, 0, 0, y],        # +y, +yaw
            [0, b, z, 0, 0, y],        # +z
            [0, 0, z, 0, 0, 0],        # (drop back toward center-z)
            [-b, 0, 0, 0, 0, -y],      # -x, -yaw
            [-b, -b, -z, 0, 0, 0],     # opposite corner
            [0, 0, 0, 0, 0, 0],        # back home
        ]

    # ========================================================================
    # gain application
    # ========================================================================
    def push_gains(self, arrays):
        """Live apply: direct set_parameters + readback verify (takes effect immediately)."""
        if not self.applier.apply(arrays):
            self.get_logger().warn("gain apply failed; controller may still hold previous gains")
            return False
        return True

    def apply_original_gains(self):
        self.push_gains(self.orig_gains)

    def apply_candidate_gains(self, params):
        arrays = copy.deepcopy(self.orig_gains)
        touched = {}
        for (pname, axis), val in params.items():
            arrays[pname][axis] = val
            touched[pname] = arrays[pname]
        self.push_gains(touched)

    # ========================================================================
    # subscriptions / helpers
    # ========================================================================
    def odom_cb(self, msg):
        if self.odom_queue.full():
            try:
                self.odom_queue.get_nowait()
            except Empty:
                pass
        self.odom_queue.put_nowait(msg)

    def get_odom(self, timeout):
        if not self.odom_queue.empty():
            try:
                self.odom_queue.get_nowait()
            except Empty:
                pass
        return self.odom_queue.get(True, timeout)

    def publish_ff(self, wrench):
        m = Twist()
        m.linear = Vector3(x=float(wrench[0]), y=float(wrench[1]), z=float(wrench[2]))
        m.angular = Vector3(x=float(wrench[3]), y=float(wrench[4]), z=float(wrench[5]))
        self.ff_pub.publish(m)

    def publish_mode(self, mode, position=None, quat=None):
        lin = ControllerCommand()
        lin.mode = mode
        if position is not None:
            lin.setpoint_vect = Vector3(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        ang = ControllerCommand()
        ang.mode = mode
        if quat is not None:
            ang.setpoint_quat = Quaternion(x=float(quat[0]), y=float(quat[1]),
                                           z=float(quat[2]), w=float(quat[3]))
        ang.setpoint_vect = Vector3(x=0.0, y=0.0, z=0.0)
        self.ctrl_lin_pub.publish(lin)
        self.ctrl_ang_pub.publish(ang)

    def request_halt(self):
        self.halt_pub.publish(EmptyMsg())

    @staticmethod
    def pose_of(odom):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])

    @staticmethod
    def vel_of(odom):
        lv = odom.twist.twist.linear
        av = odom.twist.twist.angular
        return np.array([lv.x, lv.y, lv.z, av.x, av.y, av.z])

    # ---- quaternion utils ([x,y,z,w]) ----
    @staticmethod
    def q_mult(a, b):
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return np.array([aw * bx + ax * bw + ay * bz - az * by,
                         aw * by - ax * bz + ay * bw + az * bx,
                         aw * bz + ax * by - ay * bx + az * bw,
                         aw * bw - ax * bx - ay * by - az * bz])

    @staticmethod
    def q_conj(q):
        return np.array([-q[0], -q[1], -q[2], q[3]])

    @staticmethod
    def q_axis_angle(axis3, angle):
        s = math.sin(angle / 2.0)
        return np.array([axis3[0] * s, axis3[1] * s, axis3[2] * s, math.cos(angle / 2.0)])

    @classmethod
    def rpy_to_quat(cls, roll, pitch, yaw):
        return cls.q_mult(cls.q_mult(cls.q_axis_angle([0, 0, 1], yaw),
                                     cls.q_axis_angle([0, 1, 0], pitch)),
                          cls.q_axis_angle([1, 0, 0], roll))

    @classmethod
    def q_to_rotvec(cls, q):
        q = q / np.linalg.norm(q)
        if q[3] < 0:
            q = -q
        w = min(1.0, max(-1.0, q[3]))
        angle = 2.0 * math.acos(w)
        s = math.sqrt(max(1e-12, 1.0 - w * w))
        if s < 1e-6:
            return np.zeros(3)
        return (q[0:3] / s) * angle

    @classmethod
    def quat_angle(cls, q1, q2):
        d = min(1.0, max(-1.0, abs(float(np.dot(q1, q2)))))
        return 2.0 * math.acos(d)

    @classmethod
    def tilt_of(cls, quat):
        x, y = quat[0], quat[1]
        return math.acos(min(1.0, max(-1.0, 1.0 - 2.0 * (x * x + y * y))))

    @classmethod
    def yaw_of(cls, quat):
        x, y, z, w = quat
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @classmethod
    def level_quat(cls, quat):
        half = cls.yaw_of(quat) / 2.0
        return np.array([0.0, 0.0, math.sin(half), math.cos(half)])

    def waypoint_pose(self, wp):
        pos = self.home_pos + np.array(wp[0:3])
        pos[2] = min(pos[2], self.max_z)     # never command shallower than max_z
        quat = self.q_mult(self.home_quat, self.rpy_to_quat(wp[3], wp[4], wp[5]))
        return pos, quat

    def pose_error6(self, cur_pos, cur_quat, tgt_pos, tgt_quat):
        """6-vector signed error [dx,dy,dz, rotvec_x,y,z] (position world, rotation body)."""
        ep = tgt_pos - cur_pos
        er = self.q_to_rotvec(self.q_mult(self.q_conj(cur_quat), tgt_quat))
        return np.concatenate([ep, er])

    # ========================================================================
    # recover / trajectory
    # ========================================================================
    def recover_to_home(self):
        start = time.time()
        hold_start = None
        dt = 1.0 / self.publish_rate
        while True:
            if time.time() - start > self.recovery_timeout:
                raise HardHaltException(f"recovery timed out after {self.recovery_timeout:.0f}s")
            self.publish_mode(MODE_POSITION, position=self.home_pos, quat=self.home_quat)
            self.publish_ff(self.base_wrench)
            try:
                odom = self.get_odom(self.odom_timeout)
            except Empty:
                raise HardHaltException("odometry went stale during recovery")
            pos, quat = self.pose_of(odom)
            v = self.vel_of(odom)
            pos_err = float(np.linalg.norm(pos - self.home_pos))
            ang_err = self.quat_angle(quat, self.home_quat)
            if pos_err > self.hard_radius:
                raise HardHaltException(f"diverging during recovery (pos_err={pos_err:.2f})")
            if (pos_err < self.settle_pos and ang_err < self.settle_ang
                    and float(np.linalg.norm(v[0:3])) < self.settle_lin_vel
                    and float(np.linalg.norm(v[3:6])) < self.settle_ang_vel):
                if hold_start is None:
                    hold_start = time.time()
                elif time.time() - hold_start >= self.settle_hold:
                    return
            else:
                hold_start = None
            time.sleep(dt)

    def segment_budget(self, prev_pos, prev_quat, tgt_pos, tgt_quat):
        """Time budget to reach a waypoint, tied to the distance to travel."""
        lin = float(np.linalg.norm(tgt_pos - prev_pos))
        ang = self.quat_angle(prev_quat, tgt_quat)
        t = max(lin / self.cruise_speed, ang / self.yaw_rate) + self.settle_margin
        return float(min(self.max_segment_time, max(self.min_segment_time, t)))

    def run_segment(self, tgt_pos, tgt_quat, seg_time):
        """Fly to one waypoint until reached (or seg_time); return (cost, aborted, axis_costs)."""
        ref = self.ref_scale
        tuned = np.array(self.tuned_axes)
        ts, Es = [], []
        start = time.time()
        dt = 1.0 / self.publish_rate
        aborted = False
        reached_since = None
        while time.time() - start < seg_time:
            self.publish_mode(MODE_POSITION, position=tgt_pos, quat=tgt_quat)
            self.publish_ff(self.base_wrench)
            try:
                odom = self.get_odom(self.odom_timeout)
            except Empty:
                aborted = True
                break
            cur_pos, cur_quat = self.pose_of(odom)
            e6 = self.pose_error6(cur_pos, cur_quat, tgt_pos, tgt_quat)
            v = self.vel_of(odom)
            ts.append(time.time() - start)
            Es.append(e6)
            if np.any(np.abs(e6) > ref + self.abort_margin) or self.tilt_of(cur_quat) > self.abort_tilt:
                aborted = True
                break
            # advance early once the target is actually reached (all tuned axes close + slow)
            if (np.all(np.abs(e6[tuned]) < self.reach_frac * ref[tuned])
                    and float(np.linalg.norm(v[0:3])) < self.settle_lin_vel
                    and float(np.linalg.norm(v[3:6])) < self.settle_ang_vel):
                if reached_since is None:
                    reached_since = time.time()
                elif time.time() - reached_since >= self.reach_hold:
                    break
            else:
                reached_since = None
            time.sleep(dt)

        if aborted or len(ts) < 5:
            self.apply_original_gains()   # re-stabilize before recovery
            return ABORT_PENALTY, True, np.zeros(6)

        T = np.array(ts)
        E = np.array(Es)                  # (n, 6)
        dts = np.gradient(T) if len(T) > 1 else np.array([0.0])
        n_late = max(1, int(self.late_frac * len(T)))
        axis_costs = np.zeros(6)
        for ax in self.tuned_axes:
            ea = E[:, ax]
            # normalize by this segment's own time budget (which scales with distance), so the
            # score reflects tracking QUALITY, not whether the move was reachable in a fixed window
            itae = float(np.sum(T * np.abs(ea) * dts)) / (ref[ax] * 0.5 * seg_time ** 2 + 1e-9)
            band = self.settle_frac * ref[ax]
            sig = np.sign(np.where(np.abs(ea) > band, ea, 0.0))
            nz = sig[sig != 0]
            crossings = int(np.sum(nz[1:] * nz[:-1] < 0)) if nz.size > 1 else 0
            residual = float(np.mean(np.abs(ea[-n_late:]))) / (ref[ax] + 1e-9)
            axis_costs[ax] = self.w_itae * itae + self.w_osc * crossings + self.w_settle * residual
        return float(np.sum(axis_costs)), False, axis_costs

    def run_trajectory(self):
        total = 0.0
        axis_costs = np.zeros(6)
        n = len(self.waypoints)
        prev_pos, prev_quat = self.home_pos, self.home_quat
        for i, wp in enumerate(self.waypoints):
            tgt_pos, tgt_quat = self.waypoint_pose(wp)
            seg_time = self.segment_budget(prev_pos, prev_quat, tgt_pos, tgt_quat)
            cost, aborted, seg_axis_costs = self.run_segment(tgt_pos, tgt_quat, seg_time)
            total += cost
            axis_costs += seg_axis_costs
            prev_pos, prev_quat = tgt_pos, tgt_quat
            if aborted:
                self.get_logger().info(f"    aborted at waypoint {i} -> penalized")
                total += ABORT_PENALTY * (n - i - 1)
                return total, True, axis_costs
        return total, False, axis_costs

    # ========================================================================
    # optuna driver
    # ========================================================================
    def suggest(self, trial):
        params = {}
        for dim_name, pname, axis, current in self.tune_dims:
            if current > 0:
                val = trial.suggest_float(dim_name, current * self.gain_lo_mult,
                                          current * self.gain_hi_mult, log=True)
            else:
                val = trial.suggest_float(dim_name, 0.0, self.zero_gain_max)
            params[(pname, axis)] = val
        return params

    def objective(self, trial):
        params = self.suggest(trial)
        self.apply_original_gains()
        self.recover_to_home()
        self.apply_candidate_gains(params)
        total, _aborted, axis_costs = self.run_trajectory()
        self.apply_original_gains()
        breakdown = "  ".join(f"{AXIS_NAMES[a]}={axis_costs[a]:.2f}" for a in self.tuned_axes)
        self.get_logger().info(f"  trial {trial.number}: cost={total:.3f}   [{breakdown}]")
        return total

    def run_study(self):
        try:
            self.startup()
        except HardHaltException as e:
            self.escalate(str(e))
            return
        sampler = (optuna.samplers.CmaEsSampler(seed=0) if self.sampler_name == "cmaes"
                   else optuna.samplers.TPESampler(seed=0))
        self.study = optuna.create_study(study_name=self.study_name, storage=self.storage,
                                         direction="minimize", sampler=sampler, load_if_exists=True)
        # anchor a trial at the current gains so we never end up worse than today
        self.study.enqueue_trial({dn: cur for dn, _p, _a, cur in self.tune_dims if cur > 0})
        self._best_val = float("inf")
        self._no_improve = 0
        try:
            self.study.optimize(self.objective, n_trials=self.n_trials, callbacks=[self._early_stop_cb])
        except HardHaltException as e:
            self.escalate(str(e))
            return
        except Exception as e:
            self.get_logger().error(f"Tuning aborted: {e}")
            self.disarm("unexpected error")
            return
        self.finish()

    def _early_stop_cb(self, study, trial):
        try:
            bv = study.best_value
        except ValueError:
            return
        if bv < self._best_val - 1e-6:
            self._best_val = bv
            self._no_improve = 0
        else:
            self._no_improve += 1
            if self._no_improve >= self.early_stop_patience:
                self.get_logger().info(f"Early stop: no improvement in {self.early_stop_patience} "
                                       f"trials (best={bv:.4f} after {len(study.trials)}).")
                study.stop()

    def startup(self):
        self.get_logger().info("Starting controller gain tuner. ENSURE the vehicle is armed in "
                               "RViz and staged with clearance for the box maneuver.")
        self.apply_original_gains()
        deadline = time.time() + 30.0
        while True:
            try:
                odom = self.get_odom(self.odom_timeout)
                break
            except Empty:
                if time.time() > deadline:
                    raise HardHaltException("no odometry at startup")
        self.home_pos, raw = self.pose_of(odom)
        self.home_pos[2] = min(self.home_pos[2], self.max_z)   # keep home at/below the depth limit
        self.home_quat = self.level_quat(raw)
        self.get_logger().info(f"HOME captured: pos={np.round(self.home_pos, 2).tolist()} "
                               f"(z capped at {self.max_z})")

    def escalate(self, reason):
        self.get_logger().error("=" * 60)
        self.get_logger().error(f"HARD FAILURE: {reason}")
        self.get_logger().error("Restoring original gains, requesting RViz HALT, waiting for human.")
        self.get_logger().error("=" * 60)
        self.apply_original_gains()
        for _ in range(25):
            self.request_halt()
            self.publish_mode(MODE_DISABLED)
            time.sleep(0.05)

    def disarm(self, reason, restore=True):
        self.get_logger().warn(f"Disarming ({reason}): "
                               f"{'restoring original gains + ' if restore else ''}HALT + DISABLED.")
        if restore:
            self.apply_original_gains()
        for _ in range(10):
            self.request_halt()
            self.publish_mode(MODE_DISABLED)
            time.sleep(0.05)

    def finish(self):
        persisted = self.persist_best()
        self.disarm("tuning complete", restore=not persisted)
        self.save_best()

    def persist_best(self):
        """Apply the study's best gains through the official path iff they beat the anchor."""
        study = self.study
        if study is None or not study.trials:
            return False
        try:
            best = study.best_params
            best_value = study.best_value
        except ValueError:
            return False
        # the anchor (current gains) is enqueued as the first trial; with a reused sqlite study
        # this is the anchor of the run that created the study, which is still the right baseline
        anchor = next((t.value for t in study.trials
                       if t.state == optuna.trial.TrialState.COMPLETE and t.value is not None), None)
        if anchor is not None and best_value > anchor * (1.0 - self.persist_min_improvement):
            self.get_logger().info(
                f"Best cost {best_value:.4f} does not beat the anchor {anchor:.4f} by "
                f">{self.persist_min_improvement * 100:.0f}% -- keeping original gains.")
            return False
        result = copy.deepcopy(self.orig_gains)
        for dim_name, pname, axis, _cur in self.tune_dims:
            if dim_name in best:
                result[pname][axis] = round(best[dim_name], 6)
        touched = {p: v for p, v in result.items() if v != self.orig_gains[p]}
        if not touched:
            return False
        if self.push_gains(touched):
            # live on the controller now; patch the yaml too so a reload/restart keeps the tune
            self.applier.persist(touched)
            self.get_logger().info(f"BEST gains applied + persisted (cost {best_value:.4f} vs "
                                   f"anchor {anchor if anchor is None else round(anchor, 4)}).")
            return True
        self.get_logger().error("Failed to apply best gains -- restoring originals.")
        return False

    def save_best(self):
        study = self.study
        if study is None or not study.trials:
            return
        try:
            best = study.best_params
            best_value = study.best_value
        except ValueError:
            return
        result = copy.deepcopy(self.orig_gains)
        for dim_name, pname, axis, _cur in self.tune_dims:
            if dim_name in best:
                result[pname][axis] = round(best[dim_name], 6)
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"BEST SO FAR: cost = {best_value:.4f}  ({len(study.trials)} trials)")
        for pname, vals in result.items():
            self.get_logger().info(f"  {pname} = {vals}")
        self.get_logger().info("=" * 60)
        try:
            os.makedirs(os.path.dirname(self.results_path), exist_ok=True)
            with open(self.results_path, "w") as f:
                yaml.safe_dump({"robot": self.robot, "best_value": float(best_value),
                                "best_params": {k: float(v) for k, v in best.items()},
                                "gain_arrays": {k: [float(x) for x in v] for k, v in result.items()}}, f)
            self.get_logger().info(f"Saved best gains to {self.results_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save results: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ControllerOptunaTuner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=node.run_study, daemon=True)
    worker.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.save_best()
        node.disarm("KeyboardInterrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
