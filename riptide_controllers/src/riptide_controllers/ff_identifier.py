#! /usr/bin/env python3
#
# Closed-loop feedforward (FF) identifier for the Riptide controller.
#
# Instead of blindly searching the FF wrench (see ff_optuna_tuner.py), this exploits the physics:
# in FEEDFORWARD mode the vehicle obeys  M * a = FF_applied - FF_wanted, so
#
#         FF_wanted = FF_applied - M * a
#
# where M = [mass, mass, mass, Ixx, Iyy, Izz] and `a` is the body-frame acceleration measured
# right after switching from a settled hold into feedforward. With accurate M this is a Newton
# step: it converges in a handful of iterations rather than hundreds of trials, and the SHORT
# measurement window sidesteps the open-loop pitch instability (the unstable mode has not grown
# yet when starting from rest).
#
# Per iteration:  RECOVER (position-hold at level home with the trusted base_wrench)
#              -> MEASURE (short FEEDFORWARD window; fit body-frame acceleration from odom twist)
#              -> UPDATE  (FF -= lr * M * a, clamped)   until |correction| < tol
#
# Safety is identical to the Optuna tuner: soft limits abort a measurement and recover; a hard
# failure requests an authoritative kill through the RViz ControlPanel (topic <ns>/ff_tuner/halt).
#
# Run:  ros2 run riptide_controllers2 ff_identifier.py --ros-args -r __ns:=/talos

import os
import math
import time
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
from std_msgs.msg import Empty as EmptyMsg          # aliased: `Empty` below is the queue exception
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from queue import Queue, Empty

MODE_DISABLED = ControllerCommand.DISABLED
MODE_FEEDFORWARD = ControllerCommand.FEEDFORWARD
MODE_POSITION = ControllerCommand.POSITION

DISABLE_NATIVE_FF_PARAM = "disable_native_ff"
AXES = ["fx", "fy", "fz", "tx", "ty", "tz"]


class HardHaltException(Exception):
    """Raised when the vehicle cannot be recovered; halts and asks the panel to kill."""


class FFIdentifier(Node):

    def __init__(self):
        super().__init__("ff_identifier")
        cb = ReentrantCallbackGroup()

        # ---- parameters -----------------------------------------------------
        self.robot = self.declare_parameter("robot", "talos").value
        self.vehicle_config = self.declare_parameter("vehicle_config", "").value

        self.n_iterations = self.declare_parameter("n_iterations", 200).value
        self.measure_window = self.declare_parameter("measure_window", 4.0).value   # s of feedforward
        self.learning_rate = self.declare_parameter("learning_rate", 0.8).value     # 1.0 = full Newton
        # each iteration averages the measured initial acceleration over this many short windows
        # (re-leveling between them) for a clean, trustworthy gradient
        self.windows_per_iter = self.declare_parameter("windows_per_iter", 3).value
        # mild gain decay for final settling; honest convergence (below) prevents premature freeze
        self.lr_decay = self.declare_parameter("lr_decay", 0.1).value
        self.avg_last = self.declare_parameter("avg_last", 4).value
        # converge only when the AVERAGED residual correction stays < tol this many iters in a row
        self.converge_patience = self.declare_parameter("converge_patience", 2).value
        # which axes to identify (1=on). x/y/yaw usually have ~0 static disturbance underwater
        self.axis_mask = list(self.declare_parameter("axis_mask", [1, 1, 1, 1, 1, 1]).value)
        # per-iteration clamp on how far FF may move (safety)
        self.max_step_force = self.declare_parameter("max_step_force", 2.0).value    # N
        self.max_step_torque = self.declare_parameter("max_step_torque", 1.0).value  # N*m
        # absolute runaway clamp: FF is never allowed to leave base_wrench +/- this
        self.max_dev_force = self.declare_parameter("max_dev_force", 6.0).value      # N
        self.max_dev_torque = self.declare_parameter("max_dev_torque", 3.0).value    # N*m
        # convergence tolerance on the correction magnitude
        self.tol_force = self.declare_parameter("tol_force", 0.1).value              # N
        self.tol_torque = self.declare_parameter("tol_torque", 0.05).value          # N*m

        # settle / recovery (same semantics as the Optuna tuner)
        self.settle_pos = self.declare_parameter("settle_pos", 0.15).value
        self.settle_ang = self.declare_parameter("settle_ang", 0.15).value
        self.settle_lin_vel = self.declare_parameter("settle_lin_vel", 0.12).value
        self.settle_ang_vel = self.declare_parameter("settle_ang_vel", 0.25).value
        self.settle_hold = self.declare_parameter("settle_hold", 1.0).value
        self.recovery_timeout = self.declare_parameter("recovery_timeout", 25.0).value

        # soft / hard safety limits
        self.abort_radius = self.declare_parameter("abort_radius", 1.5).value
        self.abort_speed = self.declare_parameter("abort_speed", 1.0).value
        self.abort_tilt = self.declare_parameter("abort_tilt", 0.785).value
        self.odom_timeout = self.declare_parameter("odom_timeout", 2.5).value
        self.hard_radius = self.declare_parameter("hard_radius", 2.5).value

        self.publish_rate = self.declare_parameter("publish_rate", 30.0).value
        self.results_path = self.declare_parameter(
            "results_path", os.path.expanduser("~/osu-uwrt/ff_identified.yaml")).value
        self.write_config = self.declare_parameter("write_config", False).value

        # ---- vehicle model + trusted FF from the config ---------------------
        self.config_path, self.base_wrench, mass, inertia = self.load_config()
        # generalized inertia vector M = [m, m, m, Ixx, Iyy, Izz]
        self.M = np.array([mass, mass, mass, inertia[0], inertia[1], inertia[2]])
        self.get_logger().info(f"Trusted base_wrench: {self.base_wrench}")
        self.get_logger().info(f"Inertia vector M = {self.M.tolist()}")

        # ---- ROS I/O --------------------------------------------------------
        self.odom_queue = Queue(1)
        self.create_subscription(Odometry, "odometry/filtered", self.odom_cb,
                                 qos_profile_system_default, callback_group=cb)
        self.ff_pub = self.create_publisher(Twist, "controller/FF_body_force",
                                            qos_profile_system_default)
        self.ctrl_lin_pub = self.create_publisher(ControllerCommand, "controller/linear",
                                                  qos_profile_system_default)
        self.ctrl_ang_pub = self.create_publisher(ControllerCommand, "controller/angular",
                                                  qos_profile_system_default)
        self.halt_pub = self.create_publisher(EmptyMsg, "ff_tuner/halt",
                                              qos_profile_system_default)
        self.overseer_param_client = self.create_client(
            SetParameters, "controller_overseer/set_parameters", callback_group=cb)

    # ========================================================================
    # config loading
    # ========================================================================
    def load_config(self):
        path = self.vehicle_config
        if not path:
            try:
                share = get_package_share_directory("riptide_descriptions2")
                path = self.prefer_source_config(share, os.path.join(share, "config",
                                                                     self.robot + ".yaml"))
            except Exception:
                path = ""
        try:
            with open(path, "r") as f:
                tree = yaml.safe_load(f)
            bw = [float(v) for v in tree["controller"]["feed_forward"]["base_wrench"]]
            mass = float(tree["mass"])
            inertia = [float(v) for v in tree["inertia"]]
            return path, bw, mass, inertia
        except Exception as e:
            self.get_logger().error(
                f"Could not read config from '{path}': {e}. Using safe fallbacks.")
            return path, [0.0] * 6, 30.0, [1.0, 1.0, 1.0]

    def prefer_source_config(self, share_dir, fallback):
        parts = os.path.normpath(share_dir).split(os.sep)
        if "install" not in parts:
            return fallback
        root = os.sep.join(parts[:parts.index("install")])
        sub = os.path.join("config", self.robot + ".yaml")
        for candidate in (
            os.path.join(root, "src", "riptide_core", "riptide_descriptions", sub),
            os.path.join(root, "src", "riptide_descriptions", sub),
        ):
            if os.path.exists(candidate):
                return candidate
        return fallback

    # ========================================================================
    # subscriptions / helpers (shared with ff_optuna_tuner)
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
        msg = Twist()
        msg.linear = Vector3(x=float(wrench[0]), y=float(wrench[1]), z=float(wrench[2]))
        msg.angular = Vector3(x=float(wrench[3]), y=float(wrench[4]), z=float(wrench[5]))
        self.ff_pub.publish(msg)

    def publish_mode(self, mode, position=None, quat=None):
        lin = ControllerCommand()
        lin.mode = mode
        if position is not None:
            lin.setpoint_vect = Vector3(x=float(position[0]), y=float(position[1]),
                                        z=float(position[2]))
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

    def set_native_ff_disabled(self, disabled):
        if not self.overseer_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("controller_overseer/set_parameters unavailable; the overseer "
                                   "may fight our FF at ~1 Hz.")
            return
        req = SetParameters.Request()
        p = Parameter()
        p.name = DISABLE_NATIVE_FF_PARAM
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(disabled))
        req.parameters = [p]
        self.overseer_param_client.call_async(req)
        self.get_logger().info(f"Set overseer {DISABLE_NATIVE_FF_PARAM}={disabled}")

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

    @staticmethod
    def quat_angle(q1, q2):
        d = min(1.0, max(-1.0, abs(float(np.dot(q1, q2)))))
        return 2.0 * math.acos(d)

    @staticmethod
    def tilt_of(quat):
        x, y, _z, _w = quat
        return math.acos(min(1.0, max(-1.0, 1.0 - 2.0 * (x * x + y * y))))

    @staticmethod
    def yaw_of(quat):
        x, y, z, w = quat
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @classmethod
    def level_quat(cls, quat):
        half = cls.yaw_of(quat) / 2.0
        return np.array([0.0, 0.0, math.sin(half), math.cos(half)])

    @staticmethod
    def rpy_deg(quat):
        x, y, z, w = quat
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return tuple(round(math.degrees(a), 1) for a in (roll, pitch, yaw))

    # ========================================================================
    # recover / measure
    # ========================================================================
    def recover_to_home(self):
        self.get_logger().info("RECOVER: position-hold at level home with trusted FF...")
        start = time.time()
        hold_start = None
        last_log = 0.0
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
            lin_speed = float(np.linalg.norm(v[0:3]))
            ang_speed = float(np.linalg.norm(v[3:6]))

            now = time.time()
            if now - last_log > 3.0:
                last_log = now
                self.get_logger().info(
                    f"  recovering: pos_err={pos_err:.2f}m ang_err={math.degrees(ang_err):.0f}deg "
                    f"lin_v={lin_speed:.2f} ang_v={ang_speed:.2f}")

            if pos_err > self.hard_radius:
                raise HardHaltException(f"diverging during recovery (pos_err={pos_err:.2f})")

            if (pos_err < self.settle_pos and ang_err < self.settle_ang
                    and lin_speed < self.settle_lin_vel and ang_speed < self.settle_ang_vel):
                if hold_start is None:
                    hold_start = time.time()
                elif time.time() - hold_start >= self.settle_hold:
                    self.get_logger().info("RECOVER: settled.")
                    return
            else:
                hold_start = None
            time.sleep(dt)

    def measure_window_accel(self, feedforward, ff):
        """Collect body-frame velocity over one window and return initial acceleration a(0).

        feedforward=True  -> FEEDFORWARD mode (open loop, publishing `ff`): the vehicle responds to
                             the FF error, so a(0) ~= FF_error / M.
        feedforward=False -> POSITION hold at home (still publishing `ff`): true accel is ~0, so a(0)
                             is the ESTIMATOR BIAS (e.g. the vertical channel reads ~+0.03 at rest).
        The per-iteration correction uses (a_ff - a_hold), which cancels that constant bias -- this
        is why fz was running away.
        """
        ts, vs = [], []
        start = time.time()
        dt = 1.0 / self.publish_rate
        aborted = False
        while time.time() - start < self.measure_window:
            if feedforward:
                self.publish_mode(MODE_FEEDFORWARD)
            else:
                self.publish_mode(MODE_POSITION, position=self.home_pos, quat=self.home_quat)
            self.publish_ff(ff)
            try:
                odom = self.get_odom(self.odom_timeout)
            except Empty:
                self.get_logger().warn("MEASURE: odometry stale -> abort window")
                aborted = True
                break
            pos, quat = self.pose_of(odom)
            v = self.vel_of(odom)
            ts.append(time.time() - start)
            vs.append(v)

            if feedforward and (float(np.linalg.norm(pos - self.home_pos)) > self.abort_radius
                                or float(np.linalg.norm(v[0:3])) > self.abort_speed
                                or self.tilt_of(quat) > self.abort_tilt):
                self.get_logger().warn("MEASURE: soft limit hit -> abort window")
                aborted = True
                break
            time.sleep(dt)

        # hand control back to the trusted FF immediately
        self.publish_ff(self.base_wrench)

        if len(ts) < 5:
            return None
        T = np.array(ts)
        V = np.array(vs)
        # Initial acceleration a(0): fit each velocity component to a QUADRATIC and take the linear
        # coefficient. a(0) isolates the STATIC wrench error from (a) hydrodynamic drag and (b) the
        # open-loop pitch instability -- both only contaminate later samples (drag grows with
        # velocity, the unstable mode grows with tilt; both are ~0 at t=0 from a level rest).
        accel = np.array([np.polyfit(T, V[:, i], 2)[1] for i in range(6)])
        if aborted:
            self.get_logger().info(f"  (window aborted early after {len(ts)} samples; using partial fit)")
        return accel

    # ========================================================================
    # driver
    # ========================================================================
    def run(self):
        try:
            self.startup()
            ff = np.array(self.base_wrench, dtype=float)
            mask = np.array(self.axis_mask, dtype=float)
            step_clamp = np.array([self.max_step_force] * 3 + [self.max_step_torque] * 3)
            dev_clamp = np.array([self.max_dev_force] * 3 + [self.max_dev_torque] * 3)
            base = np.array(self.base_wrench, dtype=float)
            tol = np.array([self.tol_force] * 3 + [self.tol_torque] * 3)
            history = []
            settled_count = 0

            for it in range(self.n_iterations):
                # Average a DIFFERENTIAL acceleration over several windows. Each window: recover,
                # measure the hold baseline a(0) (estimator bias, since true accel ~= 0 when held),
                # then measure the feedforward a(0); the difference cancels the constant bias.
                diffs, holds = [], []
                for _w in range(self.windows_per_iter):
                    self.recover_to_home()
                    a_hold = self.measure_window_accel(feedforward=False, ff=self.base_wrench)
                    a_ff = self.measure_window_accel(feedforward=True, ff=ff)
                    if a_hold is not None and a_ff is not None:
                        diffs.append(a_ff - a_hold)
                        holds.append(a_hold)
                if not diffs:
                    self.get_logger().warn(f"iter {it}: no usable windows, retrying")
                    continue
                accel = np.mean(diffs, axis=0)
                bias = np.mean(holds, axis=0)

                # Newton step: FF_wanted = FF_applied - M * a(0). Mild gain decay aids settling.
                lr = self.learning_rate / (1.0 + self.lr_decay * it)
                correction = self.M * accel
                step = np.clip(lr * correction, -step_clamp, step_clamp) * mask
                # absolute runaway clamp: never leave base_wrench +/- dev_clamp
                ff = np.clip(ff - step, base - dev_clamp, base + dev_clamp)
                history.append(ff.copy())

                self.get_logger().info(
                    f"iter {it}: a_diff={np.round(accel, 4).tolist()} (avg of {len(diffs)})  "
                    f"bias={np.round(bias, 4).tolist()}")
                self.get_logger().info(
                    f"         corr(M*a)={np.round(correction, 3).tolist()}  lr={lr:.2f}  "
                    f"FF -> {np.round(ff, 3).tolist()}")

                # Honest convergence: the AVERAGED residual correction is genuinely small for a
                # few iterations in a row (not merely a small step from gain decay).
                if np.all(np.abs(correction * mask) < tol):
                    settled_count += 1
                    if settled_count >= self.converge_patience:
                        self.get_logger().info(
                            f"CONVERGED after {it + 1} iterations "
                            f"(residual < tol for {self.converge_patience} in a row).")
                        break
                else:
                    settled_count = 0

            # Report the average of the last few iterates (Polyak) for a little extra smoothing.
            k = min(self.avg_last, len(history))
            ff_final = np.mean(history[-k:], axis=0) if k > 0 else ff
            self.get_logger().info(f"Averaged final estimate over last {k} iterate(s).")
            self.finish(ff_final)
        except HardHaltException as e:
            self.escalate(str(e))
        except Exception as e:
            self.get_logger().error(f"Identification aborted: {e}")
            self.disarm("unexpected error")

    def startup(self):
        self.get_logger().info("Starting FF identifier. ENSURE the vehicle is armed in RViz "
                               "and staged with clearance > abort_radius.")
        self.set_native_ff_disabled(True)
        deadline = time.time() + 30.0
        while True:
            try:
                odom = self.get_odom(self.odom_timeout)
                break
            except Empty:
                if time.time() > deadline:
                    raise HardHaltException("no odometry at startup")
        self.home_pos, raw = self.pose_of(odom)
        self.home_quat = self.level_quat(raw)   # level target (vehicle unstable in pitch)
        self.get_logger().info(
            f"HOME captured: pos={np.round(self.home_pos, 3).tolist()} "
            f"raw_rpy(deg)={self.rpy_deg(raw)} -> leveled_rpy(deg)={self.rpy_deg(self.home_quat)}")

    def escalate(self, reason):
        self.get_logger().error("=" * 60)
        self.get_logger().error(f"HARD FAILURE: {reason}")
        self.get_logger().error("Requesting RViz ControlPanel HALT (kill). Waiting for a human.")
        self.get_logger().error("=" * 60)
        for _ in range(25):
            self.request_halt()
            self.publish_mode(MODE_DISABLED)
            time.sleep(0.05)

    def disarm(self, reason):
        self.get_logger().warn(f"Disarming ({reason}): requesting HALT + DISABLED mode.")
        for _ in range(10):
            self.request_halt()
            self.publish_mode(MODE_DISABLED)
            time.sleep(0.05)
        self.set_native_ff_disabled(False)

    def finish(self, ff):
        ff = [float(round(v, 4)) for v in ff]
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"IDENTIFIED base_wrench = {ff}")
        self.get_logger().info(f"(original was          = {self.base_wrench})")
        self.get_logger().info("=" * 60)
        self.disarm("identification complete")
        try:
            os.makedirs(os.path.dirname(self.results_path), exist_ok=True)
            with open(self.results_path, "w") as f:
                yaml.safe_dump({"robot": self.robot, "base_wrench": ff,
                                "original_base_wrench": self.base_wrench}, f)
            self.get_logger().info(f"Saved to {self.results_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save results: {e}")
        if self.write_config and self.config_path:
            try:
                with open(self.config_path, "r") as f:
                    tree = yaml.safe_load(f)
                tree["controller"]["feed_forward"]["base_wrench"] = ff
                with open(self.config_path, "w") as f:
                    yaml.safe_dump(tree, f, default_flow_style=False, sort_keys=False)
                self.get_logger().warn(f"WROTE new base_wrench into {self.config_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to write config: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FFIdentifier()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=node.run, daemon=True)
    worker.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.disarm("KeyboardInterrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
