#! /usr/bin/env python3
#
# Controller verification lap: quantify stability instead of eyeballing it.
#
#   1. HOLD test: sit at level home for hold_duration and record per-axis RMS / max error --
#      this is the station-keeping number that matters for the torpedo and table tasks.
#   2. STEP tests on x, y, z, yaw (NEVER roll/pitch -- those are only ever regulated level):
#      command a small step, record rise time, overshoot, settle time, oscillation
#      (error target-crossings) and residual RMS, then recover home.
#
# Writes a YAML report (results_path, with {label} substituted) and logs a compact table.
# Run it before and after a tuning pass with different label:= values and diff the reports.
#
#   ros2 launch riptide_controllers2 controller_verify.launch.py robot:=talos label:=baseline
#
# Safety identical to the other tuning tools: trusted-FF recovery, hard radius, halt -> RViz kill.

import os
import math
import time
import datetime
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
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from gain_apply import GainApplier

from queue import Queue, Empty

MODE_DISABLED = ControllerCommand.DISABLED
MODE_POSITION = ControllerCommand.POSITION

AXIS_NAMES = ["x", "y", "z", "roll", "pitch", "yaw"]
GAIN_PARAM_NAMES = [
    "controller__PID__p_gains", "controller__PID__i_gains", "controller__PID__d_gains",
    "controller__SMC__SMC_params__eta_order_0", "controller__SMC__SMC_params__eta_order_1",
    "controller__SMC__SMC_params__lambda",
]

DISABLE_NATIVE_FF_PARAM = "disable_native_ff"


class HardHaltException(Exception):
    pass


class ControllerVerify(Node):

    def __init__(self):
        super().__init__("controller_verify")
        cb = ReentrantCallbackGroup()

        self.robot = self.declare_parameter("robot", "talos").value
        self.vehicle_config = self.declare_parameter("vehicle_config", "").value
        self.label = self.declare_parameter("label", "verify").value

        # ---- test profile ---------------------------------------------------
        self.hold_duration = self.declare_parameter("hold_duration", 15.0).value
        self.step_duration = self.declare_parameter("step_duration", 8.0).value
        self.step_xy = self.declare_parameter("step_xy", 0.5).value        # m
        self.step_z = self.declare_parameter("step_z", 0.4).value          # m
        self.step_yaw = self.declare_parameter("step_yaw", 0.6).value      # rad
        self.band_pos = self.declare_parameter("band_pos", 0.05).value     # m settle band
        self.band_yaw = self.declare_parameter("band_yaw", 0.05).value     # rad settle band

        # ---- settle / recovery / safety (ff_identifier's proven values) -----
        self.settle_pos = self.declare_parameter("settle_pos", 0.15).value
        self.settle_ang = self.declare_parameter("settle_ang", 0.15).value
        self.settle_lin_vel = self.declare_parameter("settle_lin_vel", 0.12).value
        self.settle_ang_vel = self.declare_parameter("settle_ang_vel", 0.25).value
        self.settle_hold = self.declare_parameter("settle_hold", 1.0).value
        self.recovery_timeout = self.declare_parameter("recovery_timeout", 25.0).value
        self.abort_tilt = self.declare_parameter("abort_tilt", 0.785).value
        self.abort_radius = self.declare_parameter("abort_radius", 1.5).value
        self.odom_timeout = self.declare_parameter("odom_timeout", 2.5).value
        self.hard_radius = self.declare_parameter("hard_radius", 2.5).value
        self.publish_rate = self.declare_parameter("publish_rate", 30.0).value

        self.handoff = self.declare_parameter("handoff", False).value
        self.results_path = self.declare_parameter(
            "results_path", os.path.expanduser("~/osu-uwrt/controller_verify_{label}.yaml")).value

        # ---- config ---------------------------------------------------------
        self.config_path, tree = self.load_config()
        self.base_wrench = [float(v) for v in tree["controller"]["feed_forward"]["base_wrench"]]

        # ---- ROS I/O ---------------------------------------------------------
        self.odom_queue = Queue(1)
        self.create_subscription(Odometry, "odometry/filtered", self.odom_cb,
                                 qos_profile_system_default, callback_group=cb)
        self.ff_pub = self.create_publisher(Twist, "controller/FF_body_force", qos_profile_system_default)
        self.ctrl_lin_pub = self.create_publisher(ControllerCommand, "controller/linear",
                                                  qos_profile_system_default)
        self.ctrl_ang_pub = self.create_publisher(ControllerCommand, "controller/angular",
                                                  qos_profile_system_default)
        self.halt_pub = self.create_publisher(EmptyMsg, "ff_tuner/halt", qos_profile_system_default)
        self.overseer_param_client = self.create_client(
            SetParameters, "controller_overseer/set_parameters", callback_group=cb)
        self.applier = GainApplier(self, self.config_path, callback_group=cb)  # read-only here

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
        if not path or not os.path.exists(path):
            raise RuntimeError(f"vehicle config not found (robot='{self.robot}')")
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

    # ========================================================================
    # ROS helpers (shared pattern with the other tuning tools)
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

    def set_native_ff_disabled(self, disabled):
        if not self.overseer_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("overseer set_parameters unavailable; it may fight our FF")
            return
        req = SetParameters.Request()
        p = Parameter()
        p.name = DISABLE_NATIVE_FF_PARAM
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(disabled))
        req.parameters = [p]
        self.overseer_param_client.call_async(req)

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
        x, y = quat[0], quat[1]
        return math.acos(min(1.0, max(-1.0, 1.0 - 2.0 * (x * x + y * y))))

    @staticmethod
    def yaw_of(quat):
        x, y, z, w = quat
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def rpy_of(quat):
        x, y, z, w = quat
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return roll, pitch, yaw

    @classmethod
    def level_quat(cls, quat):
        half = cls.yaw_of(quat) / 2.0
        return np.array([0.0, 0.0, math.sin(half), math.cos(half)])

    @staticmethod
    def yaw_quat(yaw):
        return np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])

    @staticmethod
    def wrap(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    # ========================================================================
    # recover
    # ========================================================================
    def recover_to_home(self):
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
            now = time.time()
            if now - last_log > 3.0:
                last_log = now
                self.get_logger().info(f"  recovering: pos_err={pos_err:.2f}m "
                                       f"ang_err={math.degrees(ang_err):.0f}deg")
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

    # ========================================================================
    # tests
    # ========================================================================
    def errors6(self, pos, quat, tgt_pos, tgt_quat):
        """Signed per-axis error target-minus-current: x,y,z [m], roll,pitch [rad], yaw [rad]."""
        e = np.zeros(6)
        e[0:3] = tgt_pos - pos
        r, p, y = self.rpy_of(quat)
        tr, tp, ty = self.rpy_of(tgt_quat)
        e[3] = self.wrap(tr - r)
        e[4] = self.wrap(tp - p)
        e[5] = self.wrap(ty - y)
        return e

    def record_window(self, tgt_pos, tgt_quat, duration):
        """POSITION-hold the target and record (t, err6) plus safety checks."""
        ts, errs = [], []
        start = time.time()
        dt = 1.0 / self.publish_rate
        while time.time() - start < duration:
            self.publish_mode(MODE_POSITION, position=tgt_pos, quat=tgt_quat)
            self.publish_ff(self.base_wrench)
            try:
                odom = self.get_odom(self.odom_timeout)
            except Empty:
                raise HardHaltException("odometry went stale during a test window")
            pos, quat = self.pose_of(odom)
            if self.tilt_of(quat) > self.abort_tilt:
                raise HardHaltException(f"tilt exceeded {self.abort_tilt:.2f} rad during a test")
            if float(np.linalg.norm(pos - self.home_pos)) > self.abort_radius:
                raise HardHaltException("left the abort radius during a test")
            ts.append(time.time() - start)
            errs.append(self.errors6(pos, quat, tgt_pos, tgt_quat))
            time.sleep(dt)
        return np.array(ts), np.array(errs)

    def hold_test(self):
        self.get_logger().info(f"HOLD test: {self.hold_duration:.0f}s at home...")
        self.recover_to_home()
        _ts, errs = self.record_window(self.home_pos, self.home_quat, self.hold_duration)
        out = {}
        for a in range(6):
            e = errs[:, a]
            out[AXIS_NAMES[a]] = {"rms": float(np.sqrt(np.mean(e * e))),
                                  "max": float(np.max(np.abs(e)))}
        return out

    def step_test(self, axis, step):
        self.get_logger().info(f"STEP test: {AXIS_NAMES[axis]} {step:+.2f}...")
        self.recover_to_home()
        tgt_pos = np.array(self.home_pos)
        tgt_quat = np.array(self.home_quat)
        if axis < 3:
            tgt_pos[axis] += step
            band = max(self.band_pos, 0.05 * abs(step))
        else:
            half = step / 2.0
            dq = self.yaw_quat(step)
            # world-frame yaw rotation composed onto home orientation
            x1, y1, z1, w1 = dq
            x2, y2, z2, w2 = tgt_quat
            tgt_quat = np.array([
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            ])
            band = max(self.band_yaw, 0.05 * abs(step))
        ts, errs = self.record_window(tgt_pos, tgt_quat, self.step_duration)
        e = errs[:, axis]
        ae = np.abs(e)
        sgn = math.copysign(1.0, step)

        rise_idx = np.argmax(ae <= 0.1 * abs(step)) if np.any(ae <= 0.1 * abs(step)) else None
        overshoot = float(max(0.0, np.max(-e * sgn)) / abs(step))
        outside = np.where(ae > band)[0]
        settle = float(ts[outside[-1]]) if len(outside) and outside[-1] < len(ts) - 1 else \
            (None if len(outside) == len(ts) else 0.0)
        crossings = int(np.sum(np.abs(np.diff(np.sign(e))) > 1))
        tail = e[int(0.75 * len(e)):]
        return {
            "step": float(step),
            "rise_s": None if rise_idx is None else float(ts[rise_idx]),
            "overshoot_frac": overshoot,
            "settle_s": settle,                     # None = never settled into the band
            "oscillation_crossings": crossings,
            "residual_rms": float(np.sqrt(np.mean(tail * tail))),
        }

    # ========================================================================
    # driver
    # ========================================================================
    def run(self):
        try:
            self.startup()
            report = {
                "robot": self.robot,
                "label": self.label,
                "generated": datetime.datetime.now().isoformat(timespec="seconds"),
                "gains": self.applier.read(GAIN_PARAM_NAMES) or "unavailable",
                "hold": self.hold_test(),
                "steps": {},
            }
            for axis, step in ((0, self.step_xy), (1, self.step_xy),
                               (2, -abs(self.step_z)), (5, self.step_yaw)):
                report["steps"][AXIS_NAMES[axis]] = self.step_test(axis, step)
            self.recover_to_home()
            self.report(report)
            self.save(report)
        except HardHaltException as e:
            self.escalate(str(e))
            return
        except Exception as e:
            self.get_logger().error(f"Verification aborted: {e}")
        self.finish()

    def startup(self):
        self.get_logger().info("Starting controller verification. ARM in RViz; stage with clearance.")
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
        self.home_quat = self.level_quat(raw)
        self.get_logger().info(f"HOME captured: pos={np.round(self.home_pos, 2).tolist()}")

    def report(self, rep):
        self.get_logger().info("=" * 64)
        self.get_logger().info(f"VERIFY [{self.label}]  hold RMS (m / rad):")
        h = rep["hold"]
        self.get_logger().info("  " + "  ".join(f"{a}={h[a]['rms']:.3f}" for a in AXIS_NAMES))
        for a, m in rep["steps"].items():
            settle = "never" if m["settle_s"] is None else f"{m['settle_s']:.1f}s"
            rise = "never" if m["rise_s"] is None else f"{m['rise_s']:.1f}s"
            self.get_logger().info(
                f"  step {a:3s} {m['step']:+.2f}: rise {rise}  overshoot {m['overshoot_frac'] * 100:.0f}%  "
                f"settle {settle}  osc x{m['oscillation_crossings']}  resid {m['residual_rms']:.3f}")
        self.get_logger().info("=" * 64)

    def save(self, rep):
        path = self.results_path.format(label=self.label)
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as f:
                yaml.safe_dump(rep, f, sort_keys=False)
            self.get_logger().info(f"Saved report to {path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save report: {e}")

    def escalate(self, reason):
        self.get_logger().error("=" * 60)
        self.get_logger().error(f"HARD FAILURE: {reason}")
        self.get_logger().error("Requesting RViz HALT, waiting for a human.")
        self.get_logger().error("=" * 60)
        for _ in range(25):
            self.request_halt()
            self.publish_mode(MODE_DISABLED)
            time.sleep(0.05)

    def finish(self):
        if self.handoff:
            # leave the vehicle armed and holding home so the next pipeline stage can take over;
            # native FF resumes so the hold has feedforward after this process exits
            self.get_logger().info("Handoff: leaving vehicle position-holding at home.")
            self.publish_mode(MODE_POSITION, position=self.home_pos, quat=self.home_quat)
            self.set_native_ff_disabled(False)
            return
        self.get_logger().warn("Disarming (verification complete): HALT + DISABLED.")
        for _ in range(10):
            self.request_halt()
            self.publish_mode(MODE_DISABLED)
            time.sleep(0.05)
        self.set_native_ff_disabled(False)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerVerify()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=node.run, daemon=True)
    worker.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.finish()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
