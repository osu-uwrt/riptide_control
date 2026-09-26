#! /usr/bin/env python3
#
# Optuna-based feedforward (FF) auto-tuner for the Riptide controller.
#
# Automates what the RViz FeedforwardPanel does by hand: searches the 6-DOF static
# feedforward wrench [fx, fy, fz, tx, ty, tz] to minimize how much the vehicle drifts
# in pure FEEDFORWARD mode (ideal FF -> zero net force -> robot holds still).
#
# Per trial:  RECOVER (position-control back to home with the *trusted* FF)
#          -> APPLY   (FEEDFORWARD mode, publish the *candidate* FF)
#          -> MEASURE (integrate drift / speed for trial_duration seconds)
#          -> SCORE   (combo cost: integrated speed + displacement + residual speed)
#
# Safety escalation ladder (works on the real robot, not just sim):
#   soft limit exceeded mid-trial  -> stop candidate, penalize trial, RECOVER
#   recovery exceeds hard limit/timeout -> assert software kill (disarm) and HALT,
#                                          wait for a human (no auto-resume)
# The operator arms the vehicle in RViz; this node never asserts *enable*, only kill.
#
# Requires: pip install optuna
# Run:  ros2 run riptide_controllers2 ff_optuna_tuner.py --ros-args -r __ns:=/talos

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
from std_msgs.msg import Empty as EmptyMsg   # aliased: `Empty` below is the queue exception
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from queue import Queue, Empty

import optuna

# Control modes (riptide_msgs2/ControllerCommand)
MODE_DISABLED = ControllerCommand.DISABLED
MODE_FEEDFORWARD = ControllerCommand.FEEDFORWARD
MODE_POSITION = ControllerCommand.POSITION

# overseer parameter that tells it to stop republishing base_wrench on FF_body_force
DISABLE_NATIVE_FF_PARAM = "disable_native_ff"

# penalty cost returned for a trial that tripped a soft safety limit
SOFT_ABORT_COST = 1.0e6

PARAM_NAMES = ["fx", "fy", "fz", "tx", "ty", "tz"]


class HardHaltException(Exception):
    """Raised when the vehicle cannot be recovered; halts the batch and waits for a human."""


class FFOptunaTuner(Node):

    def __init__(self):
        super().__init__("ff_optuna_tuner")

        cb = ReentrantCallbackGroup()

        # ---- parameters -----------------------------------------------------
        self.robot = self.declare_parameter("robot", "talos").value
        self.vehicle_config = self.declare_parameter("vehicle_config", "").value

        self.n_trials = self.declare_parameter("n_trials", 200).value
        self.trial_duration = self.declare_parameter("trial_duration", 5.0).value

        # settle / recovery
        self.settle_pos = self.declare_parameter("settle_pos", 0.15).value         # m
        self.settle_ang = self.declare_parameter("settle_ang", 0.15).value         # rad
        # velocity gates must sit ABOVE the odometry velocity-noise floor (~0.1 in sim),
        # otherwise a robot that is already home never counts as "settled"
        self.settle_lin_vel = self.declare_parameter("settle_lin_vel", 0.12).value  # m/s
        self.settle_ang_vel = self.declare_parameter("settle_ang_vel", 0.25).value  # rad/s
        self.settle_hold = self.declare_parameter("settle_hold", 1.0).value        # s
        self.recovery_timeout = self.declare_parameter("recovery_timeout", 25.0).value

        # soft limits (abort a trial, then recover)
        self.abort_radius = self.declare_parameter("abort_radius", 1.5).value      # m
        self.abort_speed = self.declare_parameter("abort_speed", 1.0).value        # m/s
        self.abort_tilt = self.declare_parameter("abort_tilt", 0.785).value        # rad (~45 deg)
        self.odom_timeout = self.declare_parameter("odom_timeout", 2.5).value      # s

        # hard limit (kill + halt)
        self.hard_radius = self.declare_parameter("hard_radius", 2.5).value        # m

        # Search is a LOCAL perturbation around the trusted base_wrench (base +/- delta), not an
        # absolute range, so candidates stay gentle/safe and Optuna refines near known-good values.
        self.delta_force = self.declare_parameter("delta_force", 1.5).value        # +/- N on fx,fy,fz
        self.delta_torque = self.declare_parameter("delta_torque", 0.75).value     # +/- N*m on tx,ty,tz

        # cost weights
        self.axis_weights = list(self.declare_parameter(
            "axis_weights", [0.5, 0.5, 1.0, 10.0, 10.0, 10.0]).value)
        self.w_speed = self.declare_parameter("w_speed", 1.0).value
        self.w_disp = self.declare_parameter("w_disp", 5.0).value
        self.w_end = self.declare_parameter("w_end", 10.0).value
        self.disp_rot_weight = self.declare_parameter("disp_rot_weight", 2.0).value

        # study persistence + output
        self.study_name = self.declare_parameter("study_name", "ff_tuning").value
        storage = self.declare_parameter("storage", "").value
        self.storage = storage if storage else \
            "sqlite:///" + os.path.expanduser("~/ff_optuna_study.db")
        self.results_path = self.declare_parameter(
            "results_path", os.path.expanduser("~/osu-uwrt/ff_optuna_best.yaml")).value
        self.write_config = self.declare_parameter("write_config", False).value

        self.publish_rate = self.declare_parameter("publish_rate", 30.0).value     # Hz cmd republish

        # ---- trusted FF (recovery wrench) read from the vehicle config ------
        self.config_path, self.base_wrench = self.load_base_wrench()
        self.get_logger().info(f"Trusted base_wrench (recovery FF): {self.base_wrench}")

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
        # Ask the RViz ControlPanel to perform an authoritative disable/kill. The tuner cannot
        # hold a software kill itself (the sim honors only kill switch id 1, which the panel
        # floods to keep the vehicle armed), so the panel owns arming/kill and we request it.
        self.halt_pub = self.create_publisher(EmptyMsg, "ff_tuner/halt",
                                              qos_profile_system_default)

        self.overseer_param_client = self.create_client(
            SetParameters, "controller_overseer/set_parameters", callback_group=cb)

        self.halted = False

    # ========================================================================
    # config loading
    # ========================================================================
    def load_base_wrench(self):
        """Find the vehicle yaml and read controller.feed_forward.base_wrench."""
        path = self.vehicle_config
        if not path:
            try:
                share = get_package_share_directory("riptide_descriptions2")
                path = os.path.join(share, "config", self.robot + ".yaml")
                path = self.prefer_source_config(share, path)
            except Exception:
                path = ""
        try:
            with open(path, "r") as f:
                tree = yaml.safe_load(f)
            bw = tree["controller"]["feed_forward"]["base_wrench"]
            return path, [float(v) for v in bw]
        except Exception as e:
            self.get_logger().error(
                f"Could not read base_wrench from '{path}': {e}. Falling back to zeros "
                "(recovery FF will be zero!). Pass vehicle_config to fix this.")
            return path, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def prefer_source_config(self, share_dir, fallback):
        """If running from a colcon install, prefer the editable src yaml (mirrors overseer)."""
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
    # subscriptions
    # ========================================================================
    def odom_cb(self, msg):
        if self.odom_queue.full():
            try:
                self.odom_queue.get_nowait()
            except Empty:
                pass
        self.odom_queue.put_nowait(msg)

    def get_odom(self, timeout):
        """Return the latest odometry, blocking up to `timeout` s. Raises Empty on stale odom."""
        # drain any stale msg so we always get a fresh one
        if not self.odom_queue.empty():
            try:
                self.odom_queue.get_nowait()
            except Empty:
                pass
        return self.odom_queue.get(True, timeout)

    # ========================================================================
    # publishing helpers
    # ========================================================================
    def publish_ff(self, wrench):
        msg = Twist()
        msg.linear = Vector3(x=float(wrench[0]), y=float(wrench[1]), z=float(wrench[2]))
        msg.angular = Vector3(x=float(wrench[3]), y=float(wrench[4]), z=float(wrench[5]))
        self.ff_pub.publish(msg)

    def publish_mode(self, mode, position=None, quat=None):
        """Publish a ControllerCommand on both linear and angular channels (mirrors ControlPanel)."""
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
        # zero commanded angular velocity for a pure hold
        ang.setpoint_vect = Vector3(x=0.0, y=0.0, z=0.0)

        self.ctrl_lin_pub.publish(lin)
        self.ctrl_ang_pub.publish(ang)

    def request_halt(self):
        """Ask the RViz ControlPanel to perform an authoritative disable/kill (it owns arming)."""
        self.halt_pub.publish(EmptyMsg())

    def set_native_ff_disabled(self, disabled):
        """Tell the overseer to stop/resume republishing base_wrench on FF_body_force."""
        if not self.overseer_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "controller_overseer/set_parameters unavailable; cannot toggle "
                f"{DISABLE_NATIVE_FF_PARAM}. The overseer may fight our FF at ~1 Hz.")
            return
        req = SetParameters.Request()
        p = Parameter()
        p.name = DISABLE_NATIVE_FF_PARAM
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(disabled))
        req.parameters = [p]
        self.overseer_param_client.call_async(req)
        self.get_logger().info(f"Set overseer {DISABLE_NATIVE_FF_PARAM}={disabled}")

    # ========================================================================
    # odometry math
    # ========================================================================
    @staticmethod
    def pose_of(odom):
        p = odom.pose.pose.position
        q = odom.pose.pose.orientation
        return np.array([p.x, p.y, p.z]), np.array([q.x, q.y, q.z, q.w])

    @staticmethod
    def vel_of(odom):
        lv = odom.twist.twist.linear
        av = odom.twist.twist.angular
        return np.array([lv.x, lv.y, lv.z]), np.array([av.x, av.y, av.z])

    @staticmethod
    def quat_angle(q1, q2):
        """Angle (rad) between two unit quaternions [x,y,z,w]."""
        d = abs(float(np.dot(q1, q2)))
        d = min(1.0, max(-1.0, d))
        return 2.0 * math.acos(d)

    @staticmethod
    def tilt_of(quat):
        """Angle (rad) between body +z and world +z, i.e. how far from upright."""
        x, y, z, w = quat
        # world-z component of the body-z axis after rotation
        cos_tilt = 1.0 - 2.0 * (x * x + y * y)
        cos_tilt = min(1.0, max(-1.0, cos_tilt))
        return math.acos(cos_tilt)

    @staticmethod
    def yaw_of(quat):
        """Yaw (rad) from a [x,y,z,w] quaternion."""
        x, y, z, w = quat
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @classmethod
    def level_quat(cls, quat):
        """Return a level quaternion (roll=pitch=0) preserving only the yaw of `quat`.

        The vehicle is open-loop unstable in pitch, so recovery must target a level,
        stable attitude rather than whatever (possibly pitched) orientation it drifted to.
        """
        half = cls.yaw_of(quat) / 2.0
        return np.array([0.0, 0.0, math.sin(half), math.cos(half)])

    @staticmethod
    def rpy_deg(quat):
        """(roll, pitch, yaw) in degrees from a [x,y,z,w] quaternion, for logging."""
        x, y, z, w = quat
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sinp)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return tuple(round(math.degrees(a), 1) for a in (roll, pitch, yaw))

    # ========================================================================
    # state-machine steps
    # ========================================================================
    def recover_to_home(self, home_pos, home_quat):
        """Position-control to home using the TRUSTED FF. Returns on settle; raises on hard fail."""
        self.get_logger().info("RECOVER: driving to home (level) with trusted FF...")
        start = time.time()
        hold_start = None
        last_log = 0.0
        dt = 1.0 / self.publish_rate
        while True:
            if time.time() - start > self.recovery_timeout:
                raise HardHaltException(
                    f"recovery timed out after {self.recovery_timeout:.0f}s")

            self.publish_mode(MODE_POSITION, position=home_pos, quat=home_quat)
            self.publish_ff(self.base_wrench)

            try:
                odom = self.get_odom(self.odom_timeout)
            except Empty:
                raise HardHaltException("odometry went stale during recovery")

            pos, quat = self.pose_of(odom)
            lv, av = self.vel_of(odom)
            pos_err = float(np.linalg.norm(pos - home_pos))
            ang_err = self.quat_angle(quat, home_quat)
            lin_speed = float(np.linalg.norm(lv))
            ang_speed = float(np.linalg.norm(av))

            now = time.time()
            if now - last_log > 3.0:
                last_log = now
                self.get_logger().info(
                    f"  recovering: pos_err={pos_err:.2f}m ang_err={math.degrees(ang_err):.0f}deg "
                    f"lin_v={lin_speed:.2f} ang_v={ang_speed:.2f} rpy(deg)={self.rpy_deg(quat)}")

            if pos_err > self.hard_radius:
                raise HardHaltException(
                    f"diverging during recovery (pos_err={pos_err:.2f} > "
                    f"hard_radius={self.hard_radius:.2f})")

            settled = (pos_err < self.settle_pos and ang_err < self.settle_ang
                       and lin_speed < self.settle_lin_vel and ang_speed < self.settle_ang_vel)
            if settled:
                if hold_start is None:
                    hold_start = time.time()
                elif time.time() - hold_start >= self.settle_hold:
                    self.get_logger().info(
                        f"RECOVER: settled (pos_err={pos_err:.3f}, ang_err={ang_err:.3f}).")
                    return
            else:
                hold_start = None
            time.sleep(dt)

    def measure(self, wrench, home_pos, home_quat):
        """FEEDFORWARD mode with the candidate wrench. Integrate drift. Returns (cost, aborted)."""
        self.get_logger().info(f"APPLY/MEASURE: candidate FF = "
                               f"[{', '.join(f'{w:.2f}' for w in wrench)}]")
        integ = np.zeros(6)            # integrated |velocity| per axis
        max_disp = 0.0
        last_lv = np.zeros(3)
        last_av = np.zeros(3)
        start = time.time()
        prev_t = None
        aborted = False

        while time.time() - start < self.trial_duration:
            self.publish_mode(MODE_FEEDFORWARD)
            self.publish_ff(wrench)

            try:
                odom = self.get_odom(self.odom_timeout)
            except Empty:
                self.get_logger().warn("MEASURE: odometry stale -> soft abort")
                aborted = True
                break

            now = time.time()
            dt = 0.0 if prev_t is None else (now - prev_t)
            prev_t = now

            pos, quat = self.pose_of(odom)
            lv, av = self.vel_of(odom)
            last_lv, last_av = lv, av

            integ[0:3] += np.abs(lv) * dt
            integ[3:6] += np.abs(av) * dt

            disp = float(np.linalg.norm(pos - home_pos))
            max_disp = max(max_disp, disp)
            tilt = self.tilt_of(quat)
            speed = float(np.linalg.norm(lv))

            if disp > self.abort_radius or speed > self.abort_speed or tilt > self.abort_tilt:
                self.get_logger().warn(
                    f"MEASURE: soft limit hit (disp={disp:.2f}, speed={speed:.2f}, "
                    f"tilt={math.degrees(tilt):.0f}deg) -> abort")
                aborted = True
                break

        # stop commanding the candidate; hand control back to the trusted FF
        self.publish_ff(self.base_wrench)

        if aborted:
            return SOFT_ABORT_COST + max_disp, True

        # combo cost: integrated speed (axis-weighted) + displacement + residual end-speed
        speed_term = float(np.dot(self.axis_weights, integ))
        end_pos_err = float(np.linalg.norm(self.pose_of(odom)[0] - home_pos))
        end_rot_err = self.quat_angle(self.pose_of(odom)[1], home_quat)
        disp_term = end_pos_err + self.disp_rot_weight * end_rot_err
        end_speed = float(np.linalg.norm(last_lv)) + float(np.linalg.norm(last_av))

        cost = (self.w_speed * speed_term
                + self.w_disp * disp_term
                + self.w_end * end_speed)
        self.get_logger().info(
            f"SCORE: cost={cost:.3f} (speed_term={speed_term:.3f}, disp_term={disp_term:.3f}, "
            f"end_speed={end_speed:.3f}, max_disp={max_disp:.2f})")
        return cost, False

    # ========================================================================
    # optuna objective + driver
    # ========================================================================
    def suggest_wrench(self, trial):
        bw = self.base_wrench
        margins = [self.delta_force, self.delta_force, self.delta_force,
                   self.delta_torque, self.delta_torque, self.delta_torque]
        return [trial.suggest_float(name, bw[i] - margins[i], bw[i] + margins[i])
                for i, name in enumerate(PARAM_NAMES)]

    def objective(self, trial):
        wrench = self.suggest_wrench(trial)
        # recover before every trial so all measurements start from the same home
        self.recover_to_home(self.home_pos, self.home_quat)
        cost, aborted = self.measure(wrench, self.home_pos, self.home_quat)
        trial.set_user_attr("aborted", aborted)
        return cost

    def run_study(self):
        try:
            self.startup()
        except HardHaltException as e:
            self.escalate(str(e))
            return

        sampler = optuna.samplers.TPESampler(seed=0)
        study = optuna.create_study(
            study_name=self.study_name, storage=self.storage,
            direction="minimize", sampler=sampler, load_if_exists=True)

        # seed trial 0 with the known-good base_wrench so we never do worse than today
        study.enqueue_trial(dict(zip(PARAM_NAMES, self.base_wrench)))

        try:
            study.optimize(self.objective, n_trials=self.n_trials)
        except HardHaltException as e:
            self.escalate(str(e))
            return
        except Exception as e:
            self.get_logger().error(f"Tuning aborted: {e}")
            self.disarm("unexpected error")
            return

        self.finish(study)

    def startup(self):
        """Wait for fresh odometry, take ownership of FF, and capture the home pose."""
        self.get_logger().info(
            "Starting FF Optuna tuner. ENSURE the vehicle is armed in RViz and "
            "positioned with clearance > abort_radius on all sides.")
        self.set_native_ff_disabled(True)

        # wait for a fresh odom and a brief settled window to define home
        deadline = time.time() + 30.0
        while True:
            try:
                odom = self.get_odom(self.odom_timeout)
            except Empty:
                if time.time() > deadline:
                    raise HardHaltException("no odometry at startup")
                continue
            break

        self.home_pos, raw_quat = self.pose_of(odom)
        # Target a LEVEL attitude (roll=pitch=0, keep yaw): the vehicle is unstable in pitch,
        # so recovering toward a pitched orientation would never settle.
        self.home_quat = self.level_quat(raw_quat)
        self.get_logger().info(
            f"HOME captured: pos={np.round(self.home_pos, 3).tolist()} "
            f"raw_rpy(deg)={self.rpy_deg(raw_quat)} -> leveled_rpy(deg)={self.rpy_deg(self.home_quat)}")

    def escalate(self, reason):
        self.halted = True
        self.get_logger().error("=" * 60)
        self.get_logger().error(f"HARD FAILURE: {reason}")
        self.get_logger().error("Requesting RViz ControlPanel HALT (kill) and stopping. Waiting "
                                "for a human to re-enable in RViz. Ctrl-C to exit.")
        self.get_logger().error("=" * 60)
        # ask the panel to kill (authoritative) and command DISABLED as a node-side fallback
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
        # let the overseer resume its native FF behavior
        self.set_native_ff_disabled(False)

    def finish(self, study):
        best = study.best_params
        best_wrench = [best[k] for k in PARAM_NAMES]
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"TUNING COMPLETE. best cost = {study.best_value:.4f}")
        self.get_logger().info(f"best base_wrench = {best_wrench}")
        self.get_logger().info("=" * 60)

        # leave the vehicle safe
        self.disarm("tuning complete")

        # save the result (never silently edit the vehicle config)
        try:
            os.makedirs(os.path.dirname(self.results_path), exist_ok=True)
            with open(self.results_path, "w") as f:
                yaml.safe_dump({
                    "robot": self.robot,
                    "best_value": float(study.best_value),
                    "base_wrench": [float(v) for v in best_wrench],
                    "original_base_wrench": self.base_wrench,
                    "n_trials": len(study.trials),
                }, f)
            self.get_logger().info(f"Saved best result to {self.results_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save results: {e}")

        if self.write_config and self.config_path:
            self.write_base_wrench_to_config(best_wrench)

    def write_base_wrench_to_config(self, wrench):
        """Patch base_wrench in the vehicle yaml (only when write_config:=true)."""
        try:
            with open(self.config_path, "r") as f:
                tree = yaml.safe_load(f)
            tree["controller"]["feed_forward"]["base_wrench"] = [float(v) for v in wrench]
            with open(self.config_path, "w") as f:
                yaml.safe_dump(tree, f, default_flow_style=False, sort_keys=False)
            self.get_logger().warn(f"WROTE new base_wrench into {self.config_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write config: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FFOptunaTuner()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    worker = threading.Thread(target=node.run_study, daemon=True)
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
