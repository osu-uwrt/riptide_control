#! /usr/bin/env python3
#
# Model-based controller-gain designer for the Riptide controller.
#
# Instead of searching gains, this IDENTIFIES the (few) plant parameters it needs -- effective
# inertia and, for pitch, the metacentric stiffness -- by short experiments in the sim, then
# COMPUTES stable gains by pole placement. It never uses CAD mass (untrusted); the effective
# inertia it measures includes added mass, which CAD cannot give anyway.
#
# Physics (per angular axis, small angle):   I_eff*thetaddot + c*thetadot - k*theta = tau
#   - I_eff : effective inertia (rigid + added). MEASURED: apply known tau from rest, I = tau/alpha.
#   - k     : restoring/destabilizing stiffness. k>0 => unstable (pitch). MEASURED from free tip.
#   - c     : linear drag. Conservatively IGNORED in D (real drag only adds damping -> stays stable).
#
# PID design (roll/pitch/yaw), overdamped for stability (zeta >= 1):
#       P = I_eff*wn^2 + max(k,0)      D = 2*zeta*I_eff*wn      (I-gain kept small/unchanged)
#   The pitch P automatically clears the stability floor k. zeta is the ONE speed/stability knob.
#
# SMC design (x/y/z): sliding mode is inherently robust to mass uncertainty. We optionally set the
#   surface slope lambda = closed-loop bandwidth and keep the (working, robust) reaching gains.
#
# Safety is identical to the FF tools: recover with the trusted base_wrench; a hard failure asks
# the RViz ControlPanel to kill via <ns>/ff_tuner/halt.
#
# Run:  ros2 launch riptide_controllers2 model_based_designer.launch.py robot:=talos

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
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

from gain_apply import GainApplier

from queue import Queue, Empty

MODE_DISABLED = ControllerCommand.DISABLED
MODE_FEEDFORWARD = ControllerCommand.FEEDFORWARD
MODE_POSITION = ControllerCommand.POSITION

AXIS_NAMES = ["x", "y", "z", "roll", "pitch", "yaw"]

DISABLE_NATIVE_FF_PARAM = "disable_native_ff"
P_PARAM = "controller__PID__p_gains"
D_PARAM = "controller__PID__d_gains"
I_PARAM = "controller__PID__i_gains"
LAMBDA_PARAM = "controller__SMC__SMC_params__lambda"


class HardHaltException(Exception):
    pass


class ModelBasedDesigner(Node):

    def __init__(self):
        super().__init__("model_based_designer")
        cb = ReentrantCallbackGroup()

        self.robot = self.declare_parameter("robot", "talos").value
        self.vehicle_config = self.declare_parameter("vehicle_config", "").value

        # ---- design target (stability first) -------------------------------
        self.zeta = self.declare_parameter("zeta", 1.2).value            # >=1 overdamped, no overshoot
        # closed-loop bandwidth (rad/s) for the PID axes roll,pitch,yaw
        self.wn = list(self.declare_parameter("wn_pid", [2.5, 2.5, 2.0]).value)  # [roll,pitch,yaw]
        # safety margin on the measured pitch stiffness for the P floor (underestimating k is the
        # dangerous direction -> pad it so P always clears the instability)
        self.stiffness_margin = self.declare_parameter("stiffness_margin", 1.3).value
        # optional SMC surface slope for x,y,z; <=0 means "leave the current value unchanged"
        self.smc_lambda = list(self.declare_parameter("smc_lambda", [0.0, 0.0, 0.0]).value)

        # ---- identification experiment -------------------------------------
        # test torques (N*m) applied per angular axis for the inertia measurement
        self.test_torques = list(self.declare_parameter("test_torques", [1.5, 3.0]).value)
        self.id_repeats = self.declare_parameter("id_repeats", 2).value
        self.measure_window = self.declare_parameter("measure_window", 1.0).value
        self.stiffness_theta_max = self.declare_parameter("stiffness_theta_max", 0.15).value  # rad
        self.stiffness_window = self.declare_parameter("stiffness_window", 3.0).value

        # ---- settle / recovery / safety ------------------------------------
        # settle tolerances match ff_identifier's proven values: the CURRENT gains are exactly what
        # this tool exists to fix, so recovery must be satisfiable with a wobbly controller
        self.settle_pos = self.declare_parameter("settle_pos", 0.15).value
        self.settle_ang = self.declare_parameter("settle_ang", 0.15).value
        self.settle_lin_vel = self.declare_parameter("settle_lin_vel", 0.12).value
        self.settle_ang_vel = self.declare_parameter("settle_ang_vel", 0.25).value
        self.settle_hold = self.declare_parameter("settle_hold", 1.0).value
        self.recovery_timeout = self.declare_parameter("recovery_timeout", 25.0).value
        self.abort_tilt = self.declare_parameter("abort_tilt", 0.6).value      # rad, during experiments
        self.abort_speed = self.declare_parameter("abort_speed", 1.0).value
        self.abort_radius = self.declare_parameter("abort_radius", 1.5).value
        self.odom_timeout = self.declare_parameter("odom_timeout", 2.5).value
        self.hard_radius = self.declare_parameter("hard_radius", 2.5).value
        self.publish_rate = self.declare_parameter("publish_rate", 30.0).value

        # apply = set the params live on complete_controller (takes effect immediately);
        # persist = also patch the gain lines in the vehicle yaml, otherwise the next overseer
        # reload or stack restart reverts the tune to the yaml values
        self.apply_gains_flag = self.declare_parameter("apply", True).value
        self.persist_gains_flag = self.declare_parameter("persist", True).value
        self.results_path = self.declare_parameter(
            "results_path", os.path.expanduser("~/osu-uwrt/model_based_gains.yaml")).value

        # ---- inertia prior (sanity bounds on identification) ----------------
        # produced by stl_inertia_prior.py (uniform-density mesh scaled to measured mass);
        # falls back to the vehicle yaml's inertia entry if the file doesn't exist
        self.inertia_prior_path = self.declare_parameter(
            "inertia_prior_path", os.path.expanduser("~/osu-uwrt/stl_inertia_prior.yaml")).value
        # identified I_eff must land within [prior/bound, prior*bound] or we hard-fail:
        # a wildly-off measurement would scale straight into the P/D gains
        self.prior_bound = self.declare_parameter("prior_bound", 4.0).value

        # ---- config -------------------------------------------------------
        self.config_path, tree = self.load_config()
        self.base_wrench = [float(v) for v in tree["controller"]["feed_forward"]["base_wrench"]]
        self.cad_inertia = [float(v) for v in tree["inertia"]]          # prior, for comparison only
        self.cad_mass = float(tree["mass"])
        self.inertia_prior, self.prior_source = self.load_inertia_prior()
        self.gains = {                                                   # current arrays (preserved)
            P_PARAM: [float(v) for v in tree["controller"]["PID"]["p_gains"]],
            I_PARAM: [float(v) for v in tree["controller"]["PID"]["i_gains"]],
            D_PARAM: [float(v) for v in tree["controller"]["PID"]["d_gains"]],
            LAMBDA_PARAM: [float(v) for v in tree["controller"]["SMC"]["SMC_params"]["lambda"]],
        }

        # ---- ROS I/O ------------------------------------------------------
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
        # live apply via direct set_parameters + readback verify; yaml patch only to persist
        self.applier = GainApplier(self, self.config_path, callback_group=cb)

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
            raise RuntimeError(
                f"vehicle config not found (robot='{self.robot}', vehicle_config='{self.vehicle_config}')"
                " -- pass vehicle_config:=/path/to/talos.yaml")
        with open(path, "r") as f:
            return path, yaml.safe_load(f)

    def load_inertia_prior(self):
        """[Ixx, Iyy, Izz] prior for the identification sanity bounds, and where it came from."""
        path = os.path.expanduser(self.inertia_prior_path)
        if path and os.path.exists(path):
            try:
                with open(path, "r") as f:
                    prior = yaml.safe_load(f)
                diag = [float(v) for v in prior["inertia_diag"]]
                self.get_logger().info(f"Inertia prior from STL: {[round(v, 3) for v in diag]} ({path})")
                return diag, "STL"
            except Exception as e:
                self.get_logger().warn(f"Could not read inertia prior {path}: {e}; using vehicle yaml")
        self.get_logger().info(f"Inertia prior from vehicle yaml: {self.cad_inertia} "
                               "(run stl_inertia_prior.py for a mesh-based prior)")
        return list(self.cad_inertia), "vehicle yaml"

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
    # ROS helpers (shared with the FF tools)
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
    def pitch_of(quat):
        x, y, z, w = quat
        return math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))

    @classmethod
    def level_quat(cls, quat):
        half = cls.yaw_of(quat) / 2.0
        return np.array([0.0, 0.0, math.sin(half), math.cos(half)])

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
                    and lin_speed < self.settle_lin_vel
                    and ang_speed < self.settle_ang_vel):
                if hold_start is None:
                    hold_start = time.time()
                elif time.time() - hold_start >= self.settle_hold:
                    return
            else:
                hold_start = None
            time.sleep(dt)

    # ========================================================================
    # identification
    # ========================================================================
    def measure_initial_accel(self, extra_wrench):
        """FEEDFORWARD window applying base_wrench+extra_wrench; return a(0) per axis (or None)."""
        wrench = list(np.array(self.base_wrench) + np.array(extra_wrench))
        ts, vs = [], []
        start = time.time()
        dt = 1.0 / self.publish_rate
        while time.time() - start < self.measure_window:
            self.publish_mode(MODE_FEEDFORWARD)
            self.publish_ff(wrench)
            try:
                odom = self.get_odom(self.odom_timeout)
            except Empty:
                break
            pos, quat = self.pose_of(odom)
            ts.append(time.time() - start)
            vs.append(self.vel_of(odom))
            if (self.tilt_of(quat) > self.abort_tilt
                    or float(np.linalg.norm(self.vel_of(odom)[0:3])) > self.abort_speed
                    or float(np.linalg.norm(pos - self.home_pos)) > self.abort_radius):
                break
            time.sleep(dt)
        self.publish_ff(self.base_wrench)
        if len(ts) < 5:
            return None
        T = np.array(ts)
        V = np.array(vs)
        return np.array([np.polyfit(T, V[:, i], 2)[1] for i in range(6)])   # a(0): quadratic linear term

    def identify_inertia(self, axis):
        """I_eff = tau / delta_alpha, regressed over several test torques (both signs)."""
        self.recover_to_home()
        a_base = self.measure_initial_accel(np.zeros(6))
        if a_base is None:
            raise HardHaltException(f"could not measure baseline accel for {AXIS_NAMES[axis]}")
        taus, dalphas = [], []
        for mag in self.test_torques:
            for sign in (+1.0, -1.0):
                for _r in range(self.id_repeats):
                    self.recover_to_home()
                    extra = np.zeros(6)
                    extra[axis] = sign * mag
                    a = self.measure_initial_accel(extra)
                    if a is None:
                        continue
                    taus.append(sign * mag)
                    dalphas.append(a[axis] - a_base[axis])
        taus = np.array(taus)
        dalphas = np.array(dalphas)
        if len(taus) < 3:
            raise HardHaltException(
                f"inertia ID for {AXIS_NAMES[axis]}: only {len(taus)} usable windows (aborts/stale odom?)")
        # tau = I_eff * delta_alpha  ->  slope of tau-vs-delta_alpha through the origin
        I_eff = float(np.sum(taus * dalphas) / (np.sum(dalphas * dalphas) + 1e-12))
        prior = self.inertia_prior[axis - 3]
        if not math.isfinite(I_eff) or I_eff <= 0.0:
            raise HardHaltException(
                f"inertia ID for {AXIS_NAMES[axis]}: unphysical I_eff={I_eff:.4f} "
                f"(vehicle did not respond to the test torques?)")
        if prior > 0.0 and not (prior / self.prior_bound <= I_eff <= prior * self.prior_bound):
            raise HardHaltException(
                f"inertia ID for {AXIS_NAMES[axis]}: I_eff={I_eff:.4f} outside sanity bounds "
                f"[{prior / self.prior_bound:.4f}, {prior * self.prior_bound:.4f}] "
                f"from {self.prior_source} prior {prior:.4f} -- refusing to design gains from it")
        self.get_logger().info(
            f"  {AXIS_NAMES[axis]}: I_eff = {I_eff:.4f} kg*m^2  ({self.prior_source} prior {prior:.4f}, "
            f"{len(taus)} samples)")
        return I_eff

    def identify_pitch_stiffness(self, I_pitch):
        """Let pitch tip freely in FF; fit thetaddot = (k/I)*theta -> k. k>0 => unstable."""
        self.recover_to_home()
        ts, thetas, rates = [], [], []
        start = time.time()
        dt = 1.0 / self.publish_rate
        while time.time() - start < self.stiffness_window:
            self.publish_mode(MODE_FEEDFORWARD)
            self.publish_ff(self.base_wrench)
            try:
                odom = self.get_odom(self.odom_timeout)
            except Empty:
                break
            _pos, quat = self.pose_of(odom)
            theta = self.pitch_of(quat)
            ts.append(time.time() - start)
            thetas.append(theta)
            rates.append(self.vel_of(odom)[4])       # pitch rate
            if abs(theta) > self.stiffness_theta_max:
                break
            time.sleep(dt)
        self.publish_ff(self.base_wrench)
        if len(ts) < 8:
            self.get_logger().warn("  pitch stiffness: too few samples, assuming k=0")
            return 0.0
        T = np.array(ts)
        theta = np.array(thetas)
        thetaddot = np.gradient(np.array(rates), T)   # d(pitch rate)/dt
        # a through-origin fit on near-zero theta is pure noise (0/0) -> would put NaN/garbage
        # into k and from there into the P gain; treat "never tipped" as k=0 instead
        if float(np.max(np.abs(theta))) < 0.02:
            self.get_logger().warn("  pitch stiffness: pitch never left level (<1.1deg), assuming k=0")
            return 0.0
        # fit thetaddot = s*theta through origin ; k = I*s
        s = float(np.sum(theta * thetaddot) / (np.sum(theta * theta) + 1e-12))
        k = I_pitch * s
        if not math.isfinite(k):
            self.get_logger().warn("  pitch stiffness: fit produced non-finite k, assuming k=0")
            return 0.0
        self.get_logger().info(f"  pitch stiffness k = {k:.3f} N*m/rad "
                               f"({'UNSTABLE' if k > 0 else 'stable'}); reached "
                               f"{math.degrees(theta[-1]):.0f}deg")
        return k

    # ========================================================================
    # design
    # ========================================================================
    def design(self, I_eff, k_pitch):
        """Pole-place overdamped PID for roll/pitch/yaw; optionally set SMC lambda for x/y/z."""
        p = list(self.gains[P_PARAM])
        d = list(self.gains[D_PARAM])
        lam = list(self.gains[LAMBDA_PARAM])
        for j, axis in enumerate((3, 4, 5)):           # roll, pitch, yaw
            I = I_eff[axis]
            wn = self.wn[j]
            # only pitch gets the stability-floor boost, padded by the stiffness margin
            k = self.stiffness_margin * max(k_pitch, 0.0) if axis == 4 else 0.0
            p[axis] = I * wn * wn + k                   # P = I*wn^2 + margin*max(k,0)
            d[axis] = 2.0 * self.zeta * I * wn          # D = 2*zeta*I*wn  (drag ignored -> >= zeta damped)
        for axis in (0, 1, 2):                          # x, y, z SMC surface (optional)
            if self.smc_lambda[axis] > 0:
                lam[axis] = self.smc_lambda[axis]
        return {P_PARAM: p, D_PARAM: d, I_PARAM: list(self.gains[I_PARAM]), LAMBDA_PARAM: lam}

    # ========================================================================
    # driver
    # ========================================================================
    def run(self):
        try:
            self.startup()
            self.get_logger().info("=== IDENTIFYING effective inertia (roll, pitch, yaw) ===")
            I_eff = list(self.cad_mass for _ in range(3)) + [0.0, 0.0, 0.0]
            for axis in (3, 4, 5):
                I_eff[axis] = self.identify_inertia(axis)
            self.get_logger().info("=== IDENTIFYING pitch stiffness ===")
            k_pitch = self.identify_pitch_stiffness(I_eff[4])

            self.get_logger().info("=== DESIGNING gains (overdamped, zeta=%.2f) ===" % self.zeta)
            new_gains = self.design(I_eff, k_pitch)
            for name, vals in new_gains.items():
                if not all(math.isfinite(v) for v in vals):
                    raise HardHaltException(f"non-finite value in designed {name}: {vals}")
            self.report(new_gains, I_eff, k_pitch)

            arrays = {P_PARAM: new_gains[P_PARAM], D_PARAM: new_gains[D_PARAM]}
            if new_gains[LAMBDA_PARAM] != self.gains[LAMBDA_PARAM]:
                arrays[LAMBDA_PARAM] = new_gains[LAMBDA_PARAM]
            if self.apply_gains_flag:
                if self.applier.apply(arrays):
                    self.get_logger().info("APPLIED live to complete_controller (verified).")
                else:
                    self.get_logger().error("Gain APPLY FAILED -- results saved to file only.")
            if self.persist_gains_flag:
                self.applier.persist(arrays)
            self.save(new_gains, I_eff, k_pitch)
        except HardHaltException as e:
            self.escalate(str(e))
            return
        except Exception as e:
            self.get_logger().error(f"Design aborted: {e}")
        self.disarm("done")

    def startup(self):
        self.get_logger().info("Starting model-based designer. ARM in RViz; stage with clearance.")
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

    def report(self, g, I_eff, k):
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Effective inertia [roll,pitch,yaw] = "
                               f"{[round(I_eff[a], 4) for a in (3, 4, 5)]}  "
                               f"({self.prior_source} prior {[round(v, 4) for v in self.inertia_prior]})")
        self.get_logger().info(f"Pitch stiffness k = {k:.3f} N*m/rad")
        for axis in (3, 4, 5):
            self.get_logger().info(
                f"  {AXIS_NAMES[axis]:5s}  P {self.gains[P_PARAM][axis]:.2f} -> {g[P_PARAM][axis]:.2f}   "
                f"D {self.gains[D_PARAM][axis]:.3f} -> {g[D_PARAM][axis]:.3f}")
        for axis in (0, 1, 2):
            if g[LAMBDA_PARAM][axis] != self.gains[LAMBDA_PARAM][axis]:
                self.get_logger().info(f"  {AXIS_NAMES[axis]:5s}  lambda "
                                       f"{self.gains[LAMBDA_PARAM][axis]:.3f} -> {g[LAMBDA_PARAM][axis]:.3f}")
        self.get_logger().info("=" * 60)

    def save(self, g, I_eff, k):
        try:
            os.makedirs(os.path.dirname(self.results_path), exist_ok=True)
            with open(self.results_path, "w") as f:
                yaml.safe_dump({
                    "robot": self.robot, "zeta": self.zeta, "wn_pid": self.wn,
                    "effective_inertia_rpy": [float(I_eff[a]) for a in (3, 4, 5)],
                    "pitch_stiffness_k": float(k),
                    "p_gains": [float(v) for v in g[P_PARAM]],
                    "d_gains": [float(v) for v in g[D_PARAM]],
                    "smc_lambda": [float(v) for v in g[LAMBDA_PARAM]],
                }, f)
            self.get_logger().info(f"Saved to {self.results_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save: {e}")

    def escalate(self, reason):
        self.get_logger().error("=" * 60)
        self.get_logger().error(f"HARD FAILURE: {reason}")
        self.get_logger().error("Requesting RViz HALT, waiting for a human.")
        self.get_logger().error("=" * 60)
        for _ in range(25):
            self.request_halt()
            self.publish_mode(MODE_DISABLED)
            time.sleep(0.05)

    def disarm(self, reason):
        self.get_logger().warn(f"Disarming ({reason}): HALT + DISABLED.")
        for _ in range(10):
            self.request_halt()
            self.publish_mode(MODE_DISABLED)
            time.sleep(0.05)
        self.set_native_ff_disabled(False)


def main(args=None):
    rclpy.init(args=args)
    node = ModelBasedDesigner()
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
