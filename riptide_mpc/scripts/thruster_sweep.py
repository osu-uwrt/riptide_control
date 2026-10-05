#!/usr/bin/env python3
"""Thruster-model sweep on the real vehicle: which MPC thruster model holds best?

Holds the pose the vehicle is at when it starts and, for each config, sets the
running MPC's thruster_model.* parameters live, lets it settle, then measures the
hold (attitude/position error vs the MPC reference, body-rate wobble, thrust use
and chatter). The baseline (the MPC's model at start) is re-flown every few
configs to show drift (battery). The baseline is restored at the end, on kill,
on a divergence and on Ctrl-C.

    python3 thruster_sweep.py [--robot talos] [--configs sweep.yaml] [--settle 5] [--measure 15]

Run with the MPC active, the vehicle armed at depth, kill switch in hand, and no
autonomy tree running. --configs: a YAML list of {name: ..., <key>: value}, where a key is a
thruster_model key (delay, reverse_scale, ...) or a full MPC parameter name weights.<name>
(e.g. weights.attitude: [3000, 1500, 300]). --preset weights sweeps the pitch weights.
"""
import argparse
import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.signals import SignalHandlerOptions
from riptide_msgs2.msg import ControllerCommand
from std_msgs.msg import Bool, Float32MultiArray

# forward_scale / reverse_scale multiply the model file's (possibly per-thruster) scales; 1 = the file's.
KEYS = ["delay", "rise_time_constant", "fall_time_constant", "slew_rate", "force_deadband",
        "forward_scale", "reverse_scale", "efficiencies", "startup_time_constant", "startup_force"]

# Hypotheses from the 2026-10-03 hold: actuation slower than modelled, reverse thrust
# weaker (pitch comes from the upper surge pair pushing against the lower one),
# thrust weaker overall, near-zero commands ineffective, lower surge pair weaker.
DEFAULT_CONFIGS = [
    {"name": "delay_0.15", "delay": 0.15},
    {"name": "delay_0.20", "delay": 0.20},
    {"name": "rise_0.15", "rise_time_constant": 0.15, "fall_time_constant": 0.12},
    {"name": "reverse_0.8", "reverse_scale": 0.8},
    {"name": "reverse_0.6", "reverse_scale": 0.6},
    {"name": "both_0.8", "forward_scale": 0.8, "reverse_scale": 0.8},
    {"name": "deadband_0.3", "force_deadband": 0.3},
    {"name": "lower_surge_0.8", "efficiencies": [1, 1, 1, 1, 1, 1, 0.8, 0.8]},
]

# Pitch-wobble candidates (body axes: index 1 is pitch). Defaults: attitude [3000, 3000, 300],
# angular_damping [10, 10, 10], thrust 0.001, thrust_rate 0.005.
WEIGHT_CONFIGS = [
    {"name": "pitch_att_1500", "weights.attitude": [3000, 1500, 300]},
    {"name": "pitch_att_800", "weights.attitude": [3000, 800, 300]},
    {"name": "pitch_damp_40", "weights.angular_damping": [10, 40, 10]},
    {"name": "pitch_damp_100", "weights.angular_damping": [10, 100, 10]},
    {"name": "thrust_rate_0.02", "weights.thrust_rate": 0.02},
    {"name": "thrust_rate_0.05", "weights.thrust_rate": 0.05},
    {"name": "att1500_damp40", "weights.attitude": [3000, 1500, 300], "weights.angular_damping": [10, 40, 10]},
]
PRESETS = {"thrusters": DEFAULT_CONFIGS, "weights": WEIGHT_CONFIGS}


def full_name(key):
    """Config key -> MPC parameter: bare names are thruster_model keys."""
    if key.startswith("weights."):
        return key
    if key in KEYS:
        return f"thruster_model.{key}"
    raise RuntimeError(f"unknown key '{key}' (thruster_model keys: {KEYS}, or weights.<name>)")


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def quat_log(q):
    q = q / np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    s = np.linalg.norm(q[1:])
    return 2 * q[1:] if s < 1e-12 else 2 * math.atan2(s, q[0]) * q[1:] / s


def att_error(q, q_ref):  # body-axis rotation from reference to q, as the MPC penalizes it
    conj = np.array([q_ref[0], -q_ref[1], -q_ref[2], -q_ref[3]])
    return quat_log(quat_mul(conj, q))


def to_value(v):
    if isinstance(v, (list, tuple)):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=[float(x) for x in v])
    return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))


def from_value(v):
    if v.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(v.double_array_value)
    if v.type == ParameterType.PARAMETER_DOUBLE:
        return v.double_value
    raise RuntimeError("parameter missing on the MPC (is the MPC build new enough for live thruster_model/weights?)")


class Sweep:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("thruster_sweep", namespace=args.robot)
        mpc = f"/{args.robot}/mpc_controller"
        self.get_cli = self.node.create_client(GetParameters, f"{mpc}/get_parameters")
        self.set_cli = self.node.create_client(SetParametersAtomically, f"{mpc}/set_parameters_atomically")
        self.lin_pub = self.node.create_publisher(ControllerCommand, "controller/linear", 10)
        self.ang_pub = self.node.create_publisher(ControllerCommand, "controller/angular", 10)
        self.state = self.reference = self.thrust = None
        self.killed = None
        self.samples = []  # filled while measuring
        self.recording = False
        self.node.create_subscription(Odometry, "controller/mpc/state", self.on_state, 10)
        self.node.create_subscription(PoseStamped, "controller/mpc/reference", self.on_reference, 10)
        self.node.create_subscription(Float32MultiArray, "thruster_forces", self.on_thrust, 10)
        self.node.create_subscription(Bool, "state/kill", self.on_kill, 10)
        self.hold = None
        self.reference_time = -1e9

    # ---------- ROS plumbing
    def on_state(self, m):
        self.state = m
        if self.recording and self.reference is not None:
            p, o, w = m.pose.pose.position, m.pose.pose.orientation, m.twist.twist.angular
            r = self.reference.pose
            self.samples.append({
                "t": time.monotonic(),
                "p": [p.x, p.y, p.z], "q": [o.w, o.x, o.y, o.z], "w": [w.x, w.y, w.z],
                "rp": [r.position.x, r.position.y, r.position.z],
                "rq": [r.orientation.w, r.orientation.x, r.orientation.y, r.orientation.z],
                "u": list(self.thrust) if self.thrust is not None else None})

    def on_reference(self, m):
        self.reference = m
        self.reference_time = time.monotonic()

    def on_thrust(self, m):
        self.thrust = np.array(m.data)

    def on_kill(self, m):
        self.killed = m.data

    def spin_for(self, seconds, guard=False):
        """Spin, re-publishing the hold; with guard, return False on a divergence."""
        end = time.monotonic() + seconds
        bad_since = None
        rates = []  # (time, |rate|^2) over the last second: a diverging model shows up as fast chatter
        last_pub = 0
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.killed:
                raise Abort("vehicle killed")
            if time.monotonic() - last_pub > 0.5:
                self.publish_hold()
                last_pub = time.monotonic()
            if guard and self.state is not None and self.reference is not None:
                o, w = self.state.pose.pose.orientation, self.state.twist.twist.angular
                r = self.reference.pose.orientation
                err = np.linalg.norm(att_error(np.array([o.w, o.x, o.y, o.z]), np.array([r.w, r.x, r.y, r.z])))
                now = time.monotonic()
                rates = [r for r in rates if now - r[0] < 1.0] + [(now, w.x ** 2 + w.y ** 2 + w.z ** 2)]
                rate_rms = math.sqrt(sum(r[1] for r in rates) / len(rates))
                if err > math.radians(self.args.max_error_deg) or (len(rates) > 10 and rate_rms > self.args.max_rate):
                    bad_since = bad_since or time.monotonic()
                    if time.monotonic() - bad_since > 0.5:
                        return False
                else:
                    bad_since = None
        return True

    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"{client.srv_name} not available")
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        if not future.done():
            raise RuntimeError(f"{client.srv_name} timed out")
        return future.result()

    def get_params(self, names):
        res = self.call(self.get_cli, GetParameters.Request(names=list(names)))
        if len(res.values) != len(names):
            raise RuntimeError("MPC returned the wrong number of parameters")
        return {n: from_value(v) for n, v in zip(names, res.values)}

    def set_params(self, values):
        req = SetParametersAtomically.Request(
            parameters=[Parameter(name=n, value=to_value(v)) for n, v in values.items()])
        res = self.call(self.set_cli, req).result
        return res.successful, res.reason

    def publish_hold(self):
        if self.hold is None:
            return
        p, q = self.hold
        lin, ang = ControllerCommand(), ControllerCommand()
        lin.mode = ang.mode = ControllerCommand.POSITION
        lin.setpoint_vect.x, lin.setpoint_vect.y, lin.setpoint_vect.z = p
        ang.setpoint_quat.w, ang.setpoint_quat.x, ang.setpoint_quat.y, ang.setpoint_quat.z = q
        self.lin_pub.publish(lin)
        self.ang_pub.publish(ang)

    # ---------- sweep
    def wait_ready(self):
        print("Waiting for: MPC state + reference (MPC active), armed ...")
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            fresh = self.reference is not None and time.monotonic() - self.reference_time < 0.5
            if self.state is not None and fresh and self.killed is False:
                if self.state.pose.pose.position.z > -self.args.min_depth:
                    print(f"  too shallow ({self.state.pose.pose.position.z:.2f} m)", end="\r")
                    continue
                break
        p, o = self.state.pose.pose.position, self.state.pose.pose.orientation
        yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y ** 2 + o.z ** 2))
        self.hold = ((p.x, p.y, p.z), (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)))
        print(f"Holding [{p.x:.2f} {p.y:.2f} {p.z:.2f}], heading {math.degrees(yaw):.0f} deg")

    def metrics(self, samples):
        s = [x for x in samples if x["u"] is not None]
        if len(s) < 10:
            return None
        e = np.array([att_error(np.array(x["q"]), np.array(x["rq"])) for x in s])
        pe = np.array([np.subtract(x["p"], x["rp"]) for x in s])
        w = np.array([x["w"] for x in s])
        u = np.array([x["u"] for x in s])
        du = np.abs(np.diff(u, axis=0)).sum(1)
        big = np.abs(u) > 0.05
        flips = ((np.sign(u[1:]) != np.sign(u[:-1])) & big[1:] & big[:-1]).sum()
        secs = s[-1]["t"] - s[0]["t"]
        deg = 180 / math.pi
        m = {
            "roll_rms_deg": float(np.sqrt((e[:, 0] ** 2).mean()) * deg),
            "pitch_rms_deg": float(np.sqrt((e[:, 1] ** 2).mean()) * deg),
            "yaw_rms_deg": float(np.sqrt((e[:, 2] ** 2).mean()) * deg),
            "att_max_deg": float(np.linalg.norm(e, axis=1).max() * deg),
            "pos_rms_cm": float(np.sqrt((pe ** 2).sum(1).mean()) * 100),
            "pitch_rate_rms_dps": float(np.sqrt((w[:, 1] ** 2).mean()) * deg),
            "rate_rms_dps": float(np.sqrt((w ** 2).sum(1).mean()) * deg),
            "thrust_mean_N": float(np.abs(u).sum(1).mean()),
            "chatter_N": float(du.mean()),
            "flips_per_s": float(flips / secs) if secs > 0 else 0.0,
        }
        m["score"] = (m["roll_rms_deg"] + m["pitch_rms_deg"] + m["yaw_rms_deg"] + 0.2 * m["pos_rms_cm"]
                      + 0.05 * m["rate_rms_dps"])
        return m

    def run(self, configs):
        self.wait_ready()
        names = [full_name(k) for k in KEYS]
        for c in configs:
            for key in c:
                if key != "name" and full_name(key) not in names:
                    names.append(full_name(key))
        base = self.get_params(names)
        print("Baseline:", base)
        plan = [{"name": "baseline"}]
        for i, c in enumerate(configs):
            plan.append(c)
            if self.args.baseline_every and (i + 1) % self.args.baseline_every == 0 and i + 1 < len(configs):
                plan.append({"name": "baseline"})
        plan.append({"name": "baseline"})
        per_run = self.args.settle + self.args.measure
        print(f"{len(plan)} runs x {per_run:.0f} s = {len(plan) * per_run / 60:.1f} min")

        results = []
        try:
            for k, c in enumerate(plan):
                values = dict(base)
                values.update({full_name(key): v for key, v in c.items() if key != "name"})
                ok, reason = self.set_params(values)
                label = f"[{k + 1}/{len(plan)}] {c['name']}"
                if not ok:
                    print(f"{label}: rejected by the MPC: {reason}")
                    results.append({"name": c["name"], "status": "rejected", "reason": reason})
                    continue
                print(f"{label}: settling {self.args.settle:.0f} s ...")
                stable = self.spin_for(self.args.settle, guard=True)
                if stable:
                    self.samples, self.recording = [], True
                    stable = self.spin_for(self.args.measure, guard=True)
                    self.recording = False
                if not stable:
                    print(f"{label}: DIVERGED, restoring baseline")
                    self.set_params(base)
                    results.append({"name": c["name"], "status": "unstable", "config": c})
                    if c["name"] == "baseline":
                        raise Abort("baseline diverged")
                    self.spin_for(self.args.settle)
                    continue
                m = self.metrics(self.samples)
                results.append({"name": c["name"], "status": "ok" if m else "no data", "config": c, "metrics": m})
                if m:
                    print(f"{label}: score {m['score']:.2f}  pitch {m['pitch_rms_deg']:.2f} deg rms, "
                          f"pitch rate {m['pitch_rate_rms_dps']:.1f} deg/s, pos {m['pos_rms_cm']:.1f} cm, "
                          f"thrust {m['thrust_mean_N']:.0f} N, chatter {m['chatter_N']:.1f} N/tick")
        except (Abort, KeyboardInterrupt) as e:
            print(f"\nStopped: {e or 'Ctrl-C'}")
        finally:
            try:
                ok, reason = self.set_params(base)
                print("Baseline restored" if ok else f"RESTORE FAILED: {reason}")
            except Exception as e:  # keep going so the results are saved
                print(f"RESTORE FAILED: {e}  -> restart the MPC or set the parameters by hand")
            if not self.killed:
                self.publish_hold()
        self.report(base, results)

    def report(self, base, results):
        out = os.path.expanduser(self.args.output_dir)
        os.makedirs(out, exist_ok=True)
        path = os.path.join(out, time.strftime(f"{self.args.robot}_thruster_sweep_%Y%m%d_%H%M%S.json"))
        with open(path, "w") as f:
            json.dump({"baseline": base, "settle_s": self.args.settle, "measure_s": self.args.measure,
                       "results": results}, f, indent=2)
        ok = [r for r in results if r.get("metrics")]
        baselines = [r["metrics"]["score"] for r in ok if r["name"] == "baseline"]
        print(f"\n{'config':<18}{'score':>7}{'roll':>7}{'pitch':>7}{'yaw':>7}{'pos cm':>8}{'wy dps':>8}"
              f"{'thrust':>8}{'chat':>7}{'flip/s':>8}")
        for r in sorted(ok, key=lambda r: r["metrics"]["score"]):
            m = r["metrics"]
            print(f"{r['name']:<18}{m['score']:7.2f}{m['roll_rms_deg']:7.2f}{m['pitch_rms_deg']:7.2f}"
                  f"{m['yaw_rms_deg']:7.2f}{m['pos_rms_cm']:8.1f}{m['pitch_rate_rms_dps']:8.1f}"
                  f"{m['thrust_mean_N']:8.1f}{m['chatter_N']:7.1f}{m['flips_per_s']:8.1f}")
        for r in results:
            if r["status"] != "ok":
                print(f"{r['name']:<18}{r['status']}")
        if len(baselines) > 1:
            print(f"baseline score range {min(baselines):.2f} .. {max(baselines):.2f}: "
                  "differences smaller than this are noise/drift")
        print(f"Saved {path}")


class Abort(Exception):
    pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default="talos")
    ap.add_argument("--configs", help="YAML list of configs (overrides --preset)")
    ap.add_argument("--preset", choices=sorted(PRESETS), default="thrusters",
                    help="built-in configs: thrusters (thruster model hypotheses) or weights (pitch weights)")
    ap.add_argument("--settle", type=float, default=5.0, help="s after each change, not scored")
    ap.add_argument("--measure", type=float, default=15.0, help="s scored per config")
    ap.add_argument("--baseline-every", type=int, default=3, help="re-fly the baseline after this many configs")
    ap.add_argument("--max-error-deg", type=float, default=15.0, help="divergence guard: attitude error")
    ap.add_argument("--max-rate", type=float, default=0.6,
                    help="divergence guard: body-rate RMS over 1 s [rad/s] (the 2026-10-03 hold wobbled at ~0.23)")
    ap.add_argument("--min-depth", type=float, default=0.5, help="m below the surface to start")
    ap.add_argument("--output-dir", default="~/osu-uwrt/mpc_thruster_sweep")
    args = ap.parse_args()
    configs = PRESETS[args.preset]
    if args.configs:
        with open(args.configs) as f:
            configs = yaml.safe_load(f)
    # No rclpy SIGINT handler: Ctrl-C must leave ROS up long enough to restore the baseline.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        Sweep(args).run(configs)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
