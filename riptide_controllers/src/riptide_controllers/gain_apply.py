#! /usr/bin/env python3
#
# Shared gain-apply helper for the riptide tuning tools.
#
# How gains actually work (established empirically, 2026-07-01):
#   - complete_controller consumes set_parameters updates LIVE -- a direct set is a real apply.
#   - controller_overseer's update_complete_controller_params (std_srvs/Trigger) is NOT an apply
#     mechanism: it re-reads the vehicle yaml and pushes THOSE values, reverting any live-set
#     gains. Same thing happens when the overseer (re)discovers the controller after a restart.
#   - Therefore: apply = direct set_parameters (+ readback verify); persist = patch the gain
#     lines in the source vehicle yaml (surgical, comments preserved) so reloads/restarts keep
#     the tune; revert_to_yaml = the Trigger.

import re
import time

from std_srvs.srv import Trigger as TriggerSrv
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

PARAMETER_SCALE = 1000000


class GainApplier:

    def __init__(self, node, config_path, callback_group=None):
        self.node = node
        self.config_path = config_path
        self.set_client = node.create_client(
            SetParameters, "complete_controller/set_parameters", callback_group=callback_group)
        self.get_client = node.create_client(
            GetParameters, "complete_controller/get_parameters", callback_group=callback_group)
        self.reload_client = node.create_client(
            TriggerSrv, "controller_overseer/update_complete_controller_params",
            callback_group=callback_group)

    def _log(self):
        return self.node.get_logger()

    # ------------------------------------------------------------------ live apply
    def apply(self, arrays, verify=True):
        """Directly set gain arrays on the running controller. arrays: {param name: [6 floats]}.

        Returns True only when the service accepted every parameter (and, with verify, the
        values read back correctly). NOTE: an overseer yaml reload will revert these -- call
        persist() once the values are final.
        """
        if not self.set_client.wait_for_service(timeout_sec=3.0):
            self._log().error("complete_controller/set_parameters unavailable")
            return False
        req = SetParameters.Request()
        names = list(arrays.keys())
        req.parameters = [
            Parameter(name=n, value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                integer_array_value=[int(round(float(v) * PARAMETER_SCALE)) for v in arrays[n]]))
            for n in names]
        fut = self.set_client.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 3.0:
            time.sleep(0.02)
        if not fut.done():
            self._log().error("set_parameters call timed out; gains NOT applied")
            return False
        rejected = [names[i] for i, r in enumerate(fut.result().results) if not r.successful]
        if rejected:
            self._log().error(f"controller rejected parameters: {rejected}")
            return False
        if verify and not self.matches(arrays):
            self._log().error("readback after set_parameters does not match; gains NOT applied")
            return False
        return True

    # ------------------------------------------------------------------ persist
    def persist(self, arrays):
        """Patch the gain lines in the vehicle yaml (surgical line edit, keeps comments) so the
        next overseer reload / stack restart keeps these values. arrays keys may be controller
        param names (controller__SMC__SMC_params__lambda) or plain yaml keys (lambda)."""
        with open(self.config_path, "r") as f:
            text = f.read()
        for name, vals in arrays.items():
            key = name.split("__")[-1]
            pattern = re.compile(rf"^(?P<indent>[ \t]*){key}:\s*\[[^\]]*\]", re.MULTILINE)
            if len(pattern.findall(text)) != 1:
                self._log().error(f"expected exactly one uncommented '{key}:' line in "
                                  f"{self.config_path}; not persisting")
                return False
            formatted = "[" + ", ".join(f"{float(v):.6g}" for v in vals) + "]"
            text = pattern.sub(lambda m: f"{m.group('indent')}{key}: {formatted}", text, count=1)
        with open(self.config_path, "w") as f:
            f.write(text)
        self._log().info(f"Persisted {', '.join(n.split('__')[-1] for n in arrays)} "
                         f"into {self.config_path}")
        return True

    # ------------------------------------------------------------------ revert
    def revert_to_yaml(self):
        """Ask the overseer to re-read the vehicle yaml and push those values (rate-limited)."""
        if not self.reload_client.wait_for_service(timeout_sec=5.0):
            self._log().error("update_complete_controller_params service unavailable")
            return False
        for _attempt in range(3):
            fut = self.reload_client.call_async(TriggerSrv.Request())
            t0 = time.time()
            while not fut.done() and time.time() - t0 < 5.0:
                time.sleep(0.05)
            if fut.done():
                res = fut.result()
                if res.success:
                    return True
                self._log().warn(f"overseer reload refused: {res.message}")
            else:
                self._log().warn("overseer reload call timed out")
            time.sleep(2.5)   # overseer rate-limits reloads (RELOAD_TIME=2s)
        return False

    # ------------------------------------------------------------------ read / compare
    def read(self, names, timeout=3.0):
        """Read current gain arrays off the controller -> {name: [floats]}, or None."""
        names = list(names)
        if not self.get_client.wait_for_service(timeout_sec=timeout):
            return None
        fut = self.get_client.call_async(GetParameters.Request(names=names))
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        if not fut.done():
            return None
        return {n: [v / PARAMETER_SCALE for v in val.integer_array_value]
                for n, val in zip(names, fut.result().values)}

    def matches(self, arrays, timeout=3.0):
        """True if complete_controller currently holds these param arrays (x1e6 ints)."""
        names = list(arrays.keys())
        if not self.get_client.wait_for_service(timeout_sec=timeout):
            return False
        fut = self.get_client.call_async(GetParameters.Request(names=names))
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)
        if not fut.done():
            return False
        for name, val in zip(names, fut.result().values):
            want = [int(round(float(v) * PARAMETER_SCALE)) for v in arrays[name]]
            got = list(val.integer_array_value)
            if len(got) != len(want) or any(abs(a - b) > 2 for a, b in zip(got, want)):
                return False
        return True
