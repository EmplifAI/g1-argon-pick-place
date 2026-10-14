"""SO-101-style tools; motion decisions remain with Argon.

Planning/verification execute locally. Absolute motion fails closed until the
camera/tool transform has empirical verification. No model-generated code runs.
"""
import json
import math
import time
import urllib.request
from pathlib import Path
import numpy as np
from left_arm_kinematics import LeftArmKinematics


class PickPlaceTools:
    def __init__(self, root, robot_call, cancel, camera_frame, record):
        self.root = Path(root)
        self.robot = robot_call
        self.cancel = cancel
        self.frame = camera_frame
        self.record = record
        self.kin = LeftArmKinematics(root)
        self.baseline = None
        self.grasp_verified = False
        self.boot_id = self.robot('/state')['boot_id']

    def scene(self):
        path = self.root / "captures/scene-targets.json"
        if not path.exists():
            raise ValueError("RGB-D scene has not been reconstructed")
        s = json.loads(path.read_text())
        if time.time() - s.get("captured_at_pc_s", 0) > 30:
            raise ValueError("RGB-D scene is stale; capture a new scene")
        if not s.get("calibration_verified"):
            raise ValueError("Camera/tool calibration is not empirically verified")
        if not s.get('scene_segmentation_verified'):
            raise ValueError('Robot/object/obstacle segmentation has not been verified')
        return s

    def observe(self):
        state = self.robot("/state")
        if state["state_age"] > 0.1 or state["hand_state_age"] > 0.3:
            raise ValueError("Robot observation is stale")
        image = self.frame()
        full = self.kin.configuration(state)
        result = {"state": state, "left_tool_pose": self.kin.pose(full).homogeneous.tolist(),
                  "grasp_verified": self.grasp_verified}
        self.record("coarse_observe", state=state, tool_pose=result["left_tool_pose"])
        return result, image

    def check_running(self, generation):
        if self.cancel.is_set():
            raise ValueError("PC STOP")
        s = self.robot("/state")
        if s.get('boot_id') != self.boot_id:
            raise ValueError('Robot rebooted; discard old primitive')
        if s["stopped"] or s["generation"] != generation:
            raise ValueError("Robot STOP or obsolete tool invocation")
        return s

    def move_left_hand(self, xyz_m, rotation, generation, speed_rad_s=0.06):
        scene = self.scene()
        state = self.check_running(generation)
        # Begin the path at the existing commanded pose, then verify measured
        # tracking before accepting a new Cartesian primitive.
        if max(abs(m["q"]-q) for m,q in zip(state["motors"][15:22],state["target_q"])) > 0.04:
            raise ValueError("Arm is not tracking its current pose")
        planning_state = dict(state, motors=[dict(m) for m in state["motors"]])
        for m,q in zip(planning_state["motors"][15:22],state["target_q"]):
            m["q"] = q
        R = np.array(rotation, dtype=float)
        if R.shape != (3,3) or not np.isfinite(R).all() or np.linalg.norm(R.T@R-np.eye(3)) > 1e-5 or np.linalg.det(R) < 0.999:
            raise ValueError("Invalid hand rotation matrix")
        plan = self.kin.plan(planning_state, xyz_m, R, speed_rad_s)
        self.kin.collision_check(plan, scene)
        self.record("cartesian_plan", plan=plan, generation=generation)
        ack = self.robot("/trajectory", {"waypoints": plan["waypoints"], "generation": generation,'boot_id':self.boot_id})
        deadline = time.monotonic()+plan["duration_s"]+2
        while time.monotonic() < deadline:
            state = self.check_running(generation)
            if state.get("motion_done"):
                full = self.kin.configuration(state)
                error = float(np.linalg.norm(self.kin.pose(full).translation-np.array(xyz_m)))
                if error > 0.01:
                    raise ValueError("Measured hand failed to reach target: %.3f m" % error)
                self.record("cartesian_complete", error_m=error)
                return {"reached": True, "error_m": error}
            self.cancel.wait(0.04)
        self.robot("/stop", {})
        raise ValueError("Cartesian primitive deadline exceeded")

    def set_grip(self, target, generation, closing=False):
        target = np.array(target, dtype=float)
        if target.shape != (6,) or not np.isfinite(target).all() or np.any(target<0) or np.any(target>1):
            raise ValueError("Invalid DFQ grip")
        if closing and self.baseline is None:
            raise ValueError("No stable unloaded force baseline")
        deadline = time.monotonic()+22
        while time.monotonic() < deadline:
            state = self.check_running(generation)
            telemetry = state["left_hand"]
            if any(telemetry["errors"]) or max(telemetry["temperature_c"]) >= 60:
                raise ValueError("Hand fault")
            if closing:
                delta = np.array(telemetry["force_n"])-self.baseline
                if np.max(np.abs(telemetry["current_ma"])) > 200:
                    self.robot("/stop", {})
                    raise ValueError("Hand current limit")
                if max(abs(delta[:5])) > 2.0:
                    # Freeze the existing grip instead of squeezing harder.
                    held = telemetry["q"]
                    self.robot("/move-left-hand", {"q": held, "generation": generation,'boot_id':self.boot_id})
                    return {"contact_candidate": True, "relative_force_n": delta.tolist()}
            commanded = np.array(state.get("commanded_hand_q") or telemetry["q"])
            if max(abs(commanded-target)) < 0.002:
                return {"command_complete": True, "actual_q": telemetry["q"]}
            next_q = commanded+np.clip(target-commanded,-0.09,0.09)
            self.robot("/move-left-hand", {"q": next_q.tolist(), "generation": generation,'boot_id':self.boot_id})
            self.cancel.wait(1.85)
        self.robot("/stop", {})
        raise ValueError("Hand primitive deadline exceeded")

    def open_hand(self, generation):
        self.grasp_verified = False
        result = self.set_grip([1,1,1,1,1,0.83],generation)
        samples = []
        for _ in range(10):
            s = self.check_running(generation)
            if min(s["left_hand"]["q"][:5]) < 0.95:
                raise ValueError("Hand did not open")
            samples.append(s["left_hand"]["force_n"])
            self.cancel.wait(0.1)
        values = np.array(samples)
        if np.max(np.std(values,axis=0)) > 0.15:
            raise ValueError("Unstable unloaded force signal")
        self.baseline = np.median(values,axis=0)
        self.record("unloaded_force_baseline", baseline_n=self.baseline.tolist(),
                    method="relative baseline; not calibrated absolute force")
        return result

    def grasp_bottle(self, xyz_m, rotation, generation, grip_q):
        self.scene()
        self.move_left_hand(xyz_m,rotation,generation)
        result = self.set_grip(grip_q,generation,closing=True)
        self.record("grasp_candidate", result=result)
        # Only Argon can confirm from a subsequent fresh observation. Merely
        # reaching a finger position or threshold is never reported as success.
        return dict(result, grasp_verified=False, next_required="verify_grasp")

    def verify_grasp(self, evidence, generation):
        self.check_running(generation)
        if not evidence.get("opposing_fingers_visible") or not evidence.get("bottle_held_visible"):
            raise ValueError("Argon did not provide visual grasp evidence")
        if time.time()-evidence.get("observed_at_pc_s",0) > 10:
            raise ValueError("Grasp evidence is stale")
        hand=self.check_running(generation)['left_hand']
        if self.baseline is None or np.count_nonzero(np.abs(np.array(hand['force_n'])-self.baseline)>0.3)<2:
            raise ValueError('No independent finger contact signals; grasp unverified')
        self.grasp_verified = True
        self.record("grasp_verified", evidence=evidence)
        return {"grasp_verified": True}

    def place_at(self, xyz_m, rotation, generation):
        if not self.grasp_verified:
            raise ValueError("Grasp not verified; do not transport")
        self.move_left_hand(xyz_m,rotation,generation)
        # Placement requires explicit release after Argon verifies contact at B.
        return {"at_place_pose": True, "released": False, "next_required": "verify_place_contact"}

    def release_at_B(self, evidence, generation):
        if not self.grasp_verified or not evidence.get("bottle_base_on_table"):
            raise ValueError("No tabletop contact evidence")
        if evidence.get("target_error_m",math.inf) > 0.02 or time.time()-evidence.get("observed_at_pc_s",0)>5:
            raise ValueError("Placement alignment not verified")
        result = self.open_hand(generation)
        self.record("released_at_B", evidence=evidence)
        return result
