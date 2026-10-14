"""Official DFQ model: left-only FK, IK, and continuous path validation."""
import json
from pathlib import Path
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares

BODY_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint", "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"]


class LeftArmKinematics:
    def __init__(self, root):
        self.root = Path(root)
        cfg = json.loads((self.root / "config/robot.json").read_text())
        self.model = pin.buildModelFromUrdf(str(self.root / cfg["urdf"]))
        self.data = self.model.createData()
        self.indices = [self.model.joints[self.model.getJointId(n)].idx_q for n in BODY_JOINTS]
        self.left = self.indices[15:22]
        self.lower = self.model.lowerPositionLimit[self.left] + 0.02
        self.upper = self.model.upperPositionLimit[self.left] - 0.02
        self.tool_frame = self.model.getFrameId("L_hand_base_link")
        # This offset must be verified against the real hand before grasping.
        self.tool_offset = np.array(cfg.get("grasp_offset_m", [0.0, -0.09, 0.03]))

    def configuration(self, state):
        q = pin.neutral(self.model)
        for i, idx in enumerate(self.indices):
            q[idx] = state["motors"][i]["q"]
        h = state.get("left_hand")
        if h:
            for jid, name in enumerate(self.model.names):
                if not name.startswith("L_") or self.model.joints[jid].nq != 1:
                    continue
                motor = next((i for text, i in [("little", 0), ("ring", 1), ("middle", 2), ("index", 3), ("thumb", 4)] if text in name), None)
                if motor is None:
                    continue
                if "thumb" in name and "yaw" in name:
                    motor = 5
                idx = self.model.joints[jid].idx_q
                q[idx] = self.model.upperPositionLimit[idx] - h["q"][motor] * (self.model.upperPositionLimit[idx] - self.model.lowerPositionLimit[idx])
        return q

    def pose(self, full_q, left_q=None):
        q = full_q.copy()
        if left_q is not None:
            q[self.left] = left_q
        pin.framesForwardKinematics(self.model, self.data, q)
        frame = self.data.oMf[self.tool_frame]
        return pin.SE3(frame.rotation.copy(), frame.act(self.tool_offset))

    def solve(self, full_q, xyz, rotation=None, seed=None):
        xyz = np.asarray(xyz, dtype=float)
        if xyz.shape != (3,) or not np.isfinite(xyz).all():
            raise ValueError("Invalid Cartesian target")
        seed = full_q[self.left].copy() if seed is None else np.array(seed, dtype=float)
        rotation = self.pose(full_q, seed).rotation if rotation is None else np.array(rotation)
        def residual(q):
            p = self.pose(full_q, q)
            return np.r_[10 * (p.translation - xyz), pin.log3(rotation.T @ p.rotation), 0.01 * (q - seed)]
        solution = least_squares(residual, np.clip(seed, self.lower, self.upper), bounds=(self.lower, self.upper), max_nfev=120)
        achieved = self.pose(full_q, solution.x)
        error = float(np.linalg.norm(achieved.translation - xyz))
        angle = float(np.linalg.norm(pin.log3(rotation.T @ achieved.rotation)))
        if not solution.success or error > 0.005 or angle > 0.10:
            raise ValueError("IK did not reach target: %.4f m, %.3f rad" % (error, angle))
        return solution.x

    def plan(self, state, xyz, rotation=None, speed_rad_s=0.06):
        if not 0 < speed_rad_s <= 0.06:
            raise ValueError("Joint speed limit 0.06 rad/s")
        full = self.configuration(state)
        start = self.pose(full)
        xyz = np.array(xyz, dtype=float)
        distance = float(np.linalg.norm(xyz - start.translation))
        if not np.isfinite(distance) or distance > 0.25:
            raise ValueError("Single Cartesian primitive limit 0.25 m")
        count = max(1, int(np.ceil(distance / 0.005)))
        qs = [full[self.left].copy()]
        for t in np.linspace(0, 1, count + 1)[1:]:
            qs.append(self.solve(full, start.translation + t * (xyz - start.translation), rotation, qs[-1]))
        result = [{"t_s": 0.0, "q": qs[0].tolist()}]
        elapsed = 0.0
        for a, b in zip(qs, qs[1:]):
            if max(abs(b - a)) > 0.15:
                raise ValueError("Discontinuous IK branch")
            elapsed += max(0.08, float(max(abs(b - a))) / speed_rad_s)
            result.append({"t_s": elapsed, "q": b.tolist()})
        if elapsed > 30:
            raise ValueError("Primitive exceeds 30-second lease")
        return {"waypoints": result, "duration_s": elapsed, "target_xyz_m": xyz.tolist(),
                "start_xyz_m": start.translation.tolist(), "full_configuration": full.tolist()}

    def collision_check(self, plan, scene):
        if not scene.get("calibration_verified"):
            raise ValueError("Empirical camera/hand calibration is not verified")
        normal = np.array(scene["table_plane_base"][:3])
        offset = scene["table_plane_base"][3]
        qfull = np.array(plan["full_configuration"])
        cloud = np.array(scene.get("obstacle_points_base", []))
        tree = None
        if cloud.size:
            from scipy.spatial import cKDTree
            tree = cKDTree(cloud)
        for waypoint in plan["waypoints"]:
            q = np.array(waypoint["q"])
            if np.any(q < self.lower) or np.any(q > self.upper):
                raise ValueError("URDF joint limit violation")
            tool = self.pose(qfull, q).translation
            if float(normal @ tool + offset) < 0.04:
                raise ValueError("Hand too close to tabletop")
            # Conservative arm capsules. Scene points must exclude robot points
            # and the explicitly allowed contact object, never unrelated obstacles.
            if tree is not None:
                if tree.query(tool)[0] < 0.07:
                    raise ValueError("Hand envelope intersects an obstacle")
            # FK for each waypoint updated self.data. Conservative arm
            # capsules check forearm and upper arm against measured obstacles.
            frames = [self.model.getFrameId(n) for n in
                      ['left_shoulder_roll_link','left_elbow_link','left_wrist_yaw_link']]
            points = [self.data.oMf[f].translation.copy() for f in frames]
            for a,b in zip(points,points[1:]):
                for t in np.linspace(0,1,12):
                    p=a+t*(b-a)
                    if tree is not None and tree.query(p)[0]<0.035:
                        raise ValueError('Arm capsule intersects measured obstacle')
                    if normal@p+offset<0.025:
                        raise ValueError('Arm capsule intersects tabletop')
        return True
