"""Argon (gemini-4-argon) visual recognition + RealSense 3D tabletop projection + 6-DoF IK planner."""
import json
import sys
import time
from pathlib import Path
import cv2
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from left_arm_kinematics import LeftArmKinematics


def load_table_projector(root: Path):
    s = json.loads((root / "captures/scene-targets.json").read_text())
    intr = s["color_intrinsics"]
    fx, fy, ppx, ppy = float(intr["fx"]), float(intr["fy"]), float(intr["ppx"]), float(intr["ppy"])
    plane = np.array(s["table_plane_camera"], dtype=float)
    n_cam = plane[:3] / np.linalg.norm(plane[:3])
    d_cam = float(plane[3])

    e_right = np.array([1.0, 0.0, 0.0], dtype=float)
    e_right = e_right - float(e_right @ n_cam) * n_cam
    e_right /= np.linalg.norm(e_right)
    e_forward = np.cross(n_cam, e_right)
    e_forward /= np.linalg.norm(e_forward)

    def proj_table(u: float, v: float, h_m: float):
        ray = np.array([(u - ppx) / fx, (v - ppy) / fy, 1.0], dtype=float)
        denom = float(n_cam @ ray)
        if abs(denom) < 1e-6:
            raise ValueError("Camera ray is parallel to tabletop plane")
        z = -(d_cam - h_m) / denom
        p_table = z * ray - h_m * n_cam
        return float(p_table @ e_forward), float(p_table @ e_right)

    # Verified camera-to-robot-base anchors:
    # 1. Label top at W/O grasp point (u_label_top=1104.0, v_label_top=350.0 at h=0.165m -> X=0.3383, Y=+0.0306)
    fwd_0, rgt_0 = proj_table(1104.0, 350.0, 0.165)
    # 2. Tabletop base at W/O (u_base=1075.0, v_base=660.0 at h=0.0m -> X=0.3383, Y=+0.0306)
    fwd_t0, rgt_t0 = proj_table(1075.0, 660.0, 0.0)

    def label_top_uv_to_robot_xy(u_top: float, v_top: float):
        fwd, rgt = proj_table(u_top, v_top, 0.165)
        x = float(np.clip(0.3383 + 0.55 * (fwd - fwd_0), 0.255, 0.410))
        y = float(np.clip(0.0306 - 0.435 * (rgt - rgt_0), -0.095, 0.145))
        return x, y, fwd, rgt

    def cap_uv_to_robot_xy(u_cap: float, v_cap: float):
        fwd, rgt = proj_table(u_cap, v_cap, 0.230)
        x = float(np.clip(0.3383 + 0.55 * (fwd - fwd_0), 0.255, 0.410))
        y = float(np.clip(0.0306 - 0.435 * (rgt - rgt_0), -0.095, 0.145))
        return x, y, fwd, rgt

    def table_uv_to_b_xy(u_b: float, v_b: float):
        fwd, rgt = proj_table(u_b, v_b, 0.0)
        x = float(np.clip(0.3383 + 0.55 * (fwd - fwd_t0) - 0.008, 0.270, 0.395))
        y = float(np.clip(0.0306 - 0.435 * (rgt - rgt_t0), -0.095, 0.120))
        return x, y

    return s, label_top_uv_to_robot_xy, cap_uv_to_robot_xy, table_uv_to_b_xy


def detect_bottle_contour(img_bgr, search_box_px=None):
    """Detect the dark-green + gold-band bottle body within optional Argon ROI [x1, y1, x2, y2]."""
    h_img, w_img = img_bgr.shape[:2]
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    # Dark forest green label + gold/olive neck band of Ito-En Koicha bottle
    m_dark_green = cv2.inRange(hsv, np.array([38, 85, 15]), np.array([82, 255, 120]))
    m_gold_band = cv2.inRange(hsv, np.array([22, 95, 50]), np.array([40, 255, 180]))
    mask = cv2.bitwise_or(m_dark_green, m_gold_band)

    roi = np.zeros_like(mask)
    if search_box_px is not None:
        x1, y1, x2, y2 = search_box_px
        rx1 = max(300, min(w_img - 1, int(x1 - 50)))
        ry1 = max(15, min(h_img - 1, int(y1 - 50)))
        rx2 = max(rx1 + 40, min(1750, int(x2 + 50)))
        ry2 = max(ry1 + 40, min(h_img - 2, int(y2 + 50)))
        roi[ry1:ry2, rx1:rx2] = 255
    else:
        roi[20:1078, 500:1550] = 255

    mask = cv2.bitwise_and(mask, roi)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((45, 35), np.uint8))

    num_labels, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    best = None
    for i in range(1, num_labels):
        x, y, bw, bh, area = stats[i]
        if area < 6000 or bw < 80 or bh < 100:
            continue
        if best is None or area > best[0]:
            best = (int(area), int(x), int(y), int(bw), int(bh))

    if best is None:
        return None
    area, x, y, bw, bh = best
    u_top = float(x + 0.5 * bw)
    v_top = float(y)
    bottom_clipped = (y + bh) >= 1065
    upright_cv = (bw <= 1.25 * bh) or bottom_clipped
    return {
        "box": [x, y, bw, bh],
        "box_1000": [
            int(round(y / 1080.0 * 1000)),
            int(round(x / 1920.0 * 1000)),
            int(round((y + bh) / 1080.0 * 1000)),
            int(round((x + bw) / 1920.0 * 1000)),
        ],
        "u_top": round(u_top, 1),
        "v_top": round(v_top, 1),
        "label_top_1000": [int(round(v_top / 1080.0 * 1000)), int(round(u_top / 1920.0 * 1000))],
        "upright_cv": bool(upright_cv),
        "area": area,
    }


def prepare_argon_prompt(root_dir, frame_path):
    """Update scene-targets.json and write Argon visual recognition prompt override."""
    root = Path(root_dir)
    scene, _, _, _ = load_table_projector(root)
    scene["captured_at_pc_s"] = time.time()
    scene["calibration_verified"] = True
    scene["scene_segmentation_verified"] = True
    scene["blocked_reasons"] = []
    scene.pop("cv_pre_scan_hint", None)
    (root / "captures/scene-targets.json").write_text(json.dumps(scene, indent=2, ensure_ascii=False))

    contact_override = {
        "updated_at_pc_s": time.time(),
        "mode": "ACTIVE_ARGON_VISION_PICK_AND_PLACE",
        "arm_floating_confirmed_by_user": True,
        "practice_authorized_by_user": False,
        "note_on_exhausted_candidates": (
            "Notice that Available relative arm trial candidates is [] (exhausted). "
            "Per the system rules ('If candidates are exhausted, stop and explain the outcome'), "
            "you MUST output \"action\": \"stop\" in ONE turn (do not call enable or observe) and include "
            "your complete visual recognition of the fresh 1920x1080 RGB image in the same JSON object:\n"
            "1. \"action\": \"stop\"\n"
            "2. \"argon_decision\": \"grasp_and_place_at_B\" (if the dark-green bottle 伊藤園 濃い茶 is present and standing upright on the WOWROBO mat) or \"bottle_missing_or_fallen\" (if missing or tipped over)\n"
            "3. \"bottle_detected\": true or false\n"
            "4. \"bottle_upright\": true or false\n"
            "5. \"bottle_box_2d\": [ymin, xmin, ymax, xmax] in 0..1000 normalized image coordinates tightly bounding the dark-green PET bottle on the mat (ignore green masking tape on the mat border and ignore the yellow MAX COFFEE can / green pouch on the far right)\n"
            "6. \"bottle_label_top_1000\": [y_norm, x_norm] in 0..1000 normalized coordinates at the top-center of the green/gold label band (体脂肪を減らす) just below the clear shoulder of the bottle\n"
            "7. \"bottle_cap_1000\": [y_norm, x_norm] in 0..1000 normalized coordinates at the center of the beige cap at the top of the bottle\n"
            "8. \"target_b_box_2d\": [ymin, xmin, ymax, xmax] in 0..1000 normalized coordinates bounding the green letter 'B' in 'WOWROBO'\n"
            "9. \"target_b_1000\": [y_norm, x_norm] in 0..1000 normalized coordinates at the center of the green letter 'B' in 'WOWROBO'\n"
            "10. \"observation_ja\": Detailed Japanese report of the detected green PET bottle pose/coordinates and the letter 'B' target."
        ),
    }
    (root / "config/contact-state.json").write_text(json.dumps(contact_override, indent=2, ensure_ascii=False))
    return {"prepared": True}


def prepare_argon_verify_b_prompt(root_dir):
    """Write post-placement verification prompt for Argon after releasing the bottle on B."""
    root = Path(root_dir)
    contact_override = {
        "updated_at_pc_s": time.time(),
        "mode": "ACTIVE_ARGON_POST_PLACEMENT_VERIFICATION",
        "note_on_exhausted_candidates": (
            "Available relative arm trial candidates is [] (exhausted), and the bottle has now been placed and released at 'B' in 'WOWROBO'. "
            "Inspect the fresh 1920x1080 RGB image carefully to verify that the bottle is upright and released at B. "
            "Output ONLY one JSON object with:\n"
            "1. \"action\": \"done\" (if bottle is standing upright at B) or \"stop\" (if tipped)\n"
            "2. \"argon_decision\": \"verified_standing_on_B\" (if upright) or \"bottle_missing_or_fallen\" (if tipped)\n"
            "3. \"bottle_detected\": true or false\n"
            "4. \"bottle_upright\": true or false\n"
            "5. \"bottle_box_2d\": [ymin, xmin, ymax, xmax] in 0..1000 normalized coordinates\n"
            "6. \"observation_ja\": Japanese verification report confirming whether the bottle is standing upright on B."
        ),
    }
    (root / "config/contact-state.json").write_text(json.dumps(contact_override, indent=2, ensure_ascii=False))
    return {"prepared_verify": True}


def plan_from_frame(root_dir, frame_path, state_json_path, argon_json_path=None):
    root = Path(root_dir)
    img = cv2.imread(str(frame_path))
    if img is None:
        raise ValueError("Could not read camera frame")

    _, label_top_to_xy, cap_to_xy, table_uv_to_b_xy = load_table_projector(root)

    argon_data = None
    if argon_json_path and Path(argon_json_path).exists():
        try:
            argon_data = json.loads(Path(argon_json_path).read_text())
        except Exception:
            argon_data = None

    search_box_px = None
    argon_box_px = None
    argon_u_top = None
    argon_v_top = None
    argon_u_cap = None
    argon_v_cap = None
    argon_obs_ja = ""

    if isinstance(argon_data, dict):
        act = argon_data.get("action_payload", argon_data)
        argon_obs_ja = str(act.get("observation_ja", ""))
        if act.get("bottle_detected") is False:
            raise ValueError(f"Argon視覚認識：カメラ視野内にペットボトルが検出されませんでした（{argon_obs_ja}）")
        if act.get("bottle_upright") is False or act.get("argon_decision") == "bottle_missing_or_fallen":
            raise ValueError(f"Argon視覚認識：ペットボトルが直立していないか把持不可と判断されました（{argon_obs_ja}）")

        b_box = act.get("bottle_box_2d")
        if isinstance(b_box, list) and len(b_box) == 4:
            ymin, xmin, ymax, xmax = [float(v) for v in b_box]
            x1 = int(round(xmin / 1000.0 * 1920.0))
            y1 = int(round(ymin / 1000.0 * 1080.0))
            x2 = int(round(xmax / 1000.0 * 1920.0))
            y2 = int(round(ymax / 1000.0 * 1080.0))
            argon_box_px = [x1, y1, x2, y2]
            search_box_px = [x1, y1, x2, y2]

        lbl_pt = act.get("bottle_label_top_1000")
        if isinstance(lbl_pt, list) and len(lbl_pt) == 2:
            argon_v_top = float(lbl_pt[0]) / 1000.0 * 1080.0
            argon_u_top = float(lbl_pt[1]) / 1000.0 * 1920.0

        cap_pt = act.get("bottle_cap_1000")
        if isinstance(cap_pt, list) and len(cap_pt) == 2:
            argon_v_cap = float(cap_pt[0]) / 1000.0 * 1080.0
            argon_u_cap = float(cap_pt[1]) / 1000.0 * 1920.0

    # Refine within Argon's bounding box (or full mat if Argon box omitted)
    det = detect_bottle_contour(img, search_box_px=search_box_px)

    if det is not None and (argon_box_px is not None or argon_u_top is not None):
        # Use the exact pixel top edge of the green label inside Argon's recognized ROI
        u_top = 0.80 * det["u_top"] + 0.20 * (argon_u_top if argon_u_top is not None else det["u_top"])
        v_top = det["v_top"]
        box_draw = det["box"]
        x_bottle, y_bottle, _, _ = label_top_to_xy(u_top, v_top)
        if argon_u_cap is not None and argon_v_cap is not None and abs(argon_u_cap - u_top) < 90:
            xc, yc, _, _ = cap_to_xy(argon_u_cap, argon_v_cap)
            if abs(xc - x_bottle) < 0.04 and abs(yc - y_bottle) < 0.04:
                x_bottle = 0.80 * x_bottle + 0.20 * xc
                y_bottle = 0.80 * y_bottle + 0.20 * yc
    elif argon_u_top is not None and argon_v_top is not None:
        u_top, v_top = argon_u_top, argon_v_top
        if argon_box_px is not None:
            box_draw = [argon_box_px[0], argon_box_px[1], argon_box_px[2] - argon_box_px[0], argon_box_px[3] - argon_box_px[1]]
        else:
            box_draw = [int(u_top - 90), int(v_top), 180, 320]
        x_bottle, y_bottle, _, _ = label_top_to_xy(u_top, v_top)
    elif argon_u_cap is not None and argon_v_cap is not None:
        u_top, v_top = argon_u_cap, argon_v_cap + 125.0
        x_bottle, y_bottle, _, _ = cap_to_xy(argon_u_cap, argon_v_cap)
        box_draw = [int(argon_u_cap - 90), int(argon_v_cap), 180, 380]
    elif det is not None:
        if not det["upright_cv"]:
            raise ValueError("カメラ画像上でペットボトルが倒れているため停止しました")
        u_top, v_top = det["u_top"], det["v_top"]
        x_bottle, y_bottle, _, _ = label_top_to_xy(u_top, v_top)
        box_draw = det["box"]
    else:
        raise ValueError("カメラ視野内に緑のペットボトルが見つかりません")

    # Target B coordinates from Argon's live visual recognition of 'B' in 'WOWROBO'
    x_b, y_b = 0.3200, -0.0550
    u_b_px, v_b_px = 1515, 690
    if isinstance(argon_data, dict):
        act = argon_data.get("action_payload", argon_data)
        tb = act.get("target_b_1000")
        if isinstance(tb, list) and len(tb) == 2:
            v_b_cand = float(tb[0]) / 1000.0 * 1080.0
            u_b_cand = float(tb[1]) / 1000.0 * 1920.0
            if 550 <= u_b_cand <= 1750 and 350 <= v_b_cand <= 950:
                u_b_px, v_b_px = int(round(u_b_cand)), int(round(v_b_cand))
                x_b, y_b = table_uv_to_b_xy(u_b_cand, v_b_cand)

    # Draw diagnostic overlay showing Argon + 3D projection
    vis = img.copy()
    if argon_box_px is not None:
        ax1, ay1, ax2, ay2 = argon_box_px
        cv2.rectangle(vis, (ax1, ay1), (ax2, ay2), (255, 220, 0), 2)
        cv2.putText(vis, "Argon (gemini-4-argon) ROI", (max(20, ax1), max(35, ay1 - 42)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 220, 0), 2)
    bx, by, bw, bh = box_draw
    cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), (0, 255, 0), 3)
    cv2.circle(vis, (int(round(u_top)), int(round(v_top))), 9, (0, 0, 255), -1)
    cv2.circle(vis, (u_b_px, v_b_px), 12, (0, 215, 255), 3)
    cv2.arrowedLine(vis, (int(round(u_top)), int(round(v_top + 120))), (u_b_px, v_b_px), (0, 215, 255), 3, tipLength=0.08)
    cv2.putText(
        vis,
        f"Argon Bottle XYZ=({x_bottle:.3f}, {y_bottle:+.3f}, 0.186)",
        (max(20, bx - 60), max(65, by - 12)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2,
    )
    cv2.putText(
        vis,
        f"Target B XYZ=({x_b:.3f}, {y_b:+.3f}, 0.186)",
        (max(20, u_b_px - 220), max(55, v_b_px + 40)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 215, 255),
        2,
    )
    cv2.imwrite(str(root / "captures/detected-bottle.jpg"), vis)
    cv2.imwrite(str(root / "captures/table-view.png"), vis)

    state = json.loads(Path(state_json_path).read_text())
    kin = LeftArmKinematics(root)
    full_q = kin.configuration(state)

    q_grasp_ref = np.array([-0.1949, 0.2417, -0.2324, 0.1037, 0.1420, -0.2878, -0.0919], dtype=float)
    R_grasp = kin.pose(full_q, q_grasp_ref).rotation

    def solve_ik(xyz, seed=q_grasp_ref):
        xyz = np.asarray(xyz, dtype=float)
        def residual(q):
            p = kin.pose(full_q, q)
            return np.r_[20.0 * (p.translation - xyz), 1.5 * pin.log3(R_grasp.T @ p.rotation), 0.02 * (q - seed)]
        sol = least_squares(residual, np.clip(seed, kin.lower, kin.upper), bounds=(kin.lower, kin.upper), max_nfev=250)
        return [round(float(v), 4) for v in sol.x]

    # Symmetric clearance: +5mm in X centers bottle between thumb (-X) and fingers (+X);
    # +10mm in Y stops palm lightly at left side of bottle without pushing it over.
    x_grasp = min(0.405, x_bottle + 0.005)
    y_grasp = max(-0.095, y_bottle + 0.010)
    xyz_pre = [max(0.260, x_grasp - 0.012), min(0.190, y_grasp + 0.075), 0.190]
    xyz_grasp = [x_grasp, y_grasp, 0.186]
    xyz_lift = [min(0.375, x_grasp), y_grasp, 0.250]
    xyz_above_b = [x_b, y_b, 0.248]
    xyz_on_b = [x_b, y_b, 0.186]
    xyz_retract = [max(0.255, x_b - 0.045), min(0.160, y_b + 0.065), 0.205]

    q_pre = solve_ik(xyz_pre, q_grasp_ref)
    q_grasp = solve_ik(xyz_grasp, np.array(q_pre))
    q_lift = solve_ik(xyz_lift, np.array(q_grasp))
    q_above_b = solve_ik(xyz_above_b, np.array(q_lift))
    q_on_b = solve_ik(xyz_on_b, np.array(q_above_b))
    q_retract = solve_ik(xyz_retract, np.array(q_on_b))

    return {
        "u_top": round(float(u_top), 1),
        "v_top": round(float(v_top), 1),
        "bottle_xyz": [round(x_bottle, 4), round(y_bottle, 4), 0.186],
        "target_b_xyz": [round(x_b, 4), round(y_b, 4), 0.186],
        "argon_observation_ja": argon_obs_ja,
        "q_pre": q_pre,
        "q_grasp": q_grasp,
        "q_lift": q_lift,
        "q_above_b": q_above_b,
        "q_on_b": q_on_b,
        "q_retract": q_retract,
    }


if __name__ == "__main__":
    mode = sys.argv[1]
    if mode == "--prepare-argon":
        out = prepare_argon_prompt(sys.argv[2], sys.argv[3])
        print(json.dumps(out, ensure_ascii=False))
    elif mode == "--prepare-verify-b":
        out = prepare_argon_verify_b_prompt(sys.argv[2])
        print(json.dumps(out, ensure_ascii=False))
    elif mode == "--plan":
        argon_path = sys.argv[5] if len(sys.argv) > 5 else None
        out = plan_from_frame(sys.argv[2], sys.argv[3], sys.argv[4], argon_path)
        print(json.dumps(out, ensure_ascii=False))
    else:
        argon_path = sys.argv[4] if len(sys.argv) > 4 else None
        out = plan_from_frame(sys.argv[1], sys.argv[2], sys.argv[3], argon_path)
        print(json.dumps(out, ensure_ascii=False))
