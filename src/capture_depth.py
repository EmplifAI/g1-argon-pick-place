"""Capture native D435I depth without opening/stopping the color sensor."""
import json
from pathlib import Path
import numpy as np
import pyrealsense2 as rs

root = Path(__file__).resolve().parents[1] / "captures"
root.mkdir(exist_ok=True)
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 15)
started = False
try:
    profile = pipeline.start(config)
    started = True
    device = profile.get_device()
    frame = None
    for _ in range(8):
        frame = pipeline.wait_for_frames(2000).get_depth_frame()
    depth_profile = frame.profile.as_video_stream_profile()
    color_profiles = [p.as_video_stream_profile() for s in device.query_sensors()
                      for p in s.get_stream_profiles()
                      if p.stream_type() == rs.stream.color
                      and p.as_video_stream_profile().width() == 1920
                      and p.as_video_stream_profile().height() == 1080
                      and p.format() == rs.format.rgb8 and p.fps() == 30]
    color_profile = color_profiles[0]
    def intrinsics(p):
        i = p.get_intrinsics()
        return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy,
                "ppx": i.ppx, "ppy": i.ppy, "model": str(i.model), "coeffs": i.coeffs}
    ex = depth_profile.get_extrinsics_to(color_profile)
    info = {"model": device.get_info(rs.camera_info.name),
            "depth_scale_m": device.first_depth_sensor().get_depth_scale(),
            "depth_intrinsics": intrinsics(depth_profile),
            "color_intrinsics": intrinsics(color_profile),
            "depth_to_color": {"rotation_column_major": ex.rotation, "translation_m": ex.translation},
            "frame_number": frame.get_frame_number(), "timestamp_ms": frame.get_timestamp()}
    np.save(root / "depth.npy", np.asanyarray(frame.get_data()))
    # RealSense computes deprojection, including distortion, in its own SDK.
    points = rs.pointcloud().calculate(frame)
    vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
    np.save(root / "depth-points.npy", vertices)
    (root / "depth-info.json").write_text(json.dumps(info, indent=2))
    print(json.dumps(info))
finally:
    if started:
        pipeline.stop()
