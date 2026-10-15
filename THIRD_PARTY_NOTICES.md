# Third-party components and provenance

Apache-2.0 covers this project's original work, not every external component.
No robot URDFs, meshes, model weights, vendor SDKs or private datasets are vendored.

| Component | How it is used | Upstream / license source |
| --- | --- | --- |
| Unitree ROS G1 description | Optional URDF and meshes, fetched separately at commit `5994d4faef0a9cadd3287f8de0199a67eeb2a259` | https://github.com/unitreerobotics/unitree_ros/tree/5994d4faef0a9cadd3287f8de0199a67eeb2a259 — review upstream and asset-specific terms |
| Unitree SDK2 Python | DDS / robot integration, installed separately | https://github.com/unitreerobotics/unitree_sdk2_python |
| Cyclone DDS | Transport required by the vendor SDK | https://github.com/eclipse-cyclonedds/cyclonedds |
| Pinocchio (`pin`) | Kinematics | https://github.com/stack-of-tasks/pinocchio |
| NumPy / SciPy | Numeric and optimization routines | https://numpy.org/doc/stable/license.html / https://github.com/scipy/scipy/blob/main/LICENSE.txt |
| Pillow | Image processing | https://github.com/python-pillow/Pillow/blob/main/LICENSE |
| Paramiko | Explicit SSH deployment helper | https://github.com/paramiko/paramiko/blob/main/LICENSE |
| Optional RealSense / OpenCV / trimesh | RGB-D and geometry experiments | Their upstream distributions and accompanying licenses |
| Gemini API | Hosted inference; not redistributed software | https://ai.google.dev/gemini-api/terms |

The original experiment consulted the Unitree Inspire service and XR teleoperation
interfaces. These are references, not claims that this repository relicenses them:
https://github.com/unitreerobotics/dfx_inspire_service and
https://github.com/unitreerobotics/xr_teleoperate.

`docs/assets/hardware-demo.jpg` and the numbered demo frames are from the author's
public X video. `web/sample.svg` is an original illustration, not camera data.
UI screenshots document this app without a connected key or active robot.
Brand names and visible product marks remain the property of their owners.
