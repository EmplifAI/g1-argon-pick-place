# Hardware setup

The browser-only observation path needs Python 3.9+ and no pip packages. The
following is for experienced operators working with a suspended G1, not a
claim that a fresh machine can move the robot using only an API key.

## 1. Prepare the operator environment

Use Python 3.12 for the robot stack. From the repository root:

```sh
python3.12 -m venv .venv
. .venv/bin/activate
pip install -e '.[robot]'
bash scripts/fetch_robot_description.sh
```

The fetch script downloads the official G1 model at the pinned commit recorded
in `config/robot.json`. No vendor meshes are redistributed in this repository.

Install the official [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
and its Cyclone DDS dependency. The offline tests were checked against SDK commit
`814556d15970dd2ecf1c9984e845ca02ab07e206`. If Cyclone DDS 0.10.2 cannot find its
native library, follow the upstream build instructions and set `CYCLONEDDS_HOME`
to its installation directory before installing and running the SDK (keep the
variable set when running tests or hardware tools):

```sh
pip install -e 'git+https://github.com/unitreerobotics/unitree_sdk2_python.git@814556d15970dd2ecf1c9984e845ca02ab07e206#egg=unitree-sdk2py'
python -m unittest discover -s tests -v
```

Use the editable (`-e`) SDK installation: the pinned upstream wheel omits its
Linux CRC shared libraries. Keeping the source checkout preserves those files.
No test sends motion to a robot. CI builds the native DDS library separately.

## 2. Match the physical setup

Reference configuration: G1 29-axis rev_1_0, suspended, factory head-mounted
RealSense D435I, left Inspire RH56DFQ, serial port `/dev/ttyUSB2`. The original
hand has disabled pinky/ring channels `[0, 1]`. Verify the hand model, wiring,
operator IP and limits against your own hardware; the checked-in confirmations
are historical reference settings, not proof about your setup.

Reference network: robot Jetson `192.168.123.164`, operator `192.168.123.99`.
Use an isolated robot network. The bridge allows only the configured operator IP
and loopback; it is not an Internet-facing authenticated service.

## 3. Install the observing bridge explicitly

Set up SDK2, NumPy and the hand serial permissions on the robot first. Confirm
its SSH host key through your normal SSH workflow. Then, from the operator PC:

```sh
python scripts/deploy_robot.py --host 192.168.123.164 --user unitree
# If key-based SSH is unavailable, add --ask-password (interactive, not stored).
```

This helper installs a user service and changes serial ACL/linger settings;
review it for your deployment before execution. It refuses to replace an active
controller. It assumes the robot service user has UID 1000 and the reference
serial port. Change those deployment details explicitly for a different setup.
Never restart a controller while it is supporting a load.

Robot service environment overrides: `G1_BIND_IP`, `G1_OPERATOR_IP`,
`G1_NETWORK_INTERFACE`. Put nonsecret overrides in a systemd service override;
defaults are the reference robot IP, operator IP and `eth0` respectively.
The bridge starts with no automatic motor command. An explicit enable is needed.

## 4. Start the camera and verify read-only status

The camera uses Unitree's video hub; its robot-side service must already be
available. It is not automatically installed or restarted by the web console.

```sh
G1_NETWORK_INTERFACE=eth0 python src/camera_preview.py
# Replace eth0 with the operator PC's actual robot-facing interface.
```

Check `http://127.0.0.1:8765/health` for a recent frame. In a separate terminal:

```sh
python scripts/start_web.py --enable-motion \
  --robot-url http://192.168.123.164:8767 \
  --camera-url http://127.0.0.1:8765
```

Enter your key, click Connect bridge, and capture an image. This does not enable
control. Verify left-only command isolation, suspended support, workspace and
remote stop before checking the two per-run confirmations and starting practice.

## 5. Understand the limits

Practice permits enable, observe, bounded trials, stop and done. Only candidate
joints 15, 16 and 18 may be probed; the planner constrains the trajectory and
limits each run to three trials. Hand and Cartesian commands are rejected even
if the model requests them. The gateway verifies boot ID, generation, expiry,
joint limits, speed and heartbeat.

Stop requests a hold rather than opening the hand. The existing gateway also
has thermal protection: while stopped, a left-arm motor at 85°C or above can
trigger a stiffness ramp-down. A software stop is not a guarantee that a hot,
unpowered, faulted or disconnected robot can hold a load. Use the physical
remote and supported workspace procedures; do not rely on a browser button as
an emergency stop.

Full grasp/place primitives in `coarse_primitives.py` require independently
verified camera/tool calibration, scene segmentation and fresh RGB-D. The old
`stop_panel.py` one-click macro is retired and refuses direct startup. Legacy
trial scripts are research references, not alternative launch instructions.

The new web path has offline tests, not fresh physical validation. Revalidate
on hardware before any real motion after code or configuration changes.
