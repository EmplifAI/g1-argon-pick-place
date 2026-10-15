"""G1-local left-arm/hand gateway with latching stop and heartbeat watchdog.

Starts observing only. No motor or hand writes until an explicit enable.
Never sends active commands to right arm, waist or legs.
"""
import os
import json
import math
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from safety_core import LEFT, SafetyCore
from inspire_serial import InspireLeft
from holding_transfer import validate_checkpoint
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient

lock = threading.RLock()
core = SafetyCore()
state = None
hand_data = None
hand_updated = 0.0
hand_error = None
arm_writes = 0
hand_writes = 0
hand_write_enabled = False
last_loop = time.monotonic()
last_fault = None
enable_lock = threading.Lock()
root = Path(__file__).resolve().parents[1]
root.joinpath("logs").mkdir(exist_ok=True)
config = json.loads(root.joinpath("config/robot.json").read_text())
limits = json.loads(root.joinpath("config/left-arm-limits.json").read_text())


def log(event):
    with root.joinpath("logs/robot-events.jsonl").open("a") as f:
        f.write(json.dumps(dict(event, timestamp=time.time())) + "\n")


stopped_since = time.monotonic()
relax_requested_at = None


def stop(reason):
    global stopped_since
    with lock:
        if not core.stopped:
            stopped_since = time.monotonic()
        core.stop(reason)
        log({"event": "stop", "reason": reason, "generation": core.generation})


def relax(reason="PC relax"):
    global stopped_since, relax_requested_at
    with lock:
        now = time.monotonic()
        stopped_since = now
        relax_requested_at = now
        core.stop(reason)
        log({"event": "relax", "reason": reason, "generation": core.generation})


def receive(msg):
    global state
    with lock:
        state = msg
        core.observe()


def snapshot():
    with lock:
        core.check_health()
        motors = [] if state is None else [
            {"id": i, "mode": m.mode, "q": m.q, "dq": m.dq, "tau_est": m.tau_est,
             "temperature": list(m.temperature), "error": m.motorstate}
            for i, m in enumerate(state.motor_state[:29])]
        return {"stopped": core.stopped, "reason": core.reason,
                "generation": core.generation, "controller_active": core.active,
                "heartbeat_age": min(999, time.monotonic() - core.last_heartbeat),
                "state_age": min(999, time.monotonic() - core.last_state),
                "mode_machine": None if state is None else state.mode_machine,
                "motors": motors, "left_hand": hand_data,
                "hand_state_age": time.monotonic() - hand_updated,
                "hand_error": hand_error, "arm_writes": arm_writes,
                "finger_force_calibrated": False,
                "disabled_left_hand_channels": config.get('disabled_left_hand_channels', []),
                "hand_writes": hand_writes, "target_q": core.q,
                "hand_write_enabled": hand_write_enabled,
                "goal_q": core.target, "commanded_hand_q": core.hand_q,
                "hand_goal": core.hand_target, "motion_done": core.motion_done,
                "trajectory_active": core.trajectory is not None,
                "boot_id": Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                "stop_backend": "robot-local left-only lowcmd+serial"}


def enable(expected_generation):
    # Motion mode query is read-only; never release or switch modes here.
    code, mode = switcher.CheckMode()
    if code != 0 or not isinstance(mode, dict) or mode.get("name"):
        raise ValueError("High-level controller still selected; cannot enable lowcmd: " + str((code, mode)))
    with lock:
        if core.generation != expected_generation:
            raise ValueError("STOP arrived during start preflight")
        if state is None or state.mode_machine != 5:
            raise ValueError("Expected verified G1 29-axis mode_machine=5")
        if config.get("hand_model") != "RH56DFQ" or not config.get("suspended_confirmed_by_user"):
            raise ValueError("Unverified hardware configuration")
        if any(m.mode != 0 for i, m in enumerate(state.motor_state[:29]) if i not in LEFT):
            raise ValueError("Initial test requires all non-left-arm joints inactive; do not take over another controller")
        if any(m.motorstate != 0 for m in state.motor_state[:29]):
            raise ValueError("Motor fault")
        if any(abs(m.dq) > 0.35 for m in state.motor_state[:29]):
            raise ValueError("Robot is moving")
        if hand_error or hand_data is None or time.monotonic() - hand_updated > 0.3:
            raise ValueError("No healthy fresh left-hand state")
        global relax_requested_at
        relax_requested_at = None
        core.enable([state.motor_state[i].q for i in LEFT], hand_data["q"])
        log({"event": "enabled", "generation": core.generation})


def prepare_lowlevel():
    # User confirmed the robot is suspended. Only release a high-level
    # controller if every non-left-arm motor is currently inactive.
    with lock:
        if state is None or time.monotonic() - core.last_state > 0.1:
            raise ValueError("Missing fresh state")
        if any(m.mode != 0 for i, m in enumerate(state.motor_state[:29]) if i not in LEFT):
            raise ValueError("Do not release an active controller")
    code, mode = switcher.CheckMode()
    if code:
        raise ValueError("Motion mode query failed")
    if mode.get("name"):
        code, _ = switcher.ReleaseMode()
        if code:
            raise ValueError("Could not release high-level controller")
    code, mode = switcher.CheckMode()
    if code or mode.get("name"):
        raise ValueError("High-level controller remains selected")
    log({"event": "prepared", "suspended_confirmed_by_user": True})


def control_loop():
    global arm_writes, last_loop, last_fault, stopped_since, relax_requested_at
    was_stopped = True
    while True:
        started = time.monotonic()
        with lock:
            dt = started - last_loop
            last_loop = started
            core.tick(dt)
            if core.stopped and not was_stopped:
                stopped_since = started
            was_stopped = core.stopped

            if core.active:
                if state is not None and not core.stopped:
                    if hand_error or time.monotonic() - hand_updated > 0.3:
                        stopped_since = started
                        core.stop("Left hand state unavailable")
                    for i in LEFT:
                        m = state.motor_state[i]
                        if m.motorstate or max(m.temperature) >= 92:
                            stopped_since = started
                            core.stop("Left-arm motor fault or temperature")
                        if abs(m.dq) > 0.8 or abs(m.q - core.q[i - 15]) > 0.32:
                            stopped_since = started
                            core.stop("Left-arm tracking fault")
                    if any(state.motor_state[i].mode != 0 for i in range(29) if i not in LEFT):
                        stopped_since = started
                        core.stop("Unexpected other-joint activation")

                # Relax when /relax was requested or if motor is hot (>=85C) while stopped
                hot_while_stopped = (
                    core.stopped
                    and state is not None
                    and any(max(state.motor_state[i].temperature) >= 85 for i in LEFT)
                )
                if hot_while_stopped and relax_requested_at is None:
                    relax_requested_at = started
                relax_s = (started - relax_requested_at) if (core.stopped and relax_requested_at is not None) else -1.0

                msg = unitree_hg_msg_dds__LowCmd_()
                msg.mode_pr = 0
                msg.mode_machine = 5
                if relax_s >= 2.5:
                    # Send mode=0 (zero torque) and deactivate until next /enable
                    for i in LEFT:
                        m = msg.motor_cmd[i]
                        m.mode = 0
                        m.q = 0.0
                        m.dq = 0.0
                        m.tau = 0.0
                        m.kp = 0.0
                        m.kd = 0.0
                    if relax_s >= 2.65:
                        core.active = False
                        relax_requested_at = None
                elif relax_s >= 0.0:
                    # Smooth 2.5s damping ramp-down so arm lowers slowly and gently without dropping
                    scale = max(0.0, 1.0 - (relax_s / 2.5))
                    for i in LEFT:
                        m = msg.motor_cmd[i]
                        m.mode = 1
                        m.q = core.q[i - 15]
                        m.dq = 0.0
                        m.tau = 0.0
                        base_kp = 40.0 if i == 15 else (45.0 if i < 19 else 18.0)
                        m.kp = base_kp * scale
                        m.kd = 3.5 if i < 19 else 1.8
                else:
                    for i in LEFT:
                        m = msg.motor_cmd[i]
                        m.mode = 1
                        m.q = core.q[i - 15]
                        m.dq = 0.0
                        m.tau = 0.0
                        m.kp = 40.0 if i == 15 else (45.0 if i < 19 else 18.0)
                        m.kd = 2.0 if i < 19 else 1.0
                # All other motor commands keep default inactive mode=0,
                # zero gains/torque. They must have been inactive at enable.
                msg.crc = crc.Crc(msg)
                try:
                    publisher.Write(msg)
                    arm_writes += 1
                except Exception as exc:
                    if not core.stopped:
                        stopped_since = started
                        core.stop("DDS publish failed")
                    last_fault = type(exc).__name__
        time.sleep(max(0, 0.004 - (time.monotonic() - started)))


def hand_loop():
    global hand_data, hand_updated, hand_error, hand_writes
    hand = None
    while hand is None:
        try:
            hand = InspireLeft()
        except Exception as exc:
            with lock:
                hand_error = type(exc).__name__ + ": " + str(exc)
            time.sleep(1.0)
    while True:
        try:
            data = hand.telemetry()
            with lock:
                hand_data = data
                hand_updated = time.monotonic()
                hand_error = None
                if any(data["errors"]) or max(data["temperature_c"]) >= 60:
                    hand_error = "Left hand fault or overtemperature"
                    core.stop(hand_error)
                write_q = core.hand_q.copy() if core.active and hand_write_enabled else None
            if write_q is not None:
                # Serial I/O must not block the arm watchdog or STOP lock.
                # At most one already-queued incremental hand write may finish
                # when STOP races this write; the next write is the frozen grip.
                hand.set_positions(write_q, config.get('disabled_left_hand_channels', []))
                with lock:
                    hand_writes += 1
        except Exception as exc:
            with lock:
                hand_error = type(exc).__name__ + ": " + str(exc)
                core.stop("Left hand telemetry/write failure")
        time.sleep(0.03)


def record_loop():
    while True:
        data = snapshot()
        data["robot_monotonic_s"] = time.monotonic()
        data["robot_wall_time_s"] = time.time()
        with root.joinpath("logs/robot-telemetry.jsonl").open("a") as f:
            f.write(json.dumps(data) + "\n")
        time.sleep(0.1)


class Handler(BaseHTTPRequestHandler):
    def reply(self, data, status=200):
        body = json.dumps(data, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def allowed(self):
        return self.client_address[0] in (os.environ.get("G1_OPERATOR_IP", "192.168.123.99"), "127.0.0.1")

    def do_GET(self):
        if not self.allowed():
            self.reply({"error": "PC address only"}, 403)
        elif self.path == "/state":
            self.reply(snapshot())
        else:
            self.reply({"error": "Not found"}, 404)

    def do_POST(self):
        global hand_write_enabled
        if not self.allowed():
            self.reply({"error": "PC address only"}, 403)
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length > 4096:
                raise ValueError("Request too large")
            data = json.loads(self.rfile.read(length)) if length else {}
            if self.path not in ('/stop','/relax','/heartbeat','/prepare-lowlevel'):
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
                if data.get('boot_id') != boot_id:
                    raise ValueError('Robot boot ID changed or is missing')
            if self.path == "/enable":
                with enable_lock:
                    with lock:
                        generation_before = data["generation"]
                    enable(generation_before)
            elif self.path == "/prepare-lowlevel":
                with enable_lock:
                    prepare_lowlevel()
            else:
                with lock:
                    if self.path == "/stop":
                        stop("PC stop")
                    elif self.path == "/relax":
                        relax("PC relax")
                    elif self.path == "/heartbeat":
                        core.heartbeat()
                    elif self.path == "/move-left-arm":
                        joint = data["joint_id"]
                        if type(joint) is not int or joint not in LEFT:
                            raise ValueError("Left-arm-only scope")
                        target = core.q[joint - 15] + float(data["delta_rad"])
                        lower, upper = limits[str(joint)]
                        if not lower + 0.02 <= target <= upper - 0.02:
                            raise ValueError("Official URDF joint limit")
                        core.move(data["joint_id"], float(data["delta_rad"]),
                                  float(data["speed_rad_s"]), data["generation"])
                        log({"event": "argon_arm_command", "command": data, "goal_q": core.target})
                    elif self.path == "/move-left-hand":
                        if any(abs(data['q'][i]-core.hand_q[i])>1e-6 for i in config.get('disabled_left_hand_channels', [])):
                            raise ValueError('Damaged pinky/ring commands are forbidden')
                        core.hand_move(data["q"], data["generation"])
                        hand_write_enabled = True
                        log({"event": "argon_hand_command", "command": data})
                    elif self.path == "/trajectory":
                        for w in data["waypoints"]:
                            if len(w["q"]) != 7:
                                raise ValueError("Left-arm trajectory requires 7 joints")
                            for i, v in enumerate(w["q"]):
                                lo, hi = limits[str(15+i)]
                                if not lo+0.02 <= v <= hi-0.02:
                                    raise ValueError("Official URDF joint limit")
                        core.submit_trajectory(data["waypoints"], data["generation"])
                        log({"event": "argon_trajectory", "command": data})
                    else:
                        raise ValueError("Unknown operation")
            self.reply(snapshot())
        except Exception as exc:
            log({"event": "request_rejected", "path": self.path, "reason": str(exc)})
            self.reply({"error": str(exc), "stopped": core.stopped}, 409)

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    ChannelFactoryInitialize(0, os.environ.get("G1_NETWORK_INTERFACE", "eth0"))
    subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    subscriber.Init(receive, 1)
    publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
    publisher.Init()
    crc = CRC()
    switcher = MotionSwitcherClient()
    switcher.SetTimeout(1.0)
    switcher.Init()
    checkpoint_path=root/'config/holding-transfer-state.json'
    if checkpoint_path.exists():
        checkpoint=json.loads(checkpoint_path.read_text())
        checkpoint_path.unlink()  # Single-use; a failed transfer cannot replay.
        deadline=time.monotonic()+3
        while state is None and time.monotonic()<deadline:time.sleep(.01)
        with lock:
            if time.monotonic()-core.last_state>.1:raise ValueError('Stale hardware state during transfer')
            q,h,generation=validate_checkpoint(checkpoint,state,Path('/proc/sys/kernel/random/boot_id').read_text().strip(),time.monotonic())
            core.q=q.copy();core.target=q.copy();core.hand_q=h.copy();core.hand_target=h.copy()
            core.active=True;core.stopped=True;core.generation=generation
            core.reason='Restored previous stopped left-arm hold; no queued movement'
        log({'event':'hold_restored','generation':generation,'hand_write_enabled':False})
    signal.signal(signal.SIGTERM, lambda *args: stop("Service termination requested; holding"))
    signal.signal(signal.SIGINT, lambda *args: stop("Keyboard stop; holding"))
    threading.Thread(target=control_loop, daemon=True).start()
    threading.Thread(target=hand_loop, daemon=True).start()
    threading.Thread(target=record_loop, daemon=True).start()
    log({"event": "startup", "stopped": True, "motor_writes": 0})
    ThreadingHTTPServer((os.environ.get("G1_BIND_IP", "192.168.123.164"), 8767), Handler).serve_forever()
