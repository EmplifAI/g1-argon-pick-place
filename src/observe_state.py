"""Read G1 state only; never create a command publisher."""
import json
import time
from pathlib import Path
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

ChannelFactoryInitialize(0, "enp12s0")
subscriber = ChannelSubscriber("rt/lowstate", LowState_)
subscriber.Init()
deadline = time.monotonic() + 5
msg = None
while time.monotonic() < deadline:
    msg = subscriber.Read()
    if msg is not None:
        break
    time.sleep(0.02)
if msg is None:
    raise SystemExit("No fresh G1 lowstate received")
client = LocoClient()
client.SetTimeout(3)
client.Init()
result = {
    "observed_at": time.time(),
    "mode_machine": int(msg.mode_machine),
    "fsm": client.GetFsmId(),
    "imu_rpy": list(msg.imu_state.rpy),
    "imu_quaternion": list(msg.imu_state.quaternion),
    "imu_gyroscope": list(msg.imu_state.gyroscope),
    "imu_accelerometer": list(msg.imu_state.accelerometer),
    "motors": [{"id": i, "mode": m.mode, "q": m.q, "dq": m.dq,
                "tau_est": m.tau_est, "temperature": list(m.temperature),
                "voltage": m.vol, "motorstate": m.motorstate}
               for i, m in enumerate(msg.motor_state)],
    "left_arm": [{"id": i, "q": msg.motor_state[i].q,
                  "dq": msg.motor_state[i].dq} for i in range(15, 22)],
    "right_arm": [{"id": i, "q": msg.motor_state[i].q,
                   "dq": msg.motor_state[i].dq} for i in range(22, 29)],
}
destination = Path(__file__).resolve().parents[1] / "captures/state.json"
destination.write_text(json.dumps(result, indent=2))
print(json.dumps(result))
