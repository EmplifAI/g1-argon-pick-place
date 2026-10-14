"""Send a fresh camera frame and measured state to Argon. No motion publishing."""
import base64
import json
import os
import time
import urllib.request
import urllib.error
from pathlib import Path

root = Path(__file__).resolve().parents[1]
config = json.loads((root / "config/argon.json").read_text())
key = os.environ["GEMINI_API_KEY"]
state = json.loads((root / "captures/state.json").read_text())
if time.time() - state["observed_at"] > 30:
    raise SystemExit("Robot state is stale; rerun observe_state.py")
with urllib.request.urlopen("http://127.0.0.1:8765/stream", timeout=12) as response:
    buffer = b""
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline and len(buffer) < 10_000_000:
        buffer += response.read(4096)
        start = buffer.find(b"\xff\xd8")
        end = buffer.find(b"\xff\xd9", max(0, start) + 2)
        if start >= 0 and end >= 0:
            image = buffer[start:end + 2]
            break
    else:
        raise SystemExit("No complete fresh camera frame")
(root / "captures/argon-input.jpg").write_bytes(image)
prompt = """You are observing a real Unitree G1 with Inspire hands. The user wants a
very slow, small left-arm-only initial test, eventually bottle pick and place at B.
Do NOT execute anything: this observer has no robot tools. Analyze visible clearance
and measured state. Return a brief Japanese observation and a JSON proposal with
action = stop or propose_left_arm_test, blocking_reasons, joint_id, delta_rad,
speed_rad_s. A proposal must be only one joint 15..21, abs(delta)<=0.03 rad,
speed<=0.01 rad/s. No right arm, hands, waist, legs or mode changes allowed.
Do not assume a camera view proves clearance for hidden limbs. The control mode
query failed (FSM response code 3102); left-only command isolation is not yet
verified. Explain what must be verified before any motion. Distinguish observed
facts from unknowns. State: """ + json.dumps(state)
request_body = dict(config, input=[{"type": "text", "text": prompt},
    {"type": "image", "mime_type": "image/jpeg",
     "data": base64.b64encode(image).decode()}])
request = urllib.request.Request(
    "https://generativelanguage.googleapis.com/v1beta/interactions",
    data=json.dumps(request_body).encode(),
    headers={"Content-Type": "application/json", "x-goog-api-key": key})
print("Argon request: thinking_level=high, fresh RGB+state, motion disabled", flush=True)
try:
    with urllib.request.urlopen(request, timeout=180) as response:
        result = json.load(response)
except urllib.error.HTTPError as error:
    report = {"http_status": error.code, "model": config["model"],
              "thinking_level": config["generation_config"]["thinking_level"],
              "motion_sent": False}
    # Do not log raw server error bodies: these may echo sensitive request data.
    try:
        parsed = json.load(error).get("error", {})
        report["error_status"] = parsed.get("status")
        report["error_reasons"] = [d["reason"] for d in parsed.get("details", []) if "reason" in d]
    except (ValueError, AttributeError, TypeError):
        pass
    (root / "logs/argon-observation.json").write_text(json.dumps(report, indent=2))
    raise SystemExit(json.dumps(report)) from None
# Save model outputs only, never request headers or input credentials.
report = {k: result[k] for k in ("id", "model", "status", "output_text", "outputs", "usage") if k in result}
if "steps" in result:
    report["model_outputs"] = [step for step in result["steps"] if step.get("type") == "model_output"]
(root / "logs/argon-observation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps(report, ensure_ascii=False))
