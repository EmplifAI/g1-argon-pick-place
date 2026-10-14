"""Fail-closed local motion gate; not a hardware emergency-stop circuit."""
import json
import math
import os
import time
from pathlib import Path

STATE = Path(__file__).resolve().parents[1] / "config/stop-state.json"


def latch_stop(reason="PC stop"):
    STATE.parent.mkdir(parents=True, exist_ok=True)
    record = {"stopped": True, "reason": reason, "updated_at": time.time(),
              "hardware_connected": False}
    temporary = STATE.with_suffix(".tmp")
    with temporary.open("w") as f:
        json.dump(record, f)
        f.flush()
        os.fsync(f.fileno())
    temporary.replace(STATE)
    return record


def require_motion_permission():
    """Future controller must call before every command, not only at startup.

    This phase has no enable mechanism. Motion remains prohibited until a
    verified controller, hardware watchdog and deliberate enable flow exist.
    """
    try:
        record = json.loads(STATE.read_text())
    except (OSError, ValueError):
        raise RuntimeError("STOP: missing or invalid motion gate") from None
    age = time.time() - record.get("updated_at", 0)
    if not math.isfinite(age) or not 0 <= age < 0.5:
        raise RuntimeError("STOP: stale motion gate")
    if record.get("stopped") is not False:
        raise RuntimeError("STOP: latched")
    # Fail closed until actual robot stop handling is implemented and tested.
    raise RuntimeError("STOP: hardware stop integration not implemented")
