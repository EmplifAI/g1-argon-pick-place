"""Robot-local latched motion state. No DDS/network dependencies."""
import math
import time

LEFT = tuple(range(15, 22))


class SafetyCore:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.stopped = True
        self.reason = "startup"
        self.generation = 0
        self.last_heartbeat = -math.inf
        self.last_state = -math.inf
        self.active = False
        self.q = None
        self.target = None
        self.speed = 0.01
        self.hand_q = None
        self.hand_target = None
        self.trajectory = None
        self.trajectory_started = None
        self.motion_done = False

    def heartbeat(self):
        self.last_heartbeat = self.clock()

    def observe(self):
        self.last_state = self.clock()

    def stop(self, reason="PC stop"):
        self.generation += 1
        self.stopped = True
        self.reason = reason
        self.trajectory = None
        self.motion_done = False
        if self.q is not None:
            self.target = self.q.copy()
        if self.hand_q is not None:
            self.hand_target = self.hand_q.copy()

    def enable(self, q, hand_q=None):
        self.check_health()
        if self.clock() - self.last_heartbeat > 0.4:
            raise ValueError("No recent PC heartbeat")
        if self.clock() - self.last_state > 0.1:
            raise ValueError("No recent robot state")
        if len(q) != 7 or not all(math.isfinite(v) for v in q):
            raise ValueError("Invalid left-arm state")
        self.q = list(q)
        self.target = self.q.copy()
        self.hand_q = list(hand_q) if hand_q is not None else None
        self.hand_target = self.hand_q.copy() if self.hand_q is not None else None
        self.active = True
        self.stopped = False
        self.reason = "enabled at measured position"
        self.trajectory = None
        self.generation += 1

    def check_health(self):
        if self.active and not self.stopped and self.clock() - self.last_heartbeat > 0.4:
            self.stop("PC heartbeat timeout")
        if self.active and not self.stopped and self.clock() - self.last_state > 0.1:
            self.stop("Robot state timeout")

    def move(self, joint_id, delta, speed, generation):
        self.check_health()
        if self.stopped or not self.active:
            raise ValueError("STOP latched")
        if generation != self.generation:
            raise ValueError("Stale command generation")
        if type(joint_id) is not int or joint_id not in LEFT:
            raise ValueError("Only left-arm joints 15..21 allowed")
        if not math.isfinite(delta) or abs(delta) > 0.03:
            raise ValueError("Initial test delta limit is 0.03 rad")
        if not math.isfinite(speed) or not 0 < speed <= 0.01:
            raise ValueError("Initial test speed limit is 0.01 rad/s")
        self.target = self.q.copy()
        self.target[joint_id - 15] += delta
        self.speed = speed

    def hand_move(self, target, generation):
        self.check_health()
        if self.stopped or self.hand_q is None or generation != self.generation:
            raise ValueError("Hand command unavailable or STOP latched")
        if len(target) != 6 or not all(math.isfinite(v) and 0 <= v <= 1 for v in target):
            raise ValueError("Invalid hand target")
        if max(abs(a - b) for a, b in zip(target, self.hand_q)) > 0.1 + 1e-12:
            raise ValueError("Initial hand delta limit is 0.1")
        self.hand_target = list(target)

    def submit_trajectory(self, waypoints, generation):
        self.check_health()
        if self.stopped or not self.active or generation != self.generation:
            raise ValueError("STOP latched or stale trajectory")
        if not isinstance(waypoints, list) or not 2 <= len(waypoints) <= 200:
            raise ValueError("Invalid waypoint count")
        previous = None
        for w in waypoints:
            t, q = w["t_s"], w["q"]
            if not math.isfinite(t) or len(q) != 7 or not all(math.isfinite(v) for v in q):
                raise ValueError("Invalid trajectory data")
            if previous is None:
                if t != 0 or max(abs(a-b) for a,b in zip(q,self.q)) > 0.005:
                    raise ValueError("Trajectory does not begin at current commanded pose")
            else:
                dt = t - previous["t_s"]
                if dt <= 0 or max(abs(a-b) for a,b in zip(q,previous["q"])) / dt > 0.060001:
                    raise ValueError("Trajectory velocity limit")
                if max(abs(a-b) for a,b in zip(q,previous["q"])) > 0.15:
                    raise ValueError("Trajectory discontinuity")
            previous = w
        if not 0 < waypoints[-1]["t_s"] <= 30:
            raise ValueError("Trajectory duration limit")
        self.trajectory = [{"t_s": w["t_s"], "q": list(w["q"])} for w in waypoints]
        self.trajectory_started = self.clock()
        self.motion_done = False

    def tick(self, dt):
        self.check_health()
        if self.active and (not math.isfinite(dt) or dt > 0.05 or dt <= 0):
            self.stop("Control timing fault")
        if not self.stopped:
            if self.trajectory:
                elapsed = self.clock() - self.trajectory_started
                if elapsed > self.trajectory[-1]["t_s"] + 2:
                    self.stop("Trajectory lease expired")
                elif elapsed >= self.trajectory[-1]["t_s"]:
                    self.q = self.trajectory[-1]["q"].copy()
                    self.target = self.q.copy()
                    self.motion_done = True
                    self.trajectory = None
                else:
                    for a, b in zip(self.trajectory, self.trajectory[1:]):
                        if a["t_s"] <= elapsed <= b["t_s"]:
                            f = (elapsed-a["t_s"])/(b["t_s"]-a["t_s"])
                            self.q = [x+f*(y-x) for x,y in zip(a["q"],b["q"])]
                            self.target = self.q.copy()
                            break
            else:
                for i in range(7):
                    d = self.target[i] - self.q[i]
                    self.q[i] += max(-self.speed * dt, min(self.speed * dt, d))
            if self.hand_q is not None:
                for i in range(6):
                    d = self.hand_target[i] - self.hand_q[i]
                    self.hand_q[i] += max(-0.05 * dt, min(0.05 * dt, d))
        return self.q
