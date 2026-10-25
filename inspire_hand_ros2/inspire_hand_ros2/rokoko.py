"""Rokoko Smartgloves (via Rokoko Studio's live stream) -> RH56 finger targets. No ROS dependency.

Rokoko Studio: open a project -> Streaming panel -> Custom -> wrench icon: Forward IP = the machine
running this code, port 14043, format JSON -> Activate. (LZ4-compressed JSON works too after
`pip install lz4`.)

Each UDP packet is one JSON v3 frame; scene.actors[].body maps bone names (leftHand,
leftIndexProximal, ...) to world-space rotations {x, y, z, w}. Curl is measured from each finger
joint's rotation relative to its parent bone, so moving or turning the whole hand doesn't move the
robot's fingers.

Calibration: "open" = hand flat, fingers straight; "closed" = fist with the thumb folded across the
fingers. After both, each joint is measured along the axis it moved between the two poses, so
sideways finger spread is ignored. Uncalibrated, total joint angle is compared to a typical range.
"""
import json
import math
import os
import socket
import time

DEFAULT_PORT = 14043
DEFAULT_CALIBRATION = "~/.rokoko_rh56_calibration.json"
FINGERS = ("Little", "Ring", "Middle", "Index")  # RH56 DOF order, then thumb bend, thumb rotate
# Typical full joint motion (rad) used before calibration: finger MCP+PIP+DIP, thumb MCP+IP, thumb CMC.
DEFAULT_RANGE = [math.radians(a) for a in (220, 220, 220, 220, 100, 45)]
MIN_RANGE = math.radians(8)  # joints that moved less than this during calibration are ignored

# Rokoko Studio's reference T-pose (w, x, y, z), from Rokoko's Blender plugin; finger bones match the hand.
TPOSE = {
    "leftHand": (-0.5, -0.5, -0.5, 0.5),
    "leftThumbProximal": (-0.0923, -0.56098, -0.70106, 0.43046),
    "leftThumbMedial": (-0.2706, -0.65328, -0.65328, 0.2706),
    "leftThumbDistal": (-0.2706, -0.65328, -0.65328, 0.2706),
    "rightHand": (0.5, 0.5, -0.5, 0.5),
    "rightThumbProximal": (0.0923, 0.56099, -0.70106, 0.43046),
    "rightThumbMedial": (0.2706, 0.65328, -0.65328, 0.2706),
    "rightThumbDistal": (0.2706, 0.65328, -0.65328, 0.2706),
}


# -- quaternions as (w, x, y, z) ------------------------------------------------------------------

def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def qconj(q):
    return (q[0], -q[1], -q[2], -q[3])


def qnormalize(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def qlog(q):
    """Rotation vector (axis * angle) of q, shortest way round."""
    w, x, y, z = q if q[0] >= 0 else tuple(-c for c in q)
    s = math.sqrt(x * x + y * y + z * z)
    if s < 1e-9:
        return (0.0, 0.0, 0.0)
    k = 2.0 * math.atan2(s, w) / s
    return (x * k, y * k, z * k)


def _norm(v):
    return math.sqrt(sum(c * c for c in v))


# -- stream parsing -------------------------------------------------------------------------------

def decode(packet):
    try:
        return json.loads(packet)
    except (UnicodeDecodeError, ValueError):
        pass
    try:
        import lz4.frame
    except ImportError:
        raise ValueError("packet isn't plain JSON: set Rokoko Studio's Custom stream to JSON, "
                         "or `pip install lz4`")
    try:
        return json.loads(lz4.frame.decompress(packet))
    except Exception as e:
        raise ValueError("unreadable packet: %s" % e)


def dof_joints(side):
    """(parent, child) bone pairs behind each RH56 DOF: little, ring, middle, index, thumb bend, thumb rotate."""
    hand = side + "Hand"
    dofs = []
    for finger in FINGERS:
        p, m, d = (side + finger + seg for seg in ("Proximal", "Medial", "Distal"))
        dofs.append([(hand, p), (p, m), (m, d)])
    tp, tm, td = (side + "Thumb" + seg for seg in ("Proximal", "Medial", "Distal"))
    dofs.append([(tp, tm), (tm, td)])
    dofs.append([(hand, tp)])
    return dofs


def find_glove(frame, side, actor=None):
    """(actor name, body) of the first actor streaming this glove, else (None, None)."""
    bones = {b for dof in dof_joints(side) for joint in dof for b in joint}
    scene = frame.get("scene", frame) if isinstance(frame, dict) else {}
    for a in scene.get("actors") or []:
        if actor and a.get("name") != actor:
            continue
        if (a.get("meta") or {}).get("hasGloves") is False:
            continue
        body = a.get("body") or {}
        if bones <= body.keys():
            return a.get("name"), body
    return None, None


def _rotation(body, bone):
    r = body[bone]["rotation"]
    return qnormalize((r["w"], r["x"], r["y"], r["z"]))


def _tpose(bone):
    return TPOSE.get(bone) or TPOSE[("left" if bone.startswith("left") else "right") + "Hand"]


# -- mapping --------------------------------------------------------------------------------------

class GloveMapper:
    """One glove's bones -> six RH56 targets (1000 open .. 0 closed)."""

    def __init__(self, side="left"):
        if side not in ("left", "right"):
            raise ValueError("glove side must be left or right")
        self.side = side
        self.dofs = dof_joints(side)
        self.joints = [j for dof in self.dofs for j in dof]
        self.open = {j: qmul(qconj(_tpose(j[0])), _tpose(j[1])) for j in self.joints}
        self.closed = None
        self._axes = {}

    def relative(self, body):
        return {(p, c): qmul(qconj(_rotation(body, p)), _rotation(body, c)) for p, c in self.joints}

    def calibrate(self, pose, body):
        if pose not in ("open", "closed"):
            raise ValueError(pose)
        setattr(self, pose, self.relative(body))
        self._update_axes()

    def _update_axes(self):
        self._axes = {}
        for j in self.joints if self.closed else ():
            v = qlog(qmul(qconj(self.open[j]), self.closed[j]))
            n = _norm(v)
            if n >= MIN_RANGE:
                self._axes[j] = (tuple(c / n for c in v), n)

    def closure(self, body):
        """Per DOF: 0 = like the open pose, 1 = like the closed pose."""
        rel = self.relative(body)
        out = []
        for dof, default in zip(self.dofs, DEFAULT_RANGE):
            deltas = {j: qlog(qmul(qconj(self.open[j]), rel[j])) for j in dof}
            calibrated = [j for j in dof if j in self._axes]
            if calibrated:
                moved = sum(sum(a * b for a, b in zip(deltas[j], self._axes[j][0])) for j in calibrated)
                span = sum(self._axes[j][1] for j in calibrated)
            else:
                moved, span = sum(_norm(v) for v in deltas.values()), default
            out.append(min(1.0, max(0.0, moved / span)))
        return out

    def targets(self, body):
        return [int(round(1000 * (1.0 - c))) for c in self.closure(body)]

    def to_json(self):
        enc = lambda d: None if d is None else {"%s>%s" % j: list(q) for j, q in d.items()}
        return {"side": self.side, "open": enc(self.open), "closed": enc(self.closed)}

    def load_json(self, data):
        if data.get("side") != self.side:
            raise ValueError("calibration is for the %s glove" % data.get("side"))
        dec = lambda d: None if d is None else {tuple(k.split(">")): tuple(v) for k, v in d.items()}
        opened, closed = dec(data["open"]), dec(data.get("closed"))
        for d in (opened, closed):
            if d is not None and set(self.joints) - d.keys():
                raise ValueError("calibration file is missing joints")
        self.open, self.closed = opened, closed
        self._update_axes()


# -- live input -----------------------------------------------------------------------------------

class StudioReceiver:
    def __init__(self, port=DEFAULT_PORT, host=""):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.port = self.sock.getsockname()[1]

    def poll(self):
        """Newest frame since the last call (older ones are dropped so we never lag), or None."""
        latest = None
        while True:
            try:
                latest, _ = self.sock.recvfrom(1 << 17)
            except (BlockingIOError, InterruptedError):
                break
            except ConnectionResetError:  # Windows reports stray ICMP errors on UDP sockets
                continue
        return None if latest is None else decode(latest)

    def close(self):
        self.sock.close()


class GloveTeleop:
    """Receiver + mapper + smoothing. Call update() regularly; it returns six targets or None."""

    def __init__(self, side="left", port=DEFAULT_PORT, actor=None, calibration_file=DEFAULT_CALIBRATION,
                 smoothing=0.5, timeout=0.5):
        self.mapper = GloveMapper(side)
        self.receiver = StudioReceiver(port)
        self.actor_filter = actor or None
        self.calibration_file = os.path.expanduser(calibration_file) if calibration_file else None
        self.smoothing, self.timeout = smoothing, timeout
        self.actor = self.body = self.filtered = self.error = None
        self.seen = 0.0
        if self.calibration_file and os.path.exists(self.calibration_file):
            with open(self.calibration_file) as f:
                self.mapper.load_json(json.load(f))

    @property
    def calibrated(self):
        return self.mapper.closed is not None

    def _live(self):
        return self.body is not None and time.monotonic() - self.seen < self.timeout

    def update(self):
        try:
            frame = self.receiver.poll()
        except ValueError as e:
            frame, self.error = None, str(e)
        if frame is not None:
            name, body = find_glove(frame, self.mapper.side, self.actor_filter)
            if body is None:
                self.error = "stream has no %s glove%s" % (
                    self.mapper.side, " on actor %r" % self.actor_filter if self.actor_filter else "")
            else:
                self.actor, self.body, self.seen, self.error = name, body, time.monotonic(), None
        if not self._live():
            self.filtered = None
            return None
        target = self.mapper.targets(self.body)
        if self.filtered is None:
            self.filtered = [float(t) for t in target]
        else:
            self.filtered = [f + (1.0 - self.smoothing) * (t - f) for f, t in zip(self.filtered, target)]
        return [int(round(f)) for f in self.filtered]

    def calibrate(self, pose):
        if not self._live():
            raise RuntimeError("no live glove data to calibrate from")
        self.mapper.calibrate(pose, self.body)
        if self.calibration_file:
            with open(self.calibration_file, "w") as f:
                json.dump(self.mapper.to_json(), f, indent=1)
        return "calibrated %s pose" % pose

    def status(self):
        if self.error:
            return self.error
        if self.body is None:
            return "waiting for Rokoko Studio on UDP port %d" % self.receiver.port
        if not self._live():
            return "glove stream stopped (%s)" % self.actor
        return "%s glove of %s%s" % (self.mapper.side, self.actor,
                                     "" if self.calibrated else " - not calibrated yet")

    def close(self):
        self.receiver.close()
