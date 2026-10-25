"""Glove -> RH56 mapping tests on synthetic Rokoko Studio frames (JSON v3 layout)."""
import json
import math
import socket
import time
import unittest

from inspire_hand_ros2 import rh56, rokoko
from inspire_hand_ros2.rokoko import qconj, qmul
from inspire_hand_ros2.sim import SimulatedHand

FLEX, SPREAD, TWIST = (0, 0, 1), (0, 1, 0), (1, 0, 0)  # arbitrary bone-local axes for the test hand


def aa(axis, deg):
    s = math.sin(math.radians(deg) / 2)
    return (math.cos(math.radians(deg) / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def make_frame(side="left", fingers=((0, 0, 0),) * 4, cmc=0, thumb=(0, 0), spread=0,
               hand_rot=(1, 0, 0, 0), actor="Actor1"):
    """World-space bones like Rokoko Studio sends: T-pose, then the given joint angles in degrees."""
    T = rokoko.TPOSE
    hand = qmul(hand_rot, T[side + "Hand"])
    bones = {side + "Hand": hand}
    for name, (a1, a2, a3) in zip(rokoko.FINGERS, fingers):
        q = qmul(hand, qmul(aa(SPREAD, spread), aa(FLEX, a1)))
        bones[side + name + "Proximal"] = q
        q = qmul(q, aa(FLEX, a2))
        bones[side + name + "Medial"] = q
        bones[side + name + "Distal"] = qmul(q, aa(FLEX, a3))
    tp = qmul(qmul(hand, qmul(qconj(T[side + "Hand"]), T[side + "ThumbProximal"])), aa(TWIST, cmc))
    tm = qmul(qmul(tp, qmul(qconj(T[side + "ThumbProximal"]), T[side + "ThumbMedial"])), aa(FLEX, thumb[0]))
    bones.update({side + "ThumbProximal": tp, side + "ThumbMedial": tm,
                  side + "ThumbDistal": qmul(tm, aa(FLEX, thumb[1]))})
    body = {k: {"position": {"x": 0, "y": 0, "z": 0},
                "rotation": {"w": q[0], "x": q[1], "y": q[2], "z": q[3]}} for k, q in bones.items()}
    return {"version": 3, "fps": 60, "scene": {"timestamp": 0.0, "props": [], "actors": [
        {"name": actor, "meta": {"hasGloves": True, "hasBody": False, "hasFace": False}, "body": body}]}}


def body(frame):
    return rokoko.find_glove(frame, "left")[1]


OPEN = dict(fingers=((10, 10, 5),) * 4, cmc=5, thumb=(5, 5))
FIST = dict(fingers=((80, 95, 60),) * 4, cmc=40, thumb=(45, 60))
TILTED = qmul(aa((0.3, 0.9, 0.1), 70), aa((1, 0, 0), -35))
TILTED = tuple(c / math.sqrt(sum(x * x for x in TILTED)) for c in TILTED)


class MapperTests(unittest.TestCase):
    def test_tpose_is_open(self):
        self.assertEqual(rokoko.GloveMapper().targets(body(make_frame())), [1000] * 6)

    def test_uncalibrated_curl(self):
        m = rokoko.GloveMapper()
        t = m.targets(body(make_frame(fingers=((0, 0, 0), (0, 0, 0), (40, 40, 30), (80, 80, 60)),
                                      cmc=45, thumb=(50, 50))))
        self.assertEqual(t[:2], [1000, 1000])
        self.assertAlmostEqual(t[2], 500, delta=2)
        self.assertEqual(t[3:], [0, 0, 0])

    def test_whole_hand_rotation_is_ignored(self):
        m = rokoko.GloveMapper()
        pose = dict(fingers=((30, 40, 20),) * 4, cmc=20, thumb=(30, 10))
        self.assertEqual(m.targets(body(make_frame(**pose))),
                         m.targets(body(make_frame(hand_rot=TILTED, **pose))))

    def test_calibrated_open_fist_and_halfway(self):
        m = rokoko.GloveMapper()
        m.calibrate("open", body(make_frame(**OPEN)))
        m.calibrate("closed", body(make_frame(**FIST)))
        self.assertEqual(m.targets(body(make_frame(**OPEN))), [1000] * 6)
        self.assertEqual(m.targets(body(make_frame(**FIST))), [0] * 6)
        half = m.targets(body(make_frame(fingers=((45, 52.5, 32.5),) * 4, cmc=22.5, thumb=(25, 32.5))))
        for t in half:
            self.assertAlmostEqual(t, 500, delta=15)

    def test_calibration_ignores_sideways_spread(self):
        m = rokoko.GloveMapper()
        spread_open = body(make_frame(spread=20, **OPEN))
        self.assertLess(m.targets(spread_open)[0], 900)  # uncalibrated: spread looks like curl
        m.calibrate("open", body(make_frame(**OPEN)))
        m.calibrate("closed", body(make_frame(**FIST)))
        self.assertGreater(m.targets(spread_open)[0], 990)

    def test_calibration_round_trip(self):
        m = rokoko.GloveMapper()
        m.calibrate("open", body(make_frame(**OPEN)))
        m.calibrate("closed", body(make_frame(**FIST)))
        m2 = rokoko.GloveMapper()
        m2.load_json(json.loads(json.dumps(m.to_json())))
        pose = body(make_frame(fingers=((50, 30, 20),) * 4, cmc=10, thumb=(20, 40)))
        self.assertEqual(m.targets(pose), m2.targets(pose))
        with self.assertRaises(ValueError):
            rokoko.GloveMapper("right").load_json(m.to_json())

    def test_find_glove(self):
        right = make_frame(side="right", actor="Bob")
        self.assertEqual(rokoko.find_glove(right, "left"), (None, None))
        self.assertEqual(rokoko.find_glove(right, "right")[0], "Bob")
        self.assertEqual(rokoko.find_glove(right, "right", actor="Alice"), (None, None))
        self.assertEqual(rokoko.GloveMapper("right").targets(rokoko.find_glove(right, "right")[1]), [1000] * 6)

    def test_non_json_packet(self):
        with self.assertRaises(ValueError):
            rokoko.decode(b"\x04\x22\x4d\x18 not json")


class StreamTests(unittest.TestCase):
    def test_udp_to_simulated_hand(self):
        teleop = rokoko.GloveTeleop(port=0, calibration_file=None, smoothing=0.0)
        port = teleop.receiver.sock.getsockname()[1]
        out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        hand = rh56.RH56(SimulatedHand())
        try:
            self.assertIsNone(teleop.update())
            self.assertIn("waiting", teleop.status())
            out.sendto(json.dumps(make_frame(fingers=((80, 80, 60),) * 4)).encode(), ("127.0.0.1", port))
            time.sleep(0.05)
            target = teleop.update()
            self.assertEqual(target, [0, 0, 0, 0, 1000, 1000])
            self.assertIn("not calibrated", teleop.status())
            hand.set_angles(target)
            self.assertEqual(hand.port.target, [0, 0, 0, 0, 1000, 1000])
            time.sleep(0.6)  # stream stops -> no more targets
            self.assertIsNone(teleop.update())
            self.assertIn("stopped", teleop.status())
        finally:
            out.close()
            teleop.close()


if __name__ == "__main__":
    unittest.main()
