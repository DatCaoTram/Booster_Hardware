"""Protocol tests against the worked examples in the RH56 manual (section 2.2) plus the simulator."""
import struct
import time
import unittest

from inspire_hand_ros2 import rh56
from inspire_hand_ros2.sim import SimulatedHand

H = rh56.HOLD
# Manual 2.2.1: read ANGLE_ACT from hand 1, and the hand's reply (100,100,100,100,1000,0).
READ_REQ = bytes.fromhex("EB90010411 0A06 0C 32")
READ_REPLY = bytes.fromhex("90EB010F11 0A06 6400640064006400E8030000 AC")
# Manual 2.2.2: write ANGLE_SET = 100,100,100,100,1000,0 and the ack.
WRITE_REQ = bytes.fromhex("EB90010F12 CE05 6400640064006400E8030000 70")
WRITE_ACK = bytes.fromhex("90EB010412 CE05 01 EB")


class ScriptedPort:
    """Records writes and replays a canned reply, optionally one byte per read()."""

    def __init__(self, reply, trickle=False):
        self.reply, self.trickle, self.written, self.out = reply, trickle, b"", b""

    def write(self, data):
        self.written += data
        self.out = self.reply

    def read(self, n):
        n = 1 if self.trickle else n
        data, self.out = self.out[:n], self.out[n:]
        return data

    def reset_input_buffer(self):
        self.out = b""

    def close(self):
        pass


class FrameTests(unittest.TestCase):
    def test_read_request_matches_manual(self):
        self.assertEqual(rh56.build_frame(1, rh56.CMD_READ, rh56.ANGLE_ACT, [12]), READ_REQ)

    def test_write_request_matches_manual(self):
        payload = struct.pack("<6h", 100, 100, 100, 100, 1000, 0)
        self.assertEqual(rh56.build_frame(1, rh56.CMD_WRITE, rh56.ANGLE_SET, payload), WRITE_REQ)

    def test_parse_manual_replies(self):
        data = rh56.parse_reply(READ_REPLY, 1, rh56.CMD_READ, rh56.ANGLE_ACT, 12)
        self.assertEqual(struct.unpack("<6h", data), (100, 100, 100, 100, 1000, 0))
        self.assertEqual(rh56.parse_reply(WRITE_ACK, 1, rh56.CMD_WRITE, rh56.ANGLE_SET, 1), b"\x01")

    def test_bad_checksum_rejected(self):
        with self.assertRaises(rh56.ProtocolError):
            rh56.parse_reply(READ_REPLY[:-1] + b"\x00", 1, rh56.CMD_READ, rh56.ANGLE_ACT, 12)

    def test_reply_from_other_hand_rejected(self):
        with self.assertRaises(rh56.ProtocolError):
            rh56.parse_reply(READ_REPLY, 2, rh56.CMD_READ, rh56.ANGLE_ACT, 12)


class RH56Tests(unittest.TestCase):
    def test_read_angles(self):
        port = ScriptedPort(READ_REPLY)
        self.assertEqual(rh56.RH56(port).read_angles(), [100, 100, 100, 100, 1000, 0])
        self.assertEqual(port.written, READ_REQ)

    def test_set_angles(self):
        port = ScriptedPort(WRITE_ACK)
        rh56.RH56(port).set_angles([100, 100, 100, 100, 1000, 0])
        self.assertEqual(port.written, WRITE_REQ)

    def test_junk_and_split_reply(self):
        port = ScriptedPort(b"\x00\x90\x13" + READ_REPLY, trickle=True)
        self.assertEqual(rh56.RH56(port).read_angles(), [100, 100, 100, 100, 1000, 0])

    def test_no_reply_times_out(self):
        port = ScriptedPort(b"")
        with self.assertRaises(rh56.ProtocolError):
            rh56.RH56(port, timeout=0.02).read_angles()

    def test_clamping(self):
        self.assertEqual(rh56.clamp_angles([-5, 1200, 500, None, 0, 1000]), [H, 1000, 500, H, 0, 1000])
        self.assertEqual(rh56.clamp_forces([2000] * 6), [1000, 1000, 1000, 1000, 1500, 1500])

    def test_summarize(self):
        self.assertEqual(rh56.summarize([2] * 6, [0] * 6), (0, "OK"))
        level, msg = rh56.summarize([2, 2, 2, 6, 2, 2], [0, 0, 0, 0b101, 0, 0])
        self.assertEqual(level, 2)
        self.assertEqual(msg, "index: locked rotor, over-current")


class SimulatorTests(unittest.TestCase):
    def test_moves_toward_target_and_holds(self):
        hand = rh56.RH56(SimulatedHand())
        hand.set_speeds([1000] * 6)
        hand.set_angles([0, H, 500, H, H, H])
        time.sleep(0.9)
        hand.read_angles()  # the sim integrates motion on each transaction
        angles = hand.read_angles()
        self.assertEqual(angles, [0, 1000, 500, 1000, 1000, 1000])
        self.assertEqual(hand.read_feedback()["status"], [2] * 6)

    def test_feedback_and_clear(self):
        sim = SimulatedHand()
        hand = rh56.RH56(sim)
        sim.regs[rh56.ERROR + 3] = 0b1
        self.assertEqual(hand.read_feedback()["error"], [0, 0, 0, 1, 0, 0])
        hand.clear_errors()
        fb = hand.read_feedback()
        self.assertEqual(fb["error"], [0] * 6)
        self.assertEqual(fb["temp"], [32] * 6)

    def test_ignores_other_ids(self):
        with self.assertRaises(rh56.ProtocolError):
            rh56.RH56(SimulatedHand(hand_id=2), hand_id=1, timeout=0.02).read_angles()


if __name__ == "__main__":
    unittest.main()
