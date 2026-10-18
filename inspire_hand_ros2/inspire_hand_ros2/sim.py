"""In-memory stand-in for a serial port with an RH56 hand on it.

Speaks the same byte protocol as the real hand, so RH56(SimulatedHand()) exercises
the full stack. Fingers move toward ANGLE_SET at the configured speed; there are
no objects to grasp, so force stays 0.
"""
import struct
import time

from . import rh56


class SimulatedHand:
    def __init__(self, hand_id=1):
        self.hand_id = hand_id
        self.regs = bytearray(2400)
        self.target = [1000.0] * 6
        self.angle = [1000.0] * 6
        self._put(rh56.ANGLE_SET, [1000] * 6)
        self._put(rh56.SPEED_SET, [1000] * 6)
        self._put(rh56.FORCE_SET, [500] * 6)
        self.regs[rh56.TEMP:rh56.TEMP + 6] = bytes([32] * 6)
        self._out = b""
        self._last = time.monotonic()
        self._update()

    # serial-port interface
    def write(self, frame):
        self._step()
        reply = self._handle(bytes(frame))
        if reply:
            self._out += reply
        return len(frame)

    def read(self, n):
        data, self._out = self._out[:n], self._out[n:]
        return data

    def reset_input_buffer(self):
        self._out = b""

    def close(self):
        pass

    # internals
    def _put(self, addr, values):
        self.regs[addr:addr + 2 * len(values)] = struct.pack("<%dh" % len(values), *values)

    def _get(self, addr, count=6):
        return list(struct.unpack("<%dh" % count, self.regs[addr:addr + 2 * count]))

    def _handle(self, frame):
        if len(frame) < 9 or frame[:2] != b"\xEB\x90" or frame[-1] != rh56.checksum(frame[2:-1]):
            return None  # the real hand ignores garbage
        hand_id, cmd, addr, payload = frame[2], frame[4], frame[5] | frame[6] << 8, frame[7:-1]
        if hand_id != self.hand_id:
            return None
        if cmd == rh56.CMD_READ:
            return self._reply(hand_id, cmd, addr, self.regs[addr:addr + payload[0]])
        if cmd == rh56.CMD_WRITE:
            self.regs[addr:addr + len(payload)] = payload
            if addr == rh56.CLEAR_ERROR:
                self.regs[rh56.ERROR:rh56.ERROR + 6] = bytes(6)
                self.regs[rh56.CLEAR_ERROR] = 0
            if addr <= rh56.ANGLE_SET < addr + len(payload):
                for i, v in enumerate(self._get(rh56.ANGLE_SET)):
                    if v >= 0:
                        self.target[i] = float(v)
            self._update()
            return self._reply(hand_id, cmd, addr, b"\x01")
        return None

    @staticmethod
    def _reply(hand_id, cmd, addr, data):
        body = bytes([hand_id, len(data) + 3, cmd, addr & 0xFF, addr >> 8]) + bytes(data)
        return b"\x90\xEB" + body + bytes([rh56.checksum(body)])

    def _step(self):
        now = time.monotonic()
        dt, self._last = now - self._last, now
        speeds = self._get(rh56.SPEED_SET)
        for i in range(6):
            max_move = 1250.0 * speeds[i] / 1000.0 * dt  # speed 1000 = full stroke in 0.8 s
            delta = self.target[i] - self.angle[i]
            self.angle[i] += max(-max_move, min(max_move, delta))
        self._update()

    def _update(self):
        angles = [int(round(a)) for a in self.angle]
        self._put(rh56.ANGLE_ACT, angles)
        status = []
        for a, t in zip(self.angle, self.target):
            if abs(a - t) < 1:
                status.append(2)
            else:
                status.append(1 if t < a else 0)
        self.regs[rh56.STATUS:rh56.STATUS + 6] = bytes(status)
