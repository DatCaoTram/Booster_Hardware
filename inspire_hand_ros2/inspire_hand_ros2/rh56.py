"""Inspire RH56 dexterous hand (RH56DFX etc.) over RS485 -- no ROS dependency.

Protocol from the RH56 user manual V1.0.9, section 2.2 (115200 8N1 by default):

    request   EB 90 <id> <len> <cmd> <addr_lo> <addr_hi> <payload...> <checksum>
    response  90 EB <id> <len> <cmd> <addr_lo> <addr_hi> <payload...> <checksum>

len = payload length + 3, checksum = low byte of the sum of every byte after the
2-byte header. cmd 0x11 reads (payload = number of bytes to read), 0x12 writes
(payload = data; the hand acks with a single 0x01 byte).

DOF order everywhere: little, ring, middle, index, thumb bend, thumb rotation.
Angles are 0..1000 with 1000 = open and 0 = closed; -1 means "don't move".
"""
import struct
import time

CMD_READ = 0x11
CMD_WRITE = 0x12

# Register addresses (manual section 2.4). "6 x int16" groups are little-endian.
HAND_ID = 1000            # u8, 1..254
REDU_RATIO = 1002         # u8, 0=115200 1=57600 2=19200
CLEAR_ERROR = 1004        # u8, write 1
SAVE = 1005               # u8, write 1 to store parameters in flash
RESET_PARA = 1006         # u8, write 1 to restore factory defaults
FORCE_CALIBRATE = 1009    # u8, write 1 (hand must be unloaded, takes ~6 s)
CURRENT_LIMIT = 1020      # 6 x int16, mA
DEFAULT_SPEED_SET = 1032  # 6 x int16
DEFAULT_FORCE_SET = 1044  # 6 x int16, g
ANGLE_SET = 1486          # 6 x int16, 0..1000 or -1
FORCE_SET = 1498          # 6 x int16, g
SPEED_SET = 1522          # 6 x int16, 0..1000 (1000 = full stroke in ~0.8 s)
ANGLE_ACT = 1546          # 6 x int16, read only
FORCE_ACT = 1582          # 6 x int16, g, read only
CURRENT = 1594            # 6 x int16, mA, read only
ERROR = 1606              # 6 x u8 bit flags, read only
STATUS = 1612             # 6 x u8, read only
TEMP = 1618               # 6 x u8, deg C, read only

DOF_NAMES = ("little", "ring", "middle", "index", "thumb_bend", "thumb_rotate")
HOLD = -1
OPEN = [1000] * 6
FORCE_MAX = (1000, 1000, 1000, 1000, 1500, 1500)  # thumb axes accept up to 1500 g

ERROR_BITS = ("locked rotor", "over-temperature", "over-current", "motor fault", "comm error")
STATUS_TEXT = {
    0: "opening",
    1: "closing",
    2: "at target",
    3: "at force limit",
    5: "current protection",
    6: "stalled",
    7: "actuator fault",
}


class ProtocolError(Exception):
    """The hand didn't answer, or answered with a malformed frame."""


def checksum(body):
    return sum(body) & 0xFF


def build_frame(hand_id, cmd, addr, payload):
    body = bytes([hand_id, len(payload) + 3, cmd, addr & 0xFF, addr >> 8]) + bytes(payload)
    return b"\xEB\x90" + body + bytes([checksum(body)])


def parse_reply(frame, hand_id, cmd, addr, data_len):
    """Validate a response frame and return its payload."""
    if frame[:2] != b"\x90\xEB":
        raise ProtocolError("bad header %s" % frame[:2].hex())
    if frame[-1] != checksum(frame[2:-1]):
        raise ProtocolError("bad checksum")
    got_id, length, got_cmd = frame[2], frame[3], frame[4]
    got_addr = frame[5] | frame[6] << 8
    if (got_id, length, got_cmd, got_addr) != (hand_id, data_len + 3, cmd, addr):
        raise ProtocolError(
            "unexpected reply id=%d len=%d cmd=0x%02x addr=%d" % (got_id, length, got_cmd, got_addr))
    return bytes(frame[7:-1])


def clamp_angles(values):
    return [HOLD if v is None or v < 0 else min(int(v), 1000) for v in values]


def clamp_forces(values):
    return [max(0, min(int(v), hi)) for v, hi in zip(values, FORCE_MAX)]


def clamp_speeds(values):
    return [max(0, min(int(v), 1000)) for v in values]


def error_text(code):
    return ", ".join(name for bit, name in enumerate(ERROR_BITS) if code >> bit & 1)


def summarize(status, error):
    """(level, message) for a hand: level 0 = OK, 1 = warning, 2 = error."""
    problems = []
    level = 0
    for name, st, err in zip(DOF_NAMES, status, error):
        if err:
            problems.append("%s: %s" % (name, error_text(err)))
            level = 2
        elif st in (5, 6, 7):
            problems.append("%s: %s" % (name, STATUS_TEXT[st]))
            level = max(level, 1)
    return level, "; ".join(problems) or "OK"


class RH56:
    """One hand on a serial port. `port` is anything with write/read/reset_input_buffer."""

    def __init__(self, port, hand_id=1, timeout=0.1):
        self.port = port
        self.hand_id = hand_id
        self.timeout = timeout

    @classmethod
    def open(cls, device, hand_id=1, baudrate=115200, timeout=0.1):
        import serial  # pyserial; imported lazily so the protocol code has no dependencies

        return cls(serial.Serial(device, baudrate, timeout=0.02), hand_id, timeout)

    def close(self):
        self.port.close()

    # -- raw register access --------------------------------------------------

    def read(self, addr, n):
        return self._transact(CMD_READ, addr, bytes([n]), n)

    def write(self, addr, data):
        if self._transact(CMD_WRITE, addr, data, 1) != b"\x01":
            raise ProtocolError("write to %d not acknowledged" % addr)

    def read_shorts(self, addr, count=6):
        return list(struct.unpack("<%dh" % count, self.read(addr, 2 * count)))

    def write_shorts(self, addr, values):
        self.write(addr, struct.pack("<%dh" % len(values), *values))

    def _transact(self, cmd, addr, payload, data_len):
        self.port.reset_input_buffer()
        self.port.write(build_frame(self.hand_id, cmd, addr, payload))
        need = data_len + 8
        buf = b""
        deadline = time.monotonic() + self.timeout
        while True:
            start = buf.find(b"\x90\xEB")
            buf = buf[start:] if start >= 0 else buf[-1:]  # drop junk before the header
            if len(buf) >= need:
                return parse_reply(buf[:need], self.hand_id, cmd, addr, data_len)
            if time.monotonic() > deadline:
                raise ProtocolError("no reply from hand id %d (got %d of %d bytes)"
                                    % (self.hand_id, len(buf), need))
            buf += self.port.read(need - len(buf))

    # -- convenience ------------------------------------------------------------

    def set_angles(self, angles):
        self.write_shorts(ANGLE_SET, clamp_angles(angles))

    def set_speeds(self, speeds):
        self.write_shorts(SPEED_SET, clamp_speeds(speeds))

    def set_forces(self, forces):
        self.write_shorts(FORCE_SET, clamp_forces(forces))

    def clear_errors(self):
        self.write(CLEAR_ERROR, b"\x01")

    def read_angles(self):
        return self.read_shorts(ANGLE_ACT)

    def read_feedback(self):
        """Force (g), current (mA), error flags, status codes and temperature (C) in one read."""
        raw = self.read(FORCE_ACT, 42)  # FORCE_ACT..TEMP are contiguous
        return {
            "force": list(struct.unpack("<6h", raw[0:12])),
            "current": list(struct.unpack("<6h", raw[12:24])),
            "error": list(raw[24:30]),
            "status": list(raw[30:36]),
            "temp": list(raw[36:42]),
        }
