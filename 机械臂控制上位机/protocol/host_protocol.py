"""PC <-> F407 host-link framing used on the DAP-Link virtual COM port."""
import struct

SOF = b"\xA5\x5A"
CMD_TARGET, CMD_ENABLE, CMD_ZERO, CMD_STOP, CMD_STATUS, CMD_CLOG, CMD_VELOCITY, CMD_SERVO = 1, 2, 3, 4, 5, 6, 7, 8
RESP_ACK, RESP_STATUS = 0x80, 0x81

def pack(kind: int, payload: bytes = b"") -> bytes:
    body = bytes((kind, len(payload))) + payload
    checksum = 0
    for value in body:
        checksum ^= value
    return SOF + body + bytes((checksum,))

def target(axis: int, angle_tenths: int, speed_rpm: int, accel: int) -> bytes:
    return pack(CMD_TARGET, struct.pack(">BhHB", axis, angle_tenths, speed_rpm, accel))

def enable(axis: int, state: bool) -> bytes:
    return pack(CMD_ENABLE, bytes((axis, int(state))))

def zero(axis: int) -> bytes:
    return pack(CMD_ZERO, bytes((axis,)))

def stop(axis: int) -> bytes:
    return pack(CMD_STOP, bytes((axis,)))

def status_request() -> bytes:
    return pack(CMD_STATUS)

def reset_clog(axis: int) -> bytes:
    return pack(CMD_CLOG, bytes((axis,)))

def velocity(axis: int, direction: int, speed_rpm: int, accel: int) -> bytes:
    speed = min(max(speed_rpm, 0), 5000)
    return pack(CMD_VELOCITY, bytes((axis, direction & 0x01, (speed >> 8) & 0xFF, speed & 0xFF, accel & 0xFF)))

def servo(angle: int) -> bytes:
    return pack(CMD_SERVO, bytes((min(max(int(angle), 0), 180),)))

class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data: bytes):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(SOF)
            if start < 0:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 5:
                break
            length = self.buffer[3]
            total = length + 5
            if length > 58:
                del self.buffer[:2]
                continue
            if len(self.buffer) < total:
                break
            raw = bytes(self.buffer[:total])
            del self.buffer[:total]
            checksum = 0
            for value in raw[2:-1]:
                checksum ^= value
            if checksum == raw[-1]:
                frames.append((raw[2], raw[4:-1]))
        return frames

def decode_status(payload: bytes):
    if len(payload) != 40:
        return None
    masks = payload[:4]
    axes = []
    offset = 4
    for axis in range(4):
        angle, rpm, flags, errors = struct.unpack_from(">hhBI", payload, offset)
        offset += 9
        axes.append((angle, rpm, flags, errors))
    return masks, axes
