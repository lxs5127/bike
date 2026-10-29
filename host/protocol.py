"""Version 1 Wi-Fi/UDP packets shared with main/link/bike_protocol.c."""
from dataclasses import dataclass
import struct
import zlib

MAGIC = b"BKR1"
HEADER = struct.Struct("<4sBBHI")
STATE = struct.Struct("<IQIIfffIIIII")
COMMAND = struct.Struct("<IQfI")
STATE_TYPE, COMMAND_TYPE = 1, 2
HELLO_TYPE = 3
VALID, DRIVE_READY, MOCK, MOTOR_DISABLED, COMMAND_VALID = 1, 2, 4, 8, 16
STOP = 1
LINK_FAULT = 64


@dataclass(frozen=True)
class State:
    sequence: int
    boot_id: int
    timestamp_us: int
    imu_age_us: int
    encoder_age_us: int
    roll_rad: float
    roll_rate_rad_s: float
    flywheel_rad_s: float
    fault: int
    flags: int
    acknowledged_sequence: int
    command_age_us: int
    round_trip_us: int


def packet(kind, sequence, payload):
    header = HEADER.pack(MAGIC, 1, kind, len(payload), sequence)
    body = header + payload
    return body + struct.pack("<I", zlib.crc32(body))


def encode_state(state):
    values = list(vars(state).values())[1:]
    return packet(STATE_TYPE, state.sequence, STATE.pack(*values))


def encode_command(state, torque_nm, stop=False):
    return packet(COMMAND_TYPE, state.sequence, COMMAND.pack(
        state.boot_id, state.timestamp_us, torque_nm, STOP if stop else 0))


def encode_hello():
    return packet(HELLO_TYPE, 0, b"")


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_packets = 0

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        while len(self.buffer) >= 4:
            start = self.buffer.find(MAGIC)
            if start < 0:
                del self.buffer[:-3]
                break
            del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                break
            _, version, kind, length, sequence = HEADER.unpack_from(self.buffer)
            expected = {STATE_TYPE: STATE.size, COMMAND_TYPE: COMMAND.size, HELLO_TYPE: 0}.get(kind)
            if version != 1 or length != expected:
                self.bad_packets += 1
                del self.buffer[0]
                continue
            size = HEADER.size + length + 4
            if len(self.buffer) < size:
                break
            body = self.buffer[:size - 4]
            crc, = struct.unpack_from("<I", self.buffer, size - 4)
            if zlib.crc32(body) != crc:
                self.bad_packets += 1
                del self.buffer[0]
                continue
            packets.append((kind, sequence, bytes(self.buffer[HEADER.size:size - 4])))
            del self.buffer[:size]
        return packets


def decode_state(sequence, payload):
    return State(sequence, *STATE.unpack(payload))


def decode_datagram(data, kind=STATE_TYPE):
    """UDP must carry exactly one complete packet, never concatenate fragments."""
    if len(data) < HEADER.size + 4:
        raise ValueError("Truncated UDP packet")
    magic, version, packet_kind, length, sequence = HEADER.unpack_from(data)
    expected = {STATE_TYPE: STATE.size, COMMAND_TYPE: COMMAND.size, HELLO_TYPE: 0}.get(kind)
    if (magic != MAGIC or version != 1 or packet_kind != kind or expected is None or
            length != expected or len(data) != HEADER.size + length + 4):
        raise ValueError("Invalid UDP packet header/length")
    crc, = struct.unpack_from("<I", data, len(data) - 4)
    if zlib.crc32(data[:-4]) != crc:
        raise ValueError("UDP CRC mismatch")
    payload = data[HEADER.size:-4]
    return decode_state(sequence, payload) if kind == STATE_TYPE else (sequence, payload)
