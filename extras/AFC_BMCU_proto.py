# AFCProject Automated Filament Changer Software
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Wire format of the BMCU-Klipper firmware (https://github.com/jarczakpawel/BMCU-Klipper),
# protocol version 1, firmware 1.0.4.

"""
BMCU wire codec: COBS framing with CRC32, packet header, and the payloads
AFC_BMCU sends and parses. Pure Python, no Klipper imports.
"""

from __future__ import annotations

import binascii
import math
import struct
from typing import Any, Dict, Iterable, List, Sequence, Tuple

PROTO_VERSION = 1

# Host requests
MSG_HELLO = 0x01
MSG_PING = 0x02
MSG_GET_STATUS = 0x03
MSG_GET_CAPS = 0x04
MSG_GET_OP_RESULT = 0x06
MSG_SESSION_CONFIRM = 0x08
MSG_SET_MOTION = 0x10
MSG_ABORT_OP = 0x16
MSG_FEED_TO_CONTACT = 0x18
MSG_FEED_DISTANCE = 0x19
MSG_MARK_UNLOADED = 0x1A
MSG_MARK_LOADED = 0x1B
MSG_CAL_AUTO_START = 0x1C
MSG_SET_SLOTS = 0x22
MSG_TAIL_RETRACT = 0x25
MSG_RUNTIME_SYNC = 0x34
MSG_STOP_ALL = 0x50
MSG_UPDATE_CANCEL = 0x62

# Firmware replies: every one has REPLY_BIT set.
REPLY_BIT = 0x80
MSG_STATUS = 0x90
MSG_OP_RESULT = 0x95
MSG_ERROR = 0xA4
MSG_ACK = 0xB0

MOTION_IDLE = 0
MOTION_SEND_OUT = 1
MOTION_BEFORE_ON_USE = 2
MOTION_ON_USE = 3
MOTION_BEFORE_PULL_BACK = 4
MOTION_PULL_BACK = 5
MOTION_STOP_ON_USE = 6
MOTION_NAMES = {
    MOTION_IDLE: "IDLE", MOTION_SEND_OUT: "SEND_OUT", MOTION_BEFORE_ON_USE: "BEFORE_ON_USE",
    MOTION_ON_USE: "ON_USE", MOTION_BEFORE_PULL_BACK: "BEFORE_PULL_BACK",
    MOTION_PULL_BACK: "PULL_BACK", MOTION_STOP_ON_USE: "STOP_ON_USE",
}

# Operation types run 1..OP_TAIL_RETRACT; 0 means none.
OP_TAIL_RETRACT = 7

OP_STATE_IDLE = 0
OP_STATE_RUNNING = 1
OP_STATE_DONE = 2
OP_STATE_FAILED = 3
OP_STATE_ABORTED = 4

ROUTE_EMPTY = 0
ROUTE_LOADED = 1
ROUTE_UNCERTAIN = 2
ROUTE_NAMES = {ROUTE_EMPTY: "EMPTY", ROUTE_LOADED: "LOADED", ROUTE_UNCERTAIN: "UNCERTAIN"}

# Slot colors and names live in RAM only: a reboot clears them.
CAP_VOLATILE_SLOTS = 1 << 9
CAP_TAIL_RETRACT = 1 << 25

# The firmware lost the session (it restarted, or the host was silent too long).
ERR_SESSION = 96

OP_REASON = {
    0: "none", 1: "target", 2: "timeout", 3: "no_filament",
    4: "encoder_io", 5: "buffer_limit", 6: "aborted", 7: "busy",
    8: "not_calibrated", 9: "motion_stopped", 10: "contact",
    11: "distance_limit", 12: "calibration_range", 13: "nvm_write",
    14: "sensor",
}

# Keys pack_runtime_sync() needs, in wire order. Distances are in mm.
RUNTIME_SYNC_KEYS: Tuple[str, ...] = (
    "load_pressure_pct", "load_speed", "pull_speed", "pull_speed_end",
    "jam_timeout_ms", "before_pullback_pct", "retract_mm", "autoload_mm",
)

_MOTION_STATES = frozenset(range(MOTION_IDLE, MOTION_STOP_ON_USE + 1))
_OP_TYPES = frozenset(range(0, OP_TAIL_RETRACT + 1))
_OP_STATES = frozenset(range(OP_STATE_IDLE, OP_STATE_ABORTED + 1))
_CHANNELS_OR_NONE = frozenset((0, 1, 2, 3, 0xFF))

_HEADER = struct.Struct("<BBHII")
_CRC = struct.Struct("<I")
_STATUS = struct.Struct("<B4B4B4B4BB4BBBH4f4f4hIII4B6B")
_HELLO_ACK = struct.Struct("<12sBBBBIIBB")
_CAPS = struct.Struct("<BBBBBBBBI12s16s")
_OP_RESULT = struct.Struct("<IBBBBffI")
_ACK = struct.Struct("<IBH")
_ERROR = struct.Struct("<BHI")
_FEED = struct.Struct("<BfBI")
_TAIL_RETRACT = struct.Struct("<BfI")
_RUNTIME_SYNC = struct.Struct("<14f3B4B")
# channel, RGBA, min and max temperature, name, Bambu filament id
_SLOT = struct.Struct("<B4Bhh20s8s")
# The firmware keeps the last name byte as its terminator.
SLOT_NAME_BYTES = 19
_BLANK_UIDS = (b"\x00" * 12, b"\xff" * 12)


def crc32(data: bytes) -> int:
    """
    CRC32 as the firmware computes it.

    :param data: bytes to checksum
    :return int: unsigned 32-bit CRC
    """
    return binascii.crc32(data) & 0xFFFFFFFF


def cobs_encode(data: bytes) -> bytes:
    """
    COBS-encode data so the result contains no zero byte.

    :param data: raw bytes
    :return bytes: encoded bytes, without delimiters
    """
    out = bytearray([0])
    code_index = 0
    code = 1
    for value in bytearray(data):
        if value == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
            continue
        out.append(value)
        code += 1
        if code == 0xFF:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """
    Decode one COBS frame (delimiters already stripped).

    :param data: encoded bytes
    :return bytes: decoded bytes
    """
    src = bytearray(data)
    out = bytearray()
    index = 0
    while index < len(src):
        code = src[index]
        if code == 0:
            error_str = "zero byte in COBS frame"
            raise ValueError(error_str)
        index += 1
        end = index + code - 1
        if end > len(src):
            error_str = "truncated COBS frame"
            raise ValueError(error_str)
        out.extend(src[index:end])
        index = end
        if (code != 0xFF
                and index < len(src)):
            out.append(0)
    return bytes(out)


class ProtocolVersionError(ValueError):
    """A frame carried a protocol version this host does not speak."""

    def __init__(self, version: int) -> None:
        """
        Record the version the frame carried.

        :param version: protocol version from the frame header
        """
        self.version = int(version)
        message = f"unsupported BMCU protocol version {self.version}"
        ValueError.__init__(self, message)


def encode_packet(msg_type: int, seq: int, cmd_id: int, payload: bytes = b"") -> bytes:
    """
    Build one framed packet: header, payload, CRC, COBS, zero delimiters.

    :param msg_type: message id
    :param seq: wire sequence number
    :param cmd_id: command id the reply echoes
    :param payload: message payload
    :return bytes: bytes ready to write
    """
    payload = bytes(payload)
    body = _HEADER.pack(PROTO_VERSION, msg_type, len(payload), seq, cmd_id) + payload
    return b"\x00" + cobs_encode(body + _CRC.pack(crc32(body))) + b"\x00"


def decode_packet(decoded: bytes) -> Tuple[int, int, int, bytes]:
    """
    Check and split one COBS-decoded packet.

    :param decoded: COBS-decoded frame
    :return Tuple[int, int, int, bytes]: msg_type, seq, cmd_id, payload
    """
    if len(decoded) < _HEADER.size + _CRC.size:
        error_str = "short packet"
        raise ValueError(error_str)
    body = decoded[:-_CRC.size]
    stored_crc = _CRC.unpack_from(decoded, len(decoded) - _CRC.size)[0]
    if crc32(body) != stored_crc:
        error_str = "CRC mismatch"
        raise ValueError(error_str)
    version, msg_type, length, seq, cmd_id = _HEADER.unpack_from(body)
    if version != PROTO_VERSION:
        raise ProtocolVersionError(version)
    payload = bytes(body[_HEADER.size:])
    if len(payload) != length:
        error_str = "payload length mismatch"
        raise ValueError(error_str)
    return msg_type, seq, cmd_id, payload


class StreamDecoder:
    """Splits a byte stream into decoded packets, dropping bad frames."""

    def __init__(self, max_frame: int = 256) -> None:
        """
        Start with an empty frame buffer.

        :param max_frame: longest encoded frame kept before it is discarded
        """
        self._buffer = bytearray()
        self._discarding = False
        self.max_frame = max_frame
        self.errors = 0
        self.incompatible_version: Any = None

    @property
    def pending(self) -> bool:
        """
        Whether bytes of an unterminated frame have been received.

        :return bool: True mid-frame
        """
        return bool(self._buffer) or self._discarding

    def feed(self, data: bytes) -> List[Tuple[int, int, int, bytes]]:
        """
        Consume received bytes.

        :param data: bytes read from the port
        :return List[Tuple[int, int, int, bytes]]: every packet completed by data
        """
        packets = []
        for value in bytearray(data):
            if value == 0:
                if self._discarding:
                    self._discarding = False
                elif self._buffer:
                    self._finish_frame(bytes(self._buffer), packets)
                self._buffer.clear()
                continue
            if self._discarding:
                continue
            if len(self._buffer) >= self.max_frame:
                self._buffer.clear()
                self._discarding = True
                self.errors += 1
                continue
            self._buffer.append(value)
        return packets

    def _finish_frame(self, frame: bytes, packets: List[Tuple[int, int, int, bytes]]) -> None:
        """
        Decode one complete frame into packets, counting it as an error if bad.

        :param frame: encoded frame without delimiters
        :param packets: list the decoded packet is appended to
        """
        try:
            packets.append(decode_packet(cobs_decode(frame)))
        except ProtocolVersionError as exc:
            self.incompatible_version = exc.version
            self.errors += 1
        except (ValueError, struct.error):
            self.errors += 1


def _require_finite(values: Iterable[float], label: str) -> None:
    """
    Raise ValueError unless every value is finite.

    :param values: numbers to check
    :param label: what the numbers are, for the message
    """
    if not all(math.isfinite(float(value)) for value in values):
        error_str = f"{label} contains a non-finite number"
        raise ValueError(error_str)


def _require_int(value: Any, label: str, minimum: int, maximum: int) -> int:
    """
    Check an integer field against its wire range.

    :param value: value to check
    :param label: field name for the message
    :param minimum: smallest allowed value
    :param maximum: largest allowed value
    :return int: the value
    """
    if (isinstance(value, bool)
            or not isinstance(value, int)):
        error_str = f"{label} must be an integer"
        raise ValueError(error_str)
    if (value < minimum
            or value > maximum):
        error_str = f"{label} must be within {minimum}..{maximum}"
        raise ValueError(error_str)
    return value


def _require_float(value: float, label: str, minimum: float, maximum: float) -> float:
    """
    Check a float field is finite and within its wire range.

    :param value: value to check
    :param label: field name for the message
    :param minimum: smallest allowed value
    :param maximum: largest allowed value
    :return float: the value as a float
    """
    value = float(value)
    if (not math.isfinite(value)
            or value < minimum
            or value > maximum):
        error_str = f"{label} must be within {minimum:g}..{maximum:g}"
        raise ValueError(error_str)
    return value


def parse_hello(payload: bytes) -> Dict[str, Any]:
    """
    Parse a HELLO_ACK payload.

    :param payload: HELLO_ACK payload
    :return Dict[str, Any]: firmware, protocol, session_id, channels
    """
    if len(payload) != _HELLO_ACK.size:
        error_str = "invalid hello length"
        raise ValueError(error_str)
    (uid, major, minor, patch, protocol, _caps, session, channels,
     _load_pressure_pct) = _HELLO_ACK.unpack_from(payload)
    if uid in _BLANK_UIDS:
        error_str = "invalid hardware UID"
        raise ValueError(error_str)
    if session == 0:
        error_str = "invalid zero session id"
        raise ValueError(error_str)
    return {
        "firmware": f"{major}.{minor}.{patch}",
        "protocol": protocol,
        "session_id": session,
        "channels": channels,
    }


def parse_caps(payload: bytes) -> int:
    """
    Parse a CAPS payload.

    :param payload: CAPS payload
    :return int: the capability mask (CAP_* bits)
    """
    if len(payload) != _CAPS.size:
        error_str = "invalid caps length"
        raise ValueError(error_str)
    values = _CAPS.unpack_from(payload)
    if values[9] in _BLANK_UIDS:
        error_str = "invalid capabilities hardware UID"
        raise ValueError(error_str)
    return values[8]


def _check_status_fields(v: Sequence[Any]) -> None:
    """
    Raise ValueError for any STATUS field outside its wire range.

    :param v: unpacked STATUS tuple
    """
    route, motion, present, buffer_pct = v[1:5], v[5:9], v[9:13], v[13:17]
    cal_mask, capture = v[17], v[18:22]
    encoder_mask, connected_mask = v[22], v[23]
    op_type, op_channel, op_state = v[40], v[41], v[42]
    auto_stage, auto_progress, auto_mask, auto_done, auto_state = v[44:49]
    checks = (
        (any(value not in ROUTE_NAMES for value in route),
         "status contains an invalid per-channel route state"),
        (v[0] not in _CHANNELS_OR_NONE, f"invalid active channel {v[0]}"),
        (any(value not in _MOTION_STATES for value in motion),
         "status contains an invalid motion state"),
        (any(value not in (0, 1) for value in present),
         "status contains an invalid filament-present flag"),
        (any(value > 100 for value in buffer_pct),
         "status contains an invalid buffer percentage"),
        (bool((cal_mask | encoder_mask | connected_mask) & ~0x0F),
         "status contains an invalid channel mask"),
        (any(value & ~0x07 for value in capture),
         "status contains an invalid calibration capture mask"),
        (op_type not in _OP_TYPES, "status contains an invalid operation type"),
        (op_channel not in _CHANNELS_OR_NONE, "status contains an invalid operation channel"),
        (op_state not in _OP_STATES, "status contains an invalid operation state"),
        (auto_stage > 6, "status contains an invalid automatic calibration stage"),
        (auto_progress > 100, "status contains invalid automatic calibration progress"),
        (bool((auto_mask | auto_done) & ~0x0F),
         "status contains invalid automatic calibration progress"),
        (auto_state not in _OP_STATES,
         "status contains an invalid automatic calibration state"),
    )
    for failed, message in checks:
        if failed:
            raise ValueError(message)


def parse_status(payload: bytes) -> Dict[str, Any]:
    """
    Parse and validate a STATUS payload.

    :param payload: STATUS payload
    :return Dict[str, Any]: per-channel lists, the masks and fault flags AFC_BMCU
        reads, the session and the active operation's type and state
    """
    if len(payload) != _STATUS.size:
        error_str = f"invalid status payload length {len(payload)}"
        raise ValueError(error_str)
    v = _STATUS.unpack(payload)
    meters, buffer_raw = list(v[25:29]), list(v[29:33])
    _require_finite(meters + buffer_raw, "status")
    _check_status_fields(v)
    error_flags = v[24]
    return {
        "route_state_name": [ROUTE_NAMES[value] for value in v[1:5]],
        "motion": list(v[5:9]),
        "present": list(v[9:13]),
        "buffer_pct": list(v[13:17]),
        "calibration_valid_mask": v[17],
        "encoder_io_mask": v[22],
        # Bit 1 is route-uncertain, set by every feed and pull-back: not a fault.
        "controller_fault_flags": int(error_flags & ~0x0002),
        "nvm_fault": bool(error_flags & 0x0001),
        "meters": meters,
        "session_id": v[37],
        "active_op_type": v[40],
        "active_op_state": v[42],
    }


def parse_op_result(payload: bytes) -> Dict[str, Any]:
    """
    Parse an OP_RESULT payload.

    :param payload: OP_RESULT payload
    :return Dict[str, Any]: op_id, state, reason name, measured_mm and ok
    """
    if len(payload) != _OP_RESULT.size:
        error_str = "invalid operation result length"
        raise ValueError(error_str)
    (op_id, op_type, channel, state, reason, target_mm, measured_mm,
     _duration_ms) = _OP_RESULT.unpack_from(payload)
    _require_finite((target_mm, measured_mm), "operation result")
    if op_type not in _OP_TYPES:
        error_str = "invalid operation result type"
        raise ValueError(error_str)
    if channel not in _CHANNELS_OR_NONE:
        error_str = "invalid operation result channel"
        raise ValueError(error_str)
    if state not in _OP_STATES:
        error_str = "invalid operation result state"
        raise ValueError(error_str)
    return {
        "op_id": op_id,
        "state": state,
        "reason": OP_REASON.get(reason, f"reason_{reason}"),
        "measured_mm": measured_mm,
        "ok": state == OP_STATE_DONE,
    }


def parse_ack(payload: bytes) -> Tuple[int, bool, int]:
    """
    Parse an ACK payload.

    :param payload: ACK payload
    :return Tuple[int, bool, int]: echoed cmd_id, accepted, firmware error code
    """
    if len(payload) != _ACK.size:
        error_str = "invalid ACK length"
        raise ValueError(error_str)
    echo, ok, error = _ACK.unpack(payload)
    return echo, bool(ok), error


def parse_error(payload: bytes) -> Tuple[int, int, int]:
    """
    Parse an ERROR payload.

    :param payload: ERROR payload
    :return Tuple[int, int, int]: channel (0xFF for none), error code, details
    """
    if len(payload) != _ERROR.size:
        error_str = "invalid ERROR length"
        raise ValueError(error_str)
    channel, code, details = _ERROR.unpack(payload)
    return channel, code, details


def pack_runtime_sync(values: Dict[str, float], rgb: Tuple[int, int, int]) -> bytes:
    """
    Build a RUNTIME_SYNC payload with standalone mode off, so the firmware
    moves no motor on its own (no insert autoload, no pull-to-unload).

    :param values: every key in RUNTIME_SYNC_KEYS; retract_mm and autoload_mm
        apply to all four channels
    :param rgb: system LED colour, three values 0-255
    :return bytes: the 63-byte payload
    """
    for key in RUNTIME_SYNC_KEYS:
        if key not in values:
            error_str = f"runtime sync is missing {key}"
            raise ValueError(error_str)
    numbers = [float(values[key]) for key in RUNTIME_SYNC_KEYS]
    _require_finite(numbers, "runtime sync")
    retract_mm = _require_float(numbers[6], "channel unload distance", 10.0, 2000.0)
    autoload_mm = _require_float(numbers[7], "channel autoload distance", 10.0, 1000.0)
    if (not isinstance(rgb, (tuple, list))
            or len(rgb) != 3):
        error_str = "runtime RGB must contain exactly three values"
        raise ValueError(error_str)
    colour = [_require_int(value, "runtime RGB value", 0, 255) for value in rgb]
    retract = [retract_mm / 1000.0] * 4
    autoload = [autoload_mm / 1000.0] * 4
    # standalone, autonomous_assist, autonomous_unload all 0; policy version 1
    policy = [0, 0, 0, 1]
    return _RUNTIME_SYNC.pack(*(numbers[:6] + retract + autoload + colour + policy))


def pack_feed(channel: int, millimeters: float, pct: int, timeout_ms: int) -> bytes:
    """
    Build a FEED_TO_CONTACT or FEED_DISTANCE payload.

    :param channel: channel 0-3
    :param millimeters: distance limit, 5-5000 mm
    :param pct: buffer percentage that ends (contact) or bounds (distance) the feed
    :param timeout_ms: operation timeout in ms
    :return bytes: the 10-byte payload
    """
    channel = _require_int(channel, "channel", 0, 3)
    millimeters = _require_float(millimeters, "distance", 5.0, 5000.0)
    pct = _require_int(pct, "contact percentage", 55, 98)
    timeout_ms = _require_int(timeout_ms, "operation timeout ms", 250, 300000)
    return _FEED.pack(channel, millimeters, pct, timeout_ms)


def pack_tail_retract(channel: int, millimeters: float, timeout_ms: int) -> bytes:
    """
    Build a TAIL_RETRACT payload.

    :param channel: channel 0-3
    :param millimeters: distance to retract by the encoder, 5-2000 mm
    :param timeout_ms: operation timeout in ms
    :return bytes: the 9-byte payload
    """
    channel = _require_int(channel, "channel", 0, 3)
    millimeters = _require_float(millimeters, "distance", 5.0, 2000.0)
    timeout_ms = _require_int(timeout_ms, "operation timeout ms", 250, 300000)
    return _TAIL_RETRACT.pack(channel, millimeters, timeout_ms)


def pack_set_motion(channel: int, motion: int) -> bytes:
    """
    Build a SET_MOTION payload.

    :param channel: channel 0-3
    :param motion: one of the MOTION_* states
    :return bytes: the 2-byte payload
    """
    channel = _require_int(channel, "channel", 0, 3)
    motion = _require_int(motion, "motion", MOTION_IDLE, MOTION_STOP_ON_USE)
    return bytes((channel, motion))


def pack_channel(channel: int) -> bytes:
    """
    Build a one-channel payload (MARK_LOADED, MARK_UNLOADED).

    :param channel: channel 0-3
    :return bytes: the 1-byte payload
    """
    return bytes((_require_int(channel, "channel", 0, 3),))


def pack_cal_auto_start(channel: int) -> bytes:
    """
    Build a CAL_AUTO_START payload selecting one channel.

    :param channel: channel 0-3
    :return bytes: the 1-byte channel mask
    """
    return bytes((1 << _require_int(channel, "channel", 0, 3),))


def pack_slots(slots: Sequence[Tuple[int, int, int, str]]) -> bytes:
    """
    Build a SET_SLOTS payload: color and name for all four channels. The
    temperatures and the Bambu filament id are sent empty.

    :param slots: (red, green, blue, name) for channels 0-3 in order
    :return bytes: the 148-byte payload
    """
    if len(slots) != 4:
        error_str = "SET_SLOTS needs four slots"
        raise ValueError(error_str)
    payload = b""
    for channel, (red, green, blue, name) in enumerate(slots):
        rgb = [_require_int(v, "color", 0, 255) for v in (red, green, blue)]
        encoded = name.encode("ascii", "replace")[:SLOT_NAME_BYTES]
        payload += _SLOT.pack(channel, *rgb, 0xFF, 0, 0, encoded, b"")
    return payload


def pack_u32(value: int) -> bytes:
    """
    Build a little-endian uint32 payload (HELLO nonce, SESSION_CONFIRM, GET_OP_RESULT).

    :param value: value 0 to 0xFFFFFFFF
    :return bytes: the 4-byte payload
    """
    return struct.pack("<I", _require_int(value, "value", 0, 0xFFFFFFFF))
