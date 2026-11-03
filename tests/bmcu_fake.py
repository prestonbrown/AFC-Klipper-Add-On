"""
In-process fake BMCU for AFC_BMCU tests. It speaks the real codec
(extras/AFC_BMCU_proto.py) and behaves the way BMCU-Klipper firmware
(firmware/src/bmcu_protocol.cpp, branch feature/tail-retract) does, with no
pty and no threads: every byte moves on a FakeReactor clock the test controls.
Wire values the firmware defines (motions, routes, op types, states, reasons,
error codes) are written out here as literals, so a wrong constant in the host
codec does not change both ends of the wire at once.

Builders
--------
``status_payload(**fields)``, ``hello_payload(...)``, ``caps_payload(...)``,
``op_result_payload(...)``, ``ack_payload(...)`` and ``error_payload(...)``
build firmware payloads with valid defaults; keyword arguments override one
field each.

FakeReactor
-----------
Klipper reactor stand-in on a controllable clock (``.now``, starts at 100.0).
``register_timer``/``update_timer``/``unregister_timer``, ``register_fd``/
``unregister_fd``, ``register_callback``, ``completion()``, ``pause()``,
``monotonic()``, ``NOW``/``NEVER``. A timer's waketime is NEVER while its
callback runs and becomes the callback's return value afterwards, as in
klippy. ``completion().wait(t)`` and ``pause(t)`` run every timer due before
t, in time order, moving the clock to each one, until the completion is done
or t is reached (the clock then sits at t). ``advance(seconds)`` does the same
for a test. ``fire_fd(fd)`` runs the read callbacks registered for fd.

FakeBMCU
--------
``bmcu = FakeBMCU(reactor)``, then ``monkeypatch.setitem(sys.modules, "serial",
bmcu.serial_module())`` so ``import serial`` inside the connection gets a
FakeSerial wired to this board. Every reply is delivered by a reactor timer
``reply_delay`` seconds (default 0) after the request is written.

State the test sets: ``bmcu.channels[i]`` (a FakeChannel: ``present``,
``calibrated``, ``route``, ``motion``, ``buffer_pct``, ``meters``,
``encoder_ok``, ``contact_mm``), ``capabilities``, ``protocol`` (HELLO field),
``header_version`` (every frame's header), ``firmware``, ``move_speed`` (mm/s
of a feed or tail retract), ``nvm_ok`` (False: every route save fails, as with
a faulted flash), ``update_mode``, ``echo`` (hand every written byte back, as
an RS485 adapter that does not gate its receiver does).

Observing the host: ``bmcu.received`` is every frame the host wrote, decoded
(Frame: time, msg_type, seq, cmd_id, payload, note), including dropped ones;
``bmcu.frames(*msg_types)`` filters it; ``bmcu.motion_frames()`` returns the
ones that can move a motor (MOTION_TYPES), for "zero motion frames" asserts.
``bmcu.sent`` is every frame the board sent. ``bmcu.open_times`` records each
open attempt. ``bmcu.runtime_sync`` holds the last accepted RUNTIME_SYNC
payload. ``bmcu.started`` lists the op ids that started running;
``running_ops()`` the one still running; ``result(op_id)`` the stored result.
``channel_count`` (default 4) is what HELLO reports.

Session: HELLO aborts a running operation without reporting it (its ABORTED
result goes in the ring), stops every motor, discards frames not yet sent and
drops RUNTIME_SYNC's motor enable; requests other than HELLO and
SESSION_CONFIRM get error 96 until SESSION_CONFIRM. After HOST_SILENCE_S with
no accepted request the board aborts the operation, stops every motor and
answers 96 again. ``reboot()`` starts a new session with no operation and an
empty result ring; ``reboot(announce=True)`` also sends the boot HELLO_ACK,
CAPS and STATUS with cmd_id 0.

Operations (FEED_TO_CONTACT, FEED_DISTANCE, CAL_AUTO_START, TAIL_RETRACT): a
refusal is a FAILED OP_RESULT alone, with no ACK, and is kept in the result
ring like any result. The ring holds the last four results and
survives HELLO; GET_OP_RESULT with op_id 0 returns the newest, an op id it
does not hold gets error 22, and a repeated op id replays (ACK, the stored
OP_RESULT, STATUS) instead of moving again. A feed needs filament at the inlet
and turns the route UNCERTAIN until a MARK; a tail retract needs RUNTIME_SYNC.
Feeds and tail retracts move ``move_speed`` mm/s once RUNTIME_SYNC has
enabled the motors, and the board ends them itself as the firmware's
process_distance_op does: no filament, encoder fault, buffer limits, contact
(the buffer compresses once a feed has moved ``contact_mm``), distance, and
its own timeout_ms; a tail retract that pulled its target is DONE only when
the EMPTY route saves, else FAILED nvm_write. ``complete_op(op_id, state,
reason, measured_mm)`` ends the running operation with an outcome the fake
does not model (``send=False`` loses the OP_RESULT, so only GET_OP_RESULT
finds it); ``auto_complete[msg_type] = (state, reason)`` ends that kind as
soon as it starts. ABORT_OP and STOP_ALL send the aborted OP_RESULT and a
STATUS before their ACK.

Unsolicited STATUS: on every motion, inlet or route change, every 0.5 s while
anything moves and every 30 s idle, only while the host session is up.
``set_present(ch, value)`` changes an inlet and sends the change now;
``send_status()`` sends one; ``send_raw(data, delay)`` puts arbitrary bytes
on the wire (partial frames, noise).

Faults: ``bmcu.add_fault(action, msg_type=None, count=1, seconds=0.0)`` with
action ``"drop"`` (no reply), ``"drop_ack"`` (the ACK lost, the frames after it
sent) or ``"delay"`` (reply ``seconds`` later); msg_type None matches any
request, count -1 never runs out.
``bmcu.silent = True`` sends nothing. ``bmcu.unplug()`` makes reads, writes
and opens fail (the host sees the fd go readable and the read raise);
``bmcu.replug()`` undoes it.
"""

from __future__ import annotations

import collections
import math
import struct
import types
from typing import (
    Any, Callable, Deque, Dict, List, NamedTuple, Optional, Sequence, Tuple,
)

from extras import AFC_BMCU_proto as proto

DEFAULT_UID = bytes(range(0x11, 0x1D))
DEFAULT_SESSION = 0x5E55
DEFAULT_FIRMWARE = (1, 0, 4)
# BMCU_CAP_TAIL_RETRACT and BMCU_CAP_VOLATILE_SLOTS in bmcu_protocol.h.
CAP_TAIL_RETRACT = 1 << 25
CAP_VOLATILE_SLOTS = 1 << 9

_STATUS = struct.Struct("<B4B4B4B4BB4BBBH4f4f4hIII4B6B")
_STATUS_DEFAULTS = {
    "now_channel": 0xFF,
    "route": (0,) * 4,
    "motion": (0,) * 4,
    "present": (0,) * 4,
    "buffer_pct": (50,) * 4,
    "cal_valid_mask": 0x0F,
    "capture_mask": (0,) * 4,
    "encoder_mask": 0x0F,
    "connected_mask": 0x0F,
    "error_flags": 0,
    "meters": (0.0,) * 4,
    "buffer_raw": (1.65,) * 4,
    "pwm": (0,) * 4,
    "session_id": DEFAULT_SESSION,
    "event_counter": 1,
    "active_op_id": 0,
    "op_type": 0,
    "op_channel": 0xFF,
    "op_state": 0,
    "op_reason": 0,
    "auto_stage": 0,
    "auto_progress": 0,
    "auto_mask": 0,
    "auto_done_mask": 0,
    "auto_state": 0,
    "auto_reason": 0,
}


def status_payload(**fields: Any) -> bytes:
    """
    Build a STATUS payload. Field names are the keys of _STATUS_DEFAULTS;
    per-channel fields take four values.

    :param fields: fields to override
    :return bytes: the STATUS payload
    """
    unknown = set(fields) - set(_STATUS_DEFAULTS)
    if unknown:
        raise TypeError(f"unknown status fields {sorted(unknown)}")
    values = dict(_STATUS_DEFAULTS)
    values.update(fields)
    flat = []
    for name in _STATUS_DEFAULTS:
        value = values[name]
        if isinstance(value, (list, tuple)):
            flat.extend(value)
        else:
            flat.append(value)
    return _STATUS.pack(*flat)


def hello_payload(uid: bytes = DEFAULT_UID, firmware: Sequence[int] = DEFAULT_FIRMWARE,
                  protocol: int = 1, capabilities: int = 0,
                  session_id: int = DEFAULT_SESSION, channels: int = 4,
                  load_pressure_pct: int = 82) -> bytes:
    """
    Build a HELLO_ACK payload.

    :return bytes: the payload
    """
    return struct.pack("<12sBBBBIIBB", uid, firmware[0], firmware[1], firmware[2],
                       protocol, capabilities, session_id, channels, load_pressure_pct)


def caps_payload(uid: bytes = DEFAULT_UID, firmware: Sequence[int] = DEFAULT_FIRMWARE,
                 protocol: int = 1, capabilities: int = 0,
                 name: bytes = b"BMCU") -> bytes:
    """
    Build a CAPS payload.

    :return bytes: the payload
    """
    return struct.pack("<BBBBBBBBI12s16s", 4, 0, 1, firmware[0], firmware[1],
                       firmware[2], protocol, 0, capabilities, uid, name)


def op_result_payload(op_id: int, op_type: int, channel: int, state: int,
                      reason: int = 1, target_mm: float = 100.0,
                      measured_mm: float = 100.0, duration_ms: int = 1000) -> bytes:
    """
    Build an OP_RESULT payload.

    :return bytes: the payload
    """
    return struct.pack("<IBBBBffI", op_id, op_type, channel, state, reason, target_mm,
                       measured_mm, duration_ms)


def ack_payload(cmd_id: int, ok: int = 1, error: int = 0) -> bytes:
    """
    Build an ACK payload.

    :return bytes: the payload
    """
    return struct.pack("<IBH", cmd_id, ok, error)


def error_payload(code: int, channel: int = 0xFF, details: int = 0) -> bytes:
    """
    Build an ERROR payload.

    :return bytes: the payload
    """
    return struct.pack("<BHI", channel, code, details)


class FakeLogger:
    """AFC logger stand-in: every call lands in ``messages`` as (level, text)."""

    def __init__(self) -> None:
        self.messages: List[Tuple[str, str]] = []

    def debug(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self.messages.append(("debug", msg))

    def info(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self.messages.append(("info", msg))

    def warning(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self.messages.append(("warning", msg))

    def error(self, msg: str, *args: Any, **kwargs: Any) -> None:
        self.messages.append(("error", msg))


# FakeReactor

class FakeTimer:
    """A registered timer: callback(eventtime) returns the next waketime."""

    def __init__(self, callback: Callable[[float], float], waketime: float, seq: int) -> None:
        self.callback = callback
        self.waketime = waketime
        self.seq = seq


class FakeCompletion:
    """ReactorCompletion stand-in whose wait() runs the fake reactor."""

    def __init__(self, reactor: "FakeReactor") -> None:
        self._reactor = reactor
        self.done = False
        self.result: Any = None

    def complete(self, result: Any) -> None:
        if not self.done:
            self.done = True
            self.result = result

    def test(self) -> bool:
        return self.done

    def wait(self, waketime: float = 9_999_999_999.0, waketime_result: Any = None) -> Any:
        self._reactor.run_until(lambda: self.done, waketime)
        return self.result if self.done else waketime_result


class FakeReactor:
    """See the module docstring."""

    NOW = 0.0
    NEVER = 9_999_999_999.0
    MAX_EVENTS = 100_000

    def __init__(self, start: float = 100.0) -> None:
        self.now = start
        self.timers: List[FakeTimer] = []
        self.fds: Dict[int, Tuple[int, Callable[[float], None]]] = {}
        self._seq = 0

    def monotonic(self) -> float:
        return self.now

    def register_timer(self, callback: Callable[[float], float],
                       waketime: float = 9_999_999_999.0) -> FakeTimer:
        self._seq += 1
        timer = FakeTimer(callback, waketime, self._seq)
        self.timers.append(timer)
        return timer

    def update_timer(self, timer: FakeTimer, waketime: float) -> None:
        timer.waketime = waketime

    def unregister_timer(self, timer: FakeTimer) -> None:
        if timer in self.timers:
            self.timers.remove(timer)

    def register_callback(self, callback: Callable[[float], Any],
                          waketime: float = 0.0) -> FakeCompletion:
        completion = FakeCompletion(self)

        def _once(eventtime: float) -> float:
            self.unregister_timer(timer)
            completion.complete(callback(eventtime))
            return self.NEVER

        timer = self.register_timer(_once, max(waketime, self.now))
        return completion

    def register_fd(self, fd: int, callback: Callable[[float], None]) -> int:
        self._seq += 1
        self.fds[self._seq] = (fd, callback)
        return self._seq

    def unregister_fd(self, handle: int) -> None:
        self.fds.pop(handle, None)

    def fire_fd(self, fd: int) -> None:
        for handle, (registered, callback) in list(self.fds.items()):
            if registered == fd and handle in self.fds:
                callback(self.now)

    def completion(self) -> FakeCompletion:
        return FakeCompletion(self)

    def mutex(self, is_locked: bool = False) -> Any:
        return types.SimpleNamespace(test=lambda: is_locked)

    def pause(self, waketime: float) -> float:
        self.run_until(lambda: False, waketime)
        return self.now

    def advance(self, seconds: float) -> None:
        self.run_until(lambda: False, self.now + seconds)

    def run_until(self, predicate: Callable[[], bool], deadline: float) -> bool:
        """Run due timers in time order until predicate() or deadline."""
        for _ in range(self.MAX_EVENTS):
            if predicate():
                return True
            due = [t for t in self.timers if t.waketime <= deadline]
            if not due:
                if deadline < self.NEVER:
                    self.now = max(self.now, deadline)
                return predicate()
            timer = min(due, key=lambda t: (t.waketime, t.seq))
            self.now = max(self.now, timer.waketime)
            timer.waketime = self.NEVER
            timer.waketime = timer.callback(self.now)
        raise RuntimeError("FakeReactor: too many events; a timer never lets the clock move")


# FakeSerial

class FakeSerialException(OSError):
    """serial.SerialException stand-in."""


class FakeSerial:
    """pyserial Serial stand-in wired to one FakeBMCU."""

    def __init__(self, bmcu: "FakeBMCU") -> None:
        self._bmcu = bmcu
        self.port: Optional[str] = None
        self.baudrate = 9600
        self.timeout: Optional[float] = None
        self.write_timeout: Optional[float] = None
        self.rtscts = False
        self.dsrdtr = False
        self.exclusive: Optional[bool] = None
        self.dtr = True
        self.rts = True
        self.is_open = False
        self.rx = bytearray()
        self.writes: List[bytes] = []

    def open(self) -> None:
        self._bmcu.open_times.append(self._bmcu.reactor.now)
        if self._bmcu.unplugged:
            raise FakeSerialException(f"could not open port {self.port}: No such file")
        self.is_open = True
        self._bmcu.attach(self)

    def fileno(self) -> int:
        return self._bmcu.fd

    def read(self, size: int = 1) -> bytes:
        if self._bmcu.unplugged:
            raise FakeSerialException(
                "device reports readiness to read but returned no data")
        data = bytes(self.rx[:size])
        del self.rx[:size]
        return data

    def write(self, data: bytes) -> int:
        if self._bmcu.unplugged:
            raise FakeSerialException("write failed: [Errno 5] Input/output error")
        self.writes.append(bytes(data))
        self._bmcu.receive(bytes(data))
        return len(data)

    def close(self) -> None:
        self.is_open = False
        self._bmcu.detach(self)


# FakeBMCU

class Frame(NamedTuple):
    """One decoded frame and when it crossed the wire."""
    time: float
    msg_type: int
    seq: int
    cmd_id: int
    payload: bytes
    note: str = ""


class Fault:
    """One armed fault, consumed by the next ``count`` matching requests."""

    def __init__(self, action: str, msg_type: Optional[int], count: int,
                 seconds: float) -> None:
        self.action = action
        self.msg_type = msg_type
        self.count = count
        self.seconds = seconds

    def matches(self, msg_type: int) -> bool:
        return (self.count != 0
                and (self.msg_type is None or self.msg_type == msg_type))


class FakeChannel:
    """Per-channel board state the test can set directly."""

    def __init__(self) -> None:
        self.present = 0
        self.calibrated = True
        self.encoder_ok = True
        self.route = ROUTE_EMPTY
        self.motion = MOTION_IDLE
        self.buffer_pct = 50
        self.meters = 0.0
        # mm a feed travels before the filament meets the extruder gears and the
        # buffer compresses; None for a path the filament never gets through.
        self.contact_mm: Optional[float] = None


# Wire values from firmware/src/bmcu_protocol.h and bmcu_protocol.cpp, written
# out so the fake does not share a wrong constant with the host.
MOTION_IDLE = 0
MOTION_SEND_OUT = 1
MOTION_BEFORE_ON_USE = 2
MOTION_ON_USE = 3
MOTION_BEFORE_PULL_BACK = 4
MOTION_PULL_BACK = 5
MOTION_STOP_ON_USE = 6
ROUTE_EMPTY = 0
ROUTE_LOADED = 1
ROUTE_UNCERTAIN = 2
OP_FEED_TO_CONTACT = 3
OP_FEED_DISTANCE = 4
OP_BUFFER_CALIBRATION = 5
OP_TAIL_RETRACT = 7
STATE_IDLE = 0
STATE_RUNNING = 1
STATE_DONE = 2
STATE_FAILED = 3
STATE_ABORTED = 4
REASON_NONE = 0
REASON_TARGET = 1
REASON_TIMEOUT = 2
REASON_NO_FILAMENT = 3
REASON_ENCODER_IO = 4
REASON_BUFFER_LIMIT = 5
REASON_ABORTED = 6
REASON_BUSY = 7
REASON_NOT_CALIBRATED = 8
REASON_MOTION_STOPPED = 9
REASON_CONTACT = 10
REASON_DISTANCE_LIMIT = 11
REASON_NVM = 13
ERR_SET_MOTION_REFUSED = 3
ERR_SET_MOTION_FAILED = 4
ERR_MARK_REFUSED = 5
ERR_OP_RESULT_UNAVAILABLE = 22
ERR_BAD_ARGUMENT = 31
ERR_RUNTIME_RANGE = 35
ERR_RUNTIME_POLICY = 36
ERR_NVM = 71
ERR_UPDATE_MODE = 90
ERR_SESSION = 96
ERR_LENGTH = 100
ERR_UNKNOWN = 0xFFFF
ERROR_FLAG_NVM = 0x0001
ERROR_FLAG_ROUTE_UNCERTAIN = 0x0002
# Replies the host does not name: it reads HELLO_ACK and CAPS by position in
# the handshake and never checks a PONG.
MSG_HELLO_ACK = 0x81
MSG_PONG = 0x82
MSG_CAPS = 0x91

# The firmware drops the session after this long without a valid request.
HOST_SILENCE_S = 15.0
# How often the fake's firmware loop runs while filament moves, and while idle.
LOOP_ACTIVE_S = 0.1
LOOP_IDLE_S = 1.0
# Unsolicited STATUS period while moving, and while idle.
STATUS_ACTIVE_S = 0.5
STATUS_IDLE_S = 30.0
# Long enough that a calibration never ends on its own in a test.
CALIBRATION_FOREVER_S = 1.0e9
# BMCU_LOAD_PRESSURE_MIN_PCT / _MAX_PCT in bmcu_config.h.
LOAD_PRESSURE_MIN = 75
LOAD_PRESSURE_MAX = 95

MOTION_TYPES = frozenset((
    proto.MSG_SET_MOTION, proto.MSG_FEED_TO_CONTACT, proto.MSG_FEED_DISTANCE,
    proto.MSG_TAIL_RETRACT, proto.MSG_CAL_AUTO_START,
))
# Fixed payload sizes from command_payload_size() in the firmware.
_PAYLOAD_SIZE = {
    proto.MSG_HELLO: 4, proto.MSG_SESSION_CONFIRM: 4, proto.MSG_GET_STATUS: 0,
    proto.MSG_GET_CAPS: 0, proto.MSG_GET_OP_RESULT: 4, proto.MSG_ABORT_OP: 0,
    proto.MSG_STOP_ALL: 0, proto.MSG_SET_MOTION: 2, proto.MSG_MARK_LOADED: 1,
    proto.MSG_MARK_UNLOADED: 1, proto.MSG_FEED_TO_CONTACT: 10, proto.MSG_FEED_DISTANCE: 10,
    proto.MSG_CAL_AUTO_START: 1, proto.MSG_TAIL_RETRACT: 9, proto.MSG_RUNTIME_SYNC: 63,
    proto.MSG_UPDATE_CANCEL: 0, proto.MSG_SET_SLOTS: 148,
}
# What firmware accepts in update mode (bmcu_protocol.cpp update_command_allowed).
_UPDATE_MODE_ALLOWED = frozenset((
    proto.MSG_PING, proto.MSG_GET_STATUS, proto.MSG_GET_CAPS, proto.MSG_UPDATE_CANCEL,
    proto.MSG_STOP_ALL,
))
# runtime_value_valid() in the firmware: the range of each RUNTIME_SYNC float.
_RUNTIME_RANGES = (
    (10.0, 120.0), (10.0, 120.0), (4.0, 40.0), (1000.0, 120000.0), (20.0, 60.0),
    (0.010, 2.000), (0.010, 2.000), (0.010, 2.000), (0.010, 2.000),
    (0.010, 1.000), (0.010, 1.000), (0.010, 1.000), (0.010, 1.000),
)

Reply = Tuple[int, int, bytes]


def _f32(value: float) -> float:
    """Round to a C float, as the firmware compares."""
    return struct.unpack("<f", struct.pack("<f", value))[0]


def _runtime_value_valid(index: int, value: float) -> bool:
    if not math.isfinite(value):
        return False
    if index == 0:
        in_range = (LOAD_PRESSURE_MIN <= value <= LOAD_PRESSURE_MAX
                    or 0.0 <= value <= 2.0)
        return in_range and value == float(int(value))
    low, high = _RUNTIME_RANGES[index - 1]
    return _f32(low) <= value <= _f32(high)


class FakeBMCU:
    """See the module docstring."""

    _next_fd = 1000

    def __init__(self, reactor: FakeReactor, uid: bytes = DEFAULT_UID,
                 firmware: Sequence[int] = DEFAULT_FIRMWARE, protocol: int = 1,
                 capabilities: int = CAP_TAIL_RETRACT, header_version: int = 1,
                 reply_delay: float = 0.0) -> None:
        self.reactor = reactor
        self.uid = uid
        self.firmware = tuple(firmware)
        self.protocol = protocol
        self.capabilities = capabilities
        self.header_version = header_version
        self.reply_delay = reply_delay
        self.channel_count = 4
        FakeBMCU._next_fd += 1
        self.fd = FakeBMCU._next_fd
        self.channels = [FakeChannel() for _ in range(4)]
        self.event_counter = 1
        self.session_id = DEFAULT_SESSION
        self.received: List[Frame] = []
        self.sent: List[Frame] = []
        self.faults: List[Fault] = []
        self.silent = False
        # mm/s a running feed or tail retract covers.
        self.move_speed = 60.0
        # An RS485 adapter that does not gate its receiver hands our own TX back.
        self.echo = False
        # Survives HELLO, as on the firmware; only UPDATE_CANCEL or a reboot clears it.
        self.update_mode = False
        # RGB and name per channel from SET_SLOTS; RAM only, a reboot clears them.
        self.slot_colors = [(0, 0, 0)] * 4
        self.slot_names = [""] * 4
        # False: every NVM save fails (Flash_saves_faulted), so route changes are refused.
        self.nvm_ok = True
        self.unplugged = False
        self.serial: Optional[FakeSerial] = None
        self.open_times: List[float] = []
        # (state, reason) to end that kind of operation with as soon as it starts.
        self.auto_complete: Dict[int, Tuple[int, int]] = {}
        self.runtime_sync: Optional[bytes] = None
        # Op ids that started running, in order.
        self.started: List[int] = []
        self.op: Optional[Dict[str, Any]] = None
        # The firmware's four-entry ring of finished results; the newest is last.
        self.history: Deque[bytes] = collections.deque(maxlen=4)
        self._decoder = proto.StreamDecoder()
        self._pending_tx: List[FakeTimer] = []
        # Registered on first use, so a board nobody talks to adds no timer.
        self._loop_timer: Optional[FakeTimer] = None
        self._power_on()

    def _power_on(self) -> None:
        """RAM state as bmcu_protocol_init leaves it; routes are in flash and survive."""
        self._end_session()
        self.seq_rx_last = 0
        self.seq_tx = 1
        self.last_rx = self.reactor.now
        self.last_status = self.reactor.now
        self.op = None
        self.history.clear()
        self.last_op_state = STATE_IDLE
        self.last_op_reason = REASON_NONE
        self.update_mode = False
        for channel in self.channels:
            channel.motion = MOTION_IDLE
        # State a test sets before the first session is how the board powered on.
        self._booted = False
        self._snapshot = self._event_snapshot()
        if self._loop_timer is not None:
            self.reactor.update_timer(self._loop_timer, self.reactor.NEVER)

    # Wiring

    def serial_module(self) -> types.ModuleType:
        module = types.ModuleType("serial")
        module.Serial = lambda: FakeSerial(self)
        module.SerialException = FakeSerialException
        return module

    def attach(self, serial: FakeSerial) -> None:
        self.serial = serial
        self._decoder = proto.StreamDecoder()

    def detach(self, serial: FakeSerial) -> None:
        if self.serial is serial:
            self.serial = None

    def unplug(self) -> None:
        self.unplugged = True
        self.reactor.fire_fd(self.fd)

    def replug(self) -> None:
        self.unplugged = False

    def reboot(self, announce: bool = False) -> None:
        """
        Power-cycle the board: a new session, no operation or result history,
        every motor stopped and every encoder back at zero, routes kept (they
        are in flash). ``announce`` sends the HELLO_ACK, CAPS and STATUS
        (cmd_id 0) of bmcu_protocol_send_hello, which this firmware build
        defines but does not call at boot.
        """
        self.session_id = (self.session_id * 7 + 1) & 0xFFFFFFFF or 1
        self.slot_colors = [(0, 0, 0)] * 4
        self.slot_names = [""] * 4
        self._discard_pending_tx()
        for channel in self.channels:
            channel.meters = 0.0
        self._power_on()
        if announce:
            self.send(MSG_HELLO_ACK, 0, self.hello_payload())
            self.send(MSG_CAPS, 0, self.caps_payload())
            self.send(proto.MSG_STATUS, 0, self.status_payload())

    # Queries

    def frames(self, *msg_types: int) -> List[Frame]:
        return [frame for frame in self.received
                if not msg_types or frame.msg_type in msg_types]

    def motion_frames(self) -> List[Frame]:
        return [frame for frame in self.received if frame.msg_type in MOTION_TYPES]

    def running_ops(self) -> List[int]:
        return [self.op["id"]] if self.op is not None else []

    def result(self, op_id: int) -> Optional[Dict[str, Any]]:
        """The firmware's stored result for op_id, parsed, or None."""
        payload = self._find_result(op_id)
        return proto.parse_op_result(payload) if payload is not None else None

    def add_fault(self, action: str, msg_type: Optional[int] = None, count: int = 1,
                  seconds: float = 0.0) -> Fault:
        fault = Fault(action, msg_type, count, seconds)
        self.faults.append(fault)
        return fault

    def _take_fault(self, msg_type: int, action: str) -> Optional[Fault]:
        for fault in self.faults:
            if (fault.action == action
                    and fault.matches(msg_type)):
                if fault.count > 0:
                    fault.count -= 1
                return fault
        return None

    # Transmit

    def _moved_mm(self, op: Dict[str, Any]) -> float:
        """mm a feed or tail retract has moved so far."""
        if op["type"] not in (OP_FEED_TO_CONTACT, OP_FEED_DISTANCE, OP_TAIL_RETRACT):
            return 0.0
        # Rounded so the timer set for a distance is not a hair short of it.
        return round((self.reactor.now - op["started"]) * op["speed"], 6)

    def _encoder_meters(self, index: int) -> float:
        channel = self.channels[index]
        op = self.op
        if op is None or op["channel"] != index:
            return channel.meters
        return channel.meters + op["direction"] * self._moved_mm(op) / 1000.0

    def _buffer_pct(self, index: int) -> int:
        channel = self.channels[index]
        op = self.op
        if (op is not None
                and op["channel"] == index
                and op["type"] in (OP_FEED_TO_CONTACT, OP_FEED_DISTANCE)
                and channel.contact_mm is not None
                and self._moved_mm(op) >= channel.contact_mm):
            return 100
        return channel.buffer_pct

    def _error_flags(self) -> int:
        flags = 0
        if not self.nvm_ok:
            flags |= ERROR_FLAG_NVM
        if any(channel.route == ROUTE_UNCERTAIN for channel in self.channels):
            flags |= ERROR_FLAG_ROUTE_UNCERTAIN
        return flags

    def status_payload(self) -> bytes:
        op = self.op
        if op is not None:
            op_fields = dict(active_op_id=op["id"], op_type=op["type"],
                             op_channel=op["channel"], op_state=STATE_RUNNING, op_reason=0)
        else:
            latest = self._find_result(0)
            if latest is not None:
                op_id, op_type, channel, state, reason = struct.unpack_from("<IBBBB", latest)
            else:
                op_id, op_type, channel = 0, 0, 0xFF
                state, reason = self.last_op_state, self.last_op_reason
            op_fields = dict(active_op_id=op_id, op_type=op_type, op_channel=channel,
                             op_state=state, op_reason=reason)
        return status_payload(
            route=[ch.route for ch in self.channels],
            motion=[ch.motion for ch in self.channels],
            present=[ch.present for ch in self.channels],
            buffer_pct=[self._buffer_pct(i) for i in range(4)],
            cal_valid_mask=sum(1 << i for i, ch in enumerate(self.channels) if ch.calibrated),
            encoder_mask=sum(1 << i for i, ch in enumerate(self.channels) if ch.encoder_ok),
            error_flags=self._error_flags(),
            meters=[self._encoder_meters(i) for i in range(4)],
            session_id=self.session_id,
            event_counter=self.event_counter,
            **op_fields,
        )

    def _encode(self, msg_type: int, seq: int, cmd_id: int, payload: bytes) -> bytes:
        body = struct.pack("<BBHII", self.header_version, msg_type, len(payload), seq,
                           cmd_id) + payload
        return b"\x00" + proto.cobs_encode(body + struct.pack("<I", proto.crc32(body))) \
            + b"\x00"

    def send(self, msg_type: int, cmd_id: int, payload: bytes = b"",
             delay: float = 0.0) -> None:
        seq = self.seq_tx
        self.seq_tx = (self.seq_tx + 1) & 0xFFFFFFFF
        self.sent.append(Frame(self.reactor.now + delay, msg_type, seq, cmd_id, bytes(payload)))
        self.send_raw(self._encode(msg_type, seq, cmd_id, bytes(payload)), delay)

    def send_raw(self, data: bytes, delay: float = 0.0) -> None:
        def _deliver(eventtime: float) -> float:
            self.reactor.unregister_timer(timer)
            if timer in self._pending_tx:
                self._pending_tx.remove(timer)
            serial = self.serial
            if (serial is not None
                    and not self.unplugged):
                serial.rx.extend(data)
                self.reactor.fire_fd(self.fd)
            return self.reactor.NEVER

        timer = self.reactor.register_timer(_deliver, self.reactor.now + delay)
        self._pending_tx.append(timer)

    def _discard_pending_tx(self) -> None:
        """Drop every frame not yet on the wire."""
        for timer in self._pending_tx:
            self.reactor.unregister_timer(timer)
        self._pending_tx.clear()

    def _emit(self, msg_type: int, cmd_id: int, payload: bytes) -> None:
        """A frame the firmware sends on its own; a silent board sends nothing."""
        if not self.silent:
            self.send(msg_type, cmd_id, payload)

    def send_status(self) -> None:
        self._snapshot = self._event_snapshot()
        self.last_status = self.reactor.now
        self.send(proto.MSG_STATUS, 0, self.status_payload())

    def set_present(self, channel: int, value: int) -> None:
        """
        Set an inlet sensor. A feed on that channel fails no_filament at once,
        and the change STATUS goes out now instead of on the next firmware loop.
        """
        self.channels[channel].present = 1 if value else 0
        self._process_op()
        self._send_changes()

    # Firmware loop: operations, the host-silence watchdog and unsolicited STATUS

    def _event_snapshot(self) -> Tuple[Any, ...]:
        return (tuple(ch.motion for ch in self.channels),
                tuple(ch.present for ch in self.channels),
                tuple(ch.route for ch in self.channels))

    def _active(self) -> bool:
        return (self.op is not None
                or self._any_motion())

    def _arm_loop(self) -> None:
        if not self.host_online:
            return
        if self._loop_timer is None:
            self._loop_timer = self.reactor.register_timer(self._loop, self.reactor.NEVER)
        self.reactor.update_timer(self._loop_timer, self._next_loop())

    def _next_loop(self) -> float:
        step = LOOP_ACTIVE_S if self._active() else LOOP_IDLE_S
        wake = self.reactor.now + step
        op = self.op
        if op is not None:
            wake = min(wake, op["deadline"])
            for distance in op["distances"]:
                if op["speed"] > 0:
                    wake = min(wake, op["started"] + distance / op["speed"])
        return max(wake, self.reactor.now)

    def _loop(self, eventtime: float) -> float:
        if not self.update_mode:
            self._process_op()
        if not self.host_online:
            return self.reactor.NEVER
        if eventtime - self.last_rx > HOST_SILENCE_S:
            self._host_lost()
            return self.reactor.NEVER
        self._send_changes()
        period = STATUS_ACTIVE_S if self._active() else STATUS_IDLE_S
        if eventtime - self.last_status >= period:
            self.last_status = eventtime
            self._emit(proto.MSG_STATUS, 0, self.status_payload())
        return self._next_loop()

    def _send_changes(self) -> None:
        if not self.host_online:
            return
        snapshot = self._event_snapshot()
        if snapshot != self._snapshot:
            self._snapshot = snapshot
            self.event_counter += 1
            self.last_status = self.reactor.now
            self._emit(proto.MSG_STATUS, 0, self.status_payload())

    def _host_lost(self) -> None:
        """process_events after HOST_SILENCE_S without a request."""
        if self.op is not None:
            self._end_op(STATE_ABORTED, REASON_ABORTED)
        self._stop_all_motion()
        self._end_session()

    def _end_session(self) -> None:
        """No host session: requests get error 96 and RUNTIME_SYNC's motor enable is gone."""
        self.host_session_ready = False
        self.host_online = False
        self.host_motion = False

    def _process_op(self) -> None:
        """process_distance_op: end the running operation for the firmware's reasons."""
        op = self.op
        if op is None:
            return
        channel = self.channels[op["channel"]]
        moved = self._moved_mm(op)
        timed_out = self.reactor.now >= op["deadline"]
        if op["type"] == OP_TAIL_RETRACT:
            if not channel.encoder_ok:
                self._end_op(STATE_FAILED, REASON_ENCODER_IO)
            elif channel.buffer_pct < 5:
                self._end_op(STATE_FAILED, REASON_BUFFER_LIMIT)
            elif moved >= op["target"]:
                self._end_op(*self._tail_outcome(STATE_DONE, REASON_TARGET), op["target"])
            elif timed_out:
                self._end_op(STATE_FAILED, REASON_TIMEOUT)
            return
        if op["type"] == OP_BUFFER_CALIBRATION:
            return
        contact = channel.contact_mm
        buffer_pct = self._buffer_pct(op["channel"])
        if not channel.present:
            self._end_op(STATE_FAILED, REASON_NO_FILAMENT)
        elif not channel.encoder_ok:
            self._end_op(STATE_FAILED, REASON_ENCODER_IO)
        elif buffer_pct <= 3:
            self._end_op(STATE_FAILED, REASON_BUFFER_LIMIT)
        elif buffer_pct >= op["contact_pct"]:
            measured = min(moved, contact) if contact is not None else moved
            if op["stop_on_contact"]:
                self._end_op(STATE_DONE, REASON_CONTACT, measured)
            else:
                self._end_op(STATE_FAILED, REASON_BUFFER_LIMIT, measured)
        elif moved >= op["target"]:
            if op["stop_on_contact"]:
                self._end_op(STATE_FAILED, REASON_DISTANCE_LIMIT, op["target"])
            else:
                self._end_op(STATE_DONE, REASON_TARGET, op["target"])
        elif channel.motion == MOTION_IDLE:
            self._end_op(STATE_FAILED, REASON_MOTION_STOPPED)
        elif timed_out:
            self._end_op(STATE_FAILED, REASON_TIMEOUT)

    def _tail_outcome(self, state: int, reason: int) -> Tuple[int, int]:
        """
        A tail retract is DONE only once its EMPTY route is saved: with a
        working NVM the route empties, else the result is FAILED nvm_write.
        Any other outcome passes through.
        """
        op = self.op
        if (op["type"] != OP_TAIL_RETRACT
                or state != STATE_DONE):
            return state, reason
        if not self.nvm_ok:
            return STATE_FAILED, REASON_NVM
        self.channels[op["channel"]].route = ROUTE_EMPTY
        return state, reason

    def _result_payload(self, op: Dict[str, Any], state: int, reason: int,
                        measured: float) -> bytes:
        duration_ms = int((self.reactor.now - op["started"]) * 1000.0)
        return op_result_payload(op["id"], op["type"], op["channel"], state, reason,
                                 op["target"], measured, duration_ms)

    def _finish(self, state: int, reason: int, measured: Optional[float]) -> bytes:
        """Stop the running operation and record its result in the ring."""
        op = self.op
        if measured is None:
            measured = min(self._moved_mm(op), op["target"]) if op["target"] else 0.0
        if op["channel"] < 4:
            channel = self.channels[op["channel"]]
            channel.meters += op["direction"] * measured / 1000.0
            channel.motion = MOTION_IDLE
        payload = self._result_payload(op, state, reason, measured)
        self.op = None
        self.history.append(payload)
        self.last_op_state, self.last_op_reason = state, reason
        self.event_counter += 1
        return payload

    def _end_op(self, state: int, reason: int, measured: Optional[float] = None) -> None:
        """finish_distance_op: the OP_RESULT, then a STATUS."""
        op_id = self.op["id"]
        payload = self._finish(state, reason, measured)
        self._emit(proto.MSG_OP_RESULT, op_id, payload)
        if not self.silent:
            self.send_status()

    def complete_op(self, op_id: int, state: int = STATE_DONE,
                    reason: Optional[int] = None, measured_mm: Optional[float] = None,
                    send: bool = True) -> None:
        """
        End the running operation now with the outcome the test names, as the
        firmware would for a reason the fake does not model (a calibration a
        person finished, a sensor the test decides). measured_mm defaults to
        what the encoder moved so far, or the target for a DONE target. A tail
        retract DONE still needs a working NVM save, as on the firmware.
        send=False loses the OP_RESULT and STATUS frames, so only
        GET_OP_RESULT can find the result.
        """
        op = self.op
        assert op is not None and op["id"] == op_id, f"operation {op_id} is not running"
        if reason is None:
            reason = REASON_CONTACT if op["type"] == OP_FEED_TO_CONTACT else REASON_TARGET
        if (state == STATE_DONE
                and reason == REASON_TARGET
                and measured_mm is None):
            measured_mm = op["target"]
        payload = self._finish(*self._tail_outcome(state, reason), measured_mm)
        if send:
            self.send(proto.MSG_OP_RESULT, op_id, payload)
            self.send_status()

    def _stop_all_motion(self) -> None:
        for channel in self.channels:
            channel.motion = MOTION_IDLE

    # Receive

    def receive(self, data: bytes) -> None:
        if self.echo:
            self.send_raw(data)
        for msg_type, seq, cmd_id, payload in self._decoder.feed(data):
            self.handle_packet(msg_type, seq, cmd_id, payload)

    def handle_packet(self, msg_type: int, seq: int, cmd_id: int, payload: bytes) -> None:
        now = self.reactor.now
        replies, note = self._dispatch(msg_type, seq, cmd_id, payload)
        self.received.append(Frame(now, msg_type, seq, cmd_id, payload, note))
        self._arm_loop()
        if (self.silent
                or not replies):
            return
        if self._take_fault(msg_type, "drop") is not None:
            return
        delay = self.reply_delay
        fault = self._take_fault(msg_type, "delay")
        if fault is not None:
            delay += fault.seconds
        if self._take_fault(msg_type, "drop_ack") is not None:
            replies = [reply for reply in replies if reply[0] != proto.MSG_ACK]
        for reply_type, reply_cmd, reply_payload in replies:
            if (reply_type == proto.MSG_STATUS
                    and reply_cmd == 0):
                self._snapshot = self._event_snapshot()
                self.last_status = now
            self.send(reply_type, reply_cmd, reply_payload, delay=delay)

    def _dispatch(self, msg_type: int, seq: int, cmd_id: int,
                  payload: bytes) -> Tuple[List[Reply], str]:
        handlers = self._handlers()
        expected = _PAYLOAD_SIZE.get(msg_type)
        if msg_type not in handlers:
            return [_error(cmd_id, ERR_UNKNOWN)], ""
        if (expected is not None
                and len(payload) != expected):
            return [_error(cmd_id, ERR_LENGTH)], ""
        if msg_type == proto.MSG_HELLO:
            return self._on_hello(cmd_id, payload), ""
        if msg_type == proto.MSG_SESSION_CONFIRM:
            if struct.unpack("<I", payload)[0] != self.session_id:
                return [_error(cmd_id, ERR_SESSION)], ""
            if not self.host_session_ready:
                self.host_session_ready = True
                self.seq_rx_last = seq
            if not self._booted:
                self._booted = True
                self._snapshot = self._event_snapshot()
            self.last_rx = self.reactor.now
            self.host_online = True
            return [_ack(cmd_id)], ""
        if not self.host_session_ready:
            return [_error(cmd_id, ERR_SESSION)], ""
        delta = (seq - self.seq_rx_last) & 0xFFFFFFFF
        if (delta == 0
                or delta >= 0x80000000):
            return [], "stale seq"
        self.seq_rx_last = seq
        self.last_rx = self.reactor.now
        self.host_online = True
        if (self.update_mode
                and msg_type not in _UPDATE_MODE_ALLOWED):
            return [_error(cmd_id, ERR_UPDATE_MODE)], ""
        return handlers[msg_type](msg_type, cmd_id, payload), ""

    def _handlers(self) -> Dict[int, Callable[[int, int, bytes], List[Reply]]]:
        handlers = {
            proto.MSG_HELLO: None, proto.MSG_SESSION_CONFIRM: None,
            proto.MSG_PING: lambda t, c, p: [(MSG_PONG, c, p)],
            proto.MSG_GET_STATUS: lambda t, c, p: [(proto.MSG_STATUS, c, self.status_payload())],
            proto.MSG_GET_CAPS: lambda t, c, p: [(MSG_CAPS, c, self.caps_payload())],
            proto.MSG_GET_OP_RESULT: self._on_get_op_result,
            proto.MSG_STOP_ALL: self._on_stop,
            proto.MSG_ABORT_OP: self._on_stop,
            proto.MSG_SET_MOTION: self._on_set_motion,
            proto.MSG_MARK_LOADED: self._on_mark,
            proto.MSG_MARK_UNLOADED: self._on_mark,
            proto.MSG_RUNTIME_SYNC: self._on_runtime_sync,
            proto.MSG_UPDATE_CANCEL: self._on_update_cancel,
            proto.MSG_SET_SLOTS: self._on_set_slots,
            proto.MSG_FEED_TO_CONTACT: self._on_feed,
            proto.MSG_FEED_DISTANCE: self._on_feed,
            proto.MSG_CAL_AUTO_START: self._on_cal_auto_start,
            proto.MSG_TAIL_RETRACT: self._on_tail_retract,
        }
        return handlers

    def hello_payload(self) -> bytes:
        return hello_payload(self.uid, self.firmware, self.protocol, self.capabilities,
                             self.session_id, self.channel_count)

    def caps_payload(self) -> bytes:
        return caps_payload(self.uid, self.firmware, self.protocol, self.capabilities)

    def _on_hello(self, cmd_id: int, payload: bytes) -> List[Reply]:
        """A HELLO aborts the running operation unreported, stops every motor and
        discards frames not yet sent, then rotates the session."""
        nonce = struct.unpack("<I", payload)[0]
        self._end_session()
        if self.op is not None:
            self._finish(STATE_ABORTED, REASON_ABORTED, None)
        self._stop_all_motion()
        self._discard_pending_tx()
        self.session_id = ((self.session_id ^ nonce) & 0xFFFFFFFF) or 1
        return [(MSG_HELLO_ACK, cmd_id, self.hello_payload())]

    def _on_update_cancel(self, msg_type: int, cmd_id: int, payload: bytes) -> List[Reply]:
        self.update_mode = False
        return [_ack(cmd_id)]

    def _on_set_slots(self, msg_type: int, cmd_id: int, payload: bytes) -> List[Reply]:
        """Every slot names its own channel in order, or error 13 and nothing applied."""
        slots = [payload[ch * 37:(ch + 1) * 37] for ch in range(4)]
        for channel, slot in enumerate(slots):
            if slot[0] != channel:
                return [_error(cmd_id, 13, channel)]
        self.slot_colors = [tuple(slot[1:4]) for slot in slots]
        self.slot_names = [slot[9:29].split(b"\0", 1)[0].decode("ascii") for slot in slots]
        return [_ack(cmd_id)]

    def _on_runtime_sync(self, msg_type: int, cmd_id: int, payload: bytes) -> List[Reply]:
        values = struct.unpack_from("<14f", payload)
        if not all(_runtime_value_valid(i, value) for i, value in enumerate(values)):
            return [_error(cmd_id, ERR_RUNTIME_RANGE)]
        if (payload[62] != 1
                or any(flag > 1 for flag in payload[59:62])):
            return [_error(cmd_id, ERR_RUNTIME_POLICY)]
        self.runtime_sync = payload
        self.host_motion = True
        return [_ack(cmd_id)]

    def _blocks_channel(self, index: int) -> bool:
        """operation_blocks_channel: a calibration blocks every channel."""
        op = self.op
        return op is not None and (op["type"] == OP_BUFFER_CALIBRATION
                                   or op["channel"] == index)

    def _any_motion(self) -> bool:
        return any(ch.motion != MOTION_IDLE for ch in self.channels)

    def _load_motion_elsewhere(self, index: int) -> bool:
        return any(ch.motion in (MOTION_SEND_OUT, MOTION_BEFORE_ON_USE)
                   for i, ch in enumerate(self.channels) if i != index)

    def _begin_change(self, index: int) -> bool:
        """ams_state_begin_change: the route turns UNCERTAIN and is saved."""
        if not self.nvm_ok:
            return False
        self.channels[index].route = ROUTE_UNCERTAIN
        return True

    def _save_route(self, index: int, route: int) -> bool:
        """ams_state_set_loaded / _unloaded: a failed save keeps the old route. An
        EMPTY route marked unloaded again writes nothing, so it cannot fail."""
        if (route == ROUTE_EMPTY
                and self.channels[index].route == ROUTE_EMPTY):
            return True
        if not self.nvm_ok:
            return False
        self.channels[index].route = route
        return True

    def _set_motion(self, index: int, motion: int) -> bool:
        """set_motion() with persist_route_change: the route moves with the motion."""
        channel = self.channels[index]
        if motion == MOTION_IDLE:
            channel.motion = MOTION_IDLE
            return True
        begins = ((motion == MOTION_SEND_OUT and channel.route == ROUTE_EMPTY)
                  or motion in (MOTION_BEFORE_ON_USE, MOTION_BEFORE_PULL_BACK,
                                MOTION_PULL_BACK))
        if begins and not self._begin_change(index):
            return False
        channel.motion = motion
        if motion == MOTION_ON_USE:
            if not self._save_route(index, ROUTE_LOADED):
                channel.motion = MOTION_IDLE
                return False
        return True

    def _on_set_motion(self, msg_type: int, cmd_id: int, payload: bytes) -> List[Reply]:
        index, motion = payload[0], payload[1]
        if (index >= 4
                or self._blocks_channel(index)
                or motion > MOTION_STOP_ON_USE
                or (motion in (MOTION_SEND_OUT, MOTION_BEFORE_ON_USE)
                    and self._load_motion_elsewhere(index))):
            return [_error(cmd_id, ERR_SET_MOTION_REFUSED)]
        if not self._set_motion(index, motion):
            return [_error(cmd_id, ERR_SET_MOTION_FAILED, index)]
        self.event_counter += 1
        return [_ack(cmd_id), (proto.MSG_STATUS, 0, self.status_payload())]

    def _on_mark(self, msg_type: int, cmd_id: int, payload: bytes) -> List[Reply]:
        index = payload[0]
        if (index >= 4
                or self._blocks_channel(index)):
            return [_error(cmd_id, ERR_MARK_REFUSED)]
        self.channels[index].motion = MOTION_IDLE
        loaded = msg_type == proto.MSG_MARK_LOADED
        if not self._save_route(index, ROUTE_LOADED if loaded else ROUTE_EMPTY):
            return [_error(cmd_id, ERR_NVM, index), (proto.MSG_STATUS, 0, self.status_payload())]
        self.event_counter += 1
        return [_ack(cmd_id), (proto.MSG_STATUS, 0, self.status_payload())]

    # Operations

    def _find_result(self, op_id: int) -> Optional[bytes]:
        """find_op_result: op_id 0 is the newest result."""
        if op_id == 0:
            return self.history[-1] if self.history else None
        for payload in self.history:
            if struct.unpack_from("<I", payload)[0] == op_id:
                return payload
        return None

    def _replay(self, cmd_id: int) -> Optional[List[Reply]]:
        """replay_known_operation: a repeated op id moves nothing."""
        if self.op is not None and self.op["id"] == cmd_id:
            return [_ack(cmd_id), (proto.MSG_STATUS, 0, self.status_payload())]
        stored = self._find_result(cmd_id)
        if stored is None:
            return None
        return [_ack(cmd_id), (proto.MSG_OP_RESULT, cmd_id, stored),
                (proto.MSG_STATUS, 0, self.status_payload())]

    def _channel_fault(self, channel: FakeChannel, needs_filament: bool) -> Optional[int]:
        """The reason a channel refuses an operation, before any busy check, or None."""
        if not channel.calibrated:
            return REASON_NOT_CALIBRATED
        if (needs_filament
                and not channel.present):
            return REASON_NO_FILAMENT
        if not channel.encoder_ok:
            return REASON_ENCODER_IO
        return None

    def _refuse(self, cmd_id: int, op_type: int, channel: int, reason: int,
                target: float) -> List[Reply]:
        """A refused operation answers with its FAILED OP_RESULT alone, kept in the ring."""
        payload = op_result_payload(cmd_id, op_type, channel, STATE_FAILED, reason,
                                    target, 0.0, 0)
        self.history.append(payload)
        self.last_op_state, self.last_op_reason = STATE_FAILED, reason
        return [(proto.MSG_OP_RESULT, cmd_id, payload)]

    def _begin(self, cmd_id: int, msg_type: int, op_type: int, channel: int, target: float,
               timeout_s: float, contact_pct: int = 0, stop_on_contact: bool = False,
               direction: float = 0.0) -> List[Reply]:
        """begin_op, then ACK and a STATUS, or the auto_complete outcome."""
        distances = []
        if op_type in (OP_FEED_TO_CONTACT, OP_FEED_DISTANCE, OP_TAIL_RETRACT):
            distances.append(target)
            contact_mm = self.channels[channel].contact_mm
            if (op_type != OP_TAIL_RETRACT
                    and contact_mm is not None):
                distances.append(contact_mm)
        self.op = {"id": cmd_id, "type": op_type, "channel": channel, "target": target,
                   "started": self.reactor.now, "deadline": self.reactor.now + timeout_s,
                   "contact_pct": contact_pct, "stop_on_contact": stop_on_contact,
                   "direction": direction, "distances": distances,
                   "speed": self.move_speed if self.host_motion else 0.0}
        self.started.append(cmd_id)
        self.last_op_state, self.last_op_reason = STATE_RUNNING, REASON_NONE
        replies = [_ack(cmd_id), (proto.MSG_STATUS, 0, self.status_payload())]
        auto = self.auto_complete.get(msg_type)
        if auto is not None:
            state, reason = auto
            measured = (target if (state, reason) == (STATE_DONE, REASON_TARGET) else None)
            payload = self._finish(*self._tail_outcome(state, reason), measured)
            replies.append((proto.MSG_OP_RESULT, cmd_id, payload))
            replies.append((proto.MSG_STATUS, 0, self.status_payload()))
        return replies

    def _on_feed(self, msg_type: int, cmd_id: int, payload: bytes) -> List[Reply]:
        replay = self._replay(cmd_id)
        if replay is not None:
            return replay
        index, target, contact_pct, timeout_ms = struct.unpack("<BfBI", payload)
        if index >= 4:
            return [_error(cmd_id, ERR_BAD_ARGUMENT)]
        contact = msg_type == proto.MSG_FEED_TO_CONTACT
        op_type = OP_FEED_TO_CONTACT if contact else OP_FEED_DISTANCE
        channel = self.channels[index]
        busy = (self.op is not None
                or channel.motion != MOTION_IDLE
                or self._load_motion_elsewhere(index)
                or not (math.isfinite(target) and 5.0 <= target <= 5000.0)
                or not 250 <= timeout_ms <= 300000
                or not 55 <= contact_pct <= 98)
        reason = self._channel_fault(channel, needs_filament=True)
        if (reason is None
                and (busy or not self._begin_change(index))):
            reason = REASON_BUSY
        if reason is not None:
            return self._refuse(cmd_id, op_type, index, reason, target)
        channel.motion = MOTION_SEND_OUT
        return self._begin(cmd_id, msg_type, op_type, index, target, timeout_ms / 1000.0,
                           contact_pct, contact, 1.0)

    def _on_cal_auto_start(self, msg_type: int, cmd_id: int, payload: bytes) -> List[Reply]:
        replay = self._replay(cmd_id)
        if replay is not None:
            return replay
        mask = payload[0] & 0x0F
        occupied = sum(1 << i for i, ch in enumerate(self.channels)
                       if ch.route in (ROUTE_LOADED, ROUTE_UNCERTAIN))
        if (not mask
                or self.op is not None
                or self._any_motion()
                or occupied & mask):
            return self._refuse(cmd_id, OP_BUFFER_CALIBRATION, 0xFF, REASON_BUSY, 0.0)
        self._stop_all_motion()
        # A calibration waits for a person; only the test ends it.
        return self._begin(cmd_id, msg_type, OP_BUFFER_CALIBRATION, mask.bit_length() - 1,
                           0.0, CALIBRATION_FOREVER_S)

    def _on_tail_retract(self, msg_type: int, cmd_id: int, payload: bytes) -> List[Reply]:
        replay = self._replay(cmd_id)
        if replay is not None:
            return replay
        index, target, timeout_ms = struct.unpack("<BfI", payload)
        if (index >= 4
                or not (5.0 <= target <= 2000.0)
                or not 250 <= timeout_ms <= 300000):
            return [_error(cmd_id, ERR_BAD_ARGUMENT)]
        channel = self.channels[index]
        reason = self._channel_fault(channel, needs_filament=False)
        if (reason is None
                and (self.op is not None
                     or channel.motion != MOTION_IDLE
                     or not self.host_motion)):
            reason = REASON_BUSY
        if reason is not None:
            return self._refuse(cmd_id, OP_TAIL_RETRACT, index, reason, target)
        return self._begin(cmd_id, msg_type, OP_TAIL_RETRACT, index, target,
                           timeout_ms / 1000.0, direction=-1.0)

    def _on_stop(self, msg_type: int, cmd_id: int, payload: bytes) -> List[Reply]:
        """ABORT_OP, STOP_ALL: the aborted result first, then the ACK."""
        replies: List[Reply] = []
        if self.op is not None:
            op_id = self.op["id"]
            replies.append((proto.MSG_OP_RESULT, op_id,
                            self._finish(STATE_ABORTED, REASON_ABORTED, None)))
            replies.append((proto.MSG_STATUS, 0, self.status_payload()))
        if msg_type == proto.MSG_STOP_ALL:
            self._stop_all_motion()
        self.event_counter += 1
        return replies + [_ack(cmd_id), (proto.MSG_STATUS, 0, self.status_payload())]

    def _on_get_op_result(self, msg_type: int, cmd_id: int, payload: bytes) -> List[Reply]:
        stored = self._find_result(struct.unpack("<I", payload)[0])
        if stored is None:
            return [_error(cmd_id, ERR_OP_RESULT_UNAVAILABLE)]
        return [(proto.MSG_OP_RESULT, cmd_id, stored)]


def _ack(cmd_id: int, ok: int = 1, error: int = 0) -> Reply:
    return (proto.MSG_ACK, cmd_id, ack_payload(cmd_id, ok, error))


def _error(cmd_id: int, code: int, channel: int = 0xFF) -> Reply:
    return (proto.MSG_ERROR, cmd_id, error_payload(code, channel))
