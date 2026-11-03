# AFCProject Automated Filament Changer Software
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""
BMCU serial link: the half-duplex resend and write-gate policy, and the
reactor-driven connection that uses it.
"""

from __future__ import annotations

import collections
import contextlib
import fcntl
import random
import struct
import termios
from typing import Any, Callable, Deque, Dict, List, Optional, Set, Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from reactor import SelectReactor as Reactor

from extras import AFC_BMCU_proto as proto

# A request is resent after RESEND_TIMEOUT scaled by a factor from RESEND_JITTER,
# so host retries cannot phase-lock with the firmware's periodic STATUS, and is
# sent at most RESEND_ATTEMPTS times in all.
RESEND_TIMEOUT = 0.25
RESEND_JITTER = (0.8, 1.2)
RESEND_ATTEMPTS = 3
# A half-duplex write waits this long after the last received byte, and at most
# WRITE_GATE_MAX for a partial inbound frame.
RX_QUIET = 0.002
WRITE_GATE_MAX = 0.05
# How long a reply id stays answered, so a late duplicate reply is dropped.
ANSWERED_TTL = 5.0

# Connection timings, seconds.
CONNECT_SETTLE = 1.5
HELLO_RETRY = 0.6
HANDSHAKE_TIMEOUT = 15.0
HEARTBEAT_INTERVAL = 5.0
LINK_TIMEOUT = 15.0
SUPERVISE_INTERVAL = 1.0
RECONNECT_MIN = 1.0
RECONNECT_MAX = 30.0
WRITE_TIMEOUT = 0.5
OP_START_TIMEOUT = 2.0
OP_POLL_INTERVAL = 1.0
OP_QUERY_TIMEOUT = 0.75
STOP_TIMEOUT = 1.0
# Wire cmd ids run 1..CMD_ID_MAX and start at a random point per connection object.
CMD_ID_MAX = 0x7FFFFFFF

# Stops. Queuing one drops every pending resend; only a newer stop drops a
# pending stop.
STOP_TYPES = frozenset((proto.MSG_STOP_ALL, proto.MSG_ABORT_OP, proto.MSG_UPDATE_CANCEL))
# Requests that change no motor, route or operation state on the BMCU; they
# neither drop resends nor are dropped. SET_SLOTS only sets LED colors.
PASSIVE_TYPES = frozenset((
    proto.MSG_PING, proto.MSG_GET_STATUS, proto.MSG_GET_CAPS, proto.MSG_GET_OP_RESULT,
    proto.MSG_HELLO, proto.MSG_SESSION_CONFIRM, proto.MSG_SET_SLOTS,
))
# Never resent: HELLO has its own retry and the heartbeat PING its own expiry.
UNTRACKED_TYPES = frozenset((proto.MSG_HELLO, proto.MSG_PING))


class _Resend:
    """One tracked request awaiting a reply."""

    def __init__(self, msg_type: int, payload: bytes) -> None:
        """
        Track a request that has been queued once.

        :param msg_type: request message id
        :param payload: request payload, resent unchanged
        """
        self.msg_type = msg_type
        self.payload = bytes(payload)
        self.sends = 1
        self.due: Optional[float] = None


class BMCULinkPolicy:
    """
    Half-duplex link decisions with no I/O: which unanswered requests to
    resend and when, which resends a newer command makes stale, which
    duplicate replies to drop, and when the line is clear to write.

    With half_duplex False every method is a no-op: nothing is resent, no
    reply is dropped and writes are never held.
    """

    def __init__(self, half_duplex: bool, rng: random.Random, logger: Any) -> None:
        """
        Start with nothing tracked.

        :param half_duplex: enable resends, duplicate drops and the write gate
        :param rng: source of the resend jitter
        :param logger: AFC logger for resend decisions
        """
        self.half_duplex = half_duplex
        self.rng = rng
        self.logger = logger
        self._resend: Dict[int, _Resend] = {}
        self.answered: Dict[int, float] = {}
        self.last_rx = float("-inf")
        self.partial_since: Optional[float] = None
        self._status_motion: Optional[Tuple[int, ...]] = None
        self._status_flags = 0

    @property
    def pending_resends(self) -> Dict[int, Tuple[int, int, Optional[float]]]:
        """
        Tracked requests, for tests.

        :return Dict[int, Tuple[int, int, Optional[float]]]: cmd_id to
            (msg_type, sends so far, resend deadline or None until written)
        """
        return {cmd_id: (entry.msg_type, entry.sends, entry.due)
                for cmd_id, entry in self._resend.items()}

    def _drop_superseded(self, msg_type: int) -> None:
        """
        Drop resends made stale by a newer request or a STATUS showing a stop.
        Drops by kind, not channel: a state change on one channel also drops a
        pending resend for another, which then simply times out.

        :param msg_type: the newer request, or MSG_STATUS
        """
        if msg_type in PASSIVE_TYPES:
            return
        for cmd_id, entry in list(self._resend.items()):
            if msg_type in STOP_TYPES:
                stale = True
            else:
                stale = (entry.msg_type not in PASSIVE_TYPES
                         and entry.msg_type not in STOP_TYPES)
            if stale:
                del self._resend[cmd_id]
                self.logger.debug(f"BMCU dropped resend of 0x{entry.msg_type:02X} "
                                  f"cmd={cmd_id}: superseded by 0x{msg_type:02X}")

    def on_queued(self, msg_type: int, cmd_id: int, payload: bytes) -> None:
        """
        A new request was queued: apply the supersede rules, then track it.

        :param msg_type: request message id
        :param cmd_id: request cmd_id, kept across resends
        :param payload: request payload
        """
        if not self.half_duplex:
            return
        self._drop_superseded(msg_type)
        if msg_type not in UNTRACKED_TYPES:
            self._resend[cmd_id] = _Resend(msg_type, payload)

    def on_written(self, cmd_id: int, now: float) -> None:
        """
        A frame reached the wire: start its resend wait if none is running.

        :param cmd_id: cmd_id of the written frame
        :param now: reactor time of the write
        """
        if not self.half_duplex:
            return
        entry = self._resend.get(cmd_id)
        if (entry is not None
                and entry.due is None):
            entry.due = now + RESEND_TIMEOUT * self.rng.uniform(*RESEND_JITTER)

    def on_reply(self, cmd_id: int, now: float) -> bool:
        """
        A reply arrived for cmd_id: stop resending it.

        :param cmd_id: cmd_id the reply echoes
        :param now: reactor time of the reply
        :return bool: False when the reply duplicates one already answered
        """
        if not self.half_duplex:
            return True
        if self.answered.get(cmd_id, 0.0) > now:
            self.logger.debug(f"BMCU dropped duplicate reply cmd={cmd_id}")
            return False
        self._resend.pop(cmd_id, None)
        return True

    def mark_answered(self, cmd_id: int, now: float) -> None:
        """
        The request for cmd_id is finished; later replies to it are duplicates.

        :param cmd_id: the finished request
        :param now: reactor time
        """
        if not self.half_duplex:
            return
        for answered_id, expiry in list(self.answered.items()):
            if expiry <= now:
                del self.answered[answered_id]
        self.answered[cmd_id] = now + ANSWERED_TTL

    def forget(self, cmd_id: int) -> None:
        """
        Stop resending a request its caller gave up on.

        :param cmd_id: the abandoned request
        """
        self._resend.pop(cmd_id, None)

    def track_status(self, status: Dict[str, Any]) -> bool:
        """
        Remember a STATUS and report whether the firmware stopped something on
        its own: a channel went from moving to idle or a new fault flag appeared.

        :param status: parsed STATUS
        :return bool: True on such a stop; the first STATUS only seeds
        """
        if not self.half_duplex:
            return False
        motion = tuple(status["motion"])
        flags = int(status["controller_fault_flags"])
        previous, previous_flags = self._status_motion, self._status_flags
        self._status_motion, self._status_flags = motion, flags
        if previous is None:
            return False
        went_idle = any(old != proto.MOTION_IDLE
                        and new == proto.MOTION_IDLE
                        for old, new in zip(previous, motion))
        return (went_idle
                or bool(flags & ~previous_flags))

    def on_unsolicited_status(self, status: Dict[str, Any]) -> None:
        """
        An unsolicited STATUS arrived: one showing a stop drops pending
        state-changing resends, since a resend would restart a motor the
        firmware just stopped.

        :param status: parsed STATUS
        """
        if self.track_status(status):
            self._drop_superseded(proto.MSG_STATUS)

    def due_resends(self, now: float) -> List[Tuple[int, int, bytes]]:
        """
        Requests to send again now, with the same cmd_id. A request already
        sent RESEND_ATTEMPTS times is dropped instead.

        :param now: reactor time
        :return List[Tuple[int, int, bytes]]: (msg_type, cmd_id, payload) each
        """
        if not self.half_duplex:
            return []
        resends = []
        for cmd_id, entry in list(self._resend.items()):
            if (entry.due is None
                    or now < entry.due):
                continue
            if entry.sends >= RESEND_ATTEMPTS:
                del self._resend[cmd_id]
                self.logger.warning(f"BMCU no reply to 0x{entry.msg_type:02X} "
                                    f"cmd={cmd_id} after {entry.sends} sends")
                continue
            entry.sends += 1
            entry.due = None
            resends.append((entry.msg_type, cmd_id, entry.payload))
            self.logger.debug(f"BMCU resent 0x{entry.msg_type:02X} cmd={cmd_id} "
                              f"attempt {entry.sends}")
        return resends

    def next_resend_due(self) -> Optional[float]:
        """
        Earliest armed resend deadline.

        :return Optional[float]: the deadline, or None when none is armed
        """
        dues = [entry.due for entry in self._resend.values() if entry.due is not None]
        return min(dues) if dues else None

    def on_rx_bytes(self, now: float, decoder_pending: bool,
                    frame_ended: bool = False) -> None:
        """
        Bytes arrived: note the time, and when a partial frame started.

        :param now: reactor time of the read
        :param decoder_pending: the stream decoder holds an unfinished frame
        :param frame_ended: a frame ended (decoded or discarded) within these bytes
        """
        if not self.half_duplex:
            return
        self.last_rx = now
        if not decoder_pending:
            self.partial_since = None
        elif (frame_ended
                or self.partial_since is None):
            self.partial_since = now

    def write_blocked_until(self, now: float) -> Optional[float]:
        """
        Whether a write may go now. Half-duplex holds writes for RX_QUIET after
        the last received byte, and while a frame is arriving, bounded by
        WRITE_GATE_MAX from the frame's start.

        :param now: reactor time
        :return Optional[float]: None to write now, else the time to try again
        """
        if not self.half_duplex:
            return None
        release = self.last_rx + RX_QUIET
        blocked = now < release
        if self.partial_since is not None:
            partial_until = self.partial_since + WRITE_GATE_MAX
            if now < partial_until:
                blocked = True
                release = max(release, partial_until)
        return release if blocked else None

    def reset(self) -> None:
        """
        Forget all state; called on every disconnect.
        """
        self._resend.clear()
        self.answered.clear()
        self.last_rx = float("-inf")
        self.partial_since = None
        self._status_motion = None
        self._status_flags = 0


# Linux struct serial_struct in native layout; flags is the fifth int. ftdi_sio maps
# ASYNC_LOW_LATENCY to a 1 ms latency timer, so the write gate sees an inbound frame
# almost as soon as it starts.
_SERIAL_STRUCT = struct.Struct("@iiIiiiiiHcciHHPHIL")
_SERIAL_FLAGS_OFFSET = struct.calcsize("@iiIi")
_ASYNC_LOW_LATENCY = 0x2000
_TIOCGSERIAL = getattr(termios, "TIOCGSERIAL", 0x541E)
_TIOCSSERIAL = getattr(termios, "TIOCSSERIAL", 0x541F)
_serial_ioctl = fcntl.ioctl


def request_low_latency(fd: int) -> Optional[str]:
    """
    Set ASYNC_LOW_LATENCY on a tty.

    :param fd: open tty file descriptor
    :return Optional[str]: None when set (or already set), else why it was not
    """
    buffer = bytearray(_SERIAL_STRUCT.size)
    try:
        _serial_ioctl(fd, _TIOCGSERIAL, buffer, True)
        flags = struct.unpack_from("@i", buffer, _SERIAL_FLAGS_OFFSET)[0]
        if flags & _ASYNC_LOW_LATENCY:
            return None
        struct.pack_into("@i", buffer, _SERIAL_FLAGS_OFFSET, flags | _ASYNC_LOW_LATENCY)
        _serial_ioctl(fd, _TIOCSSERIAL, buffer, False)
    except OSError as exc:
        return exc.strerror or str(exc)
    return None


class BMCUError(Exception):
    """A BMCU request failed, was rejected, or the link dropped."""

    #: Firmware error code when the firmware answered with ERROR, else None.
    code: Optional[int] = None


class BMCUTimeoutError(BMCUError):
    """A BMCU request or operation got no answer in time."""


class BMCUVersionError(BMCUError):
    """The firmware speaks a protocol version AFC_BMCU does not support."""


def _version_message(protocol: int, firmware: str = "") -> str:
    """
    Explain a protocol mismatch.

    :param protocol: protocol version the firmware reported
    :param firmware: firmware version, when known
    :return str: message naming both sides
    """
    board = f"BMCU firmware {firmware}" if firmware else "BMCU firmware"
    return (f"{board} speaks protocol {protocol}; AFC_BMCU supports protocol "
            f"{proto.PROTO_VERSION}. Flash a supported BMCU-Klipper firmware.")


class BMCUConnection:
    """
    Serial connection to one BMCU, driven by the Klipper reactor.

    Reads arrive through ``reactor.register_fd``; requests block the calling
    greenlet on a ``reactor.completion()`` until the reply with their cmd_id.
    After the HELLO / SESSION_CONFIRM handshake a supervision timer pings an
    idle link and declares it lost when nothing arrives for LINK_TIMEOUT.
    A lost link fails every waiting request, calls ``on_link(False)``, and
    reconnects with backoff; a reconnect calls ``on_link(True)`` and sends no
    motion command. STATUS frames are parsed and handed to ``on_status``.
    """

    def __init__(self, reactor: Reactor, port: str, baud: int, half_duplex: bool,
                 logger: Any, on_status: Callable[[Dict[str, Any]], None],
                 on_link: Callable[[bool], None],
                 rng: Optional[random.Random] = None) -> None:
        """
        Set up the connection; nothing is opened until connect().

        :param reactor: Klipper reactor
        :param port: serial device path, ideally /dev/serial/by-id/...
        :param baud: serial baud rate
        :param half_duplex: RS485 mode: resends, duplicate drops and the write gate
        :param logger: AFC logger
        :param on_status: called with each parsed STATUS
        :param on_link: called with False when an established link drops, True
            when the reconnect timer brings it back
        :param rng: random source for cmd ids, nonces and resend jitter
        """
        self._reactor = reactor
        self.port = port
        self.baud = baud
        self.half_duplex = half_duplex
        self.logger = logger
        self._on_status = on_status
        self._on_link = on_link
        self._rng = rng if rng is not None else random.Random()
        self.policy = BMCULinkPolicy(half_duplex, self._rng, logger)
        self._next_cmd = self._rng.randrange(1, CMD_ID_MAX)
        self._seq = 1
        self._serial: Any = None
        self._fd_handle: Any = None
        self._decoder = proto.StreamDecoder()
        self._connected = False
        self._closed = False
        self._session_id = 0
        self._hello: Optional[Dict[str, Any]] = None
        self._caps = 0
        self._tx: Deque[Tuple[int, bytes]] = collections.deque()
        self._pending: Dict[int, Any] = {}
        self._open_ops: Set[int] = set()
        self._op_results: Dict[int, Dict[str, Any]] = {}
        self._op_waiters: Dict[int, Any] = {}
        self._last_rx = 0.0
        self._status_reject_warned = False
        self._last_tx = 0.0
        self._backoff = RECONNECT_MIN
        self._backoff_held = False
        self._generation = 0
        self.last_error = ""
        never = reactor.NEVER
        self._write_timer = reactor.register_timer(self._flush_tx, never)
        self._resend_timer = reactor.register_timer(self._resend_tick, never)
        self._supervise_timer = reactor.register_timer(self._supervise_tick, never)
        self._reconnect_timer = reactor.register_timer(self._reconnect_tick, never)

    # Properties

    @property
    def connected(self) -> bool:
        """
        Whether the handshake has completed on an open port.

        :return bool: True while the link is up
        """
        return self._connected

    @property
    def caps(self) -> int:
        """
        Capability mask from the last CAPS reply, 0 before the first handshake.

        :return int: CAP_* bits
        """
        return self._caps

    @property
    def generation(self) -> int:
        """
        How many handshakes have completed. A change means the board was
        reconnected, so state the caller set up before it may be gone.

        :return int: the handshake count, 0 before the first
        """
        return self._generation

    @property
    def reconnect_delay(self) -> float:
        """
        Seconds the next reconnect attempt will wait.

        :return float: the current backoff
        """
        return self._backoff

    @property
    def hello(self) -> Optional[Dict[str, Any]]:
        """
        The last HELLO_ACK, parsed: firmware, protocol, session_id, channels.

        :return Optional[Dict[str, Any]]: the fields, or None before the first handshake
        """
        return dict(self._hello) if self._hello is not None else None

    # Public API

    def connect(self) -> None:
        """
        Open the port and run the handshake: HELLO, protocol check,
        SESSION_CONFIRM, GET_CAPS. On failure the reconnect timer is armed,
        except for a protocol mismatch, which raises BMCUVersionError and
        stays down.
        """
        if self._connected:
            return
        self._closed = False
        try:
            self._establish()
        except BMCUVersionError:
            raise
        except BMCUError:
            self._schedule_reconnect()
            raise

    def close(self) -> None:
        """
        Close the port and stop reconnecting. Waiting requests fail.
        """
        self._closed = True
        self._reactor.update_timer(self._reconnect_timer, self._reactor.NEVER)
        self._teardown("BMCU connection closed")

    def request(self, msg_type: int, payload: bytes = b"",
                timeout: float = 2.0) -> Tuple[int, bytes]:
        """
        Send a request and block until its reply.

        :param msg_type: request message id
        :param payload: request payload
        :param timeout: seconds to wait for the reply
        :return Tuple[int, bytes]: reply msg_type and payload
        """
        self._require_connected()
        return self._request_raw(msg_type, payload, timeout)

    def start_op(self, msg_type: int, payload: bytes) -> int:
        """
        Start a firmware operation and wait for it to be accepted. When the
        start cannot be confirmed, ABORT_OP is sent so no unconfirmed motion
        is left running.

        :param msg_type: operation message id (FEED_*, TAIL_RETRACT, CAL_AUTO_START...)
        :param payload: operation payload
        :return int: the op_id, which is the request's cmd_id
        """
        self._require_connected()
        op_id = self._new_cmd_id()
        self._open_ops.add(op_id)
        try:
            self._exchange(msg_type, op_id, payload, OP_START_TIMEOUT)
        except BMCUTimeoutError:
            self._open_ops.discard(op_id)
            with contextlib.suppress(BMCUError):
                self.abort_op()
            raise
        except BMCUError:
            self._open_ops.discard(op_id)
            raise
        return op_id

    def wait_op(self, op_id: int, timeout: float) -> Dict[str, Any]:
        """
        Wait for an operation's OP_RESULT, asking for it with GET_OP_RESULT
        every OP_POLL_INTERVAL in case the unsolicited result was lost.

        :param op_id: id from start_op
        :param timeout: seconds to wait
        :return Dict[str, Any]: the parsed OP_RESULT (state DONE, FAILED or ABORTED)
        """
        deadline = self._reactor.monotonic() + timeout
        while True:
            result = self._op_results.pop(op_id, None)
            if result is not None:
                return result
            self._require_connected()
            now = self._reactor.monotonic()
            if now >= deadline:
                # Nobody waits for this op any more, so a late result is not kept.
                self._open_ops.discard(op_id)
                error_str = f"BMCU operation {op_id} did not finish within {timeout:g}s"
                raise BMCUTimeoutError(error_str)
            completion = self._reactor.completion()
            self._op_waiters[op_id] = completion
            try:
                outcome = completion.wait(min(deadline, now + OP_POLL_INTERVAL))
            finally:
                self._op_waiters.pop(op_id, None)
            if isinstance(outcome, BMCUError):
                raise outcome
            # A result or a dropped link completes the waiter; None means the
            # poll interval passed with neither.
            if (outcome is None
                    and self._reactor.monotonic() < deadline):
                with contextlib.suppress(BMCUError):
                    self._request_raw(proto.MSG_GET_OP_RESULT, proto.pack_u32(op_id),
                                      OP_QUERY_TIMEOUT)

    def stop_all(self) -> None:
        """
        Stop every motor and abort any running operation. A stop is never
        superseded by a later request in half-duplex mode.
        """
        self.request(proto.MSG_STOP_ALL, b"", STOP_TIMEOUT)

    def abort_op(self) -> None:
        """
        Abort the running operation.
        """
        self.request(proto.MSG_ABORT_OP, b"", STOP_TIMEOUT)

    def stop_all_nowait(self) -> bool:
        """
        Write one STOP_ALL straight to the port, past the queue and the write
        gate, and wait for nothing. For klippy shutdown and disconnect, where
        the reactor cannot wait for a reply.

        :return bool: True when the frame was written
        """
        if self._serial is None:
            return False
        # A resend after the stop would carry a newer seq, and the firmware would run it.
        self.policy.reset()
        frame = proto.encode_packet(proto.MSG_STOP_ALL, self._next_seq(), self._new_cmd_id())
        try:
            self._serial.write(frame)
        except Exception as exc:
            self.logger.debug(f"BMCU STOP_ALL write failed: {exc}")
            return False
        return True

    def drop(self, reason: str) -> None:
        """
        Treat the link as lost: fail waiters, call on_link(False), reconnect.
        The owner's setup failed, so the backoff keeps growing across
        reconnects until mark_ready().

        :param reason: why, for the log and the waiters' errors
        """
        self._backoff_held = True
        self._link_lost(reason)

    def mark_ready(self) -> None:
        """
        The owner finished its setup after a handshake: reset the backoff.
        """
        self._backoff_held = False
        self._backoff = RECONNECT_MIN

    def op_open(self, op_id: int) -> bool:
        """
        Whether an operation started by start_op still has no result.

        :param op_id: id from start_op
        :return bool: True until its OP_RESULT arrives or the link drops
        """
        return op_id in self._open_ops

    # Requests

    def _require_connected(self) -> None:
        """
        Raise BMCUError unless the link is up.
        """
        if not self._connected:
            error_str = f"BMCU on {self.port} is not connected"
            raise BMCUError(error_str)

    def _new_cmd_id(self) -> int:
        """
        Next wire cmd id, wrapping from CMD_ID_MAX to 1.

        :return int: the id
        """
        value = self._next_cmd
        self._next_cmd = value + 1 if value < CMD_ID_MAX else 1
        return value

    def _request_raw(self, msg_type: int, payload: bytes, timeout: float) -> Tuple[int, bytes]:
        """
        Send a request under a new cmd id and wait for its reply.

        :param msg_type: request message id
        :param payload: request payload
        :param timeout: seconds to wait
        :return Tuple[int, bytes]: reply msg_type and payload
        """
        cmd_id = self._new_cmd_id()
        result = self._exchange(msg_type, cmd_id, payload, timeout)
        self.policy.mark_answered(cmd_id, self._reactor.monotonic())
        return result

    def _exchange(self, msg_type: int, cmd_id: int, payload: bytes,
                  timeout: float) -> Tuple[int, bytes]:
        """
        Send one request with a given cmd id and wait for the first reply to it.

        :param msg_type: request message id
        :param cmd_id: wire cmd id
        :param payload: request payload
        :param timeout: seconds to wait
        :return Tuple[int, bytes]: reply msg_type and payload
        """
        completion = self._reactor.completion()
        self._pending[cmd_id] = completion
        try:
            self._queue_frame(msg_type, cmd_id, payload, track=True)
            result = completion.wait(self._reactor.monotonic() + timeout)
        finally:
            self._pending.pop(cmd_id, None)
            self.policy.forget(cmd_id)
        if result is None:
            error_str = (f"BMCU request 0x{msg_type:02X} cmd={cmd_id} got no reply "
                         f"within {timeout:g}s")
            raise BMCUTimeoutError(error_str)
        if isinstance(result, BMCUError):
            raise result
        return result

    def _reply_result(self, msg_type: int, payload: bytes) -> Any:
        """
        Turn a reply into what the waiting request returns or raises.

        :param msg_type: reply message id
        :param payload: reply payload
        :return Any: (msg_type, payload), or a BMCUError for a rejection
        """
        if msg_type == proto.MSG_ACK:
            try:
                _echo, ok, error = proto.parse_ack(payload)
            except ValueError as exc:
                error_str = f"BMCU sent a bad ACK: {exc}"
                return BMCUError(error_str)
            if not ok:
                error_str = f"BMCU rejected the command: error {error}"
                return BMCUError(error_str)
        elif msg_type == proto.MSG_ERROR:
            try:
                channel, code, details = proto.parse_error(payload)
            except ValueError as exc:
                error_str = f"BMCU sent a bad ERROR: {exc}"
                return BMCUError(error_str)
            error_str = f"BMCU error code={code} channel={channel} details={details}"
            rejected = BMCUError(error_str)
            rejected.code = code
            return rejected
        return (msg_type, payload)

    # Transmit

    def _next_seq(self) -> int:
        """
        Next wire sequence number; resends take a new one.

        :return int: the seq
        """
        value = self._seq
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        return value

    def _queue_frame(self, msg_type: int, cmd_id: int, payload: bytes, track: bool) -> None:
        """
        Encode a frame, queue it and write what the gate allows.

        :param msg_type: message id
        :param cmd_id: wire cmd id
        :param payload: payload
        :param track: a new request (apply supersede rules, track for resend);
            False for a resend
        """
        if track:
            self.policy.on_queued(msg_type, cmd_id, payload)
        frame = proto.encode_packet(msg_type, self._next_seq(), cmd_id, payload)
        self._tx.append((cmd_id, frame))
        now = self._reactor.monotonic()
        self._reactor.update_timer(self._write_timer, self._flush_tx(now))

    def _flush_tx(self, now: float) -> float:
        """
        Write queued frames, one per write(), while the write gate is open.
        Also the write timer, retrying a write the gate held.

        :param now: reactor time
        :return float: when to try again, or NEVER when the queue is empty
        """
        while self._tx:
            blocked = self.policy.write_blocked_until(now)
            if blocked is not None:
                return blocked
            cmd_id, frame = self._tx.popleft()
            try:
                self._serial.write(frame)
            except Exception as exc:
                self._link_lost(f"write failed: {exc}")
                break
            self._last_tx = now
            self.policy.on_written(cmd_id, now)
        self._arm_resend_timer()
        return self._reactor.NEVER

    def _arm_resend_timer(self) -> None:
        """
        Point the resend timer at the earliest armed resend.
        """
        due = self.policy.next_resend_due()
        waketime = due if due is not None else self._reactor.NEVER
        self._reactor.update_timer(self._resend_timer, waketime)

    def _resend_tick(self, eventtime: float) -> float:
        """
        Resend timer: send again every due request still awaited, same cmd id,
        new seq.

        :param eventtime: reactor time
        :return float: next waketime
        """
        for msg_type, cmd_id, payload in self.policy.due_resends(eventtime):
            if cmd_id in self._pending:
                self._queue_frame(msg_type, cmd_id, payload, track=False)
            else:
                self.policy.forget(cmd_id)
        due = self.policy.next_resend_due()
        return due if due is not None else self._reactor.NEVER

    # Receive

    def _handle_read(self, eventtime: float) -> None:
        """
        fd callback: read, decode, and dispatch every complete frame.

        :param eventtime: reactor time
        """
        try:
            data = self._serial.read(4096)
        except Exception as exc:
            self._link_lost(f"read failed: {exc}")
            return
        if not data:
            return
        errors = self._decoder.errors
        packets = self._decoder.feed(data)
        discarded = self._decoder.errors - errors
        frame_ended = (bool(packets)
                       or discarded > 0)
        self.policy.on_rx_bytes(eventtime, self._decoder.pending, frame_ended)
        if self._decoder.incompatible_version is not None:
            message = _version_message(self._decoder.incompatible_version)
            self._closed = True
            self._link_lost(message, BMCUVersionError)
            return
        if discarded:
            self.logger.debug(f"BMCU discarded {discarded} malformed frame(s)")
        for msg_type, _seq, cmd_id, payload in packets:
            self._handle_packet(msg_type, cmd_id, payload, eventtime)
            if self._serial is None:
                return

    def _handle_packet(self, msg_type: int, cmd_id: int, payload: bytes,
                       eventtime: float) -> None:
        """
        Route one decoded frame from the firmware.

        :param msg_type: message id
        :param cmd_id: echoed cmd id, 0 for unsolicited
        :param payload: payload
        :param eventtime: reactor time
        """
        if not msg_type & proto.REPLY_BIT:
            # Firmware frames all carry the reply bit; anything else is our own
            # TX echoed by a half-duplex adapter that does not gate its receiver.
            return
        self._last_rx = eventtime
        if cmd_id == 0:
            if msg_type == proto.MSG_STATUS:
                self._handle_status(payload, unsolicited=True)
            else:
                self.logger.debug(f"BMCU event 0x{msg_type:02X} ignored")
            return
        if not self.policy.on_reply(cmd_id, eventtime):
            return
        if msg_type == proto.MSG_STATUS:
            self._handle_status(payload, unsolicited=False)
        elif msg_type == proto.MSG_OP_RESULT:
            self._record_op_result(payload, eventtime)
        result = self._reply_result(msg_type, payload)
        completion = self._pending.get(cmd_id)
        if completion is not None:
            completion.complete(result)
        # Checked for every cmd id: the heartbeat PING has no waiter, and a
        # session the firmware dropped while idle is only visible there.
        if getattr(result, "code", None) == proto.ERR_SESSION:
            self._link_lost("BMCU session expired (firmware restarted)")

    def _record_op_result(self, payload: bytes, eventtime: float) -> None:
        """
        Store an OP_RESULT for wait_op and wake its waiter.

        :param payload: OP_RESULT payload
        :param eventtime: reactor time
        """
        try:
            result = proto.parse_op_result(payload)
        except ValueError as exc:
            self.logger.debug(f"BMCU discarded invalid OP_RESULT: {exc}")
            return
        op_id = result["op_id"]
        if op_id not in self._open_ops:
            return
        self._open_ops.discard(op_id)
        self.policy.mark_answered(op_id, eventtime)
        self._op_results[op_id] = result
        waiter = self._op_waiters.get(op_id)
        if waiter is not None:
            waiter.complete(True)

    def _handle_status(self, payload: bytes, unsolicited: bool) -> None:
        """
        Parse a STATUS, show it to the resend policy, and pass it to on_status.
        A STATUS from another session means the firmware restarted, so the
        link is treated as lost.

        :param payload: STATUS payload
        :param unsolicited: sent on the firmware's own (cmd_id 0), not a reply
        """
        try:
            status = proto.parse_status(payload)
        except ValueError as exc:
            if not self._connected:
                return
            # Once per connection: a firmware newer than this parser fails every
            # STATUS, and lanes would stop updating with nothing above debug.
            if not self._status_reject_warned:
                self._status_reject_warned = True
                self.logger.warning(f"BMCU sent a STATUS this AFC cannot read ({exc}); "
                                    "lane states will not update. Check the firmware version.")
            else:
                self.logger.debug(f"BMCU discarded invalid STATUS: {exc}")
            return
        if unsolicited:
            self.policy.on_unsolicited_status(status)
        else:
            self.policy.track_status(status)
        if not self._connected:
            return
        if status["session_id"] != self._session_id:
            self._link_lost("BMCU session changed (firmware restarted)")
            return
        self._notify("status", self._on_status, status)

    def _notify(self, what: str, callback: Callable[[Any], None], value: Any) -> None:
        """
        Call an owner callback, logging instead of raising into the reactor.

        :param what: callback name for the log
        :param callback: the callback
        :param value: its argument
        """
        try:
            callback(value)
        except Exception as exc:
            self.logger.error(f"BMCU {what} callback failed: {exc}")

    # Connection lifecycle

    def _establish(self) -> None:
        """
        Open the port and handshake. Tears down and raises on failure.
        """
        try:
            self._open()
            self._handshake()
        except BMCUError as exc:
            self.last_error = str(exc)
            self._teardown(str(exc))
            raise
        if not self._backoff_held:
            self._backoff = RECONNECT_MIN
        self.last_error = ""

    def _open(self) -> None:
        """
        Open the serial port non-blocking and register it with the reactor.
        """
        try:
            import serial
        except ImportError:
            error_str = "pyserial is not installed in Klipper's Python environment"
            raise BMCUError(error_str)
        connection = serial.Serial()
        connection.port = self.port
        connection.baudrate = self.baud
        connection.timeout = 0
        connection.write_timeout = WRITE_TIMEOUT
        connection.rtscts = False
        connection.dsrdtr = False
        connection.exclusive = True
        connection.dtr = True
        connection.rts = False
        try:
            connection.open()
        except Exception as exc:
            error_str = f"BMCU could not open {self.port}: {exc}"
            raise BMCUError(error_str)
        self._serial = connection
        if self.half_duplex:
            reason = request_low_latency(connection.fileno())
            if reason is None:
                self.logger.debug("BMCU serial low-latency on")
            else:
                self.logger.debug(f"BMCU serial low-latency not supported: {reason}")
        self._decoder = proto.StreamDecoder()
        self._seq = 1
        self.policy.reset()
        now = self._reactor.monotonic()
        self._last_rx = now
        self._last_tx = now
        self._fd_handle = self._reactor.register_fd(connection.fileno(), self._handle_read)

    def _handshake(self) -> None:
        """
        HELLO until answered, protocol check, SESSION_CONFIRM, GET_CAPS.
        """
        start = self._reactor.monotonic()
        self._reactor.pause(start + CONNECT_SETTLE)
        deadline = self._reactor.monotonic() + HANDSHAKE_TIMEOUT
        payload = None
        while payload is None:
            if self._reactor.monotonic() >= deadline:
                error_str = f"BMCU on {self.port} did not answer HELLO"
                raise BMCUTimeoutError(error_str)
            nonce = self._rng.getrandbits(32)
            with contextlib.suppress(BMCUTimeoutError):
                payload = self._request_raw(proto.MSG_HELLO, proto.pack_u32(nonce),
                                            HELLO_RETRY)[1]
        try:
            hello = proto.parse_hello(payload)
        except ValueError as exc:
            error_str = f"BMCU sent a bad HELLO reply: {exc}"
            raise BMCUError(error_str)
        if hello["protocol"] != proto.PROTO_VERSION:
            self._closed = True
            raise BMCUVersionError(_version_message(hello["protocol"], hello["firmware"]))
        if hello["channels"] != 4:
            error_str = f"BMCU reports {hello['channels']} channels; AFC_BMCU needs 4"
            raise BMCUError(error_str)
        self._hello = hello
        self._session_id = hello["session_id"]
        self._request_raw(proto.MSG_SESSION_CONFIRM, proto.pack_u32(self._session_id),
                          STOP_TIMEOUT)
        reply = self._request_raw(proto.MSG_GET_CAPS, b"", STOP_TIMEOUT)[1]
        try:
            self._caps = proto.parse_caps(reply)
        except ValueError as exc:
            error_str = f"BMCU sent a bad CAPS reply: {exc}"
            raise BMCUError(error_str)
        # Update mode survives HELLO and refuses runtime commands, so an
        # interrupted flash from another tool would otherwise block RUNTIME_SYNC.
        self._request_raw(proto.MSG_UPDATE_CANCEL, b"", STOP_TIMEOUT)
        self._connected = True
        self._status_reject_warned = False
        self._generation += 1
        now = self._reactor.monotonic()
        self._reactor.update_timer(self._supervise_timer, now + SUPERVISE_INTERVAL)
        self.logger.info(f"BMCU connected on {self.port}: firmware {hello['firmware']}, "
                         f"protocol {hello['protocol']}")

    def _teardown(self, reason: str, error_cls: type = BMCUError) -> None:
        """
        Close the port and fail everything waiting on it.

        :param reason: message for the waiting requests' errors
        :param error_cls: BMCUError or a subclass to raise in them
        """
        self._connected = False
        if self._fd_handle is not None:
            self._reactor.unregister_fd(self._fd_handle)
            self._fd_handle = None
        if self._serial is not None:
            with contextlib.suppress(Exception):
                self._serial.close()
            self._serial = None
        self._tx.clear()
        never = self._reactor.NEVER
        for timer in (self._write_timer, self._resend_timer, self._supervise_timer):
            self._reactor.update_timer(timer, never)
        self.policy.reset()
        error = error_cls(reason)
        waiting = list(self._pending.values()) + list(self._op_waiters.values())
        self._pending.clear()
        self._op_waiters.clear()
        self._open_ops.clear()
        self._op_results.clear()
        for completion in waiting:
            completion.complete(error)

    def _link_lost(self, reason: str, error_cls: type = BMCUError) -> None:
        """
        Handle a dropped link: fail waiters, tell the owner, and reconnect
        unless closed. A link that had not finished its handshake is left to
        the code establishing it.

        :param reason: why the link dropped
        :param error_cls: error class for the waiting requests
        """
        was_connected = self._connected
        self.last_error = reason
        self._teardown(reason, error_cls)
        if not was_connected:
            return
        self.logger.warning(f"BMCU link lost on {self.port}: {reason}")
        self._notify("link", self._on_link, False)
        if not self._closed:
            self._schedule_reconnect()

    def _take_backoff(self) -> float:
        """
        Current reconnect delay; doubles the next one up to RECONNECT_MAX.

        :return float: seconds to wait before the next attempt
        """
        delay = self._backoff
        self._backoff = min(self._backoff * 2.0, RECONNECT_MAX)
        return delay

    def _schedule_reconnect(self) -> None:
        """
        Arm the reconnect timer after the current backoff.
        """
        delay = self._take_backoff()
        self.logger.info(f"BMCU reconnecting to {self.port} in {delay:g}s")
        self._reactor.update_timer(self._reconnect_timer, self._reactor.monotonic() + delay)

    def _reconnect_tick(self, eventtime: float) -> float:
        """
        Reconnect timer: try once, then back off.

        :param eventtime: reactor time
        :return float: next waketime
        """
        if (self._closed
                or self._connected):
            return self._reactor.NEVER
        try:
            self._establish()
        except BMCUVersionError as exc:
            self.logger.error(str(exc))
            return self._reactor.NEVER
        except BMCUError as exc:
            self.logger.debug(f"BMCU reconnect failed: {exc}")
            return self._reactor.monotonic() + self._take_backoff()
        self._notify("link", self._on_link, True)
        return self._reactor.NEVER

    def _supervise_tick(self, eventtime: float) -> float:
        """
        Supervision timer: declare the link lost after LINK_TIMEOUT of silence,
        and PING when nothing has been sent for HEARTBEAT_INTERVAL.

        :param eventtime: reactor time
        :return float: next waketime
        """
        if not self._connected:
            return self._reactor.NEVER
        if eventtime - self._last_rx > LINK_TIMEOUT:
            self._link_lost(f"nothing received for {LINK_TIMEOUT:g}s")
            return self._reactor.NEVER
        if eventtime - self._last_tx >= HEARTBEAT_INTERVAL:
            self._queue_frame(proto.MSG_PING, self._new_cmd_id(), b"B", track=True)
        return eventtime + SUPERVISE_INTERVAL if self._connected else self._reactor.NEVER
