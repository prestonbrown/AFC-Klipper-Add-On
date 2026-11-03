"""
Unit tests for extras/AFC_BMCU_link.py
"""

from __future__ import annotations

import random
import struct
import sys

import pytest

from extras import AFC_BMCU_link as link
from extras import AFC_BMCU_proto as proto
from extras.AFC_BMCU_link import (
    BMCUConnection, BMCUError, BMCULinkPolicy, BMCUTimeoutError, BMCUVersionError,
)
from tests import bmcu_fake
from tests.bmcu_fake import FakeBMCU, FakeLogger, FakeReactor


def _policy(half_duplex=True, seed=7):
    logger = FakeLogger()
    return BMCULinkPolicy(half_duplex, random.Random(seed), logger), logger


def _written(policy, msg_type, cmd_id, payload=b"", now=10.0):
    policy.on_queued(msg_type, cmd_id, payload)
    policy.on_written(cmd_id, now)


def _status(motion=(0, 0, 0, 0), error_flags=0):
    return proto.parse_status(bmcu_fake.status_payload(motion=list(motion),
                                                       error_flags=error_flags))


# BMCULinkPolicy.__init__

class TestBMCULinkPolicyInit:
    def test_starts_with_nothing_tracked(self):
        policy, logger = _policy()
        assert policy.half_duplex is True
        assert policy.logger is logger
        assert policy.pending_resends == {}
        assert policy.answered == {}
        assert policy.last_rx == float("-inf")
        assert policy.partial_since is None


# pending_resends

class TestPendingResends:
    def test_reports_type_sends_and_due(self):
        policy, _ = _policy()
        policy.on_queued(proto.MSG_SET_MOTION, 5, b"\x00\x03")
        assert policy.pending_resends == {5: (proto.MSG_SET_MOTION, 1, None)}

    def test_is_a_copy(self):
        policy, _ = _policy()
        policy.on_queued(proto.MSG_SET_MOTION, 5, b"")
        policy.pending_resends.clear()
        assert list(policy.pending_resends) == [5]


# on_queued

class TestOnQueued:
    def test_full_duplex_tracks_nothing(self):
        policy, logger = _policy(half_duplex=False)
        policy.on_queued(proto.MSG_SET_MOTION, 5, b"\x00\x03")
        assert policy.pending_resends == {}
        assert logger.messages == []

    def test_tracks_state_change(self):
        policy, _ = _policy()
        policy.on_queued(proto.MSG_FEED_TO_CONTACT, 9, b"abc")
        assert policy.pending_resends == {9: (proto.MSG_FEED_TO_CONTACT, 1, None)}

    def test_tracks_passive_read(self):
        policy, _ = _policy()
        policy.on_queued(proto.MSG_GET_STATUS, 9, b"")
        assert policy.pending_resends == {9: (proto.MSG_GET_STATUS, 1, None)}

    @pytest.mark.parametrize("msg_type", [proto.MSG_HELLO, proto.MSG_PING])
    def test_never_tracks_untracked_types(self, msg_type):
        policy, _ = _policy()
        policy.on_queued(msg_type, 9, b"\x00")
        assert policy.pending_resends == {}

    def test_stop_drops_every_pending_resend(self):
        policy, logger = _policy()
        policy.on_queued(proto.MSG_SET_MOTION, 1, b"")
        policy.on_queued(proto.MSG_GET_STATUS, 2, b"")
        policy.on_queued(proto.MSG_ABORT_OP, 3, b"")
        policy.on_queued(proto.MSG_STOP_ALL, 4, b"")
        assert list(policy.pending_resends) == [4]
        assert logger.messages == [
            ("debug", "BMCU dropped resend of 0x10 cmd=1: superseded by 0x16"),
            ("debug", "BMCU dropped resend of 0x03 cmd=2: superseded by 0x16"),
            ("debug", "BMCU dropped resend of 0x16 cmd=3: superseded by 0x50"),
        ]

    def test_update_cancel_is_a_stop(self):
        policy, _ = _policy()
        policy.on_queued(proto.MSG_ABORT_OP, 1, b"")
        policy.on_queued(proto.MSG_UPDATE_CANCEL, 2, b"")
        assert list(policy.pending_resends) == [2]

    def test_newer_state_change_drops_older_but_not_a_stop_or_read(self):
        policy, logger = _policy()
        policy.on_queued(proto.MSG_STOP_ALL, 1, b"")
        policy.on_queued(proto.MSG_GET_STATUS, 2, b"")
        policy.on_queued(proto.MSG_SET_MOTION, 3, b"")
        policy.on_queued(proto.MSG_FEED_DISTANCE, 4, b"")
        assert list(policy.pending_resends) == [1, 2, 4]
        assert logger.messages == [
            ("debug", "BMCU dropped resend of 0x10 cmd=3: superseded by 0x19"),
        ]

    @pytest.mark.parametrize("passive", [proto.MSG_GET_OP_RESULT, proto.MSG_SET_SLOTS])
    def test_passive_request_drops_nothing(self, passive):
        policy, logger = _policy()
        policy.on_queued(proto.MSG_SET_MOTION, 1, b"")
        policy.on_queued(passive, 2, b"")
        assert list(policy.pending_resends) == [1, 2]
        assert logger.messages == []


# on_written

class TestOnWritten:
    def test_arms_jittered_deadline(self):
        policy, _ = _policy(seed=11)
        _written(policy, proto.MSG_SET_MOTION, 5, now=10.0)
        expected = 10.0 + 0.25 * random.Random(11).uniform(0.8, 1.2)
        assert policy.pending_resends[5][2] == pytest.approx(expected)

    def test_does_not_rearm_an_armed_deadline(self):
        policy, _ = _policy()
        _written(policy, proto.MSG_SET_MOTION, 5, now=10.0)
        first = policy.pending_resends[5][2]
        policy.on_written(5, 20.0)
        assert policy.pending_resends[5][2] == first

    def test_untracked_id_is_ignored(self):
        policy, _ = _policy()
        policy.on_written(5, 10.0)
        assert policy.pending_resends == {}

    def test_full_duplex_is_noop(self):
        policy, _ = _policy(half_duplex=False)
        policy.on_written(5, 10.0)
        assert policy.pending_resends == {}


# on_reply

class TestOnReply:
    def test_clears_resend_and_accepts(self):
        policy, _ = _policy()
        _written(policy, proto.MSG_SET_MOTION, 5)
        assert policy.on_reply(5, 10.1) is True
        assert policy.pending_resends == {}

    def test_duplicate_of_answered_is_dropped(self):
        policy, logger = _policy()
        policy.mark_answered(5, 10.0)
        assert policy.on_reply(5, 14.9) is False
        assert logger.messages == [("debug", "BMCU dropped duplicate reply cmd=5")]

    def test_answered_memory_expires(self):
        policy, _ = _policy()
        policy.mark_answered(5, 10.0)
        assert policy.on_reply(5, 15.0) is True

    def test_full_duplex_always_accepts(self):
        policy, _ = _policy(half_duplex=False)
        policy.mark_answered(5, 10.0)
        assert policy.on_reply(5, 10.1) is True


# mark_answered

class TestMarkAnswered:
    def test_remembers_for_five_seconds(self):
        policy, _ = _policy()
        policy.mark_answered(5, 10.0)
        assert policy.answered == {5: 15.0}

    def test_purges_expired_entries(self):
        policy, _ = _policy()
        policy.mark_answered(5, 10.0)
        policy.mark_answered(6, 15.0)
        assert policy.answered == {6: 20.0}

    def test_keeps_live_entries(self):
        policy, _ = _policy()
        policy.mark_answered(5, 10.0)
        policy.mark_answered(6, 12.0)
        assert policy.answered == {5: 15.0, 6: 17.0}

    def test_full_duplex_remembers_nothing(self):
        policy, _ = _policy(half_duplex=False)
        policy.mark_answered(5, 10.0)
        assert policy.answered == {}


# forget

class TestForget:
    def test_drops_the_resend(self):
        policy, _ = _policy()
        policy.on_queued(proto.MSG_SET_MOTION, 5, b"")
        policy.on_queued(proto.MSG_GET_STATUS, 6, b"")
        policy.forget(5)
        assert list(policy.pending_resends) == [6]

    def test_unknown_id_is_ignored(self):
        policy, _ = _policy()
        policy.forget(5)
        assert policy.pending_resends == {}


# on_unsolicited_status

class TestOnUnsolicitedStatus:
    def test_status_with_channel_going_idle_drops(self):
        policy, logger = _policy()
        policy.on_queued(proto.MSG_STOP_ALL, 1, b"")
        policy.on_queued(proto.MSG_GET_STATUS, 2, b"")
        policy.on_unsolicited_status(_status(motion=(3, 0, 0, 0)))
        policy.on_queued(proto.MSG_SET_MOTION, 3, b"")
        policy.on_unsolicited_status(_status(motion=(0, 0, 0, 0)))
        assert list(policy.pending_resends) == [1, 2]
        assert logger.messages == [
            ("debug", "BMCU dropped resend of 0x10 cmd=3: superseded by 0x90"),
        ]

    def test_status_without_stop_keeps(self):
        policy, logger = _policy()
        policy.on_unsolicited_status(_status(motion=(0, 0, 0, 0)))
        policy.on_queued(proto.MSG_SET_MOTION, 3, b"")
        policy.on_unsolicited_status(_status(motion=(1, 0, 0, 0)))
        assert list(policy.pending_resends) == [3]
        assert logger.messages == []


# track_status

class TestTrackStatus:
    def test_first_status_only_seeds(self):
        policy, _ = _policy()
        assert policy.track_status(_status(motion=(0, 0, 0, 0))) is False

    @pytest.mark.parametrize("before, after, stopped", [
        ({"motion": (0, 0, 5, 0)}, {"motion": (0, 0, 0, 0)}, True),
        ({"motion": (0, 0, 0, 0)}, {"motion": (0, 0, 0, 0)}, False),
        ({"motion": (2, 0, 0, 0)}, {"motion": (3, 0, 0, 0)}, False),
        ({"error_flags": 0x0002}, {"error_flags": 0x0003}, True),
        # Route-uncertain (bit 1) rising is not a fault.
        ({"error_flags": 0x0001}, {"error_flags": 0x0003}, False),
        ({"error_flags": 0x0003}, {"error_flags": 0x0001}, False),
    ])
    def test_stop_is_a_channel_going_idle_or_a_new_fault(self, before, after, stopped):
        policy, _ = _policy()
        policy.track_status(_status(**before))
        assert policy.track_status(_status(**after)) is stopped

    def test_full_duplex_tracks_nothing(self):
        policy, _ = _policy(half_duplex=False)
        policy.track_status(_status(motion=(3, 0, 0, 0)))
        assert policy.track_status(_status(motion=(0, 0, 0, 0))) is False


# due_resends

class TestDueResends:
    def test_not_due_before_deadline(self):
        policy, _ = _policy()
        _written(policy, proto.MSG_SET_MOTION, 5, now=10.0)
        assert policy.due_resends(10.19) == []

    def test_unwritten_entry_is_never_due(self):
        policy, _ = _policy()
        policy.on_queued(proto.MSG_SET_MOTION, 5, b"")
        assert policy.due_resends(1000.0) == []

    def test_due_entry_is_returned_and_disarmed(self):
        policy, logger = _policy()
        _written(policy, proto.MSG_SET_MOTION, 5, b"\x01\x03", now=10.0)
        assert policy.due_resends(10.31) == [(proto.MSG_SET_MOTION, 5, b"\x01\x03")]
        assert policy.pending_resends == {5: (proto.MSG_SET_MOTION, 2, None)}
        assert logger.messages == [("debug", "BMCU resent 0x10 cmd=5 attempt 2")]

    def test_three_sends_then_gives_up(self):
        policy, logger = _policy()
        _written(policy, proto.MSG_GET_STATUS, 5, now=0.0)
        sent = []
        for now in (1.0, 2.0, 3.0):
            resends = policy.due_resends(now)
            sent.extend(resends)
            for _msg_type, cmd_id, _payload in resends:
                policy.on_written(cmd_id, now)
        assert sent == [(proto.MSG_GET_STATUS, 5, b"")] * 2
        assert policy.pending_resends == {}
        assert logger.messages == [
            ("debug", "BMCU resent 0x03 cmd=5 attempt 2"),
            ("debug", "BMCU resent 0x03 cmd=5 attempt 3"),
            ("warning", "BMCU no reply to 0x03 cmd=5 after 3 sends"),
        ]

    def test_full_duplex_never_resends(self):
        policy, _ = _policy(half_duplex=False)
        _written(policy, proto.MSG_GET_STATUS, 5, now=0.0)
        assert policy.due_resends(100.0) == []


# next_resend_due

class TestNextResendDue:
    def test_none_without_armed_entries(self):
        policy, _ = _policy()
        policy.on_queued(proto.MSG_GET_STATUS, 5, b"")
        assert policy.next_resend_due() is None

    def test_earliest_armed_deadline(self):
        policy, _ = _policy()
        _written(policy, proto.MSG_GET_STATUS, 5, now=20.0)
        _written(policy, proto.MSG_GET_STATUS, 6, now=10.0)
        assert policy.next_resend_due() == policy.pending_resends[6][2]


# on_rx_bytes

class TestOnRxBytes:
    def test_records_last_byte_time(self):
        policy, _ = _policy()
        policy.on_rx_bytes(10.0, False)
        assert policy.last_rx == 10.0
        assert policy.partial_since is None

    def test_new_partial_frame_starts_its_clock(self):
        policy, _ = _policy()
        policy.on_rx_bytes(10.0, True)
        assert policy.partial_since == 10.0

    def test_continuing_frame_keeps_its_start(self):
        policy, _ = _policy()
        policy.on_rx_bytes(10.0, True)
        policy.on_rx_bytes(10.02, True)
        assert policy.partial_since == 10.0
        assert policy.last_rx == 10.02

    def test_frame_end_then_new_partial_restarts_clock(self):
        policy, _ = _policy()
        policy.on_rx_bytes(10.0, True)
        policy.on_rx_bytes(10.02, True, frame_ended=True)
        assert policy.partial_since == 10.02

    def test_frame_end_without_partial_clears(self):
        policy, _ = _policy()
        policy.on_rx_bytes(10.0, True)
        policy.on_rx_bytes(10.02, False, frame_ended=True)
        assert policy.partial_since is None

    def test_full_duplex_records_nothing(self):
        policy, _ = _policy(half_duplex=False)
        policy.on_rx_bytes(10.0, True)
        assert policy.partial_since is None
        assert policy.last_rx == float("-inf")


# write_blocked_until

class TestWriteBlockedUntil:
    def test_open_with_no_rx(self):
        policy, _ = _policy()
        assert policy.write_blocked_until(10.0) is None

    def test_quiet_gap_after_last_byte(self):
        policy, _ = _policy()
        policy.on_rx_bytes(10.0, False)
        assert policy.write_blocked_until(10.001) == pytest.approx(10.002)
        assert policy.write_blocked_until(10.002) is None

    def test_partial_frame_holds_until_bound(self):
        policy, _ = _policy()
        policy.on_rx_bytes(10.0, True)
        assert policy.write_blocked_until(10.01) == pytest.approx(10.05)
        assert policy.write_blocked_until(10.049) == pytest.approx(10.05)

    def test_partial_frame_gate_releases_after_bound(self):
        policy, _ = _policy()
        policy.on_rx_bytes(10.0, True)
        assert policy.write_blocked_until(10.05) is None

    def test_quiet_gap_still_applies_after_partial_bound(self):
        policy, _ = _policy()
        policy.on_rx_bytes(10.0, True)
        policy.on_rx_bytes(10.06, True)
        assert policy.write_blocked_until(10.061) == pytest.approx(10.062)

    def test_full_duplex_never_blocks(self):
        policy, _ = _policy(half_duplex=False)
        policy.on_rx_bytes(10.0, True)
        assert policy.write_blocked_until(10.0) is None


# reset

class TestReset:
    def test_forgets_everything(self):
        policy, _ = _policy()
        _written(policy, proto.MSG_SET_MOTION, 5)
        policy.mark_answered(6, 10.0)
        policy.on_rx_bytes(10.0, True)
        policy.track_status(_status(motion=(3, 0, 0, 0)))
        policy.reset()
        assert policy.pending_resends == {}
        assert policy.answered == {}
        assert policy.last_rx == float("-inf")
        assert policy.partial_since is None
        assert policy.write_blocked_until(10.0) is None
        assert policy.track_status(_status(motion=(0, 0, 0, 0))) is False


# BMCUConnection

PORT = "/dev/serial/by-id/usb-bmcu"
NEVER = FakeReactor.NEVER
CONNECTED_INFO = ("info", f"BMCU connected on {PORT}: firmware 1.0.4, protocol 1")
UNPLUGGED = "read failed: device reports readiness to read but returned no data"


# AFC_BMCU's own runtime values; every one inside the firmware's accepted range.
RUNTIME_SYNC = proto.pack_runtime_sync(
    {"load_pressure_pct": 82, "load_speed": 60.0, "pull_speed": 60.0,
     "pull_speed_end": 12.0, "jam_timeout_ms": 20000.0, "before_pullback_pct": 40.0,
     "retract_mm": 1000.0, "autoload_mm": 120.0}, (255, 255, 255))


class Rig:
    """A FakeReactor, a FakeBMCU behind the serial module, and a connection."""

    def __init__(self, monkeypatch, half_duplex=False, seed=1, **bmcu_kwargs):
        self.reactor = FakeReactor()
        self.bmcu = FakeBMCU(self.reactor, **bmcu_kwargs)
        # A spool at every inlet.
        for channel in self.bmcu.channels:
            channel.present = 1
        monkeypatch.setitem(sys.modules, "serial", self.bmcu.serial_module())
        monkeypatch.setattr(link, "_serial_ioctl", self._ioctl)
        self.ioctl_calls = []
        self.logger = FakeLogger()
        self.statuses = []
        self.links = []
        self.conn = BMCUConnection(self.reactor, PORT, 115200, half_duplex, self.logger,
                                   self.statuses.append, self.links.append,
                                   rng=random.Random(seed))

    def _ioctl(self, fd, request, buffer, mutate):
        self.ioctl_calls.append(request)
        raise OSError(25, "Inappropriate ioctl for device")

    def connect(self):
        """Handshake, then RUNTIME_SYNC, which the firmware needs before it drives
        a motor."""
        self.conn.connect()
        self.conn.request(proto.MSG_RUNTIME_SYNC, RUNTIME_SYNC)
        self.logger.messages.clear()
        self.bmcu.received.clear()
        return self

    def later(self, delay, fn):
        """Run fn() from a reactor timer, delay seconds from now."""
        def _tick(eventtime):
            self.reactor.unregister_timer(timer)
            fn()
            return NEVER
        timer = self.reactor.register_timer(_tick, self.reactor.now + delay)

    def last_cmd(self, msg_type):
        return self.bmcu.frames(msg_type)[-1].cmd_id


@pytest.fixture
def make_rig(monkeypatch):
    return lambda **kwargs: Rig(monkeypatch, **kwargs)


def _types(frames):
    return [frame.msg_type for frame in frames]


# request_low_latency

class TestRequestLowLatency:
    def test_sets_only_the_flag_bit(self, monkeypatch):
        calls = []

        def ioctl(fd, request, buffer, mutate):
            if request == link._TIOCGSERIAL:
                struct.pack_into("@i", buffer, link._SERIAL_FLAGS_OFFSET, 0x0040)
            calls.append((request, struct.unpack_from("@i", buffer,
                                                      link._SERIAL_FLAGS_OFFSET)[0]))

        monkeypatch.setattr(link, "_serial_ioctl", ioctl)
        assert link.request_low_latency(5) is None
        assert calls == [(link._TIOCGSERIAL, 0x0040), (link._TIOCSSERIAL, 0x2040)]

    def test_already_set_writes_nothing(self, monkeypatch):
        calls = []

        def ioctl(fd, request, buffer, mutate):
            struct.pack_into("@i", buffer, link._SERIAL_FLAGS_OFFSET, 0x2000)
            calls.append(request)

        monkeypatch.setattr(link, "_serial_ioctl", ioctl)
        assert link.request_low_latency(5) is None
        assert calls == [link._TIOCGSERIAL]

    @pytest.mark.parametrize("error, reason", [
        (OSError(25, "Inappropriate ioctl for device"), "Inappropriate ioctl for device"),
        (OSError("no serial struct"), "no serial struct"),
    ])
    def test_refusal_reports_why(self, monkeypatch, error, reason):
        def ioctl(fd, request, buffer, mutate):
            raise error

        monkeypatch.setattr(link, "_serial_ioctl", ioctl)
        assert link.request_low_latency(5) == reason

    def test_serial_struct_layout_matches_kernel(self):
        assert link._SERIAL_FLAGS_OFFSET == 16
        assert link._SERIAL_STRUCT.size in (60, 72)


# _version_message

class TestVersionMessage:
    def test_names_firmware_and_supported_set(self):
        assert link._version_message(2, "2.0.0") == (
            "BMCU firmware 2.0.0 speaks protocol 2; AFC_BMCU supports protocol 1. "
            "Flash a supported BMCU-Klipper firmware.")

    def test_without_firmware(self):
        assert link._version_message(3) == (
            "BMCU firmware speaks protocol 3; AFC_BMCU supports protocol 1. "
            "Flash a supported BMCU-Klipper firmware.")


# __init__

class TestBMCUConnectionInit:
    def test_state_before_connect(self, make_rig):
        rig = make_rig(half_duplex=True)
        conn = rig.conn
        assert conn.connected is False
        assert conn.port == PORT
        assert conn.baud == 115200
        assert conn.policy.half_duplex is True
        assert conn.last_error == ""
        assert [t.waketime for t in rig.reactor.timers] == [NEVER] * 4
        assert rig.bmcu.open_times == []

    def test_cmd_ids_start_from_the_rng(self, make_rig):
        rig = make_rig(seed=5)
        expected = random.Random(5).randrange(1, 0x7FFFFFFF)
        assert rig.conn._new_cmd_id() == expected

    def test_default_rng_is_used_without_one(self, monkeypatch):
        expected = random.Random(9).randrange(1, 0x7FFFFFFF)
        seeded = random.Random
        monkeypatch.setattr(link.random, "Random", lambda: seeded(9))
        conn = BMCUConnection(FakeReactor(), PORT, 115200, False, FakeLogger(), print, print)
        assert conn._new_cmd_id() == expected


# properties

class TestConnected:
    def test_true_after_handshake_false_after_close(self, make_rig):
        rig = make_rig().connect()
        assert rig.conn.connected is True
        rig.conn.close()
        assert rig.conn.connected is False


class TestCaps:
    def test_zero_before_handshake(self, make_rig):
        assert make_rig().conn.caps == 0

    def test_from_caps_reply(self, make_rig):
        rig = make_rig(capabilities=proto.CAP_TAIL_RETRACT | proto.CAP_VOLATILE_SLOTS)
        rig.connect()
        assert rig.conn.caps == proto.CAP_TAIL_RETRACT | proto.CAP_VOLATILE_SLOTS


class TestHello:
    def test_none_before_handshake(self, make_rig):
        assert make_rig().conn.hello is None

    def test_copy_after_handshake(self, make_rig):
        rig = make_rig(firmware=(1, 0, 4)).connect()
        hello = rig.conn.hello
        assert hello["firmware"] == "1.0.4"
        assert hello["session_id"] == rig.bmcu.session_id
        hello["firmware"] = "x"
        assert rig.conn.hello["firmware"] == "1.0.4"


# connect

class TestConnect:
    def test_handshake_order(self, make_rig):
        rig = make_rig()
        rig.conn.connect()
        assert _types(rig.bmcu.received) == [
            proto.MSG_HELLO, proto.MSG_SESSION_CONFIRM, proto.MSG_GET_CAPS,
            proto.MSG_UPDATE_CANCEL]
        confirm = rig.bmcu.frames(proto.MSG_SESSION_CONFIRM)[0]
        assert confirm.payload == struct.pack("<I", rig.bmcu.session_id)
        assert rig.bmcu.open_times == [100.0]
        assert rig.bmcu.received[0].time == pytest.approx(101.5)
        assert rig.bmcu.serial.exclusive is True
        assert rig.bmcu.serial.rts is False
        assert rig.bmcu.serial.timeout == 0
        assert rig.logger.messages == [
            ("info", f"BMCU connected on {PORT}: firmware 1.0.4, protocol 1")]
        assert rig.links == []

    def test_cancels_update_mode_left_by_another_tool(self, make_rig):
        rig = make_rig()
        rig.bmcu.update_mode = True
        rig.conn.connect()
        assert rig.bmcu.update_mode is False
        rig.conn.request(proto.MSG_RUNTIME_SYNC, RUNTIME_SYNC)

    def test_drop_before_caps_is_not_reported_as_a_lost_link(self, make_rig):
        rig = make_rig()
        dispatch = rig.bmcu._dispatch

        def caps_session_error(msg_type, seq, cmd_id, payload):
            if msg_type == proto.MSG_GET_CAPS:
                return [bmcu_fake._error(cmd_id, proto.ERR_SESSION)], ""
            return dispatch(msg_type, seq, cmd_id, payload)

        rig.bmcu._dispatch = caps_session_error
        with pytest.raises(BMCUError):
            rig.conn.connect()
        assert rig.conn.connected is False
        assert rig.links == []

    def test_echo_of_our_own_request_is_not_its_reply(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.echo = True
        rig.bmcu.channels[0].motion = 0
        with pytest.raises(BMCUError) as info:
            rig.conn.request(proto.MSG_SET_MOTION, b"\x00\x09")
        assert info.value.code == 3

    def test_already_connected_does_nothing(self, make_rig):
        rig = make_rig().connect()
        rig.conn.connect()
        assert rig.bmcu.received == []
        assert rig.bmcu.open_times == [100.0]

    def test_protocol_mismatch_raises(self, make_rig):
        rig = make_rig(protocol=2, firmware=(2, 0, 0))
        with pytest.raises(BMCUVersionError) as info:
            rig.conn.connect()
        assert str(info.value) == link._version_message(2, "2.0.0")
        assert rig.conn.connected is False
        rig.reactor.advance(120.0)
        assert rig.bmcu.open_times == [100.0]
        assert rig.conn.last_error == str(info.value)

    def test_header_version_mismatch_raises(self, make_rig):
        rig = make_rig(header_version=2)
        with pytest.raises(BMCUVersionError) as info:
            rig.conn.connect()
        assert str(info.value) == link._version_message(2)
        rig.reactor.advance(120.0)
        assert rig.bmcu.open_times == [100.0]

    def test_open_failure_raises_and_schedules_reconnect(self, make_rig):
        rig = make_rig()
        rig.bmcu.unplugged = True
        with pytest.raises(BMCUError, match="could not open"):
            rig.conn.connect()
        assert rig.logger.messages == [("info", f"BMCU reconnecting to {PORT} in 1s")]
        rig.bmcu.replug()
        rig.reactor.advance(1.0)
        assert rig.conn.connected is True
        assert rig.links == [True]

    def test_dropped_hello_reply_is_retried(self, make_rig):
        rig = make_rig()
        rig.bmcu.add_fault("drop", proto.MSG_HELLO)
        rig.conn.connect()
        hellos = rig.bmcu.frames(proto.MSG_HELLO)
        assert len(hellos) == 2
        assert hellos[1].time - hellos[0].time == pytest.approx(0.6)
        assert hellos[0].cmd_id != hellos[1].cmd_id
        assert rig.conn.connected is True

    def test_silent_board_times_out(self, make_rig):
        rig = make_rig()
        rig.bmcu.silent = True
        with pytest.raises(BMCUTimeoutError, match="did not answer HELLO"):
            rig.conn.connect()
        hellos = rig.bmcu.frames(proto.MSG_HELLO)
        assert hellos[0].time == pytest.approx(101.5)
        assert hellos[-1].time < 101.5 + 15.0
        assert rig.reactor.now >= 101.5 + 15.0
        assert len(hellos) >= 25
        assert rig.conn.connected is False

    def test_other_failure_raises_and_schedules_reconnect(self, make_rig):
        rig = make_rig()
        rig.bmcu.channel_count = 2
        with pytest.raises(BMCUError, match="reports 2 channels; AFC_BMCU needs 4"):
            rig.conn.connect()
        assert rig.reactor.timers[3].waketime == pytest.approx(rig.reactor.now + 1.0)


# close

class TestClose:
    def test_fails_waiters_and_stops_reconnecting(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.add_fault("drop", proto.MSG_GET_STATUS)
        rig.later(0.1, rig.conn.close)
        with pytest.raises(BMCUError, match="BMCU connection closed"):
            rig.conn.request(proto.MSG_GET_STATUS)
        rig.reactor.advance(60.0)
        assert rig.conn.connected is False
        assert rig.bmcu.open_times == [100.0]
        assert rig.links == []
        assert rig.bmcu.serial is None


class TestGeneration:
    def test_zero_before_the_first_handshake(self, make_rig):
        assert make_rig().conn.generation == 0

    def test_counts_every_handshake(self, make_rig):
        rig = make_rig().connect()
        assert rig.conn.generation == 1
        rig.bmcu.unplug()
        assert rig.conn.generation == 1
        rig.bmcu.replug()
        rig.reactor.advance(3.0)
        assert rig.conn.connected is True
        assert rig.conn.generation == 2

    def test_failed_handshake_does_not_count(self, make_rig):
        rig = make_rig()
        rig.bmcu.silent = True
        with pytest.raises(BMCUError):
            rig.conn.connect()
        assert rig.conn.generation == 0


class TestStopAllNowait:
    def test_writes_stop_all_without_waiting(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.silent = True
        start = rig.reactor.now
        assert rig.conn.stop_all_nowait() is True
        assert rig.reactor.now == start
        assert _types(rig.bmcu.received) == [proto.MSG_STOP_ALL]
        assert rig.conn._pending == {}

    def test_stops_a_running_operation(self, make_rig):
        rig = make_rig().connect()
        op_id = rig.conn.start_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 50.0, 70, 3000))
        rig.conn.stop_all_nowait()
        assert rig.bmcu.running_ops() == []
        assert rig.conn.wait_op(op_id, 1.0)["state"] == proto.OP_STATE_ABORTED

    def test_bypasses_the_half_duplex_write_gate(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.conn.policy.on_rx_bytes(rig.reactor.now, True)
        assert rig.conn.policy.write_blocked_until(rig.reactor.now) is not None
        assert rig.conn.stop_all_nowait() is True
        assert _types(rig.bmcu.received) == [proto.MSG_STOP_ALL]

    def test_drops_pending_resends(self, make_rig):
        # A resend after the stop would carry a newer seq, and the firmware would run it.
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.add_fault("drop", proto.MSG_SET_MOTION)
        rig.later(0.05, rig.conn.stop_all_nowait)
        with pytest.raises(BMCUError):
            rig.conn.request(proto.MSG_SET_MOTION, proto.pack_set_motion(0, 3), timeout=1.0)
        types = _types(rig.bmcu.received)
        assert types.count(proto.MSG_SET_MOTION) == 1
        assert types.index(proto.MSG_STOP_ALL) > types.index(proto.MSG_SET_MOTION)
        assert rig.conn.policy.pending_resends == {}

    def test_nothing_open(self, make_rig):
        rig = make_rig()
        assert rig.conn.stop_all_nowait() is False
        assert rig.bmcu.received == []

    def test_write_error_is_swallowed(self, make_rig):
        rig = make_rig().connect()

        def bad_write(data):
            raise OSError("gone")

        rig.bmcu.serial.write = bad_write
        assert rig.conn.stop_all_nowait() is False
        assert rig.logger.messages == [("debug", "BMCU STOP_ALL write failed: gone")]


class TestDrop:
    def test_drops_an_established_link_and_reconnects(self, make_rig):
        rig = make_rig().connect()
        rig.conn.drop("RUNTIME_SYNC failed")
        assert rig.conn.connected is False
        assert rig.links == [False]
        assert rig.conn.last_error == "RUNTIME_SYNC failed"
        assert rig.logger.messages == [
            ("warning", f"BMCU link lost on {PORT}: RUNTIME_SYNC failed"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
        ]
        rig.reactor.advance(3.0)
        assert rig.conn.connected is True
        assert rig.links == [False, True]


class TestMarkReady:
    def test_after_ready_a_plain_drop_backs_off_from_the_start(self, make_rig):
        rig = make_rig().connect()
        rig.conn.drop("setup failed")
        rig.reactor.advance(1.5)
        rig.conn.mark_ready()
        rig.bmcu.unplug()
        rig.bmcu.replug()
        rig.reactor.advance(2.5)
        assert rig.conn.connected is True
        rig.logger.messages.clear()
        rig.bmcu.unplug()
        assert rig.logger.messages == [
            ("warning", f"BMCU link lost on {PORT}: {UNPLUGGED}"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
        ]

    def test_a_drop_holds_the_backoff_until_ready(self, make_rig):
        rig = make_rig().connect()
        rig.conn.drop("setup failed")
        rig.reactor.advance(1.5)
        assert rig.conn.connected is True
        assert rig.conn.reconnect_delay == 2.0
        rig.conn.drop("setup failed")
        assert rig.logger.messages == [
            ("warning", f"BMCU link lost on {PORT}: setup failed"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
            CONNECTED_INFO,
            ("warning", f"BMCU link lost on {PORT}: setup failed"),
            ("info", f"BMCU reconnecting to {PORT} in 2s"),
        ]
        rig.reactor.advance(2.5)
        assert rig.conn.reconnect_delay == 4.0
        rig.conn.mark_ready()
        assert rig.conn.reconnect_delay == 1.0

class TestOpOpen:
    def test_open_until_its_result(self, make_rig):
        rig = make_rig().connect()
        op_id = rig.conn.start_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 50.0, 70, 3000))
        assert rig.conn.op_open(op_id) is True
        rig.bmcu.complete_op(op_id)
        rig.reactor.advance(0)
        assert rig.conn.op_open(op_id) is False

    def test_unknown_op(self, make_rig):
        assert make_rig().connect().conn.op_open(12345) is False


# request

class TestRequest:
    def test_returns_reply(self, make_rig):
        rig = make_rig().connect()
        msg_type, payload = rig.conn.request(proto.MSG_PING, b"hi")
        assert (msg_type, payload) == (bmcu_fake.MSG_PONG, b"hi")

    def test_not_connected_raises_without_sending(self, make_rig):
        rig = make_rig()
        with pytest.raises(BMCUError, match=f"BMCU on {PORT} is not connected"):
            rig.conn.request(proto.MSG_GET_STATUS)
        assert rig.bmcu.received == []

    def test_timeout_raises(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.add_fault("drop", proto.MSG_GET_STATUS)
        start = rig.reactor.now
        with pytest.raises(BMCUTimeoutError) as info:
            rig.conn.request(proto.MSG_GET_STATUS, timeout=0.5)
        cmd_id = rig.last_cmd(proto.MSG_GET_STATUS)
        assert str(info.value) == f"BMCU request 0x03 cmd={cmd_id} got no reply within 0.5s"
        assert rig.reactor.now == pytest.approx(start + 0.5)

    def test_full_duplex_never_resends(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.add_fault("drop", proto.MSG_GET_STATUS)
        with pytest.raises(BMCUTimeoutError):
            rig.conn.request(proto.MSG_GET_STATUS, timeout=1.0)
        assert len(rig.bmcu.frames(proto.MSG_GET_STATUS)) == 1

    def test_half_duplex_resend_recovers_one_lost_reply(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.add_fault("drop", proto.MSG_SET_MOTION)
        msg_type, _ = rig.conn.request(proto.MSG_SET_MOTION, proto.pack_set_motion(0, 2))
        sends = rig.bmcu.frames(proto.MSG_SET_MOTION)
        assert msg_type == proto.MSG_ACK
        assert len(sends) == 2
        assert sends[0].cmd_id == sends[1].cmd_id
        assert sends[1].seq == sends[0].seq + 1
        assert 0.2 <= sends[1].time - sends[0].time <= 0.3

    def test_half_duplex_gives_up_after_three_sends(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.add_fault("drop", proto.MSG_GET_STATUS, count=-1)
        with pytest.raises(BMCUTimeoutError):
            rig.conn.request(proto.MSG_GET_STATUS)
        sends = rig.bmcu.frames(proto.MSG_GET_STATUS)
        cmd_id = sends[0].cmd_id
        assert [frame.cmd_id for frame in sends] == [cmd_id] * 3
        assert rig.logger.messages == [
            ("debug", f"BMCU resent 0x03 cmd={cmd_id} attempt 2"),
            ("debug", f"BMCU resent 0x03 cmd={cmd_id} attempt 3"),
            ("warning", f"BMCU no reply to 0x03 cmd={cmd_id} after 3 sends"),
        ]

    def test_half_duplex_duplicate_reply_is_dropped(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.add_fault("delay", proto.MSG_GET_STATUS, seconds=0.4)
        rig.conn.request(proto.MSG_GET_STATUS)
        rig.reactor.advance(1.0)
        cmd_id = rig.last_cmd(proto.MSG_GET_STATUS)
        replies = [f for f in rig.bmcu.sent if f.cmd_id == cmd_id]
        assert len(replies) == 2
        assert len(rig.statuses) == 1
        assert rig.logger.messages == [
            ("debug", f"BMCU resent 0x03 cmd={cmd_id} attempt 2"),
            ("debug", f"BMCU dropped duplicate reply cmd={cmd_id}"),
        ]

    def test_error_reply_raises(self, make_rig):
        rig = make_rig().connect()
        with pytest.raises(BMCUError, match="BMCU error code=3 channel=255 details=0"):
            rig.conn.request(proto.MSG_SET_MOTION, b"\x00\x09")

    def test_session_error_drops_the_link(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.reboot()
        with pytest.raises(BMCUError) as info:
            rig.conn.request(proto.MSG_GET_STATUS)
        assert info.value.code == proto.ERR_SESSION
        assert rig.conn.connected is False
        assert rig.links == [False]
        assert rig.logger.messages == [
            ("warning", f"BMCU link lost on {PORT}: BMCU session expired (firmware restarted)"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
        ]


# start_op

class TestStartOp:
    def test_returns_the_wire_cmd_id(self, make_rig):
        rig = make_rig().connect()
        op_id = rig.conn.start_op(proto.MSG_FEED_TO_CONTACT, proto.pack_feed(1, 500.0, 70, 3000))
        assert op_id == rig.last_cmd(proto.MSG_FEED_TO_CONTACT)
        assert rig.bmcu.running_ops() == [op_id]
        assert op_id in rig.conn._open_ops

    def test_rejection_raises_and_forgets_the_op(self, make_rig):
        rig = make_rig().connect()
        with pytest.raises(BMCUError, match="code=31"):
            rig.conn.start_op(proto.MSG_TAIL_RETRACT, struct.pack("<BfI", 0, 4.0, 1000))
        assert rig.conn._open_ops == set()

    def test_unconfirmed_start_is_aborted(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.add_fault("drop", proto.MSG_FEED_DISTANCE)
        with pytest.raises(BMCUTimeoutError):
            rig.conn.start_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 500.0, 70, 30000))
        assert _types(rig.bmcu.received) == [proto.MSG_FEED_DISTANCE, proto.MSG_ABORT_OP]
        assert rig.bmcu.running_ops() == []
        assert rig.conn._open_ops == set()

    def test_failed_abort_after_unconfirmed_start_still_raises_timeout(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.add_fault("drop", proto.MSG_FEED_DISTANCE)
        rig.bmcu.add_fault("drop", proto.MSG_ABORT_OP)
        with pytest.raises(BMCUTimeoutError, match="0x19"):
            rig.conn.start_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 500.0, 70, 30000))
        assert _types(rig.bmcu.received) == [proto.MSG_FEED_DISTANCE, proto.MSG_ABORT_OP]

    def test_half_duplex_lost_ack_does_not_start_a_second_operation(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.add_fault("drop", proto.MSG_FEED_TO_CONTACT)
        op_id = rig.conn.start_op(proto.MSG_FEED_TO_CONTACT, proto.pack_feed(1, 500.0, 70, 3000))
        assert len(rig.bmcu.frames(proto.MSG_FEED_TO_CONTACT)) == 2
        assert rig.bmcu.started == [op_id]

    def test_half_duplex_lost_ack_survives_the_route_uncertain_status(self, make_rig):
        # Only the ACK is lost; the STATUS after it shows the route turning
        # UNCERTAIN, which must not cancel the resend that recovers the ACK.
        rig = make_rig(half_duplex=True).connect()
        rig.conn.request(proto.MSG_GET_STATUS)
        rig.bmcu.add_fault("drop_ack", proto.MSG_FEED_TO_CONTACT)
        op_id = rig.conn.start_op(proto.MSG_FEED_TO_CONTACT,
                                  proto.pack_feed(1, 500.0, 70, 30000))
        sends = rig.bmcu.frames(proto.MSG_FEED_TO_CONTACT)
        assert [frame.cmd_id for frame in sends] == [op_id, op_id]
        assert rig.statuses[-1]["route_state_name"][1] == "UNCERTAIN"
        assert rig.bmcu.running_ops() == [op_id]
        assert rig.bmcu.frames(proto.MSG_ABORT_OP) == []

    def test_not_connected_raises(self, make_rig):
        rig = make_rig()
        with pytest.raises(BMCUError, match="not connected"):
            rig.conn.start_op(proto.MSG_FEED_DISTANCE, b"")

    @pytest.mark.parametrize("setup, reason", [
        (lambda ch: setattr(ch, "calibrated", False), "not_calibrated"),
        (lambda ch: setattr(ch, "encoder_ok", False), "encoder_io"),
        (lambda ch: setattr(ch, "motion", proto.MOTION_ON_USE), "busy"),
    ])
    def test_tail_retract_refusal_is_a_failed_result(self, make_rig, setup, reason):
        rig = make_rig().connect()
        setup(rig.bmcu.channels[1])
        op_id = rig.conn.start_op(proto.MSG_TAIL_RETRACT,
                                  proto.pack_tail_retract(1, 300.0, 9000))
        result = rig.conn.wait_op(op_id, 1.0)
        assert (result["state"], result["reason"]) == (proto.OP_STATE_FAILED, reason)
        assert [f.msg_type for f in rig.bmcu.sent if f.cmd_id == op_id] == [
            proto.MSG_OP_RESULT]
        assert rig.bmcu.frames(proto.MSG_ABORT_OP) == []


# wait_op

class TestWaitOp:
    def _start(self, rig):
        return rig.conn.start_op(proto.MSG_TAIL_RETRACT, proto.pack_tail_retract(2, 300.0, 9000))

    def test_a_result_after_the_wait_gave_up_is_not_kept(self, make_rig):
        rig = make_rig().connect()
        op_id = self._start(rig)
        with pytest.raises(BMCUTimeoutError):
            rig.conn.wait_op(op_id, 0.5)
        assert rig.conn.op_open(op_id) is False
        rig.bmcu.complete_op(op_id)
        rig.reactor.advance(0.1)
        assert op_id not in rig.conn._op_results

    def test_returns_the_unsolicited_result(self, make_rig):
        rig = make_rig().connect()
        op_id = self._start(rig)
        rig.later(0.3, lambda: rig.bmcu.complete_op(op_id))
        result = rig.conn.wait_op(op_id, 5.0)
        assert result["op_id"] == op_id
        assert result["state"] == proto.OP_STATE_DONE
        assert result["reason"] == "target"
        assert rig.bmcu.frames(proto.MSG_GET_OP_RESULT) == []
        assert op_id not in rig.conn._op_results

    def test_result_already_received(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.auto_complete[proto.MSG_TAIL_RETRACT] = (proto.OP_STATE_FAILED, 5)
        op_id = self._start(rig)
        start = rig.reactor.now
        result = rig.conn.wait_op(op_id, 5.0)
        assert result["reason"] == "buffer_limit"
        assert result["ok"] is False
        assert rig.reactor.now == start

    def test_poll_recovers_a_lost_result(self, make_rig):
        rig = make_rig().connect()
        op_id = self._start(rig)
        rig.later(0.2, lambda: rig.bmcu.complete_op(op_id, send=False))
        result = rig.conn.wait_op(op_id, 5.0)
        queries = rig.bmcu.frames(proto.MSG_GET_OP_RESULT)
        assert result["op_id"] == op_id
        assert len(queries) == 1
        assert queries[0].payload == struct.pack("<I", op_id)

    def test_poll_while_running_keeps_waiting_then_times_out(self, make_rig):
        rig = make_rig().connect()
        op_id = self._start(rig)
        start = rig.reactor.now
        with pytest.raises(BMCUTimeoutError) as info:
            rig.conn.wait_op(op_id, 2.5)
        assert str(info.value) == f"BMCU operation {op_id} did not finish within 2.5s"
        assert len(rig.bmcu.frames(proto.MSG_GET_OP_RESULT)) == 2
        assert rig.reactor.now == pytest.approx(start + 2.5)

    def test_link_drop_fails_waiter(self, make_rig):
        rig = make_rig().connect()
        op_id = self._start(rig)
        rig.later(0.3, rig.bmcu.unplug)
        with pytest.raises(BMCUError, match="read failed"):
            rig.conn.wait_op(op_id, 5.0)
        assert rig.links == [False]

    def test_not_connected_raises(self, make_rig):
        rig = make_rig()
        with pytest.raises(BMCUError, match="not connected"):
            rig.conn.wait_op(5, 1.0)

    def test_link_drop_during_poll_raises_not_connected(self, make_rig):
        rig = make_rig().connect()
        op_id = self._start(rig)
        rig.bmcu.add_fault("drop", proto.MSG_GET_OP_RESULT)
        rig.later(1.1, rig.bmcu.unplug)
        with pytest.raises(BMCUError, match="not connected"):
            rig.conn.wait_op(op_id, 5.0)


# stop_all / abort_op

class TestStopAll:
    def test_stops_and_aborts_running_op(self, make_rig):
        rig = make_rig().connect()
        op_id = rig.conn.start_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 50.0, 70, 3000))
        rig.conn.stop_all()
        assert rig.bmcu.frames(proto.MSG_STOP_ALL) != []
        assert rig.conn.wait_op(op_id, 1.0)["state"] == proto.OP_STATE_ABORTED
        assert rig.bmcu.channels[0].motion == proto.MOTION_IDLE

    def test_half_duplex_stop_is_not_superseded(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.add_fault("drop", proto.MSG_STOP_ALL)
        rig.conn.stop_all()
        assert len(rig.bmcu.frames(proto.MSG_STOP_ALL)) == 2


class TestAbortOp:
    def test_aborts_running_op(self, make_rig):
        rig = make_rig().connect()
        op_id = rig.conn.start_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 50.0, 70, 3000))
        rig.conn.abort_op()
        assert _types(rig.bmcu.frames(proto.MSG_ABORT_OP)) == [proto.MSG_ABORT_OP]
        assert rig.conn.wait_op(op_id, 1.0)["reason"] == "aborted"


# cmd ids

class TestOpIdsAcrossReconnects:
    def test_an_op_after_a_reconnect_is_not_replayed_from_the_last_session(self, make_rig):
        # The firmware keeps its last four results across HELLO, and replays a
        # repeated op id instead of moving (#37), so ids keep counting up.
        rig = make_rig().connect()
        first = rig.conn.start_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 50.0, 70, 3000))
        assert rig.conn.wait_op(first, 5.0)["state"] == proto.OP_STATE_DONE
        rig.bmcu.unplug()
        rig.bmcu.replug()
        rig.reactor.advance(3.0)
        assert rig.conn.connected is True
        assert rig.bmcu.result(first)["state"] == proto.OP_STATE_DONE
        rig.conn.request(proto.MSG_RUNTIME_SYNC, RUNTIME_SYNC)
        second = rig.conn.start_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 50.0, 70, 3000))
        assert second != first
        assert rig.bmcu.started == [first, second]
        assert rig.bmcu.running_ops() == [second]


class TestNewCmdId:
    def test_wraps_from_max_to_one(self, make_rig):
        conn = make_rig().conn
        conn._next_cmd = 0x7FFFFFFE
        assert [conn._new_cmd_id() for _ in range(3)] == [0x7FFFFFFE, 0x7FFFFFFF, 1]


class TestRequireConnected:
    def test_raises_when_down(self, make_rig):
        with pytest.raises(BMCUError, match=f"BMCU on {PORT} is not connected"):
            make_rig().conn._require_connected()

    def test_passes_when_up(self, make_rig):
        rig = make_rig().connect()
        assert rig.conn._require_connected() is None


# _request_raw / _exchange

class TestRequestRaw:
    def test_marks_the_id_answered_in_half_duplex(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.conn._request_raw(proto.MSG_GET_STATUS, b"", 1.0)
        assert rig.last_cmd(proto.MSG_GET_STATUS) in rig.conn.policy.answered

    def test_works_before_the_handshake_finishes(self, make_rig):
        rig = make_rig()
        rig.conn._open()
        msg_type, _ = rig.conn._request_raw(proto.MSG_HELLO, proto.pack_u32(1), 1.0)
        assert msg_type == bmcu_fake.MSG_HELLO_ACK


class TestExchange:
    def test_uses_the_given_cmd_id_and_cleans_up(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.add_fault("drop", proto.MSG_GET_STATUS, count=-1)
        with pytest.raises(BMCUTimeoutError):
            rig.conn._exchange(proto.MSG_GET_STATUS, 4242, b"", 0.1)
        assert rig.bmcu.frames(proto.MSG_GET_STATUS)[0].cmd_id == 4242
        assert rig.conn._pending == {}
        assert rig.conn.policy.pending_resends == {}

    def test_error_result_is_raised(self, make_rig):
        rig = make_rig().connect()
        with pytest.raises(BMCUError, match="code=65535"):
            rig.conn._exchange(0x7E, 4242, b"", 1.0)


# _reply_result

class TestReplyResult:
    def test_good_ack_passes_through(self, make_rig):
        conn = make_rig().conn
        payload = bmcu_fake.ack_payload(5)
        assert conn._reply_result(proto.MSG_ACK, payload) == (proto.MSG_ACK, payload)

    def test_rejected_ack(self, make_rig):
        result = make_rig().conn._reply_result(proto.MSG_ACK, bmcu_fake.ack_payload(5, 0, 12))
        assert isinstance(result, BMCUError)
        assert str(result) == "BMCU rejected the command: error 12"
        assert result.code is None

    def test_bad_ack(self, make_rig):
        result = make_rig().conn._reply_result(proto.MSG_ACK, b"\x01")
        assert str(result) == "BMCU sent a bad ACK: invalid ACK length"

    def test_error_carries_code(self, make_rig):
        result = make_rig().conn._reply_result(proto.MSG_ERROR,
                                               bmcu_fake.error_payload(40, 2, 7))
        assert str(result) == "BMCU error code=40 channel=2 details=7"
        assert result.code == 40

    def test_bad_error(self, make_rig):
        result = make_rig().conn._reply_result(proto.MSG_ERROR, b"\x01")
        assert str(result) == "BMCU sent a bad ERROR: invalid ERROR length"
        assert result.code is None

    def test_other_reply_passes_through(self, make_rig):
        conn = make_rig().conn
        assert conn._reply_result(proto.MSG_STATUS, b"x") == (proto.MSG_STATUS, b"x")


# transmit

class TestNextSeq:
    def test_counts_from_one_and_wraps(self, make_rig):
        conn = make_rig().conn
        assert conn._next_seq() == 1
        conn._seq = 0xFFFFFFFF
        assert conn._next_seq() == 0xFFFFFFFF
        assert conn._next_seq() == 0


class TestQueueFrame:
    def test_new_request_is_tracked(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.silent = True
        rig.conn._queue_frame(proto.MSG_SET_MOTION, 77, b"\x00\x00", track=True)
        rig.reactor.advance(0.01)
        assert list(rig.conn.policy.pending_resends) == [77]
        assert rig.bmcu.frames()[-1].cmd_id == 77

    def test_resend_is_not_tracked_again(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.silent = True
        rig.conn._queue_frame(proto.MSG_SET_MOTION, 77, b"\x00\x00", track=False)
        rig.reactor.advance(0.01)
        assert rig.conn.policy.pending_resends == {}
        assert rig.bmcu.frames()[-1].cmd_id == 77


class TestFlushTx:
    def test_writes_one_frame_per_write(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.silent = True
        serial = rig.bmcu.serial
        serial.writes.clear()
        rig.conn._tx.extend([(1, proto.encode_packet(2, 50, 1)),
                             (2, proto.encode_packet(2, 51, 2))])
        assert rig.conn._flush_tx(rig.reactor.now) == NEVER
        assert len(serial.writes) == 2
        assert rig.conn._last_tx == rig.reactor.now

    def test_gate_holds_and_returns_retry_time(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.conn.policy.on_rx_bytes(rig.reactor.now, False)
        rig.conn._tx.append((1, proto.encode_packet(2, 50, 1)))
        assert rig.conn._flush_tx(rig.reactor.now) == pytest.approx(rig.reactor.now + 0.002)
        assert len(rig.conn._tx) == 1

    def test_write_failure_drops_the_link(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.unplugged = True
        rig.conn._tx.append((1, proto.encode_packet(2, 50, 1)))
        assert rig.conn._flush_tx(rig.reactor.now) == NEVER
        assert rig.conn.connected is False
        assert rig.links == [False]
        assert rig.logger.messages == [
            ("warning", f"BMCU link lost on {PORT}: write failed: write failed: "
                        "[Errno 5] Input/output error"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
        ]


class TestWrite:
    def test_gated_write_retries_after_quiet_gap(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.send_status()
        rig.reactor.advance(0.0)
        rx_time = rig.reactor.now
        rig.conn.request(proto.MSG_GET_STATUS)
        sent = rig.bmcu.frames(proto.MSG_GET_STATUS)[0]
        assert sent.time == pytest.approx(rx_time + 0.002)

    def test_partial_frame_holds_the_write_up_to_the_bound(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        frame = proto.encode_packet(proto.MSG_STATUS, 99, 0, b"")
        rig.bmcu.send_raw(frame[:4])
        rig.reactor.advance(0.0)
        start = rig.reactor.now
        rig.conn.request(proto.MSG_GET_STATUS)
        sent = rig.bmcu.frames(proto.MSG_GET_STATUS)[0]
        assert sent.time == pytest.approx(start + 0.05)


class TestArmResendTimer:
    def test_points_at_earliest_due(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.conn.policy.on_queued(proto.MSG_SET_MOTION, 9, b"")
        rig.conn.policy.on_written(9, 200.0)
        rig.conn._arm_resend_timer()
        assert rig.conn._resend_timer.waketime == rig.conn.policy.next_resend_due()

    def test_never_without_resends(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.conn._arm_resend_timer()
        assert rig.conn._resend_timer.waketime == NEVER


class TestResendTick:
    def test_resends_awaited_and_forgets_abandoned(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.silent = True
        policy = rig.conn.policy
        for cmd_id in (11, 12):
            policy.on_queued(proto.MSG_GET_STATUS, cmd_id, b"")
            policy.on_written(cmd_id, 0.0)
        rig.conn._pending[11] = rig.reactor.completion()
        rig.reactor.advance(0.01)
        waketime = rig.conn._resend_tick(rig.reactor.now)
        assert [f.cmd_id for f in rig.bmcu.frames()] == [11]
        assert list(policy.pending_resends) == [11]
        assert waketime == policy.next_resend_due()

    def test_never_when_nothing_is_due(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        assert rig.conn._resend_tick(rig.reactor.now) == NEVER


# receive

class TestHandleRead:
    def test_read_error_drops_the_link(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.unplug()
        assert rig.conn.connected is False
        assert rig.links == [False]
        assert rig.conn.last_error == (
            "read failed: device reports readiness to read but returned no data")

    def test_empty_read_changes_nothing(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        last_rx = rig.conn.policy.last_rx
        rig.reactor.advance(1.0)
        rig.conn._handle_read(rig.reactor.now)
        assert rig.conn.connected is True
        assert rig.conn.policy.last_rx == last_rx

    def test_malformed_frame_is_logged_and_link_survives(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.send_raw(b"\x00\x05\x01\x02\x00")
        rig.reactor.advance(0.0)
        assert rig.logger.messages == [("debug", "BMCU discarded 1 malformed frame(s)")]
        assert rig.conn.connected is True

    def test_frame_end_restarts_the_partial_clock(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        frame = proto.encode_packet(proto.MSG_ACK, 99, 0, b"")
        rig.bmcu.send_raw(frame[:4])
        rig.reactor.advance(0.01)
        rig.bmcu.send_raw(frame[4:] + frame[:4])
        rig.reactor.advance(0.0)
        assert rig.conn.policy.partial_since == rig.reactor.now

    def test_version_mismatch_after_connect_stays_down(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.header_version = 2
        rig.bmcu.send_status()
        rig.reactor.advance(60.0)
        assert rig.conn.connected is False
        assert rig.links == [False]
        assert rig.bmcu.open_times == [100.0]
        assert rig.conn.last_error == link._version_message(2)

    def test_stops_after_a_frame_drops_the_link(self, make_rig):
        rig = make_rig().connect()
        stale = bmcu_fake.status_payload(session_id=1)
        # An event is logged whether or not the link is up, so it shows whether
        # the read went on past the frame that dropped the link.
        rig.bmcu.send_raw(proto.encode_packet(proto.MSG_STATUS, 50, 0, stale)
                          + proto.encode_packet(bmcu_fake.MSG_CAPS, 51, 0, b"\x01"))
        rig.reactor.advance(0.0)
        assert rig.statuses == []
        assert rig.links == [False]
        assert rig.logger.messages == [
            ("warning", f"BMCU link lost on {PORT}: BMCU session changed (firmware restarted)"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
        ]


class TestHandlePacket:
    def test_unsolicited_status_reaches_on_status(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.set_present(2, 0)
        rig.reactor.advance(0.0)
        assert [s["present"] for s in rig.statuses] == [[1, 1, 0, 1]]

    def test_other_unsolicited_frame_is_logged(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.send(bmcu_fake.MSG_CAPS, 0, b"\x01")
        rig.reactor.advance(0.0)
        assert rig.logger.messages == [("debug", "BMCU event 0x91 ignored")]
        assert rig.statuses == []

    def test_status_reply_reaches_on_status_and_the_policy(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.channels[1].motion = proto.MOTION_ON_USE
        rig.conn.request(proto.MSG_GET_STATUS)
        rig.conn.policy.on_queued(proto.MSG_SET_MOTION, 9, b"")
        rig.bmcu.channels[1].motion = proto.MOTION_IDLE
        rig.bmcu.send_status()
        rig.reactor.advance(0.0)
        assert [s["motion"][1] for s in rig.statuses] == [3, 0]
        assert rig.conn.policy.pending_resends == {}

    def test_reply_nobody_waits_for_is_ignored(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.send(proto.MSG_ACK, 31337, bmcu_fake.ack_payload(31337))
        rig.reactor.advance(0.0)
        assert rig.conn.connected is True
        assert rig.logger.messages == []

    def test_last_rx_tracks_every_frame(self, make_rig):
        rig = make_rig().connect()
        rig.reactor.advance(3.0)
        rig.bmcu.send(bmcu_fake.MSG_CAPS, 0, b"")
        rig.reactor.advance(0.0)
        assert rig.conn._last_rx == rig.reactor.now


class TestRecordOpResult:
    def test_invalid_result_is_logged(self, make_rig):
        rig = make_rig().connect()
        rig.conn._record_op_result(b"\x00", rig.reactor.now)
        assert rig.logger.messages == [
            ("debug", "BMCU discarded invalid OP_RESULT: invalid operation result length")]

    def test_result_for_unknown_op_is_ignored(self, make_rig):
        rig = make_rig().connect()
        payload = bmcu_fake.op_result_payload(5, bmcu_fake.OP_FEED_DISTANCE, 0, proto.OP_STATE_DONE)
        rig.conn._record_op_result(payload, rig.reactor.now)
        assert rig.conn._op_results == {}

    def test_records_marks_answered_and_wakes_waiter(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        waiter = rig.reactor.completion()
        rig.conn._open_ops.add(5)
        rig.conn._op_waiters[5] = waiter
        payload = bmcu_fake.op_result_payload(5, bmcu_fake.OP_FEED_DISTANCE, 0, proto.OP_STATE_DONE)
        rig.conn._record_op_result(payload, rig.reactor.now)
        assert rig.conn._op_results[5]["state"] == proto.OP_STATE_DONE
        assert rig.conn._open_ops == set()
        assert waiter.result is True
        assert 5 in rig.conn.policy.answered

    def test_records_without_a_waiter(self, make_rig):
        rig = make_rig().connect()
        rig.conn._open_ops.add(5)
        payload = bmcu_fake.op_result_payload(5, bmcu_fake.OP_FEED_DISTANCE, 0, proto.OP_STATE_DONE)
        rig.conn._record_op_result(payload, rig.reactor.now)
        assert list(rig.conn._op_results) == [5]


class TestHandleStatus:
    def test_ignored_before_handshake(self, make_rig):
        rig = make_rig()
        rig.conn._handle_status(bmcu_fake.status_payload(), unsolicited=True)
        assert rig.statuses == []
        # Not taken for a STATUS from another session either.
        assert rig.conn.last_error == ""
        assert rig.links == []

    def test_invalid_status_warns_once_per_connection(self, make_rig):
        rig = make_rig().connect()
        rig.conn._handle_status(b"\x00" * 3, unsolicited=True)
        rig.conn._handle_status(b"\x00" * 3, unsolicited=True)
        assert rig.statuses == []
        warning = ("warning", "BMCU sent a STATUS this AFC cannot read (invalid status "
                              "payload length 3); lane states will not update. Check the "
                              "firmware version.")
        assert rig.logger.messages == [
            warning, ("debug", "BMCU discarded invalid STATUS: invalid status payload length 3")]
        rig.bmcu.reboot()
        rig.reactor.advance(20.0)
        assert rig.conn.connected is True
        rig.logger.messages.clear()
        rig.conn._handle_status(b"\x00" * 3, unsolicited=True)
        assert rig.logger.messages == [warning]

    def test_other_session_drops_the_link(self, make_rig):
        # A board that rebooted announces itself with cmd_id 0: HELLO_ACK and
        # CAPS are only logged, the STATUS of the new session drops the link.
        rig = make_rig().connect()
        rig.bmcu.reboot(announce=True)
        rig.reactor.advance(0.0)
        assert rig.statuses == []
        assert rig.links == [False]
        assert rig.conn.last_error == "BMCU session changed (firmware restarted)"
        assert rig.logger.messages == [
            ("debug", "BMCU event 0x81 ignored"),
            ("debug", "BMCU event 0x91 ignored"),
            ("warning", f"BMCU link lost on {PORT}: BMCU session changed (firmware restarted)"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
        ]

    def test_a_reply_showing_a_stop_keeps_resends(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        rig.bmcu.channels[0].motion = proto.MOTION_ON_USE
        rig.conn._handle_status(rig.bmcu.status_payload(), unsolicited=False)
        rig.conn.policy.on_queued(proto.MSG_SET_MOTION, 9, b"")
        rig.bmcu.channels[0].motion = proto.MOTION_IDLE
        rig.conn._handle_status(rig.bmcu.status_payload(), unsolicited=False)
        assert list(rig.conn.policy.pending_resends) == [9]
        assert [s["motion"][0] for s in rig.statuses] == [3, 0]

    def test_delivers_parsed_status(self, make_rig):
        rig = make_rig().connect()
        rig.conn._handle_status(rig.bmcu.status_payload(), unsolicited=True)
        assert rig.statuses[0]["session_id"] == rig.bmcu.session_id


class TestNotify:
    def test_calls_the_callback(self, make_rig):
        seen = []
        make_rig().conn._notify("status", seen.append, 5)
        assert seen == [5]

    def test_callback_error_is_logged(self, make_rig):
        rig = make_rig()

        def boom(value):
            raise RuntimeError("kaboom")

        rig.conn._notify("link", boom, True)
        assert rig.logger.messages == [("error", "BMCU link callback failed: kaboom")]


# lifecycle

class TestEstablish:
    def test_success_resets_backoff_and_error(self, make_rig):
        rig = make_rig()
        rig.conn._backoff = 16.0
        rig.conn.last_error = "old"
        rig.conn._establish()
        assert rig.conn._backoff == 1.0
        assert rig.conn.last_error == ""
        assert rig.conn.connected is True

    def test_failure_tears_down_and_records(self, make_rig):
        rig = make_rig()
        rig.bmcu.channel_count = 3
        with pytest.raises(BMCUError):
            rig.conn._establish()
        assert rig.conn.last_error == "BMCU reports 3 channels; AFC_BMCU needs 4"
        assert rig.bmcu.serial is None
        assert rig.reactor.fds == {}


class TestOpen:
    def test_configures_and_registers(self, make_rig):
        rig = make_rig()
        rig.conn._open()
        serial = rig.bmcu.serial
        assert (serial.port, serial.baudrate, serial.timeout, serial.write_timeout) == (
            PORT, 115200, 0, 0.5)
        assert (serial.rtscts, serial.dsrdtr, serial.dtr, serial.rts) == (
            False, False, True, False)
        assert [fd for fd, _cb in rig.reactor.fds.values()] == [rig.bmcu.fd]
        assert rig.ioctl_calls == []
        assert rig.logger.messages == []

    def test_half_duplex_requests_low_latency(self, make_rig):
        rig = make_rig(half_duplex=True)
        rig.conn._open()
        assert rig.ioctl_calls == [link._TIOCGSERIAL]
        assert rig.logger.messages == [
            ("debug", "BMCU serial low-latency not supported: Inappropriate ioctl for device")]

    def test_half_duplex_low_latency_on(self, make_rig, monkeypatch):
        rig = make_rig(half_duplex=True)
        monkeypatch.setattr(link, "_serial_ioctl", lambda fd, req, buf, mutate: None)
        rig.conn._open()
        assert rig.logger.messages == [("debug", "BMCU serial low-latency on")]

    def test_missing_pyserial_raises(self, make_rig, monkeypatch):
        rig = make_rig()
        monkeypatch.setitem(sys.modules, "serial", None)
        with pytest.raises(BMCUError,
                           match="pyserial is not installed in Klipper's Python environment"):
            rig.conn._open()

    def test_open_error_raises(self, make_rig):
        rig = make_rig()
        rig.bmcu.unplugged = True
        with pytest.raises(BMCUError) as info:
            rig.conn._open()
        assert str(info.value) == (f"BMCU could not open {PORT}: could not open port "
                                   f"{PORT}: No such file")
        assert rig.reactor.fds == {}


class TestHandshake:
    def test_bad_hello_raises(self, make_rig):
        rig = make_rig(uid=b"\x00" * 12)
        rig.conn._open()
        with pytest.raises(BMCUError, match="BMCU sent a bad HELLO reply: invalid hardware UID"):
            rig.conn._handshake()

    def test_bad_caps_raises(self, make_rig):
        rig = make_rig()
        rig.bmcu.caps_payload = lambda: b"\x00"
        rig.conn._open()
        with pytest.raises(BMCUError, match="BMCU sent a bad CAPS reply: invalid caps length"):
            rig.conn._handshake()

    def test_arms_supervision(self, make_rig):
        rig = make_rig()
        rig.conn._open()
        rig.conn._handshake()
        assert rig.conn._supervise_timer.waketime == pytest.approx(rig.reactor.now + 1.0)


class TestTeardown:
    def test_a_frame_queued_before_the_drop_is_never_written(self, make_rig):
        rig = make_rig(half_duplex=True).connect()
        frame = proto.encode_packet(proto.MSG_STATUS, 90, 0, rig.bmcu.status_payload())
        rig.bmcu.send_raw(frame[:4])
        rig.reactor.advance(0.0)
        rig.conn._queue_frame(proto.MSG_SET_MOTION, 77, proto.pack_set_motion(0, 2), track=True)
        assert len(rig.conn._tx) == 1
        rig.bmcu.unplug()
        rig.bmcu.replug()
        rig.reactor.advance(3.0)
        assert rig.conn.connected is True
        assert rig.bmcu.frames(proto.MSG_SET_MOTION) == []

    def test_fails_waiters_with_given_class(self, make_rig):
        rig = make_rig().connect()
        request = rig.reactor.completion()
        op_waiter = rig.reactor.completion()
        rig.conn._pending[1] = request
        rig.conn._op_waiters[2] = op_waiter
        rig.conn._open_ops.add(2)
        rig.conn._op_results[3] = {}
        rig.conn._teardown("gone", BMCUVersionError)
        assert isinstance(request.result, BMCUVersionError)
        assert str(op_waiter.result) == "gone"
        assert (rig.conn._pending, rig.conn._op_waiters) == ({}, {})
        assert (rig.conn._open_ops, rig.conn._op_results) == (set(), {})
        assert rig.conn.connected is False
        assert rig.reactor.fds == {}
        assert rig.conn._supervise_timer.waketime == NEVER

    def test_close_error_is_swallowed(self, make_rig):
        rig = make_rig().connect()

        def bad_close():
            raise OSError("busy")

        rig.bmcu.serial.close = bad_close
        rig.conn._teardown("gone")
        assert rig.conn._serial is None

    def test_safe_when_never_opened(self, make_rig):
        rig = make_rig()
        rig.conn._teardown("gone")
        assert rig.conn.connected is False


class TestLinkLost:
    def test_not_connected_only_tears_down(self, make_rig):
        rig = make_rig()
        rig.conn._link_lost("nope")
        assert rig.links == []
        assert rig.logger.messages == []
        assert rig.conn.last_error == "nope"

    def test_closed_does_not_reconnect(self, make_rig):
        rig = make_rig().connect()
        rig.conn._closed = True
        rig.conn._link_lost("gone")
        assert rig.links == [False]
        assert rig.logger.messages == [("warning", f"BMCU link lost on {PORT}: gone")]
        assert rig.conn._reconnect_timer.waketime == NEVER


class TestLinkLoss:
    def test_on_link_false_then_true_without_motion(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.unplug()
        assert rig.links == [False]
        rig.bmcu.replug()
        rig.reactor.advance(3.0)
        assert rig.links == [False, True]
        assert rig.conn.connected is True
        assert rig.bmcu.motion_frames() == []
        assert _types(rig.bmcu.received) == [
            proto.MSG_HELLO, proto.MSG_SESSION_CONFIRM, proto.MSG_GET_CAPS,
            proto.MSG_UPDATE_CANCEL]

    def test_backoff_doubles_to_thirty_seconds(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.unplug()
        lost_at = rig.reactor.now
        rig.reactor.advance(130.0)
        gaps = [round(t - lost_at, 3) for t in rig.bmcu.open_times[1:]]
        assert gaps == [1.0, 3.0, 7.0, 15.0, 31.0, 61.0, 91.0, 121.0]

    def test_silence_drops_the_link(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.silent = True
        rig.reactor.advance(17.0)
        assert rig.links == [False]
        assert rig.logger.messages == [
            ("warning", f"BMCU link lost on {PORT}: nothing received for 15s"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
            ("debug", f"BMCU reconnect failed: BMCU on {PORT} did not answer HELLO"),
        ]


class TestTakeBackoff:
    def test_doubles_and_caps(self, make_rig):
        conn = make_rig().conn
        assert [conn._take_backoff() for _ in range(7)] == [1, 2, 4, 8, 16, 30, 30]


class TestScheduleReconnect:
    def test_arms_timer_and_logs(self, make_rig):
        rig = make_rig()
        rig.conn._schedule_reconnect()
        assert rig.conn._reconnect_timer.waketime == rig.reactor.now + 1.0
        assert rig.logger.messages == [("info", f"BMCU reconnecting to {PORT} in 1s")]


class TestReconnectTick:
    def test_closed_does_nothing(self, make_rig):
        rig = make_rig()
        rig.conn._closed = True
        assert rig.conn._reconnect_tick(rig.reactor.now) == NEVER
        assert rig.bmcu.open_times == []

    def test_connected_does_nothing(self, make_rig):
        rig = make_rig().connect()
        assert rig.conn._reconnect_tick(rig.reactor.now) == NEVER
        assert rig.bmcu.open_times == [100.0]

    def test_version_error_stops(self, make_rig):
        rig = make_rig(protocol=2)
        assert rig.conn._reconnect_tick(rig.reactor.now) == NEVER
        assert rig.logger.messages == [("error", link._version_message(2, "1.0.4"))]
        assert rig.links == []

    def test_failure_backs_off(self, make_rig):
        rig = make_rig()
        rig.bmcu.unplugged = True
        rig.conn._backoff = 4.0
        assert rig.conn._reconnect_tick(rig.reactor.now) == rig.reactor.now + 4.0
        assert rig.conn._backoff == 8.0
        assert rig.logger.messages == [
            ("debug", f"BMCU reconnect failed: BMCU could not open {PORT}: could not open "
                      f"port {PORT}: No such file")]

    def test_success_reports_link_up(self, make_rig):
        rig = make_rig()
        assert rig.conn._reconnect_tick(rig.reactor.now) == NEVER
        assert rig.links == [True]


class TestSuperviseTick:
    def test_heartbeat_keeps_the_board_session_alive(self, make_rig):
        # The firmware drops a session after 15 s without a request.
        rig = make_rig().connect()
        rig.reactor.advance(60.0)
        assert rig.conn.connected is True
        assert rig.links == []
        pings = rig.bmcu.frames(proto.MSG_PING)
        assert len(pings) == 12
        assert [frame.note for frame in pings] == [""] * 12

    def test_not_connected_stops(self, make_rig):
        rig = make_rig()
        assert rig.conn._supervise_tick(rig.reactor.now) == NEVER

    def test_quiet_link_is_left_alone(self, make_rig):
        rig = make_rig().connect()
        now = rig.reactor.now
        assert rig.conn._supervise_tick(now) == now + 1.0
        assert rig.bmcu.received == []

    def test_pings_an_idle_link(self, make_rig):
        rig = make_rig().connect()
        rig.reactor.advance(5.5)
        pings = rig.bmcu.frames(proto.MSG_PING)
        assert len(pings) == 1
        assert pings[0].payload == b"B"
        assert rig.conn.connected is True

    def test_ping_write_failure_stops(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.unplugged = True
        later = rig.conn._last_tx + 5.0
        rig.conn._last_rx = later
        assert rig.conn._supervise_tick(later) == NEVER
        assert rig.links == [False]

    def test_session_error_on_the_heartbeat_drops(self, make_rig):
        rig = make_rig().connect()
        rig.bmcu.host_session_ready = False
        rig.reactor.advance(5.5)
        assert rig.bmcu.frames(proto.MSG_PING)
        assert rig.links == [False]
        assert rig.conn.connected is False

    def test_silence_past_timeout_drops(self, make_rig):
        rig = make_rig().connect()
        later = rig.conn._last_rx + 15.01
        assert rig.conn._supervise_tick(later) == NEVER
        assert rig.links == [False]
