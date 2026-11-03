"""
Unit tests for extras/AFC_BMCU_proto.py
"""

from __future__ import annotations

import binascii
import struct

import pytest

from extras import AFC_BMCU_proto as proto
from tests import bmcu_fake


def _decoded(msg_type=proto.MSG_PING, seq=1, cmd_id=1, payload=b"x"):
    frame = proto.encode_packet(msg_type, seq, cmd_id, payload)
    return bytearray(proto.cobs_decode(frame[1:-1]))


def _reseal(body):
    body[-4:] = struct.pack("<I", binascii.crc32(bytes(body[:-4])) & 0xFFFFFFFF)
    return bytes(body)


# constants

# Copied by hand from BMCU-Klipper firmware/src/bmcu_protocol.h (MSG_*,
# BMCU_MOTION_*, BMCU_CAP_*), bmcu_protocol.cpp (op types, OP_STATE_*,
# OP_REASON_*, the error codes passed to send_error) and ams.h (AMS_ROUTE_*).
# Never derive these from proto: they are what the host is checked against.
FIRMWARE_WIRE_VALUES = {
    "MSG_HELLO": 0x01, "MSG_PING": 0x02, "MSG_GET_STATUS": 0x03, "MSG_GET_CAPS": 0x04,
    "MSG_GET_OP_RESULT": 0x06, "MSG_SESSION_CONFIRM": 0x08, "MSG_SET_MOTION": 0x10,
    "MSG_ABORT_OP": 0x16, "MSG_FEED_TO_CONTACT": 0x18, "MSG_FEED_DISTANCE": 0x19,
    "MSG_MARK_UNLOADED": 0x1A, "MSG_MARK_LOADED": 0x1B, "MSG_CAL_AUTO_START": 0x1C,
    "MSG_SET_SLOTS": 0x22, "MSG_TAIL_RETRACT": 0x25, "MSG_RUNTIME_SYNC": 0x34,
    "MSG_STOP_ALL": 0x50, "MSG_UPDATE_CANCEL": 0x62,
    "MSG_STATUS": 0x90, "MSG_OP_RESULT": 0x95, "MSG_ERROR": 0xA4, "MSG_ACK": 0xB0,
    "MOTION_IDLE": 0, "MOTION_SEND_OUT": 1, "MOTION_BEFORE_ON_USE": 2, "MOTION_ON_USE": 3,
    "MOTION_BEFORE_PULL_BACK": 4, "MOTION_PULL_BACK": 5, "MOTION_STOP_ON_USE": 6,
    "OP_TAIL_RETRACT": 7,
    "OP_STATE_IDLE": 0, "OP_STATE_RUNNING": 1, "OP_STATE_DONE": 2, "OP_STATE_FAILED": 3,
    "OP_STATE_ABORTED": 4,
    "ROUTE_EMPTY": 0, "ROUTE_LOADED": 1, "ROUTE_UNCERTAIN": 2,
    "CAP_TAIL_RETRACT": 0x02000000, "CAP_VOLATILE_SLOTS": 0x00000200,
    "ERR_SESSION": 96,
    # Every firmware reply and event type has this bit; no request has it.
    "REPLY_BIT": 0x80,
    "PROTO_VERSION": 1,
}
FIRMWARE_OP_REASONS = {
    0: "none", 1: "target", 2: "timeout", 3: "no_filament", 4: "encoder_io",
    5: "buffer_limit", 6: "aborted", 7: "busy", 8: "not_calibrated", 9: "motion_stopped",
    10: "contact", 11: "distance_limit", 12: "calibration_range", 13: "nvm_write",
    14: "sensor",
}
WIRE_PREFIXES = ("MSG_", "MOTION_", "OP_", "ROUTE_", "CAP_", "ERR_")


class TestConstants:
    def test_every_wire_value_matches_the_firmware(self):
        values = {name: getattr(proto, name) for name in FIRMWARE_WIRE_VALUES}
        assert values == FIRMWARE_WIRE_VALUES

    def test_every_wire_constant_is_pinned(self):
        defined = {name for name, value in vars(proto).items()
                   if name.startswith(WIRE_PREFIXES) and isinstance(value, int)}
        assert defined == {name for name in FIRMWARE_WIRE_VALUES
                           if name.startswith(WIRE_PREFIXES)}

    def test_reason_codes_match_the_firmware(self):
        assert proto.OP_REASON == FIRMWARE_OP_REASONS


# crc32

class TestCrc32:
    def test_known_vector(self):
        assert proto.crc32(b"123456789") == 0xCBF43926

    def test_result_is_unsigned(self):
        assert proto.crc32(b"\xff" * 4) == 0xFFFFFFFF


# cobs_encode / cobs_decode

class TestCobsEncode:
    @pytest.mark.parametrize("data", [
        b"",
        b"\x00",
        b"\x00\x00",
        b"\x11\x00\x22",
    ])
    def test_round_trip_has_no_zero(self, data):
        encoded = proto.cobs_encode(data)
        assert b"\x00" not in encoded
        assert proto.cobs_decode(encoded) == data

    def test_known_encoding(self):
        assert proto.cobs_encode(b"\x11\x00\x22") == b"\x02\x11\x02\x22"

    def test_full_block_starts_a_new_code(self):
        encoded = proto.cobs_encode(bytes([1]) * 254)
        assert encoded == b"\xff" + bytes([1]) * 254 + b"\x01"


class TestCobsDecode:
    def test_rejects_embedded_zero(self):
        with pytest.raises(ValueError, match="zero byte"):
            proto.cobs_decode(b"\x02\x01\x00\x01")

    def test_rejects_truncated_block(self):
        with pytest.raises(ValueError, match="truncated"):
            proto.cobs_decode(b"\x05\x01\x02")

    def test_full_block_adds_no_zero(self):
        assert proto.cobs_decode(b"\xff" + bytes([1]) * 254 + b"\x01") == bytes([1]) * 254


# encode_packet / decode_packet

class TestEncodePacket:
    def test_body_layout(self):
        frame = proto.encode_packet(proto.MSG_PING, 7, 42, b"ab")
        body = proto.cobs_decode(frame[1:-1])
        header = struct.pack("<BBHII", 1, 0x02, 2, 7, 42) + b"ab"
        crc = struct.pack("<I", binascii.crc32(header) & 0xFFFFFFFF)
        assert body == header + crc

    def test_default_payload_is_empty(self):
        frame = proto.encode_packet(proto.MSG_GET_STATUS, 1, 2)
        assert proto.decode_packet(proto.cobs_decode(frame[1:-1])) == (
            proto.MSG_GET_STATUS, 1, 2, b"")


class TestDecodePacket:
    def test_round_trip(self):
        frame = proto.encode_packet(proto.MSG_PING, 7, 42, b"\x00B\x00")
        decoded = proto.cobs_decode(frame[1:-1])
        assert proto.decode_packet(decoded) == (proto.MSG_PING, 7, 42, b"\x00B\x00")

    def test_short_packet_raises(self):
        with pytest.raises(ValueError, match="short"):
            proto.decode_packet(b"\x01" * 15)

    def test_bad_crc_raises(self):
        body = _decoded()
        body[-1] ^= 0x01
        with pytest.raises(ValueError, match="CRC"):
            proto.decode_packet(bytes(body))

    def test_payload_length_mismatch_raises(self):
        body = _decoded(payload=b"abc")
        struct.pack_into("<H", body, 2, 2)
        with pytest.raises(ValueError, match="length"):
            proto.decode_packet(_reseal(body))

    def test_other_protocol_version_raises(self):
        body = _decoded()
        body[0] = 2
        with pytest.raises(proto.ProtocolVersionError) as info:
            proto.decode_packet(_reseal(body))
        assert info.value.version == 2
        assert isinstance(info.value, ValueError)


class TestProtocolVersionError:
    def test_carries_version_and_message(self):
        err = proto.ProtocolVersionError(3)
        assert err.version == 3
        assert str(err) == "unsupported BMCU protocol version 3"


# StreamDecoder

class TestStreamDecoderFeed:
    def test_reassembles_split_frames(self):
        frames = (proto.encode_packet(proto.MSG_ACK, 1, 10, b"a")
                  + proto.encode_packet(proto.MSG_ACK, 2, 11, b"bb"))
        decoder = proto.StreamDecoder()
        packets = []
        for byte in frames:
            packets.extend(decoder.feed(bytes([byte])))
        assert [(p[1], p[2], p[3]) for p in packets] == [(1, 10, b"a"), (2, 11, b"bb")]
        assert decoder.errors == 0

    def test_drops_corrupt_frame_and_keeps_the_next(self):
        good = proto.encode_packet(proto.MSG_ACK, 2, 11, b"ok")
        bad = bytearray(proto.encode_packet(proto.MSG_ACK, 1, 10, b"xx"))
        bad[3] ^= 0x40
        decoder = proto.StreamDecoder()
        packets = decoder.feed(bytes(bad) + good)
        assert [(p[1], p[2], p[3]) for p in packets] == [(2, 11, b"ok")]
        assert decoder.errors == 1

    def test_bad_cobs_counts_an_error(self):
        decoder = proto.StreamDecoder()
        assert decoder.feed(b"\x00\x05\x01\x02\x00") == []
        assert decoder.errors == 1

    def test_discards_oversized_frame_until_delimiter(self):
        decoder = proto.StreamDecoder(max_frame=32)
        good = proto.encode_packet(proto.MSG_ACK, 3, 12, b"")
        packets = decoder.feed(b"\x00" + b"\x01" * 40 + good)
        assert [(p[1], p[2]) for p in packets] == [(3, 12)]
        assert decoder.errors == 1

    def test_records_incompatible_version(self):
        body = _decoded()
        body[0] = 2
        decoder = proto.StreamDecoder()
        assert decoder.feed(b"\x00" + proto.cobs_encode(_reseal(body)) + b"\x00") == []
        assert decoder.incompatible_version == 2
        assert decoder.errors == 1

    def test_default_max_frame(self):
        assert proto.StreamDecoder().max_frame == 256


class TestStreamDecoderPending:
    def test_pending_true_mid_frame(self):
        decoder = proto.StreamDecoder()
        frame = proto.encode_packet(proto.MSG_PING, 1, 2, b"x")
        assert decoder.pending is False
        assert decoder.feed(frame[:5]) == []
        assert decoder.pending is True
        assert decoder.feed(frame[5:]) == [(proto.MSG_PING, 1, 2, b"x")]
        assert decoder.pending is False

    def test_pending_while_discarding(self):
        decoder = proto.StreamDecoder()
        decoder.feed(b"\x00" + b"\x01" * 300)
        assert decoder.pending is True
        decoder.feed(b"\x00")
        assert decoder.pending is False


# parse_hello

class TestParseHello:
    def test_fields(self):
        payload = struct.pack("<12sBBBBIIBB", bytes(range(0x11, 0x1D)), 1, 0, 4, 1,
                              0x02000000, 0xABCD, 4, 82)
        assert proto.parse_hello(payload) == {
            "firmware": "1.0.4", "protocol": 1, "session_id": 0xABCD, "channels": 4,
        }

    def test_wrong_length_raises(self):
        with pytest.raises(ValueError, match="hello length"):
            proto.parse_hello(b"\x00" * 10)

    @pytest.mark.parametrize("uid", [b"\x00" * 12, b"\xff" * 12])
    def test_blank_uid_raises(self, uid):
        payload = struct.pack("<12sBBBBIIBB", uid, 1, 0, 4, 1, 0, 5, 4, 82)
        with pytest.raises(ValueError, match="UID"):
            proto.parse_hello(payload)

    def test_zero_session_raises(self):
        payload = struct.pack("<12sBBBBIIBB", b"\x01" * 12, 1, 0, 4, 1, 0, 0, 4, 82)
        with pytest.raises(ValueError, match="session"):
            proto.parse_hello(payload)


# parse_caps

class TestParseCaps:
    def test_returns_the_capability_mask(self):
        payload = struct.pack("<BBBBBBBBI12s16s", 4, 2, 1, 1, 0, 4, 1, 0, 0x00600000,
                              b"\x22" * 12, b"BMCU 370\x00junk")
        assert proto.parse_caps(payload) == 0x00600000

    def test_wrong_length_raises(self):
        with pytest.raises(ValueError, match="caps length"):
            proto.parse_caps(b"\x00" * 3)

    @pytest.mark.parametrize("uid", [b"\x00" * 12, b"\xff" * 12])
    def test_blank_uid_raises(self, uid):
        payload = struct.pack("<BBBBBBBBI12s16s", 4, 1, 0, 1, 0, 4, 1, 0, 0, uid, b"")
        with pytest.raises(ValueError, match="UID"):
            proto.parse_caps(payload)


# parse_status

class TestParseStatus:
    def test_fields(self):
        payload = bmcu_fake.status_payload(
            # Channel 0 neither loaded nor uncertain; every mask value differs
            # from its neighbours'.
            route=[proto.ROUTE_EMPTY, proto.ROUTE_LOADED, proto.ROUTE_UNCERTAIN,
                   proto.ROUTE_LOADED],
            motion=[proto.MOTION_ON_USE, 0, 0, proto.MOTION_PULL_BACK], present=[1, 0, 1, 0],
            buffer_pct=[10, 20, 30, 40], cal_valid_mask=0x0B, encoder_mask=0x0E,
            connected_mask=0x06, error_flags=0x0007, meters=[1.5, 2.0, 0.0, -1.0],
            session_id=77, active_op_id=1234, op_type=proto.OP_TAIL_RETRACT, op_channel=2,
            op_state=proto.OP_STATE_RUNNING, op_reason=1)
        assert proto.parse_status(payload) == {
            "route_state_name": ["EMPTY", "LOADED", "UNCERTAIN", "LOADED"],
            "motion": [3, 0, 0, 5],
            "present": [1, 0, 1, 0],
            "buffer_pct": [10, 20, 30, 40],
            "calibration_valid_mask": 0x0B,
            "encoder_io_mask": 0x0E,
            "controller_fault_flags": 5,
            "nvm_fault": True,
            "meters": [1.5, 2.0, 0.0, -1.0],
            "session_id": 77,
            "active_op_type": 7,
            "active_op_state": 1,
        }

    def test_flags_clear(self):
        status = proto.parse_status(bmcu_fake.status_payload(error_flags=0x0002))
        assert status["nvm_fault"] is False
        assert status["controller_fault_flags"] == 0

    def test_wrong_length_raises(self):
        with pytest.raises(ValueError, match="status payload length 3"):
            proto.parse_status(b"\x00" * 3)

    def test_non_finite_raises(self):
        with pytest.raises(ValueError, match="non-finite"):
            proto.parse_status(bmcu_fake.status_payload(meters=[float("nan"), 0, 0, 0]))

    @pytest.mark.parametrize("fields, match", [
        ({"route": [3, 0, 0, 0]}, "route state"),
        ({"now_channel": 4}, "active channel 4"),
        ({"motion": [0, 7, 0, 0]}, "motion state"),
        ({"present": [0, 0, 2, 0]}, "filament-present"),
        ({"buffer_pct": [0, 0, 0, 101]}, "buffer percentage"),
        ({"cal_valid_mask": 0x10}, "channel mask"),
        ({"encoder_mask": 0x10}, "channel mask"),
        ({"connected_mask": 0x10}, "channel mask"),
        ({"capture_mask": [8, 0, 0, 0]}, "capture mask"),
        ({"op_type": 8}, "operation type"),
        ({"op_channel": 4}, "operation channel"),
        ({"op_state": 5}, "operation state"),
        ({"auto_stage": 7}, "calibration stage"),
        ({"auto_progress": 101}, "calibration progress"),
        ({"auto_mask": 0x10}, "calibration progress"),
        ({"auto_done_mask": 0x10}, "calibration progress"),
        ({"auto_state": 5}, "calibration state"),
    ])
    def test_invalid_field_raises(self, fields, match):
        with pytest.raises(ValueError, match=match):
            proto.parse_status(bmcu_fake.status_payload(**fields))

    def test_boundary_values_accepted(self):
        payload = bmcu_fake.status_payload(
            now_channel=0xFF, op_channel=0xFF, buffer_pct=[100] * 4, cal_valid_mask=0x0F,
            capture_mask=[7] * 4, auto_stage=6, auto_progress=100,
            op_state=proto.OP_STATE_ABORTED, auto_state=proto.OP_STATE_ABORTED,
            motion=[proto.MOTION_STOP_ON_USE] * 4)
        assert proto.parse_status(payload)["motion"] == [6, 6, 6, 6]


# parse_op_result

class TestParseOpResult:
    def test_done_result(self):
        payload = struct.pack("<IBBBBffI", 99, proto.OP_TAIL_RETRACT, 2,
                              proto.OP_STATE_DONE, 1, 300.0, 299.5, 4200)
        assert proto.parse_op_result(payload) == {
            "op_id": 99, "state": 2, "reason": "target", "measured_mm": 299.5, "ok": True,
        }

    def test_failed_result_not_ok(self):
        payload = struct.pack("<IBBBBffI", 5, 3, 0, proto.OP_STATE_FAILED, 2, 0.0, 0.0, 0)
        result = proto.parse_op_result(payload)
        assert result["ok"] is False
        assert result["reason"] == "timeout"

    def test_unknown_reason_named_by_code(self):
        payload = struct.pack("<IBBBBffI", 5, 0, 0xFF, 0, 50, 0.0, 0.0, 0)
        assert proto.parse_op_result(payload)["reason"] == "reason_50"

    def test_wrong_length_raises(self):
        with pytest.raises(ValueError, match="operation result length"):
            proto.parse_op_result(b"\x00" * 4)

    def test_non_finite_raises(self):
        payload = struct.pack("<IBBBBffI", 5, 3, 0, 2, 1, float("inf"), 0.0, 0)
        with pytest.raises(ValueError, match="non-finite"):
            proto.parse_op_result(payload)

    @pytest.mark.parametrize("op_type, channel, state, match", [
        (8, 0, 2, "result type"),
        (3, 4, 2, "result channel"),
        (3, 0, 5, "result state"),
    ])
    def test_invalid_field_raises(self, op_type, channel, state, match):
        payload = struct.pack("<IBBBBffI", 5, op_type, channel, state, 1, 0.0, 0.0, 0)
        with pytest.raises(ValueError, match=match):
            proto.parse_op_result(payload)


# parse_ack / parse_error

class TestParseAck:
    def test_fields(self):
        assert proto.parse_ack(struct.pack("<IBH", 77, 0, 31)) == (77, False, 31)

    def test_ok(self):
        assert proto.parse_ack(struct.pack("<IBH", 77, 1, 0)) == (77, True, 0)

    def test_wrong_length_raises(self):
        with pytest.raises(ValueError, match="ACK length"):
            proto.parse_ack(b"\x00" * 6)


class TestParseError:
    def test_fields(self):
        assert proto.parse_error(struct.pack("<BHI", 2, 31, 9)) == (2, 31, 9)

    def test_wrong_length_raises(self):
        with pytest.raises(ValueError, match="ERROR length"):
            proto.parse_error(b"\x00" * 8)


# pack_runtime_sync

def _sync_values(**overrides):
    values = {
        "load_pressure_pct": 82.0, "load_speed": 60.0, "pull_speed": 55.0,
        "pull_speed_end": 12.0, "jam_timeout_ms": 20000.0, "before_pullback_pct": 40.0,
        "retract_mm": 200.0, "autoload_mm": 120.0,
    }
    values.update(overrides)
    return values


class TestPackRuntimeSync:
    def test_layout_with_standalone_off(self):
        payload = proto.pack_runtime_sync(_sync_values(), (1, 2, 3))
        expected = struct.pack(
            "<14f3B4B", 82.0, 60.0, 55.0, 12.0, 20000.0, 40.0,
            0.2, 0.2, 0.2, 0.2, 0.12, 0.12, 0.12, 0.12, 1, 2, 3, 0, 0, 0, 1)
        assert payload == expected
        assert len(payload) == 63

    def test_missing_key_raises(self):
        values = _sync_values()
        del values["pull_speed"]
        with pytest.raises(ValueError, match="missing pull_speed"):
            proto.pack_runtime_sync(values, (0, 0, 0))

    def test_non_finite_raises(self):
        with pytest.raises(ValueError, match="non-finite"):
            proto.pack_runtime_sync(_sync_values(load_speed=float("nan")), (0, 0, 0))

    @pytest.mark.parametrize("retract", [9.9, 2000.1])
    def test_retract_out_of_range_raises(self, retract):
        with pytest.raises(ValueError, match="unload distance"):
            proto.pack_runtime_sync(_sync_values(retract_mm=retract), (0, 0, 0))

    @pytest.mark.parametrize("autoload", [9.9, 1000.1])
    def test_autoload_out_of_range_raises(self, autoload):
        with pytest.raises(ValueError, match="autoload distance"):
            proto.pack_runtime_sync(_sync_values(autoload_mm=autoload), (0, 0, 0))

    def test_range_edges_accepted(self):
        payload = proto.pack_runtime_sync(_sync_values(retract_mm=10.0, autoload_mm=1000.0),
                                          (0, 0, 0))
        values = struct.unpack("<14f3B4B", payload)
        assert values[6] == pytest.approx(0.01)
        assert values[10] == pytest.approx(1.0)

    @pytest.mark.parametrize("rgb", [(0, 0), (0, 0, 0, 0), "abc"])
    def test_rgb_shape_raises(self, rgb):
        with pytest.raises(ValueError, match="three values"):
            proto.pack_runtime_sync(_sync_values(), rgb)

    @pytest.mark.parametrize("rgb", [(0, 256, 0), (-1, 0, 0), (0, 0, 1.5), (True, 0, 0)])
    def test_rgb_value_raises(self, rgb):
        with pytest.raises(ValueError, match="RGB value"):
            proto.pack_runtime_sync(_sync_values(), rgb)

    def test_rgb_list_accepted(self):
        payload = proto.pack_runtime_sync(_sync_values(), [255, 0, 7])
        assert struct.unpack("<14f3B4B", payload)[14:17] == (255, 0, 7)


# pack_feed

class TestPackFeed:
    def test_layout(self):
        assert proto.pack_feed(3, 450.0, 70, 30000) == struct.pack("<BfBI", 3, 450.0, 70,
                                                                    30000)

    def test_bounds_accepted(self):
        assert len(proto.pack_feed(0, 5.0, 55, 250)) == 10
        assert len(proto.pack_feed(3, 5000.0, 98, 300000)) == 10

    @pytest.mark.parametrize("args, match", [
        ((4, 100.0, 70, 1000), "channel"),
        ((-1, 100.0, 70, 1000), "channel"),
        ((0, 4.9, 70, 1000), "distance"),
        ((0, 5000.1, 70, 1000), "distance"),
        ((0, float("nan"), 70, 1000), "distance"),
        ((0, 100.0, 54, 1000), "contact percentage"),
        ((0, 100.0, 99, 1000), "contact percentage"),
        ((0, 100.0, 70, 249), "timeout"),
        ((0, 100.0, 70, 300001), "timeout"),
    ])
    def test_out_of_range_raises(self, args, match):
        with pytest.raises(ValueError, match=match):
            proto.pack_feed(*args)


# pack_tail_retract

class TestPackTailRetract:
    def test_layout(self):
        assert proto.pack_tail_retract(2, 300.0, 20000) == struct.pack("<BfI", 2, 300.0,
                                                                        20000)

    def test_bounds_accepted(self):
        assert len(proto.pack_tail_retract(0, 5.0, 250)) == 9
        assert len(proto.pack_tail_retract(3, 2000.0, 300000)) == 9

    @pytest.mark.parametrize("args, match", [
        ((4, 100.0, 1000), "channel"),
        ((0, 4.9, 1000), "distance"),
        ((0, 2000.1, 1000), "distance"),
        ((0, float("inf"), 1000), "distance"),
        ((0, 100.0, 249), "timeout"),
        ((0, 100.0, 300001), "timeout"),
    ])
    def test_out_of_range_raises(self, args, match):
        with pytest.raises(ValueError, match=match):
            proto.pack_tail_retract(*args)


# pack_set_motion

class TestPackSetMotion:
    def test_layout(self):
        assert proto.pack_set_motion(1, proto.MOTION_ON_USE) == b"\x01\x03"

    def test_bounds_accepted(self):
        assert proto.pack_set_motion(3, proto.MOTION_STOP_ON_USE) == b"\x03\x06"
        assert proto.pack_set_motion(0, proto.MOTION_IDLE) == b"\x00\x00"

    @pytest.mark.parametrize("channel, motion, match", [
        (4, 0, "channel"), (0, 7, "motion"), (0, -1, "motion"),
    ])
    def test_out_of_range_raises(self, channel, motion, match):
        with pytest.raises(ValueError, match=match):
            proto.pack_set_motion(channel, motion)


# pack_channel / pack_cal_auto_start / pack_u32

class TestPackChannel:
    def test_layout(self):
        assert proto.pack_channel(2) == b"\x02"

    def test_out_of_range_raises(self):
        with pytest.raises(ValueError, match="channel must be within 0..3"):
            proto.pack_channel(4)

    def test_non_int_raises(self):
        with pytest.raises(ValueError, match="channel must be an integer"):
            proto.pack_channel("1")


class TestPackCalAutoStart:
    def test_mask_selects_one_channel(self):
        assert proto.pack_cal_auto_start(0) == b"\x01"
        assert proto.pack_cal_auto_start(3) == b"\x08"

    def test_out_of_range_raises(self):
        with pytest.raises(ValueError, match="channel"):
            proto.pack_cal_auto_start(4)


class TestPackSlots:
    def test_layout_matches_the_firmware_slot(self):
        payload = proto.pack_slots([(1, 2, 3, "PLA"), (0, 0, 0, ""),
                                    (255, 128, 0, "PETG-CF high temp blend"), (9, 9, 9, "x")])
        assert len(payload) == 148
        # bmcu_protocol.cpp apply_slot_payload: ch, R, G, B, A, tmin, tmax, name[20], id[8]
        assert payload[:37] == (bytes((0, 1, 2, 3, 0xFF)) + b"\0" * 4
                                + b"PLA".ljust(20, b"\0") + b"\0" * 8)
        assert [payload[i * 37] for i in range(4)] == [0, 1, 2, 3]
        assert payload[74 + 9:74 + 29] == b"PETG-CF high temp b\0"

    def test_needs_four_slots(self):
        with pytest.raises(ValueError, match="four slots"):
            proto.pack_slots([(0, 0, 0, "")] * 3)

    def test_color_out_of_range_raises(self):
        with pytest.raises(ValueError):
            proto.pack_slots([(256, 0, 0, "")] + [(0, 0, 0, "")] * 3)


class TestPackU32:
    def test_layout(self):
        assert proto.pack_u32(0x01020304) == b"\x04\x03\x02\x01"

    @pytest.mark.parametrize("value", [-1, 0x100000000])
    def test_out_of_range_raises(self, value):
        with pytest.raises(ValueError, match="value"):
            proto.pack_u32(value)
