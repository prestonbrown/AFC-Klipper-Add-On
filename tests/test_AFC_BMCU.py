"""
Unit tests for extras/AFC_BMCU.py: the afcBMCU unit, built through its real
__init__ on AFC's conftest mocks, talking to the in-process FakeBMCU through a
real BMCUConnection on a FakeReactor clock.
"""

from __future__ import annotations

import os
import struct
import sys
from types import SimpleNamespace
from configparser import Error as config_error
from unittest.mock import MagicMock, call

import pytest

from extras import AFC_BMCU_link as link
from extras import AFC_BMCU_proto as proto
from extras.AFC_BMCU import MAX_SPEED, MIN_SPEED, afcBMCU, load_config_prefix
from extras.AFC_BMCU_link import BMCUError
from extras.AFC_extruder import AFCExtruder
from extras.AFC_lane import AFCLane, AFCLaneState
from tests.bmcu_fake import FakeBMCU, FakeReactor, status_payload
from tests.conftest import CommandError, MockAFC, MockConfig, MockPrinter

PORT = "/dev/serial/by-id/usb-BMCU_test"
NEVER = FakeReactor.NEVER
CALIBRATE_HELP = "Calibrate one BMCU lane's buffer"
CONNECTED_INFO = ("info", f"BMCU connected on {PORT}: firmware 1.0.4, protocol 1")
UNPLUGGED = "read failed: device reports readiness to read but returned no data"
HANDSHAKE = [proto.MSG_HELLO, proto.MSG_SESSION_CONFIRM, proto.MSG_GET_CAPS,
             proto.MSG_UPDATE_CANCEL]


# Fakes

# A 900 mm bowden at the fake's 60 mm/s: the sensor triggers past the minimum feed.
SENSOR_REACHED_S = 10.0


class FakeHub:
    """AFC_hub stand-in with the attributes the unit reads."""

    def __init__(self, name="BMCU_1_hub", virtual=True, bowden=900.0):
        self.name = name
        self.virtual = virtual
        self.state = False
        self.afc_bowden_length = bowden
        self.use_dist_hub = False

    def is_virtual_pin(self):
        return self.virtual


class FakeExtruder:
    """AFC_extruder stand-in: toolhead sensor and load/unload distances. Whether
    it has a toolhead sensor is AFCExtruder's own rule."""

    has_toolhead_sensor = AFCExtruder.has_toolhead_sensor

    def __init__(self, tool_start="PA1"):
        self.name = "extruder"
        self.tool_start = tool_start
        # A U1 head motion sensor (u1_filament_sensor_name), None without one.
        self.fila_tool_start = None
        self.tool_start_state = False
        self.tool_stn = 72.0
        self.tool_stn_unload = 100.0
        self.tool_load_speed = 25.0
        self.tool_unload_speed = 30.0
        self.tool_end = None
        self.tool_end_state = False
        self.lane_loaded = None
        self.set_status_led = MagicMock()
        self.estats = MagicMock()


class FakeLane:
    """AFCLane stand-in with explicit state; side-effect methods are mocks. Taking
    a lane in and out of the toolhead runs AFCLane's own methods."""

    set_tool_loaded = AFCLane.set_tool_loaded
    set_tool_unloaded = AFCLane.set_tool_unloaded
    get_toolhead_pre_sensor_state = AFCLane.get_toolhead_pre_sensor_state

    def __init__(self, name, index, hub, extruder):
        self.name = name
        self.index = index
        self.hub_obj = hub
        self.extruder_obj = extruder
        self.tool_loaded = False
        self.loaded_to_hub = False
        self.prep_state = False
        self._load_state = True
        self.status = AFCLaneState.NONE
        self.spool_id = 7
        self.dist_hub = 60.0
        self.short_move_dis = 10.0
        self.color = ""
        self.current_led_state = ""
        self.led_index = None
        self.led_spool_index = None
        self.led_use_filament_color = False
        self.led_fault = "fault"
        self.led_ready = "ready"
        self.led_not_ready = "not_ready"
        self.led_loading = "loading"
        self.led_unloading = "unloading"
        self.led_tool_loaded = "tool_loaded"
        self.led_tool_loaded_idle = "tool_loaded_idle"
        self.led_tool_unloaded = "tool_unloaded"
        self.led_spool_illum = "illum"
        self.hub = hub.name if hub is not None else None
        self.afc_extruder_name = extruder.name
        self.buffer_name = None
        self.handle_load_runout = MagicMock()
        self.send_lane_data = MagicMock()
        self.set_afc_prep_done = MagicMock()
        self.activate_toolhead_extruder = MagicMock()
        self.enable_buffer = MagicMock()
        self.disable_buffer = MagicMock()
        self.do_enable = MagicMock()
        self.custom_load_cmd = None
        self.custom_unload_cmd = None
        # The AFC and unit the real AFCLane methods report to; the rig sets both.
        self.afc = None
        self.unit_obj = None

    def handle_unit_connect(self, unit):
        unit.lanes[self.name] = self

    def is_direct_hub(self):
        return False

    def map_to_string(self):
        return f"T{self.index - 1}"


def _in_toolhead(lane):
    """Put a lane in its toolhead the way a finished load leaves it: AFC's
    flags set and the toolhead sensor seeing the filament."""
    lane.tool_loaded = True
    lane.extruder_obj.lane_loaded = lane.name
    lane.extruder_obj.tool_start_state = True


def _printing(lane):
    """Make a lane the one a print is using: in its toolhead, AFC's current
    lane, and printing."""
    _in_toolhead(lane)
    lane.afc.current = lane.name
    lane.afc.function.is_printing.return_value = True
    return lane


class Rig:
    """A real afcBMCU on conftest mocks, its FakeLanes, and the FakeBMCU it drives."""

    def __init__(self, monkeypatch, lanes=("bmcu1", "bmcu2"), virtual=True, values=None,
                 caps=proto.CAP_TAIL_RETRACT, tool_start="PA1", bowden=900.0, spools=(0, 1)):
        self.reactor = FakeReactor()
        self.afc = MockAFC()
        self.afc.reactor = self.reactor
        self.afc.function.is_printing.return_value = False
        self.afc.function.in_print.return_value = False
        self.afc.move_e_pos = MagicMock(side_effect=self._extrude)
        self.afc.do_tool_cut_tip_form = MagicMock()
        self.afc.post_unload_macro = None
        self.printer = MockPrinter(afc=self.afc)
        self.printer._reactor = self.reactor
        self.printer.reactor = self.reactor
        self.printer._gcode.register_mux_command = MagicMock()
        all_values = {"serial": PORT}
        all_values.update(values or {})
        self.config = MockConfig(name="AFC_BMCU BMCU_1", printer=self.printer,
                                 values=all_values)
        self.bmcu = FakeBMCU(self.reactor, capabilities=caps)
        # A spool at each of these inlets when the board powers on; a feed meets
        # the extruder gears, and the buffer compresses, after the bowden length.
        for channel in spools:
            self.bmcu.channels[channel].present = 1
            self.bmcu.channels[channel].contact_mm = bowden
        monkeypatch.setitem(sys.modules, "serial", self.bmcu.serial_module())
        monkeypatch.setattr(link, "_serial_ioctl", self._ioctl)
        self.unit = afcBMCU(self.config)
        self.logger = self.afc.logger
        self.hub = FakeHub(virtual=virtual, bowden=bowden)
        self.extruder = FakeExtruder(tool_start)
        self.lanes = {}
        for index, name in enumerate(lanes, 1):
            lane = FakeLane(name, index, self.hub, self.extruder)
            lane.afc = self.afc
            lane.unit_obj = self.unit
            self.lanes[name] = lane
            self.printer.register_event_handler("AFC_unit_BMCU_1:connect",
                                                lane.handle_unit_connect)
        # mm of encoder travel per mm the extruder moves, None for no follow
        self.follow = None
        self.follow_channel = 0
        # True: the toolhead sensor keeps reading filament after a retract.
        self.sensor_stuck = False

    def _ioctl(self, fd, request, buffer, mutate):
        raise OSError(25, "Inappropriate ioctl for device")

    def _extrude(self, amount, speed, log_string="", wait_tool=False):
        if (amount < 0
                and not self.sensor_stuck):
            # Retracting out of the extruder takes the filament off its sensor.
            self.extruder.tool_start_state = False
        if self.follow is not None:
            self.bmcu.channels[self.follow_channel].meters += amount * self.follow / 1000.0

    def lane(self, name="bmcu1"):
        return self.lanes[name]

    def start(self, status=True):
        """handle_connect, handle_ready and the deferred connect, then a STATUS."""
        self.unit.handle_connect()
        self.unit.handle_ready()
        self.reactor.advance(0)
        if status:
            self.bmcu.send_status()
            self.reactor.advance(0)
        self.logger.messages.clear()
        self.bmcu.received.clear()
        return self

    def finish_pull_back(self, channel=0, mm=900.0):
        """A PULL_BACK ends: the encoder moved back mm and the channel went idle."""
        self.bmcu.channels[channel].meters -= mm / 1000.0
        self.bmcu.channels[channel].motion = proto.MOTION_IDLE

    def reconnect(self, *args, **kwargs):
        """The board drops off the link, reboots and is reconnected; usable as a
        side_effect of a step that lets the reactor run."""
        self.bmcu.unplug()
        self.reactor.advance(0.5)
        self.bmcu.replug()
        self.bmcu.reboot()
        self.reactor.advance(10.0)

    def later(self, delay, fn):
        """Run fn() from a reactor timer, delay seconds from now."""
        def _tick(eventtime):
            self.reactor.unregister_timer(timer)
            fn()
            return NEVER
        timer = self.reactor.register_timer(_tick, self.reactor.now + delay)

    def status(self, **fields):
        return proto.parse_status(status_payload(**fields))

    def types(self):
        return [frame.msg_type for frame in self.bmcu.received]

    def errors(self):
        return self.afc.error.AFC_error.call_args_list

    def failures(self):
        return self.afc.error.handle_lane_failure.call_args_list


@pytest.fixture
def make_rig(monkeypatch):
    return lambda **kwargs: Rig(monkeypatch, **kwargs)


@pytest.fixture
def rig(make_rig):
    """A default Rig, connected and past its first STATUS."""
    return make_rig().start()


def _runtime_fields(payload):
    return struct.unpack("<14f3B4B", payload)


# __init__

class TestInit:
    def test_defaults(self, make_rig):
        unit = make_rig().unit
        assert unit.type == "BMCU"
        assert unit.stepperless_drive is True
        assert unit.serial == PORT
        assert unit.baud == 115200
        assert unit.half_duplex is False
        assert unit.load_speed == 60.0
        assert unit.pull_speed == 60.0
        assert unit.contact_pct == 70
        assert unit.feed_timeout == 30.0
        assert unit.path_extra == 100.0
        assert unit.follow_tolerance == 0.5
        assert unit._conn is None
        assert unit._status is None
        assert unit._prev_present is None
        assert unit._channels == {}
        assert unit._connecting is False
        assert unit._op_active is False
        assert unit._follow_pending is False

    def test_reads_every_option(self, make_rig):
        unit = make_rig(values={"type": "BMCU2", "baud": 230400, "half_duplex": True,
                                "load_speed": 41.0, "pull_speed": 42.0, "contact_pct": 85,
                                "feed_timeout": 45.0, "path_extra": 150.0,
                                "follow_tolerance": 0.25}).unit
        assert unit.type == "BMCU2"
        assert unit.baud == 230400
        assert unit.half_duplex is True
        assert unit.load_speed == 41.0
        assert unit.pull_speed == 42.0
        assert unit.contact_pct == 85
        assert unit.feed_timeout == 45.0
        assert unit.path_extra == 150.0
        assert unit.follow_tolerance == 0.25

    def test_serial_is_required(self, monkeypatch):
        printer = MockPrinter(afc=MockAFC())
        config = MockConfig(name="AFC_BMCU BMCU_1", printer=printer, values={})
        with pytest.raises(config_error):
            afcBMCU(config)

    def test_registers_calibrate_command(self, make_rig):
        rig = make_rig()
        rig.printer._gcode.register_mux_command.assert_called_once_with(
            "AFC_BMCU_CALIBRATE", "UNIT", "BMCU_1", rig.unit.cmd_AFC_BMCU_CALIBRATE,
            desc=CALIBRATE_HELP)

    def test_speeds_are_bounded_to_what_the_firmware_accepts(self, make_rig):
        rig = make_rig()
        rig.config.getfloat = MagicMock(side_effect=rig.config.getfloat)
        afcBMCU(rig.config)
        assert call('load_speed', 60.0, minval=10.0, maxval=120.0) in \
            rig.config.getfloat.call_args_list
        assert call('pull_speed', 60.0, minval=10.0, maxval=120.0) in \
            rig.config.getfloat.call_args_list

    def test_registers_shutdown_and_disconnect(self, make_rig):
        rig = make_rig()
        handlers = rig.printer._event_handlers
        assert handlers["klippy:shutdown"] == [rig.unit._handle_shutdown]
        assert handlers["klippy:disconnect"] == [rig.unit._handle_disconnect]


# handle_connect

class TestHandleConnect:
    def test_coinstalled_bmcu_section_refuses(self, make_rig):
        rig = make_rig()
        rig.config.fileconfig.add_section("bmcu")
        rig.config.fileconfig.set("bmcu", "devices", f"first,{PORT},-")
        with pytest.raises(config_error) as raised:
            rig.unit.handle_connect()
        assert str(raised.value) == (f"[AFC_BMCU BMCU_1] and [bmcu] both use {PORT}. "
                                     "Remove one: two hosts cannot drive one BMCU.")
        assert rig.unit._conn is None

    def test_different_port_allowed(self, make_rig):
        rig = make_rig()
        rig.config.fileconfig.add_section("bmcu")
        rig.config.fileconfig.set("bmcu", "devices", "first,/dev/ttyACM9")
        rig.unit.handle_connect()
        assert rig.unit._channels == {"bmcu1": 0, "bmcu2": 1}
        assert isinstance(rig.unit._conn, link.BMCUConnection)
        assert rig.unit._conn.port == PORT
        assert rig.unit._conn.baud == 115200
        assert rig.unit._conn.half_duplex is False
        assert rig.unit._conn.connected is False

    def test_sets_logos_and_clears_lane_load_state(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        assert "BMCU" in rig.unit.logo
        assert "success--text" in rig.unit.logo
        assert "BMCU_1" in rig.unit.logo
        assert "X ERROR" in rig.unit.logo_error
        assert rig.lane("bmcu1")._load_state is False
        assert rig.lane("bmcu2")._load_state is False

    def test_registers_unit_with_afc(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        assert rig.afc.units["BMCU_1"] is rig.unit
        assert list(rig.unit.lanes) == ["bmcu1", "bmcu2"]


class TestHandleShutdown:
    def test_stops_a_running_feed_without_waiting(self, rig):
        rig.unit._conn.start_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 300.0, 90, 9000))
        rig.bmcu.received.clear()
        start = rig.reactor.now
        rig.unit._handle_shutdown()
        assert rig.reactor.now == start
        assert rig.types() == [proto.MSG_STOP_ALL, proto.MSG_STOP_ALL]
        assert rig.bmcu.running_ops() == []
        assert rig.unit._conn.connected is False

    def test_stops_a_running_tail_retract(self, rig):
        rig.unit._conn.start_op(proto.MSG_TAIL_RETRACT, proto.pack_tail_retract(0, 300.0, 9000))
        rig.unit._handle_shutdown()
        assert rig.bmcu.running_ops() == []

    def test_before_connect_does_nothing(self, make_rig):
        rig = make_rig()
        rig.unit._handle_shutdown()
        assert rig.bmcu.received == []

    def test_half_duplex_resend_cannot_follow_the_stop(self, make_rig):
        rig = make_rig(values={"half_duplex": True}).start()
        rig.bmcu.add_fault("drop", proto.MSG_SET_MOTION)
        rig.later(0.05, rig.unit._handle_shutdown)
        with pytest.raises(BMCUError):
            rig.unit._set_motion(0, proto.MOTION_ON_USE)
        types = rig.types()
        first_stop = types.index(proto.MSG_STOP_ALL)
        assert types[first_stop:] == [proto.MSG_STOP_ALL, proto.MSG_STOP_ALL]
        opens = list(rig.bmcu.open_times)
        rig.reactor.advance(30.0)
        assert rig.types()[first_stop:] == [proto.MSG_STOP_ALL, proto.MSG_STOP_ALL]
        assert rig.bmcu.open_times == opens

    def test_a_load_in_progress_fails_at_its_next_call(self, rig):
        lane = rig.lane()
        rig.later(1.0, rig.unit._handle_shutdown)
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        types = rig.types()
        assert types[types.index(proto.MSG_STOP_ALL):] == [proto.MSG_STOP_ALL,
                                                             proto.MSG_STOP_ALL]
        assert rig.failures() == [call(lane, "load failed: BMCU connection closed",
                                       pause=False)]
        assert lane.tool_loaded is False


class TestHandleDisconnect:
    def test_stops_motors_and_releases_the_port(self, rig):
        rig.unit._conn.start_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 300.0, 90, 9000))
        rig.bmcu.received.clear()
        rig.unit._handle_disconnect()
        assert rig.types() == [proto.MSG_STOP_ALL]
        assert rig.bmcu.running_ops() == []
        assert rig.bmcu.serial is None
        assert rig.unit._conn.connected is False
        opens = list(rig.bmcu.open_times)
        rig.reactor.advance(60.0)
        assert rig.bmcu.open_times == opens

    def test_before_connect_does_nothing(self, make_rig):
        rig = make_rig()
        rig.unit._handle_disconnect()
        assert rig.bmcu.received == []


class TestRefuseCoinstalledBmcu:
    def _rig(self, make_rig, section, option, value):
        rig = make_rig()
        rig.config.fileconfig.add_section(section)
        rig.config.fileconfig.set(section, option, value)
        return rig

    def test_named_bmcu_section_refuses(self, make_rig):
        rig = self._rig(make_rig, "bmcu box", "serial", PORT)
        with pytest.raises(config_error) as raised:
            rig.unit._refuse_coinstalled_bmcu()
        assert str(raised.value) == (f"[AFC_BMCU BMCU_1] and [bmcu box] both use {PORT}. "
                                     "Remove one: two hosts cannot drive one BMCU.")

    def test_other_section_with_bmcu_prefix_ignored(self, make_rig):
        rig = self._rig(make_rig, "bmcu_panel", "serial", PORT)
        assert rig.unit._refuse_coinstalled_bmcu() is None

    def test_symlink_to_the_same_device_refuses(self, make_rig, tmp_path):
        device = tmp_path / "ttyACM0"
        device.write_text("")
        alias = tmp_path / "by-id-link"
        os.symlink(device, alias)
        rig = make_rig(values={"serial": str(alias)})
        rig.config.fileconfig.add_section("bmcu")
        rig.config.fileconfig.set("bmcu", "devices", f"a,{device}")
        with pytest.raises(config_error):
            rig.unit._refuse_coinstalled_bmcu()

    def test_no_bmcu_sections_passes(self, make_rig):
        rig = make_rig()
        assert rig.unit._refuse_coinstalled_bmcu() is None


class TestBmcuSectionPorts:
    def test_serial_and_every_devices_line(self, make_rig):
        rig = make_rig()
        rig.config.fileconfig.add_section("bmcu")
        rig.config.fileconfig.set("bmcu", "serial", " /dev/a ")
        rig.config.fileconfig.set("bmcu", "devices",
                                  "one, /dev/b ,-\n\nbroken\ntwo,/dev/c;three,/dev/d,ABC")
        assert rig.unit._bmcu_section_ports("bmcu") == ["/dev/a", "/dev/b", "/dev/c", "/dev/d"]

    def test_section_without_ports(self, make_rig):
        rig = make_rig()
        rig.config.fileconfig.add_section("bmcu")
        assert rig.unit._bmcu_section_ports("bmcu") == []


class TestBuildChannelMap:
    @pytest.mark.parametrize("option", ["custom_load_cmd", "custom_unload_cmd"])
    def test_custom_load_or_unload_command_is_refused(self, make_rig, option):
        rig = make_rig()
        setattr(rig.lane("bmcu2"), option, "MY_MACRO")
        with pytest.raises(config_error) as raised:
            rig.unit.handle_connect()
        assert str(raised.value) == (
            f"BMCU BMCU_1: lane bmcu2 sets {option}, which would replace the BMCU's own "
            "load and unload. Remove it.")

    def test_maps_index_to_channel(self, make_rig):
        rig = make_rig(lanes=("a", "b", "c", "d"))
        rig.unit.lanes = dict(rig.lanes)
        rig.lane("c").index = 4
        rig.lane("d").index = 3
        assert rig.unit._build_channel_map() == {"a": 0, "b": 1, "c": 3, "d": 2}

    @pytest.mark.parametrize("index", [0, 5])
    def test_index_out_of_range_raises(self, make_rig, index):
        rig = make_rig(lanes=("a",))
        rig.unit.lanes = dict(rig.lanes)
        rig.lane("a").index = index
        with pytest.raises(config_error) as raised:
            rig.unit._build_channel_map()
        assert str(raised.value) == (f"BMCU BMCU_1: lane a has index {index}; "
                                     "use unit: BMCU_1:1 to BMCU_1:4.")

    def test_duplicate_channel_raises(self, make_rig):
        rig = make_rig(lanes=("a", "b"))
        rig.unit.lanes = dict(rig.lanes)
        rig.lane("b").index = 1
        with pytest.raises(config_error) as raised:
            rig.unit._build_channel_map()
        assert str(raised.value) == "BMCU BMCU_1: lanes a and b both use channel 1."


# handle_ready / connect

class TestHandleReady:
    def test_defers_the_connect(self, make_rig):
        rig = make_rig(lanes=("bmcu2", "bmcu1"))
        rig.lane("bmcu2").index = 2
        rig.lane("bmcu1").index = 1
        rig.unit.handle_connect()
        rig.unit.handle_ready()
        assert list(rig.unit.lanes) == ["bmcu1", "bmcu2"]
        assert rig.unit._connecting is True
        assert rig.bmcu.open_times == []
        rig.reactor.advance(0)
        assert rig.unit._conn.connected is True
        assert rig.unit._connecting is False


class TestDeferredConnect:
    def test_prep_waits_until_runtime_sync_is_answered(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.unit._connecting = True
        seen = []
        sync = rig.bmcu._on_runtime_sync

        def record(*args):
            seen.append(rig.unit._connecting)
            return sync(*args)

        rig.bmcu._on_runtime_sync = record
        rig.unit._deferred_connect(rig.reactor.now)
        assert seen == [True]
        assert rig.unit._connecting is False

    def test_connects_and_syncs_runtime(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.unit._connecting = True
        rig.unit._deferred_connect(rig.reactor.now)
        assert rig.unit._conn.connected is True
        assert rig.unit._connecting is False
        assert rig.types() == [proto.MSG_HELLO, proto.MSG_SESSION_CONFIRM, proto.MSG_GET_CAPS,
                               proto.MSG_UPDATE_CANCEL,
                               proto.MSG_RUNTIME_SYNC]
        assert rig.bmcu.motion_frames() == []
        assert rig.logger.messages == [CONNECTED_INFO]

    def test_connect_failure_warns(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.unit._conn.connect = MagicMock(side_effect=BMCUError("no board"))
        rig.unit._connecting = True
        rig.unit._deferred_connect(rig.reactor.now)
        assert rig.unit._connecting is False
        assert rig.logger.messages == [("warning", "BMCU BMCU_1 not connected: no board")]
        assert rig.bmcu.received == []

    def test_runtime_sync_failure_drops_the_link(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.unit._sync_runtime = MagicMock(side_effect=BMCUError("refused"))
        rig.unit._connecting = True
        rig.unit._deferred_connect(rig.reactor.now)
        rig.unit._sync_runtime.assert_called_once_with()
        assert rig.unit._connecting is False
        assert rig.unit._conn.connected is False
        assert rig.unit._conn.last_error == "RUNTIME_SYNC failed: refused"
        assert rig.logger.messages == [
            CONNECTED_INFO,
            ("error", "BMCU BMCU_1 RUNTIME_SYNC failed: refused"),
            ("warning", f"BMCU link lost on {PORT}: RUNTIME_SYNC failed: refused"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
        ]

    def test_reconnect_retries_the_sync(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.bmcu.add_fault("drop", proto.MSG_RUNTIME_SYNC)
        rig.unit._connecting = True
        rig.unit._deferred_connect(rig.reactor.now)
        assert rig.unit._conn.connected is False
        rig.reactor.advance(5.0)
        assert rig.unit._conn.connected is True
        assert len(rig.bmcu.frames(proto.MSG_RUNTIME_SYNC)) == 2
        assert rig.bmcu.runtime_sync is not None

    def test_success_marks_the_link_ready(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.unit._conn._backoff_held = True
        rig.unit._conn._backoff = 8.0
        rig.unit._deferred_connect(rig.reactor.now)
        assert rig.unit._conn.reconnect_delay == 1.0


class TestSyncOrDrop:
    def test_success(self, rig):
        assert rig.unit._sync_or_drop() is True
        assert rig.types() == [proto.MSG_RUNTIME_SYNC]
        assert rig.unit._conn.connected is True

    def test_failure_drops_the_link(self, rig):
        rig.bmcu.add_fault("drop", proto.MSG_RUNTIME_SYNC)
        assert rig.unit._sync_or_drop() is False
        assert rig.unit._conn.connected is False
        failure = (f"BMCU request 0x34 cmd={rig.bmcu.frames(proto.MSG_RUNTIME_SYNC)[-1].cmd_id}"
                   " got no reply within 2s")
        assert rig.logger.messages == [
            ("error", f"BMCU BMCU_1 RUNTIME_SYNC failed: {failure}"),
            ("warning", f"BMCU link lost on {PORT}: RUNTIME_SYNC failed: {failure}"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
        ]

    def test_success_resets_the_failure_count(self, rig):
        rig.unit._sync_failures = 3
        assert rig.unit._sync_or_drop() is True
        assert rig.unit._sync_failures == 0

    def test_failures_after_five_log_only_when_the_backoff_caps(self, rig):
        rig.unit._sync_runtime = MagicMock(side_effect=BMCUError("refused"))
        rig.unit._conn.drop = MagicMock()
        rig.unit._sync_failures = 5
        rig.unit._conn._backoff = 16.0
        rig.logger.messages.clear()
        assert rig.unit._sync_or_drop() is False
        rig.unit._conn._backoff = 30.0
        assert rig.unit._sync_or_drop() is False
        assert rig.logger.messages == [
            ("debug", "BMCU BMCU_1 RUNTIME_SYNC failed: refused"),
            ("error", "BMCU BMCU_1 RUNTIME_SYNC failed 7 times in a row, retrying every "
                      "30s: refused"),
        ]
        assert rig.unit._conn.drop.call_args_list == [call("RUNTIME_SYNC failed: refused")] * 2

    def test_the_fifth_failure_is_still_logged(self, rig):
        rig.unit._sync_runtime = MagicMock(side_effect=BMCUError("refused"))
        rig.unit._conn.drop = MagicMock()
        rig.unit._sync_failures = 4
        rig.logger.messages.clear()
        rig.unit._sync_or_drop()
        assert rig.logger.messages == [("error", "BMCU BMCU_1 RUNTIME_SYNC failed: refused")]

    def test_persistent_failure_backs_off_to_thirty_seconds(self, rig):
        rig.bmcu.add_fault("drop", proto.MSG_RUNTIME_SYNC, count=-1)
        lost_at = rig.reactor.now
        rig.unit._handle_link_restored(rig.reactor.now)
        rig.reactor.advance(200.0)
        opens = [round(t - lost_at, 1) for t in rig.bmcu.open_times[1:]]
        gaps = [round(b - a, 1) for a, b in zip(opens, opens[1:])]
        # each cycle: 30 s backoff, 1.5 s connect settle, 2 s for the unanswered sync
        assert gaps[-3:] == [33.5, 33.5, 33.5]


class TestRuntimeValues:
    def test_retract_covers_longest_bowden(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.lane("bmcu2").hub_obj = FakeHub(name="other", bowden=1200.0)
        assert rig.unit._runtime_values() == {
            "load_pressure_pct": 82.0, "load_speed": 60.0, "pull_speed": 60.0,
            "pull_speed_end": 12.0, "jam_timeout_ms": 20000.0, "before_pullback_pct": 40.0,
            "retract_mm": 1300.0, "autoload_mm": 120.0,
        }

    def test_retract_capped_at_firmware_maximum(self, make_rig):
        rig = make_rig(bowden=1950.0)
        rig.unit.handle_connect()
        assert rig.unit._runtime_values()["retract_mm"] == 2000.0

    def test_retract_floor_without_lanes(self, make_rig):
        rig = make_rig(lanes=(), values={"path_extra": 0.0})
        assert rig.unit._runtime_values()["retract_mm"] == 10.0

    def test_uses_configured_speeds(self, make_rig):
        rig = make_rig(values={"load_speed": 33.0, "pull_speed": 44.0})
        values = rig.unit._runtime_values()
        assert values["load_speed"] == 33.0
        assert values["pull_speed"] == 44.0


class TestSyncRuntime:
    def test_configured_speed_limits_are_inside_the_firmware_ranges(self, make_rig):
        rig = make_rig(values={"load_speed": MIN_SPEED, "pull_speed": MAX_SPEED,
                               "path_extra": 5000.0}).start()
        assert rig.unit._conn.connected is True
        assert _runtime_fields(rig.bmcu.runtime_sync)[1:3] == (MIN_SPEED, MAX_SPEED)

    def test_sends_runtime_sync_with_standalone_off(self, rig):
        rig.unit._sync_runtime()
        assert rig.types() == [proto.MSG_RUNTIME_SYNC]
        fields = _runtime_fields(rig.bmcu.runtime_sync)
        assert fields[:6] == (82.0, 60.0, 60.0, 12.0, 20000.0, 40.0)
        assert fields[6:10] == pytest.approx((1.0, 1.0, 1.0, 1.0))
        assert fields[10:14] == pytest.approx((0.12, 0.12, 0.12, 0.12))
        assert fields[14:17] == (255, 255, 255)
        assert fields[17:] == (0, 0, 0, 1)


class TestReconcileLane:
    def test_tool_loaded_marks_loaded(self, rig):
        _in_toolhead(rig.lane("bmcu2"))
        rig.unit._reconcile_lane(rig.lane("bmcu2"))
        frames = rig.bmcu.frames(proto.MSG_MARK_LOADED, proto.MSG_MARK_UNLOADED)
        assert [(f.msg_type, f.payload) for f in frames] == [(proto.MSG_MARK_LOADED, b"\x01")]

    def test_not_loaded_marks_unloaded(self, rig):
        rig.unit._reconcile_lane(rig.lane("bmcu2"))
        frames = rig.bmcu.frames(proto.MSG_MARK_LOADED, proto.MSG_MARK_UNLOADED)
        assert [(f.msg_type, f.payload) for f in frames] == [(proto.MSG_MARK_UNLOADED, b"\x01")]


# _sync_from_status

class TestSyncFromStatus:
    def test_insert_during_an_operation_is_dispatched_when_it_ends(self, make_rig):
        rig = make_rig(spools=()).start()
        other = rig.lane("bmcu2")
        with rig.unit._operation():
            rig.bmcu.set_present(1, 1)
            rig.reactor.advance(0)
            other.handle_load_runout.assert_not_called()
        rig.reactor.advance(0)
        assert [c.args[1] for c in other.handle_load_runout.call_args_list] == [True]

    def test_removal_during_an_operation_is_dispatched_when_it_ends(self, rig):
        rig.bmcu.set_present(1, 1)
        rig.reactor.advance(0)
        other = rig.lane("bmcu2")
        other.handle_load_runout.reset_mock()
        with rig.unit._operation():
            rig.bmcu.set_present(1, 0)
            rig.reactor.advance(0)
        rig.reactor.advance(0)
        assert [c.args[1] for c in other.handle_load_runout.call_args_list] == [False]

    def _drop(self, rig, lane="bmcu1", **fields):
        rig.unit._prev_present = None
        rig.unit._sync_from_status(rig.status(present=(1, 1, 0, 0)))
        rig.unit._sync_from_status(rig.status(present=(0, 1, 0, 0) if lane == "bmcu1"
                                              else (1, 0, 0, 0), **fields))
        rig.reactor.advance(0)

    def test_runout_edge_while_printing_is_confirmed_then_dispatched(self, make_rig):
        rig = make_rig(spools=()).start()
        lane = _printing(rig.lane())
        rig.bmcu.channels[1].present = 1
        self._drop(rig, motion=(proto.MOTION_ON_USE, 0, 0, 0))
        lane.handle_load_runout.assert_not_called()
        start = rig.reactor.now
        rig.reactor.advance(0.3)
        lane.handle_load_runout.assert_called_once_with(start + 0.3, False)
        assert rig.unit._runout_dispatched == {"bmcu1"}

    def test_present_drop_during_unit_op_is_not_runout(self, rig):
        lane = _printing(rig.lane())
        rig.unit._op_active = True
        self._drop(rig)
        lane.handle_load_runout.assert_not_called()
        assert lane.prep_state is False

    def test_present_drop_during_firmware_op_is_not_runout(self, rig):
        # A running tail retract reports its channel's motion as idle.
        lane = _printing(rig.lane())
        self._drop(rig, active_op_id=9, op_type=proto.OP_TAIL_RETRACT, op_channel=0,
                   op_state=proto.OP_STATE_RUNNING)
        lane.handle_load_runout.assert_not_called()

    def test_present_drop_during_board_tail_retract_is_not_runout(self, rig):
        rig.bmcu.channels[0].present = 1
        rig.unit._refresh_status()
        rig.reactor.advance(0)
        lane = _printing(rig.lane())
        lane.handle_load_runout.reset_mock()
        rig.unit._conn.start_op(proto.MSG_TAIL_RETRACT, proto.pack_tail_retract(0, 300.0, 20000))
        rig.reactor.advance(0)
        status = rig.unit._status
        assert status["motion"][0] == proto.MOTION_IDLE
        assert status["active_op_type"] == proto.OP_TAIL_RETRACT
        assert status["active_op_state"] == proto.OP_STATE_RUNNING
        rig.bmcu.set_present(0, 0)
        rig.reactor.advance(0)
        assert rig.unit._status["present"][0] == 0
        lane.handle_load_runout.assert_not_called()
        assert rig.bmcu.motion_frames()[-1].msg_type == proto.MSG_TAIL_RETRACT

    def test_present_drop_not_printing_is_not_runout(self, rig):
        lane = _printing(rig.lane())
        rig.afc.function.is_printing.return_value = False
        self._drop(rig)
        lane.handle_load_runout.assert_not_called()

    def test_present_drop_on_other_tool_lane_is_not_runout(self, rig):
        _printing(rig.lane())
        other = rig.lane("bmcu2")
        _in_toolhead(other)
        self._drop(rig, lane="bmcu2")
        other.handle_load_runout.assert_not_called()

    def test_present_drop_on_idle_lane_dispatches_removal(self, rig):
        self._drop(rig, lane="bmcu2")
        rig.lane("bmcu2").handle_load_runout.assert_called_once_with(rig.reactor.now, False)
        rig.lane("bmcu1").handle_load_runout.assert_not_called()

    def test_insert_edge_dispatches_true(self, make_rig):
        rig = make_rig(spools=()).start()
        lane = rig.lane("bmcu2")
        rig.unit._sync_from_status(rig.status(present=(0, 0, 0, 0)))
        rig.unit._sync_from_status(rig.status(present=(0, 1, 0, 0)))
        rig.reactor.advance(0)
        lane.handle_load_runout.assert_called_once_with(rig.reactor.now, True)

    def test_insert_edge_on_tool_loaded_lane_is_ignored(self, rig):
        lane = rig.lane("bmcu2")
        _in_toolhead(lane)
        rig.unit._sync_from_status(rig.status(present=(0, 0, 0, 0)))
        rig.unit._sync_from_status(rig.status(present=(0, 1, 0, 0)))
        rig.reactor.advance(0)
        lane.handle_load_runout.assert_not_called()
        assert lane.prep_state is True

    def test_first_status_is_a_baseline(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.unit._sync_from_status(rig.status(present=(1, 0, 0, 0)))
        rig.reactor.advance(0)
        rig.lane("bmcu1").handle_load_runout.assert_not_called()
        assert rig.unit._prev_present == [1, 0, 0, 0]

    def test_unchanged_present_does_not_dispatch(self, rig):
        rig.unit._prev_present = None
        rig.unit._sync_from_status(rig.status(present=(1, 0, 0, 0)))
        rig.unit._sync_from_status(rig.status(present=(1, 0, 0, 0)))
        rig.reactor.advance(0)
        rig.lane("bmcu1").handle_load_runout.assert_not_called()

    def test_virtual_hub_tracks_tool_loaded(self, rig):
        tooled, staged = rig.lane("bmcu1"), rig.lane("bmcu2")
        _in_toolhead(tooled)
        rig.unit._sync_from_status(rig.status(present=(0, 1, 0, 0)))
        assert tooled._load_state is True
        assert tooled.prep_state is False
        assert tooled.loaded_to_hub is True
        assert staged._load_state is False
        assert staged.prep_state is True
        assert staged.loaded_to_hub is True

    def test_real_hub_state_not_overridden(self, make_rig):
        rig = make_rig(virtual=False).start()
        rig.hub.state = True
        tooled, empty = rig.lane("bmcu1"), rig.lane("bmcu2")
        _in_toolhead(tooled)
        rig.unit._sync_from_status(rig.status(present=(1, 0, 0, 0)))
        assert rig.hub.state is True
        assert tooled._load_state is True
        assert empty._load_state is False
        assert empty.loaded_to_hub is False
        rig.unit._sync_from_status(rig.status(present=(0, 0, 0, 0)))
        assert tooled._load_state is False
        assert tooled.loaded_to_hub is True

    def test_stores_status_and_resumes_following(self, rig):
        _printing(rig.lane())
        status = rig.status(present=(1, 0, 0, 0))
        rig.unit._sync_from_status(status)
        assert rig.unit._status is status
        rig.reactor.advance(0)
        assert [(f.msg_type, f.payload) for f in rig.bmcu.motion_frames()] == [
            (proto.MSG_SET_MOTION, bytes((0, proto.MOTION_ON_USE)))]

    def test_flicker_shorter_than_the_debounce_is_not_runout(self, rig):
        lane = _printing(rig.lane())
        rig.bmcu.channels[0].present = 1
        self._drop(rig)
        rig.later(0.1, lambda: rig.bmcu.set_present(0, 1))
        rig.reactor.advance(1.0)
        lane.handle_load_runout.assert_not_called()
        assert rig.unit._runout_dispatched == set()

    def test_drop_while_paused_is_not_confirmed_until_resume(self, make_rig):
        rig = make_rig(spools=()).start()
        lane = _printing(rig.lane())
        rig.afc.function.is_printing.return_value = False
        self._drop(rig)
        rig.reactor.advance(1.0)
        lane.handle_load_runout.assert_not_called()
        rig.afc.function.is_printing.return_value = True
        rig.unit._handle_printing(0.0)
        rig.reactor.advance(0.3)
        lane.handle_load_runout.assert_called_once_with(rig.reactor.now, False)

    def test_inlet_seeing_filament_clears_the_runout_latch(self, rig):
        rig.unit._runout_dispatched.add("bmcu2")
        rig.unit._sync_from_status(rig.status(present=(0, 1, 0, 0)))
        assert rig.unit._runout_dispatched == set()

    def test_empty_printing_lane_is_checked_without_an_edge(self, make_rig):
        rig = make_rig(spools=()).start()
        lane = _printing(rig.lane())
        rig.unit._prev_present = [0, 0, 0, 0]
        rig.unit._sync_from_status(rig.status(present=(0, 0, 0, 0)))
        rig.unit._sync_from_status(rig.status(present=(0, 0, 0, 0)))
        assert rig.unit._runout_pending == {"bmcu1"}
        rig.reactor.advance(0.3)
        lane.handle_load_runout.assert_called_once_with(rig.reactor.now, False)
        rig.unit._sync_from_status(rig.status(present=(0, 0, 0, 0)))
        rig.reactor.advance(1.0)
        assert lane.handle_load_runout.call_count == 1

    def test_empty_lane_while_busy_is_not_checked(self, rig):
        _printing(rig.lane())
        rig.unit._op_active = True
        rig.unit._sync_from_status(rig.status(present=(0, 0, 0, 0)))
        assert rig.unit._runout_pending == set()

    def test_empty_lane_not_in_the_toolhead_is_not_checked(self, rig):
        # AFC names bmcu1 current while printing, but it is not in the toolhead.
        rig.afc.current = "bmcu1"
        rig.afc.function.is_printing.return_value = True
        rig.unit._sync_from_status(rig.status(present=(0, 0, 0, 0)))
        assert rig.unit._runout_pending == set()


class TestMayHaveRunOut:
    def test_empty_inlet_on_the_printing_lane(self, make_rig):
        rig = make_rig()
        assert rig.unit._may_have_run_out(_printing(rig.lane()), False) is True

    @pytest.mark.parametrize("present, change", [
        (True, lambda rig, lane: None),
        (False, lambda rig, lane: setattr(lane, "tool_loaded", False)),
        (False, lambda rig, lane: setattr(rig.unit, "_op_active", True)),
        (False, lambda rig, lane: setattr(rig.afc.function.is_printing, "return_value", False)),
        (False, lambda rig, lane: setattr(rig.afc, "current", "bmcu2")),
    ], ids=["inlet_sees_filament", "not_in_the_toolhead", "operation_running", "not_printing",
            "not_the_current_lane"])
    def test_each_condition_alone_rules_it_out(self, make_rig, present, change):
        rig = make_rig()
        lane = _printing(rig.lane())
        change(rig, lane)
        assert rig.unit._may_have_run_out(lane, present) is False


class TestScheduleRunoutCheck:
    def test_schedules_once(self, make_rig):
        rig = make_rig()
        lane = rig.lane()
        rig.unit._confirm_runout = MagicMock()
        rig.unit._schedule_runout_check(lane)
        rig.unit._schedule_runout_check(lane)
        assert rig.unit._runout_pending == {"bmcu1"}
        rig.reactor.advance(0.29)
        rig.unit._confirm_runout.assert_not_called()
        rig.reactor.advance(0.01)
        rig.unit._confirm_runout.assert_called_once_with(lane)

    def test_already_handed_over(self, make_rig):
        rig = make_rig()
        rig.unit._runout_dispatched.add("bmcu1")
        rig.unit._schedule_runout_check(rig.lane())
        assert rig.unit._runout_pending == set()


class TestConfirmRunout:
    def _ready(self, make_rig):
        rig = make_rig(spools=()).start()
        return rig, _printing(rig.lane())

    def test_still_empty_dispatches_once(self, make_rig):
        rig, lane = self._ready(make_rig)
        rig.unit._confirm_runout(lane)
        rig.unit._confirm_runout(lane)
        lane.handle_load_runout.assert_called_once_with(rig.reactor.now, False)
        assert rig.unit._runout_dispatched == {"bmcu1"}

    def test_inlet_filled_again(self, make_rig):
        rig, lane = self._ready(make_rig)
        rig.bmcu.channels[0].present = 1
        rig.unit._confirm_runout(lane)
        lane.handle_load_runout.assert_not_called()
        assert rig.unit._runout_dispatched == set()

    def test_status_failure_is_logged(self, make_rig):
        rig, lane = self._ready(make_rig)
        rig.unit._conn.request = MagicMock(side_effect=BMCUError("gone"))
        rig.unit._confirm_runout(lane)
        lane.handle_load_runout.assert_not_called()
        assert rig.logger.messages == [("debug", "BMCU BMCU_1 runout check failed: gone")]

    def test_clears_the_pending_mark(self, make_rig):
        rig, lane = self._ready(make_rig)
        rig.unit._runout_pending.add("bmcu1")
        rig.bmcu.channels[0].present = 1
        rig.unit._confirm_runout(lane)
        assert rig.unit._runout_pending == set()


class TestApplyInlet:
    def test_virtual_hub_present_lane(self, make_rig):
        rig = make_rig()
        lane = rig.lane()
        rig.unit._apply_inlet(lane, True)
        assert lane.prep_state is True
        assert lane.loaded_to_hub is True
        assert lane._load_state is False

    def test_virtual_hub_tool_loaded_lane_with_empty_inlet(self, make_rig):
        rig = make_rig()
        lane = rig.lane()
        _in_toolhead(lane)
        rig.unit._apply_inlet(lane, False)
        assert lane.prep_state is False
        assert lane.loaded_to_hub is True
        assert lane._load_state is True

    def test_empty_lane(self, make_rig):
        rig = make_rig()
        lane = rig.lane()
        lane.loaded_to_hub = True
        rig.unit._apply_inlet(lane, False)
        assert lane.loaded_to_hub is False
        assert lane._load_state is False

    def test_real_hub_follows_inlet(self, make_rig):
        rig = make_rig(virtual=False)
        lane = rig.lane()
        _in_toolhead(lane)
        rig.unit._apply_inlet(lane, False)
        assert lane._load_state is False
        rig.unit._apply_inlet(lane, True)
        assert lane._load_state is True


class TestUnitBusy:
    def test_own_operation(self, make_rig):
        rig = make_rig()
        rig.unit._op_active = True
        assert rig.unit._unit_busy() is True

    def test_no_status(self, make_rig):
        assert make_rig().unit._unit_busy() is False

    def test_firmware_operation_running(self, make_rig):
        rig = make_rig()
        rig.unit._status = rig.status(op_state=proto.OP_STATE_RUNNING)
        assert rig.unit._unit_busy() is True

    def test_firmware_idle(self, make_rig):
        rig = make_rig()
        rig.unit._status = rig.status(op_state=proto.OP_STATE_DONE)
        assert rig.unit._unit_busy() is False

    def test_restoring(self, make_rig):
        rig = make_rig()
        rig.unit._restoring = True
        assert rig.unit._unit_busy() is True


SLOT_CAPS = proto.CAP_TAIL_RETRACT | proto.CAP_VOLATILE_SLOTS


class TestSlotColors:
    def _rig(self, make_rig):
        rig = make_rig(caps=SLOT_CAPS).start()
        rig.reactor.advance(0)
        rig.bmcu.received.clear()
        return rig

    def test_lane_colors_reach_the_board(self, make_rig):
        rig = make_rig(caps=SLOT_CAPS)
        rig.lane("bmcu1").color = "#FF8000"
        rig.lane("bmcu2").color = "00ff10"
        rig.lane("bmcu1").material = "PLA"
        rig.lane("bmcu2").material = "PETG"
        rig.start()
        rig.reactor.advance(0)
        assert rig.bmcu.slot_colors == [(255, 128, 0), (0, 255, 16), (0, 0, 0), (0, 0, 0)]
        assert rig.bmcu.slot_names == ["PLA", "PETG", "", ""]

    def test_a_color_change_is_sent_on_the_next_status(self, make_rig):
        rig = self._rig(make_rig)
        rig.lane("bmcu2").color = "#0000FF"
        rig.bmcu.send_status()
        rig.reactor.advance(0)
        assert rig.bmcu.slot_colors[1] == (0, 0, 255)
        sent = len(rig.bmcu.frames(proto.MSG_SET_SLOTS))
        rig.bmcu.send_status()
        rig.reactor.advance(0)
        assert len(rig.bmcu.frames(proto.MSG_SET_SLOTS)) == sent

    def test_colors_are_sent_again_after_the_board_reboots(self, make_rig):
        rig = make_rig(caps=SLOT_CAPS)
        rig.lane("bmcu1").color = "#123456"
        rig.start()
        rig.reactor.advance(0)
        rig.bmcu.reboot()
        assert rig.bmcu.slot_colors[0] == (0, 0, 0)
        rig.reactor.advance(20.0)
        assert rig.unit._conn.connected is True
        assert rig.bmcu.slot_colors[0] == (0x12, 0x34, 0x56)

    def test_firmware_without_volatile_slots_gets_none(self, make_rig):
        rig = make_rig()
        rig.lane("bmcu1").color = "#FF0000"
        rig.start()
        rig.reactor.advance(0)
        assert rig.bmcu.slot_colors == [(0, 0, 0)] * 4

    def test_an_invalid_color_is_black(self, make_rig):
        rig = make_rig(caps=SLOT_CAPS)
        rig.lane("bmcu1").color = "#GG0000"
        rig.lane("bmcu2").color = "#FFF"
        rig.start()
        rig.reactor.advance(0)
        assert rig.bmcu.slot_colors[:2] == [(0, 0, 0), (0, 0, 0)]


class TestMaybeFollow:
    @pytest.fixture
    def rig(self, rig):
        _printing(rig.lane())
        return rig

    def test_not_armed_when_the_toolhead_sensor_is_empty(self, rig):
        rig.extruder.tool_start_state = False
        rig.unit._maybe_follow(rig.status(present=(1, 0, 0, 0)))
        assert rig.unit._follow_pending is False

    def _follow(self, rig, **fields):
        fields.setdefault("present", (1, 0, 0, 0))
        status = rig.status(**fields)
        rig.unit._status = status
        rig.unit._maybe_follow(status)
        rig.reactor.advance(0)
        return [(f.msg_type, f.payload) for f in rig.bmcu.motion_frames()]

    def test_idle_printing_lane_resumes_on_use(self, rig):
        assert self._follow(rig) == [(proto.MSG_SET_MOTION, b"\x00\x03")]
        assert rig.unit._follow_pending is False

    def test_marks_pending_until_sent(self, rig):
        status = rig.status(present=(1, 0, 0, 0))
        rig.unit._maybe_follow(status)
        assert rig.unit._follow_pending is True

    def test_no_current_lane(self, rig):
        rig.afc.current = None
        assert self._follow(rig) == []

    def test_current_lane_on_another_unit(self, rig):
        rig.afc.current = "lane1"
        assert self._follow(rig) == []

    def test_already_pending(self, rig):
        rig.unit._follow_pending = True
        assert self._follow(rig) == []

    def test_busy(self, rig):
        rig.unit._op_active = True
        assert self._follow(rig) == []

    def test_lane_not_tool_loaded(self, rig):
        rig.lane("bmcu1").tool_loaded = False
        assert self._follow(rig) == []

    def test_not_printing(self, rig):
        rig.afc.function.is_printing.return_value = False
        assert self._follow(rig) == []

    def test_lane_already_moving(self, rig):
        assert self._follow(rig, motion=(proto.MOTION_ON_USE, 0, 0, 0)) == []

    def test_inlet_empty(self, rig):
        assert self._follow(rig, present=(0, 0, 0, 0)) == []


class TestHandlePrinting:
    def test_registered_for_idle_timeout_printing(self, make_rig):
        rig = make_rig()
        assert rig.printer._event_handlers["idle_timeout:printing"] == [
            rig.unit._handle_printing]

    def test_defers_a_status_refresh(self, rig):
        rig.unit._handle_printing(55.0)
        assert rig.bmcu.received == []
        rig.reactor.advance(0)
        assert rig.types() == [proto.MSG_GET_STATUS]

    def test_a_resume_clears_the_runout_latch(self, rig):
        rig.unit._runout_dispatched.add("bmcu1")
        rig.unit._handle_printing(1.0)
        assert rig.unit._runout_dispatched == set()


class TestRefreshFollow:
    def test_resumes_on_use_for_the_printing_lane(self, rig):
        rig.bmcu.channels[0].present = 1
        _printing(rig.lane())
        rig.unit._refresh_follow(rig.reactor.now)
        rig.reactor.advance(0)
        assert rig.types() == [proto.MSG_GET_STATUS, proto.MSG_SET_MOTION]
        assert rig.bmcu.channels[0].motion == proto.MOTION_ON_USE

    def test_runout_during_a_pause_is_dispatched_on_resume(self, make_rig):
        rig = make_rig(spools=()).start()
        lane = _printing(rig.lane())
        # The refresh's own reply decides, not the STATUS callback's level trigger.
        rig.unit._conn._on_status = lambda status: None
        rig.unit._refresh_follow(rig.reactor.now)
        lane.handle_load_runout.assert_not_called()
        assert rig.unit._runout_pending == {"bmcu1"}
        rig.reactor.advance(0.3)
        lane.handle_load_runout.assert_called_once_with(rig.reactor.now, False)

    def test_status_failure_does_not_check_runout(self, rig):
        lane = _printing(rig.lane())
        rig.unit._conn.request = MagicMock(side_effect=BMCUError("busy"))
        rig.unit._refresh_follow(rig.reactor.now)
        lane.handle_load_runout.assert_not_called()

    def test_no_current_lane(self, rig):
        rig.afc.function.is_printing.return_value = True
        rig.unit._refresh_follow(rig.reactor.now)
        rig.lane().handle_load_runout.assert_not_called()

    def test_current_lane_not_tool_loaded(self, rig):
        rig.afc.current = "bmcu1"
        rig.afc.function.is_printing.return_value = True
        rig.unit._refresh_follow(rig.reactor.now)
        rig.lane().handle_load_runout.assert_not_called()

    def test_current_lane_still_fed(self, rig):
        rig.bmcu.channels[0].present = 1
        lane = _printing(rig.lane())
        rig.unit._refresh_follow(rig.reactor.now)
        lane.handle_load_runout.assert_not_called()


class TestResumeFollow:
    def test_sends_on_use(self, rig):
        rig.unit._follow_pending = True
        rig.unit._resume_follow(1)
        assert [(f.msg_type, f.payload) for f in rig.bmcu.motion_frames()] == [
            (proto.MSG_SET_MOTION, b"\x01\x03")]
        assert rig.unit._follow_pending is False
        assert rig.logger.messages == []

    def test_operation_started_since_skips(self, rig):
        rig.unit._follow_pending = True
        rig.unit._op_active = True
        rig.unit._resume_follow(1)
        assert rig.bmcu.received == []
        assert rig.unit._follow_pending is False

    def test_failure_warns_and_clears_pending(self, rig):
        rig.unit._follow_pending = True
        rig.unit._conn.request = MagicMock(side_effect=BMCUError("gone"))
        rig.unit._resume_follow(1)
        assert rig.unit._follow_pending is False
        assert rig.logger.messages == [
            ("warning", "BMCU BMCU_1 could not resume following the extruder: gone")]


class TestDispatchLoadRunout:
    def test_runs_the_lane_handler(self, make_rig):
        rig = make_rig()
        lane = rig.lane("bmcu1")
        rig.unit._dispatch_load_runout(lane, 12.5, False)
        lane.handle_load_runout.assert_called_once_with(12.5, False)
        rig.afc.error.AFC_error.assert_not_called()
        assert rig.logger.messages == []

    @pytest.mark.parametrize("load_state,what", [(True, "load"), (False, "runout")])
    def test_failure_raises_afc_error(self, make_rig, load_state, what):
        rig = make_rig()
        rig.afc.function.in_print.return_value = True
        lane = rig.lane("bmcu1")
        lane.handle_load_runout.side_effect = RuntimeError("jammed")
        rig.unit._dispatch_load_runout(lane, 1.0, load_state)
        assert rig.logger.messages == [("error", f"bmcu1: {what} handling failed: jammed")]
        assert rig.errors() == [call(
            f"Lane bmcu1 {what} handling failed: jammed\nFilament may still be loaded in "
            "the toolhead, check the toolhead before resuming.", pause=True)]

    def test_failed_runout_handling_clears_the_latch(self, make_rig):
        rig = make_rig()
        lane = rig.lane("bmcu1")
        rig.unit._runout_dispatched.add("bmcu1")
        lane.handle_load_runout.side_effect = RuntimeError("jammed")
        rig.unit._dispatch_load_runout(lane, 1.0, False)
        assert rig.unit._runout_dispatched == set()


# Link loss

class TestOnLink:
    def test_offline_defers_link_lost(self, make_rig):
        rig = make_rig()
        rig.unit._handle_link_lost = MagicMock()
        rig.unit._handle_link_restored = MagicMock()
        rig.unit._on_link(False)
        rig.unit._handle_link_lost.assert_not_called()
        rig.reactor.advance(0)
        rig.unit._handle_link_lost.assert_called_once_with(rig.reactor.now)
        rig.unit._handle_link_restored.assert_not_called()

    def test_online_defers_link_restored(self, make_rig):
        rig = make_rig()
        rig.unit._handle_link_lost = MagicMock()
        rig.unit._handle_link_restored = MagicMock()
        rig.unit._on_link(True)
        rig.reactor.advance(0)
        rig.unit._handle_link_restored.assert_called_once_with(rig.reactor.now)
        rig.unit._handle_link_lost.assert_not_called()

    def test_online_marks_restoring_at_once(self, make_rig):
        rig = make_rig()
        rig.unit._handle_link_restored = MagicMock()
        rig.unit._on_link(True)
        assert rig.unit._restoring is True

    def test_offline_does_not_mark_restoring(self, make_rig):
        rig = make_rig()
        rig.unit._handle_link_lost = MagicMock()
        rig.unit._on_link(False)
        assert rig.unit._restoring is False


class TestHandleLinkLost:
    def test_pause_while_printing(self, rig):
        rig.afc.function.is_printing.return_value = True
        rig.bmcu.unplug()
        rig.reactor.advance(0)
        assert rig.errors() == [call("BMCU BMCU_1 disconnected during a print. "
                                     "Reconnect it, then resume.", pause=True)]
        assert rig.logger.messages == [
            ("warning", f"BMCU link lost on {PORT}: {UNPLUGGED}"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
        ]
        assert rig.unit._prev_present is None

    def test_idle_only_warns(self, rig):
        rig.unit._handle_link_lost(rig.reactor.now)
        rig.afc.error.AFC_error.assert_not_called()
        assert rig.logger.messages == [("warning", "BMCU BMCU_1 disconnected.")]
        assert rig.unit._prev_present is None

    def test_reconnect_sends_no_motion(self, rig):
        _in_toolhead(rig.lane("bmcu1"))
        rig.bmcu.channels[0].motion = proto.MOTION_ON_USE
        rig.bmcu.unplug()
        rig.reactor.advance(0.5)
        rig.bmcu.replug()
        rig.bmcu.reboot()
        rig.reactor.advance(5.0)
        assert rig.unit._conn.connected is True
        assert rig.bmcu.motion_frames() == []
        assert proto.MSG_RUNTIME_SYNC in rig.types()
        assert rig.types()[-4:] == [proto.MSG_STOP_ALL, proto.MSG_MARK_LOADED,
                                    proto.MSG_MARK_UNLOADED, proto.MSG_GET_STATUS]
        assert rig.bmcu.channels[0].motion == proto.MOTION_IDLE
        assert rig.logger.messages == [
            ("warning", f"BMCU link lost on {PORT}: {UNPLUGGED}"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
            ("warning", "BMCU BMCU_1 disconnected."),
            CONNECTED_INFO,
            ("info", "BMCU BMCU_1 reconnected, every lane is idle."),
        ]


class TestHandleLinkRestored:
    def test_syncs_stops_and_reconciles(self, rig):
        rig.unit._handle_link_restored(rig.reactor.now)
        assert rig.types() == [proto.MSG_RUNTIME_SYNC, proto.MSG_STOP_ALL,
                               proto.MSG_MARK_UNLOADED, proto.MSG_MARK_UNLOADED,
                               proto.MSG_GET_STATUS]
        assert rig.logger.messages == [("info", "BMCU BMCU_1 reconnected, every lane is idle.")]

    def test_uncertain_routes_follow_afc_without_motion(self, rig):
        rig.bmcu.channels[0].route = proto.ROUTE_UNCERTAIN
        rig.bmcu.channels[1].route = proto.ROUTE_UNCERTAIN
        _in_toolhead(rig.lane("bmcu1"))
        rig.unit._handle_link_restored(rig.reactor.now)
        assert rig.types() == [proto.MSG_RUNTIME_SYNC, proto.MSG_STOP_ALL,
                               proto.MSG_MARK_LOADED, proto.MSG_MARK_UNLOADED,
                               proto.MSG_GET_STATUS]
        assert rig.bmcu.channels[0].route == proto.ROUTE_LOADED
        assert rig.bmcu.channels[1].route == proto.ROUTE_EMPTY
        assert rig.bmcu.motion_frames() == []

    def test_refreshes_lane_state_and_leds(self, rig):
        rig.bmcu.channels[1].present = 1
        tooled, staged = rig.lane("bmcu1"), rig.lane("bmcu2")
        _in_toolhead(tooled)
        rig.extruder.lane_loaded = "bmcu1"
        rig.afc.current = "bmcu1"
        rig.unit._handle_link_restored(rig.reactor.now)
        assert tooled.status == AFCLaneState.TOOLED
        assert tooled.current_led_state == "tool_loaded"
        assert staged.status == AFCLaneState.LOADED
        assert staged.current_led_state == "loaded"
        assert staged.prep_state is True

    def test_sync_failure_drops_the_link(self, rig):
        rig.unit._sync_runtime = MagicMock(side_effect=BMCUError("refused"))
        rig.unit._handle_link_restored(rig.reactor.now)
        assert rig.unit._conn.connected is False
        assert rig.bmcu.received == []
        assert rig.logger.messages == [
            ("error", "BMCU BMCU_1 RUNTIME_SYNC failed: refused"),
            ("warning", f"BMCU link lost on {PORT}: RUNTIME_SYNC failed: refused"),
            ("info", f"BMCU reconnecting to {PORT} in 1s"),
        ]

    def test_failure_drops_the_link_to_retry_the_restore(self, rig):
        rig.unit._conn.stop_all = MagicMock(side_effect=BMCUError("timeout"))
        rig.unit._handle_link_restored(rig.reactor.now)
        assert rig.types() == [proto.MSG_RUNTIME_SYNC]
        assert rig.unit._conn.connected is False
        assert rig.logger.messages == [
            ("warning", "BMCU BMCU_1 reconnect setup failed: timeout"),
            ("warning", f"BMCU link lost on {PORT}: reconnect setup failed: timeout"),
            ("info", f"BMCU reconnecting to {PORT} in 1s")]

    def test_clears_restoring_and_resets_the_backoff(self, rig):
        rig.unit._conn._backoff = 16.0
        rig.unit._restoring = True
        rig.unit._handle_link_restored(rig.reactor.now)
        assert rig.unit._restoring is False
        assert rig.unit._conn.reconnect_delay == 1.0

    def test_failure_clears_restoring_and_backs_off(self, rig):
        rig.unit._conn._backoff = 16.0
        rig.unit._restoring = True
        rig.unit._conn.stop_all = MagicMock(side_effect=BMCUError("timeout"))
        rig.unit._handle_link_restored(rig.reactor.now)
        assert rig.unit._restoring is False
        assert rig.unit._conn.reconnect_delay == 30.0

    def test_load_refused_until_the_restore_finishes(self, rig):
        lane = rig.lane()
        rig.unit._restoring = True
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "load failed: the BMCU is still being set up after a reconnect",
            pause=False)]
        assert rig.types() == [proto.MSG_STOP_ALL]


# Helpers

class TestIsVirtualHub:
    def test_virtual(self, make_rig):
        rig = make_rig()
        assert rig.unit._is_virtual_hub(rig.lane()) is True

    def test_real(self, make_rig):
        rig = make_rig(virtual=False)
        assert rig.unit._is_virtual_hub(rig.lane()) is False

    def test_no_hub(self, make_rig):
        rig = make_rig()
        rig.lane().hub_obj = None
        assert rig.unit._is_virtual_hub(rig.lane()) is False


class TestRealHub:
    def test_real(self, make_rig):
        rig = make_rig(virtual=False)
        assert rig.unit._real_hub(rig.lane()) is rig.hub

    def test_virtual(self, make_rig):
        rig = make_rig()
        assert rig.unit._real_hub(rig.lane()) is None

    def test_no_hub(self, make_rig):
        rig = make_rig()
        rig.lane().hub_obj = None
        assert rig.unit._real_hub(rig.lane()) is None


class TestBowdenLength:
    def test_hub_bowden(self, make_rig):
        rig = make_rig(bowden=850.0)
        assert rig.unit._bowden_length(rig.lane()) == 850.0

    def test_dist_hub_when_hub_uses_it(self, make_rig):
        rig = make_rig(bowden=850.0)
        rig.hub.use_dist_hub = True
        rig.lane().dist_hub = 420.0
        assert rig.unit._bowden_length(rig.lane()) == 420.0

    def test_no_hub(self, make_rig):
        rig = make_rig()
        rig.lane().hub_obj = None
        assert rig.unit._bowden_length(rig.lane()) == 0.0


class TestConnected:
    def test_no_connection(self, make_rig):
        assert make_rig().unit._connected() is False

    def test_not_yet_connected(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        assert rig.unit._connected() is False

    def test_connected(self, rig):
        assert rig.unit._connected() is True


class TestCalibrated:
    def test_no_status(self, make_rig):
        assert make_rig().unit._calibrated(0) is False

    def test_mask_bit(self, make_rig):
        rig = make_rig()
        rig.unit._status = rig.status(cal_valid_mask=0b0101)
        assert rig.unit._calibrated(0) is True
        assert rig.unit._calibrated(1) is False
        assert rig.unit._calibrated(2) is True
        assert rig.unit._calibrated(3) is False


class TestCalibrateCommand:
    def test_names_unit_and_lane(self, make_rig):
        rig = make_rig()
        assert rig.unit._calibrate_command(rig.lane("bmcu2")) == (
            "AFC_BMCU_CALIBRATE UNIT=BMCU_1 LANE=bmcu2")


class TestOpTimeout:
    def test_twice_the_travel_time_plus_the_margin(self, make_rig):
        unit = make_rig(values={"feed_timeout": 1.0}).unit
        assert unit._op_timeout(600.0, 60.0) == pytest.approx(21.5)

    def test_feed_timeout_is_the_floor(self, make_rig):
        unit = make_rig(values={"feed_timeout": 30.0}).unit
        assert unit._op_timeout(100.0, 60.0) == 30.0

    def test_capped_at_the_firmware_limit(self, make_rig):
        unit = make_rig(values={"feed_timeout": 1.0}).unit
        assert unit._op_timeout(5000.0, 10.0) == 300.0


class TestSameSession:
    def test_same_generation_passes(self, rig):
        assert rig.unit._same_session(rig.unit._conn.generation) is None

    def test_reconnect_raises(self, rig):
        with pytest.raises(BMCUError, match="the BMCU reconnected during the operation "
                                            "and lost its state"):
            rig.unit._same_session(rig.unit._conn.generation - 1)


class TestOperation:
    def test_marks_busy_and_yields_the_generation(self, rig):
        with rig.unit._operation() as generation:
            assert rig.unit._op_active is True
            assert generation == rig.unit._conn.generation == 1
        assert rig.unit._op_active is False

    def test_clears_busy_on_error(self, rig):
        with pytest.raises(RuntimeError):
            with rig.unit._operation():
                raise RuntimeError("boom")
        assert rig.unit._op_active is False

    def test_refused_while_restoring(self, rig):
        rig.unit._restoring = True
        with pytest.raises(BMCUError, match="the BMCU is still being set up after a "
                                            "reconnect"):
            with rig.unit._operation():
                pass
        assert rig.unit._op_active is False


class TestMark:
    @pytest.mark.parametrize("loaded,msg_type", [(True, proto.MSG_MARK_LOADED),
                                                 (False, proto.MSG_MARK_UNLOADED)])
    def test_sends_the_mark(self, rig, loaded, msg_type):
        rig.unit._mark(2, loaded)
        assert [(f.msg_type, f.payload) for f in rig.bmcu.received] == [(msg_type, b"\x02")]
        assert rig.bmcu.motion_frames() == []


class TestTimeoutMs:
    def test_seconds_to_ms(self, make_rig):
        unit = make_rig().unit
        assert unit._timeout_ms(30.0) == 30000
        assert unit._timeout_ms(1.2346) == 1235


class TestRefreshStatus:
    def test_requests_and_returns_status(self, make_rig):
        rig = make_rig(spools=()).start()
        rig.bmcu.channels[2].present = 1
        status = rig.unit._refresh_status()
        assert rig.types() == [proto.MSG_GET_STATUS]
        assert status["present"] == [0, 0, 1, 0]
        assert rig.unit._status["present"] == [0, 0, 1, 0]

    def test_returns_the_reply_even_when_the_callback_discards_it(self, make_rig):
        rig = make_rig(spools=()).start()
        rig.unit._status = rig.status(present=(1, 1, 1, 1))
        rig.unit._conn._on_status = lambda status: None
        rig.bmcu.channels[3].present = 1
        assert rig.unit._refresh_status()["present"] == [0, 0, 0, 1]

    def test_bad_reply_raises(self, rig):
        rig.unit._conn.request = MagicMock(return_value=(proto.MSG_STATUS, b"short"))
        with pytest.raises(BMCUError, match="BMCU sent a bad STATUS: invalid status payload "
                                            "length 5"):
            rig.unit._refresh_status()


class TestRefreshQuietly:
    def test_returns_the_status(self, make_rig):
        rig = make_rig(spools=()).start()
        rig.bmcu.channels[1].present = 1
        assert rig.unit._refresh_quietly(rig.reactor.now)["present"] == [0, 1, 0, 0]
        assert rig.types() == [proto.MSG_GET_STATUS]

    def test_not_connected(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        assert rig.unit._refresh_quietly(rig.reactor.now) is None
        assert rig.bmcu.received == []
        assert rig.logger.messages == []

    def test_failure_is_logged(self, rig):
        rig.unit._conn.request = MagicMock(side_effect=BMCUError("busy"))
        assert rig.unit._refresh_quietly(rig.reactor.now) is None
        assert rig.logger.messages == [("debug", "BMCU BMCU_1 status refresh failed: busy")]


class TestSetMotion:
    def test_sends_set_motion(self, rig):
        rig.unit._set_motion(3, proto.MOTION_PULL_BACK)
        assert [(f.msg_type, f.payload) for f in rig.bmcu.received] == [
            (proto.MSG_SET_MOTION, b"\x03\x05")]
        assert rig.bmcu.channels[3].motion == proto.MOTION_PULL_BACK


class TestRunOp:
    def test_starts_and_waits(self, rig):
        result = rig.unit._run_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 20.0, 90, 5000),
                                  5.0)
        assert result["ok"] is True
        assert result["reason"] == "target"
        assert rig.types() == [proto.MSG_FEED_DISTANCE]

    # A 2000 mm feed at 60 mm/s runs past the 5 s the host is told about; the
    # firmware's own 30 s timeout is far off.
    LONG_FEED = proto.pack_feed(0, 2000.0, 90, 30000)

    def test_waits_past_the_firmware_timeout(self, rig):
        rig.later(9.0, lambda: rig.bmcu.complete_op(rig.bmcu.running_ops()[0]))
        result = rig.unit._run_op(proto.MSG_FEED_DISTANCE, self.LONG_FEED, 5.0)
        assert result["ok"] is True

    def test_gives_up_after_the_grace(self, rig):
        start = rig.reactor.now
        with pytest.raises(link.BMCUTimeoutError):
            rig.unit._run_op(proto.MSG_FEED_DISTANCE, self.LONG_FEED, 5.0)
        assert rig.reactor.now - start == pytest.approx(10.0, abs=0.01)


class TestStopAfterFailure:
    def test_sends_stop_all(self, rig):
        rig.unit._stop_after_failure()
        assert rig.types() == [proto.MSG_STOP_ALL]
        assert rig.logger.messages == []

    def test_dead_link_is_logged(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.unit._stop_after_failure()
        assert rig.logger.messages == [
            ("debug", f"BMCU BMCU_1 stop after failure: BMCU on {PORT} is not connected")]


class TestAbortQuietly:
    def _open_op(self, rig):
        op_id = rig.unit._conn.start_op(proto.MSG_FEED_DISTANCE, proto.pack_feed(0, 50.0, 70, 3000))
        rig.bmcu.received.clear()
        return op_id

    def test_sends_abort_while_the_op_is_open(self, rig):
        op_id = self._open_op(rig)
        rig.unit._abort_quietly(op_id)
        assert rig.types() == [proto.MSG_ABORT_OP]
        assert rig.logger.messages == []

    def test_finished_op_is_not_aborted(self, rig):
        op_id = self._open_op(rig)
        rig.bmcu.complete_op(op_id)
        rig.reactor.advance(0)
        rig.bmcu.received.clear()
        rig.unit._abort_quietly(op_id)
        assert rig.bmcu.received == []

    def test_failure_is_logged(self, rig):
        op_id = self._open_op(rig)
        rig.unit._conn.abort_op = MagicMock(side_effect=BMCUError("timeout"))
        rig.unit._abort_quietly(op_id)
        assert rig.logger.messages == [("debug", "BMCU BMCU_1 abort failed: timeout")]


class TestWaitMotionIdle:
    def test_already_idle(self, rig):
        start = rig.reactor.now
        assert rig.unit._wait_motion_idle(0, 5.0) is None
        assert rig.reactor.now == start

    def test_becomes_idle(self, rig):
        rig.bmcu.channels[1].motion = proto.MOTION_PULL_BACK
        start = rig.reactor.now
        rig.later(1.1, lambda: setattr(rig.bmcu.channels[1], "motion", proto.MOTION_IDLE))
        assert rig.unit._wait_motion_idle(1, 5.0) is None
        assert rig.reactor.now - start == pytest.approx(1.25)

    def test_times_out(self, rig):
        rig.bmcu.channels[1].motion = proto.MOTION_PULL_BACK
        start = rig.reactor.now
        assert rig.unit._wait_motion_idle(1, 2.0) == "pull back did not finish within 2s."
        assert rig.reactor.now - start == pytest.approx(2.0)

    @pytest.mark.parametrize("buffer_pct", [2, 98])
    def test_buffer_held_at_an_extreme_is_a_jam(self, rig, buffer_pct):
        rig.bmcu.channels[1].motion = proto.MOTION_PULL_BACK
        rig.bmcu.channels[1].buffer_pct = buffer_pct
        start = rig.reactor.now
        assert rig.unit._wait_motion_idle(1, 5.0) == (
            f"jammed: the buffer stayed at {buffer_pct}% during the pull back.")
        assert rig.reactor.now - start == pytest.approx(0.5)

    @pytest.mark.parametrize("buffer_pct", [3, 97])
    def test_buffer_inside_the_jam_limits_is_not_a_jam(self, rig, buffer_pct):
        rig.bmcu.channels[1].motion = proto.MOTION_PULL_BACK
        rig.bmcu.channels[1].buffer_pct = buffer_pct
        assert rig.unit._wait_motion_idle(1, 2.0) == "pull back did not finish within 2s."

    def test_a_recovered_buffer_restarts_the_count(self, rig):
        channel = rig.bmcu.channels[1]
        channel.motion = proto.MOTION_PULL_BACK
        channel.buffer_pct = 2
        rig.later(0.3, lambda: setattr(channel, "buffer_pct", 50))
        rig.later(0.6, lambda: setattr(channel, "buffer_pct", 2))
        rig.later(1.0, lambda: setattr(channel, "motion", proto.MOTION_IDLE))
        assert rig.unit._wait_motion_idle(1, 5.0) is None

    def test_short_extreme_run_is_not_a_jam(self, rig):
        # The deadline shortens the last poll: three extreme polls at 0, 0.25
        # and 0.3 s span less than JAM_MIN_TIME, so this is a timeout, not a jam.
        channel = rig.bmcu.channels[1]
        channel.motion = proto.MOTION_PULL_BACK
        channel.buffer_pct = 2
        start = rig.reactor.now
        assert rig.unit._wait_motion_idle(1, 0.3) == "pull back did not finish within 0s."
        assert rig.reactor.now - start == pytest.approx(0.3)
        assert len(rig.bmcu.frames(proto.MSG_GET_STATUS)) == 3

    def test_a_jam_needs_three_extreme_polls(self, rig):
        # Slow replies space the polls 0.45 s apart, past JAM_MIN_TIME after two,
        # so only the sample count holds the jam back to the third poll.
        rig.bmcu.reply_delay = 0.2
        channel = rig.bmcu.channels[1]
        channel.motion = proto.MOTION_PULL_BACK
        channel.buffer_pct = 98
        start = rig.reactor.now
        assert rig.unit._wait_motion_idle(1, 5.0) == (
            "jammed: the buffer stayed at 98% during the pull back.")
        assert rig.reactor.now - start == pytest.approx(1.1)
        assert len(rig.bmcu.frames(proto.MSG_GET_STATUS)) == 3


# AFC hooks

class TestCheckRunout:
    def test_current_lane_while_printing(self, make_rig):
        rig = make_rig()
        rig.afc.current = "bmcu1"
        rig.afc.function.is_printing.return_value = True
        assert rig.unit.check_runout(rig.lane("bmcu1")) is True

    def test_current_lane_not_printing(self, make_rig):
        rig = make_rig()
        rig.afc.current = "bmcu1"
        assert rig.unit.check_runout(rig.lane("bmcu1")) is False

    def test_other_lane_while_printing(self, make_rig):
        rig = make_rig()
        rig.afc.current = "bmcu2"
        rig.afc.function.is_printing.return_value = True
        assert rig.unit.check_runout(rig.lane("bmcu1")) is False


class TestHandleRunout:
    def test_no_capability_pauses_and_unloads_the_lane(self, make_rig):
        rig = make_rig(caps=0).start()
        lane = rig.lane()
        _in_toolhead(lane)
        lane.status = AFCLaneState.TOOLED
        assert rig.unit.handle_runout(lane) is True
        assert rig.errors() == [call("bmcu1 ran out. This BMCU firmware cannot retract the "
                                     "tail: clear the bmcu1 path by hand, load a lane, then "
                                     "resume.", pause=True)]
        assert lane.tool_loaded is False
        assert lane.status == AFCLaneState.NONE
        assert rig.afc.spool.set_active_spool.call_args_list == [call(None)]
        assert [f.payload for f in rig.bmcu.frames(proto.MSG_MARK_UNLOADED)] == [b"\x00"]
        assert lane.current_led_state == "not_ready"
        rig.afc.save_vars.assert_called_once_with()

    def test_capability_defers_to_afc(self, rig):
        lane = rig.lane()
        lane.status = AFCLaneState.TOOLED
        assert rig.unit.handle_runout(lane) is False
        rig.afc.error.AFC_error.assert_not_called()
        assert lane.status == AFCLaneState.TOOLED


class TestGetStatus:
    def test_lane_values_are_none_while_disconnected(self, rig):
        rig.unit._conn.close()
        lanes = rig.unit.get_status()["bmcu_lanes"]
        assert lanes["bmcu1"]["buffer_pct"] is None
        assert lanes["bmcu1"]["route"] is None
        assert lanes["bmcu1"]["calibrated"] is None

    def test_before_connect(self, make_rig):
        status = make_rig().unit.get_status()
        assert status["connected"] is False
        assert status["half_duplex"] is False
        assert status["firmware"] is None
        assert status["tail_retract"] is False
        assert status["bmcu_lanes"] == {}
        assert status["lanes"] == []

    def test_connected_without_status(self, make_rig):
        rig = make_rig().start(status=False)
        rig.unit._status = None
        status = rig.unit.get_status()
        assert status["connected"] is True
        assert status["firmware"] == "1.0.4"
        assert status["tail_retract"] is True
        assert status["bmcu_lanes"]["bmcu1"] == {
            "channel": 1, "buffer_pct": None, "calibrated": None, "route": None,
            "inlet": None, "motion": None, "encoder_ok": None}
        assert status["fault_flags"] is None
        assert status["nvm_fault"] is None

    def test_per_lane_state(self, make_rig):
        rig = make_rig(values={"half_duplex": True}, caps=0).start()
        rig.unit._status = rig.status(buffer_pct=(40, 61, 0, 0), cal_valid_mask=0b10,
                                      route=(proto.ROUTE_UNCERTAIN, proto.ROUTE_LOADED, 0, 0),
                                      present=(0, 1, 0, 0), encoder_mask=0b1101,
                                      motion=(proto.MOTION_PULL_BACK, proto.MOTION_ON_USE, 0, 0),
                                      error_flags=0x0003)
        status = rig.unit.get_status()
        assert status["half_duplex"] is True
        assert status["tail_retract"] is False
        assert status["last_error"] == ""
        assert status["fault_flags"] == 0x0001
        assert status["nvm_fault"] is True
        assert status["lanes"] == ["bmcu1", "bmcu2"]
        assert status["bmcu_lanes"] == {
            "bmcu1": {"channel": 1, "buffer_pct": 40, "calibrated": False,
                      "route": "UNCERTAIN", "inlet": False, "motion": "PULL_BACK",
                      "encoder_ok": True},
            "bmcu2": {"channel": 2, "buffer_pct": 61, "calibrated": True, "route": "LOADED",
                      "inlet": True, "motion": "ON_USE", "encoder_ok": False},
        }

    def test_last_error_names_why_the_link_dropped(self, rig):
        rig.bmcu.reboot()
        with pytest.raises(BMCUError):
            rig.unit._refresh_status()
        assert rig.unit.get_status()["last_error"] == (
            "BMCU session expired (firmware restarted)")


class TestWaitFirstConnect:
    def test_returns_at_once_when_not_connecting(self, make_rig):
        rig = make_rig()
        start = rig.reactor.now
        rig.unit._wait_first_connect()
        assert rig.reactor.now == start

    def test_waits_for_the_connect(self, make_rig):
        rig = make_rig()
        rig.unit._connecting = True
        start = rig.reactor.now
        rig.later(0.35, lambda: setattr(rig.unit, "_connecting", False))
        rig.unit._wait_first_connect()
        assert rig.reactor.now - start == pytest.approx(0.4)


class TestSystemTest:
    def _prep(self, rig, lane="bmcu1", assign=True):
        return rig.unit.system_Test(rig.lane(lane), 0.1, assign, False)

    def test_waits_for_the_connect_started_at_ready(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.unit.handle_ready()
        assert rig.unit._conn.connected is False
        assert self._prep(rig) is True
        assert rig.bmcu.frames(proto.MSG_RUNTIME_SYNC) != []
        assert rig.logger.messages == [
            CONNECTED_INFO,
            ("info", "bmcu1 tool cmd: T0  <span class=success--text>LOCKED AND LOADED</span>"),
        ]

    def test_not_connected(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        assert self._prep(rig) is False
        lane = rig.lane()
        assert lane.current_led_state == "fault"
        rig.afc.function.TcmdAssign.assert_called_once_with(lane)
        lane.send_lane_data.assert_called_once_with()
        lane.set_afc_prep_done.assert_called_once_with()
        assert rig.logger.messages == [
            ("debug", f"BMCU BMCU_1 PREP: BMCU on {PORT} is not connected"),
            ("info", "bmcu1 tool cmd: T0  <span class=error--text>BMCU NOT CONNECTED</span>"),
        ]

    def test_firmware_refusing_the_mark_is_shown_not_called_a_disconnect(self, rig):
        _in_toolhead(rig.lane())
        rig.bmcu.nvm_ok = False
        assert self._prep(rig) is False
        assert rig.unit._conn.connected is True
        assert rig.lane().current_led_state == "fault"
        error = "BMCU error code=71 channel=0 details=0"
        assert rig.logger.messages == [
            ("debug", f"BMCU BMCU_1 PREP: {error}"),
            ("info", f"bmcu1 tool cmd: T0  <span class=error--text>BMCU REFUSED: {error}"
                     "</span>"),
        ]

    def test_status_request_failure_counts_as_not_connected(self, rig):
        rig.unit._refresh_status = MagicMock(side_effect=BMCUError("timed out"))
        assert self._prep(rig) is False
        assert rig.logger.messages == [
            ("debug", "BMCU BMCU_1 PREP: timed out"),
            ("info", "bmcu1 tool cmd: T0  <span class=error--text>BMCU NOT CONNECTED</span>"),
        ]

    def test_uncalibrated_names_command(self, rig):
        rig.bmcu.channels[0].calibrated = False
        rig.bmcu.channels[0].present = 1
        assert self._prep(rig, assign=False) is False
        lane = rig.lane()
        assert lane.current_led_state == "fault"
        assert lane.prep_state is True
        rig.afc.function.TcmdAssign.assert_not_called()
        assert rig.logger.messages == [
            ("info", "bmcu1 tool cmd: T0  <span class=error--text>NOT CALIBRATED, run "
                     "AFC_BMCU_CALIBRATE UNIT=BMCU_1 LANE=bmcu1</span>")]

    def test_empty_lane(self, make_rig):
        rig = make_rig(spools=()).start()
        assert self._prep(rig) is True
        lane = rig.lane()
        assert lane.prep_state is False
        assert lane.loaded_to_hub is False
        assert lane._load_state is False
        assert lane.status == AFCLaneState.NONE
        assert lane.current_led_state == "not_ready"
        assert rig.types() == [proto.MSG_MARK_UNLOADED, proto.MSG_GET_STATUS]
        assert rig.bmcu.motion_frames() == []
        assert rig.logger.messages == [
            ("info", "bmcu1 tool cmd: T0  <span class=success--text>EMPTY READY FOR "
                     "SPOOL</span>")]

    def test_spool_present(self, rig):
        rig.bmcu.channels[1].present = 1
        assert self._prep(rig, lane="bmcu2") is True
        lane = rig.lane("bmcu2")
        assert lane.prep_state is True
        assert lane.loaded_to_hub is True
        assert lane._load_state is False
        assert lane.status == AFCLaneState.LOADED
        assert lane.current_led_state == "loaded"
        assert rig.logger.messages == [
            ("info", "bmcu2 tool cmd: T1  <span class=success--text>LOCKED AND LOADED</span>")]

    def test_tool_loaded_current_lane(self, rig):
        rig.bmcu.channels[0].route = proto.ROUTE_UNCERTAIN
        lane = rig.lane()
        _in_toolhead(lane)
        rig.extruder.lane_loaded = "bmcu1"
        rig.afc.current = "bmcu1"
        assert self._prep(rig) is True
        assert lane.status == AFCLaneState.TOOLED
        assert lane.current_led_state == "tool_loaded"
        assert lane._load_state is True
        assert lane.loaded_to_hub is True
        rig.afc.spool.set_active_spool.assert_called_once_with(7)
        assert rig.types() == [proto.MSG_MARK_LOADED, proto.MSG_GET_STATUS]
        assert rig.bmcu.channels[0].route == proto.ROUTE_LOADED

    def test_tool_loaded_idle_tool(self, rig):
        lane = rig.lane()
        _in_toolhead(lane)
        rig.extruder.lane_loaded = "bmcu1"
        rig.afc.current = None
        assert self._prep(rig) is True
        assert lane.status == AFCLaneState.TOOLED
        assert lane.current_led_state == "tool_loaded_idle"
        rig.afc.spool.set_active_spool.assert_not_called()

    def test_tool_loaded_flag_without_extruder_match(self, rig):
        rig.bmcu.channels[0].present = 1
        lane = rig.lane()
        _in_toolhead(lane)
        rig.extruder.lane_loaded = "lane9"
        assert self._prep(rig) is False
        assert lane.current_led_state == "fault"

    def test_real_hub_load_state_follows_inlet(self, make_rig):
        rig = make_rig(virtual=False).start()
        rig.bmcu.channels[0].present = 1
        self._prep(rig)
        assert rig.lane()._load_state is True


class TestShowLane:
    def test_tool_loaded_seen_only_by_the_post_extruder_sensor_agrees(self, rig):
        lane = rig.lane()
        _in_toolhead(lane)
        rig.extruder.tool_start_state = False
        rig.extruder.tool_end_state = True
        usable, _msg = rig.unit._show_lane(lane, rig.status(present=(1, 0, 0, 0)))
        assert usable is True
        assert lane.status == AFCLaneState.TOOLED

    def test_tool_loaded_with_an_empty_toolhead_sensor_faults(self, rig):
        lane = rig.lane()
        _in_toolhead(lane)
        rig.extruder.tool_start_state = False
        usable, msg = rig.unit._show_lane(lane, rig.status(present=(1, 0, 0, 0)))
        assert usable is False
        assert "AFC has bmcu1 loaded but the toolhead sensor is empty." in msg
        assert lane.current_led_state == "fault"

    def test_tool_loaded_without_a_sensor_is_trusted(self, make_rig):
        rig = make_rig(tool_start=None).start()
        lane = rig.lane()
        _in_toolhead(lane)
        rig.extruder.tool_start_state = False
        usable, _msg = rig.unit._show_lane(lane, rig.status(present=(1, 0, 0, 0)))
        assert usable is True
        assert lane.status == AFCLaneState.TOOLED

    def test_uncalibrated(self, rig):
        status = rig.status(cal_valid_mask=0b1110, present=(1, 0, 0, 0))
        rig.unit._status = status
        assert rig.unit._show_lane(rig.lane(), status) == (
            False, "<span class=error--text>NOT CALIBRATED, run AFC_BMCU_CALIBRATE "
            "UNIT=BMCU_1 LANE=bmcu1</span>")
        assert rig.lane().current_led_state == "fault"
        assert rig.lane().prep_state is True

    def test_tool_loaded_current(self, rig):
        lane = rig.lane()
        _in_toolhead(lane)
        rig.extruder.lane_loaded = "bmcu1"
        rig.afc.current = "bmcu1"
        status = rig.status()
        assert rig.unit._show_lane(lane, status) == (
            True, "<span class=success--text>LOCKED AND LOADED</span>")
        assert lane.status == AFCLaneState.TOOLED
        assert lane.current_led_state == "tool_loaded"
        rig.afc.spool.set_active_spool.assert_called_once_with(7)

    def test_tool_loaded_idle(self, rig):
        lane = rig.lane()
        _in_toolhead(lane)
        rig.extruder.lane_loaded = "bmcu1"
        assert rig.unit._show_lane(lane, rig.status())[0] is True
        assert lane.current_led_state == "tool_loaded_idle"
        rig.afc.spool.set_active_spool.assert_not_called()

    def test_present(self, rig):
        lane = rig.lane()
        assert rig.unit._show_lane(lane, rig.status(present=(1, 0, 0, 0))) == (
            True, "<span class=success--text>LOCKED AND LOADED</span>")
        assert lane.status == AFCLaneState.LOADED
        assert lane.current_led_state == "loaded"

    def test_present_relights_the_spool_when_the_lane_led_is_unchanged(self, rig):
        # After a reconnect the lane LED already reads loaded, so lane_loaded
        # sets nothing; the spool light must still come back.
        lane = rig.lane()
        lane.led_spool_index = "AFC_Indicator:3"
        lane.current_led_state = "loaded"
        rig.unit._show_lane(lane, rig.status(present=(1, 0, 0, 0)))
        assert rig.afc.function.afc_led.call_args_list == [call("illum", "AFC_Indicator:3")]

    def test_empty(self, rig):
        lane = rig.lane()
        assert rig.unit._show_lane(lane, rig.status()) == (
            True, "<span class=success--text>EMPTY READY FOR SPOOL</span>")
        assert lane.current_led_state == "not_ready"


class TestCmdAfcBmcuCalibrate:
    def test_stops_and_unmarks_the_channel_before_calibrating(self, make_rig):
        rig = make_rig(spools=()).start()
        rig.bmcu.channels[0].route = proto.ROUTE_UNCERTAIN
        rig.bmcu.auto_complete[proto.MSG_CAL_AUTO_START] = (proto.OP_STATE_DONE, 1)
        rig.unit.cmd_AFC_BMCU_CALIBRATE(self._gcmd())
        sent = [t for t in rig.types() if t != proto.MSG_GET_STATUS]
        assert sent == [proto.MSG_STOP_ALL, proto.MSG_MARK_UNLOADED, proto.MSG_CAL_AUTO_START]
        assert rig.errors() == []

    def _gcmd(self, lane="bmcu1"):
        gcmd = MagicMock()
        gcmd.get.return_value = lane
        return gcmd

    def test_refuses_while_printing(self, rig):
        rig.afc.function.is_printing.return_value = True
        rig.unit.cmd_AFC_BMCU_CALIBRATE(self._gcmd())
        assert rig.errors() == [call("Cannot calibrate a BMCU lane while printing.",
                                     pause=False)]
        assert rig.bmcu.received == []

    def test_refuses_loaded_lane(self, rig):
        rig.lane().prep_state = True
        rig.unit.cmd_AFC_BMCU_CALIBRATE(self._gcmd())
        assert rig.errors() == [call("Remove the filament from bmcu1 before calibrating it.",
                                     pause=False)]
        assert rig.bmcu.received == []

    def test_refuses_tool_loaded_lane(self, rig):
        _in_toolhead(rig.lane())
        rig.unit.cmd_AFC_BMCU_CALIBRATE(self._gcmd())
        assert rig.errors() == [call("Remove the filament from bmcu1 before calibrating it.",
                                     pause=False)]

    def test_refuses_unknown_lane(self, rig):
        rig.unit.cmd_AFC_BMCU_CALIBRATE(self._gcmd("lane7"))
        assert rig.errors() == [call("lane7 is not a lane of BMCU BMCU_1.", pause=False)]

    def test_refuses_when_disconnected(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.unit.cmd_AFC_BMCU_CALIBRATE(self._gcmd())
        assert rig.errors() == [call("BMCU BMCU_1 is not connected.", pause=False)]

    def test_runs_cal_auto_start(self, make_rig):
        rig = make_rig(spools=()).start()
        busy = []
        rig.later(3.0, lambda: (busy.append(rig.unit._op_active),
                                rig.bmcu.complete_op(rig.bmcu.running_ops()[0])))
        rig.unit.cmd_AFC_BMCU_CALIBRATE(self._gcmd("bmcu2"))
        frames = rig.bmcu.frames(proto.MSG_CAL_AUTO_START)
        assert [f.payload for f in frames] == [b"\x02"]
        assert busy == [True]
        assert rig.unit._op_active is False
        rig.afc.error.AFC_error.assert_not_called()
        assert rig.logger.messages == [
            ("info", "Calibrating bmcu2: push its buffer all the way in, let it center, pull "
                     "it all the way out, let it center."),
            ("info", "bmcu2 calibrated."),
        ]

    def test_failed_calibration_reports_reason(self, make_rig):
        rig = make_rig(spools=()).start()
        rig.bmcu.auto_complete[proto.MSG_CAL_AUTO_START] = (proto.OP_STATE_FAILED, 12)
        rig.unit.cmd_AFC_BMCU_CALIBRATE(self._gcmd())
        assert rig.errors() == [call("bmcu1 calibration failed: calibration_range",
                                     pause=False)]

    def test_waits_for_the_person_then_gives_up(self, make_rig):
        rig = make_rig(spools=()).start()
        start = rig.reactor.now
        rig.unit.cmd_AFC_BMCU_CALIBRATE(self._gcmd())
        cal_id = rig.bmcu.frames(proto.MSG_CAL_AUTO_START)[0].cmd_id
        # CALIBRATION_TIMEOUT plus the 5 s grace
        assert rig.reactor.now - start == pytest.approx(305.0, abs=0.1)
        assert rig.errors() == [call(
            f"bmcu1 calibration failed: BMCU operation {cal_id} did not finish within 305s",
            pause=False)]
        assert rig.types()[-1] == proto.MSG_STOP_ALL
        assert rig.bmcu.running_ops() == []

    def test_unconfirmed_start_stops_every_motor_and_reports(self, make_rig):
        rig = make_rig(spools=()).start()
        rig.bmcu.add_fault("drop", proto.MSG_CAL_AUTO_START)
        rig.unit.cmd_AFC_BMCU_CALIBRATE(self._gcmd())
        cal_id = rig.bmcu.frames(proto.MSG_CAL_AUTO_START)[0].cmd_id
        assert rig.types() == [proto.MSG_STOP_ALL, proto.MSG_MARK_UNLOADED,
                               proto.MSG_CAL_AUTO_START, proto.MSG_ABORT_OP, proto.MSG_STOP_ALL]
        assert rig.unit._conn.connected is True
        assert rig.errors() == [call(
            f"bmcu1 calibration failed: BMCU request 0x1C cmd={cal_id} got no reply within 2s",
            pause=False)]
        assert rig.unit._op_active is False


# Load

def _feed_frames(rig):
    return [struct.unpack("<BfBI", f.payload) for f in rig.bmcu.frames(proto.MSG_FEED_TO_CONTACT)]


def _motions(rig):
    return [tuple(f.payload) for f in rig.bmcu.frames(proto.MSG_SET_MOTION)]


def _feed(rig, lane, channel, has_sensor):
    return rig.unit._feed_to_contact(lane, channel, has_sensor)


def _load_lane(rig):
    return rig.unit._load(rig.lane(), rig.extruder, 0, rig.unit._conn.generation)


def _unload_lane(rig):
    return rig.unit._unload(rig.lane(), rig.extruder, 0, rig.unit._conn.generation)


class TestHasToolheadSensor:
    @pytest.mark.parametrize("tool_start,expected", [
        ("PA1", True), (None, False), ("", False), ("virtual", False), ("buffer", False)])
    def test_kinds(self, make_rig, tool_start, expected):
        rig = make_rig(tool_start=tool_start)
        assert rig.unit._has_toolhead_sensor(rig.extruder) is expected


class TestExtrudeToToolEnd:
    def test_without_a_tool_end_sensor(self, make_rig):
        rig = make_rig()
        assert rig.unit._extrude_to_tool_end(rig.lane(), rig.extruder) == (None, 0.0)
        rig.afc.move_e_pos.assert_not_called()

    def test_short_moves_until_it_triggers(self, make_rig):
        rig = make_rig()
        rig.extruder.tool_end = "PA2"
        moves = []

        def extrude(amount, speed, log_string="", wait_tool=False):
            moves.append((amount, speed, log_string, wait_tool))
            rig.extruder.tool_end_state = len(moves) == 2

        rig.afc.move_e_pos.side_effect = extrude
        assert rig.unit._extrude_to_tool_end(rig.lane(), rig.extruder) == (None, 20.0)
        assert moves == [(10.0, 25.0, "Tool end", True)] * 2

    def test_already_triggered(self, make_rig):
        rig = make_rig()
        rig.extruder.tool_end = "PA2"
        rig.extruder.tool_end_state = True
        assert rig.unit._extrude_to_tool_end(rig.lane(), rig.extruder) == (None, 0.0)

    def test_gives_up_after_twenty_moves(self, make_rig):
        rig = make_rig()
        rig.extruder.tool_end = "PA2"
        assert rig.unit._extrude_to_tool_end(rig.lane(), rig.extruder) == (
            "did not reach the post-extruder sensor after 20 short moves.", 200.0)
        assert rig.afc.move_e_pos.call_count == 20


class TestFeedToContact:
    def test_sensor_load_feeds_to_the_high_buffer_limit(self, make_rig):
        rig = make_rig(values={"contact_pct": 70}).start()
        rig.later(SENSOR_REACHED_S, lambda: setattr(rig.extruder, "tool_start_state", True))
        _feed(rig, rig.lane(), 0, True)
        feed = rig.bmcu.frames(proto.MSG_FEED_TO_CONTACT)[0]
        assert feed.payload[5] == 98

    def test_sensorless_load_feeds_to_contact_pct(self, make_rig):
        rig = make_rig(values={"contact_pct": 70}).start()
        rig.later(1.0, lambda: rig.bmcu.complete_op(rig.bmcu.running_ops()[0]))
        _feed(rig, rig.lane(), 0, False)
        assert rig.bmcu.frames(proto.MSG_FEED_TO_CONTACT)[0].payload[5] == 70

    def test_feed_payload_and_result(self, make_rig):
        rig = make_rig(bowden=900.0).start()
        rig.bmcu.auto_complete[proto.MSG_FEED_TO_CONTACT] = (proto.OP_STATE_DONE, 10)
        result, hub_seen = _feed(rig, rig.lane("bmcu2"), 1, False)
        assert result["ok"] is True
        assert result["reason"] == "contact"
        assert hub_seen is False
        # bowden 900 + path_extra 100 + overshoot 50 = 1050 mm at 60 mm/s:
        # 1050 / 60 * 2 + 1.5 = 36.5 s
        assert _feed_frames(rig) == [(1, 1050.0, 70, 36500)]

    def test_feed_limited_to_firmware_maximum(self, make_rig):
        rig = make_rig(bowden=6000.0).start()
        rig.bmcu.auto_complete[proto.MSG_FEED_TO_CONTACT] = (proto.OP_STATE_DONE, 10)
        _feed(rig, rig.lane(), 0, False)
        assert _feed_frames(rig)[0][1] == 5000.0

    def test_toolhead_sensor_aborts_the_feed(self, rig):
        # Replies slower than the sensor poll: the feed is still open at the
        # next ticks, and only the first may send an ABORT_OP.
        rig.bmcu.reply_delay = 0.2
        start = rig.reactor.now
        rig.later(SENSOR_REACHED_S, lambda: setattr(rig.extruder, "tool_start_state", True))
        result, _ = _feed(rig, rig.lane(), 0, True)
        assert result["state"] == proto.OP_STATE_ABORTED
        assert rig.types().count(proto.MSG_ABORT_OP) == 1
        assert rig.reactor.now - start < SENSOR_REACHED_S + 2.2

    def test_sensor_ignored_without_toolhead_sensor(self, rig):
        rig.extruder.tool_start_state = True
        rig.later(3.0, lambda: rig.bmcu.complete_op(rig.bmcu.running_ops()[0]))
        result, _ = _feed(rig, rig.lane(), 0, False)
        assert result["ok"] is True
        assert proto.MSG_ABORT_OP not in rig.types()

    def test_real_hub_seen_during_the_feed(self, make_rig):
        rig = make_rig(virtual=False).start()
        rig.later(1.0, lambda: setattr(rig.hub, "state", True))
        rig.later(1.5, lambda: setattr(rig.hub, "state", False))
        rig.later(3.0, lambda: rig.bmcu.complete_op(rig.bmcu.running_ops()[0]))
        _, hub_seen = _feed(rig, rig.lane(), 0, False)
        assert hub_seen is True
        assert rig.hub.state is False

    def test_real_hub_seen_at_the_end(self, make_rig):
        rig = make_rig(virtual=False).start()
        rig.bmcu.auto_complete[proto.MSG_FEED_TO_CONTACT] = (proto.OP_STATE_DONE, 10)
        rig.hub.state = True
        _, hub_seen = _feed(rig, rig.lane(), 0, False)
        assert hub_seen is True

    def test_real_hub_never_seen(self, make_rig):
        rig = make_rig(virtual=False).start()
        rig.bmcu.auto_complete[proto.MSG_FEED_TO_CONTACT] = (proto.OP_STATE_DONE, 10)
        _, hub_seen = _feed(rig, rig.lane(), 0, False)
        assert hub_seen is False

    def test_sensor_timer_removed_after_the_feed(self, rig):
        rig.bmcu.auto_complete[proto.MSG_FEED_TO_CONTACT] = (proto.OP_STATE_DONE, 10)
        _feed(rig, rig.lane(), 0, True)
        ticks = [t for t in rig.reactor.timers
                 if t.callback.__qualname__.endswith("_feed_to_contact.<locals>._tick")]
        assert ticks == []

    def test_sensor_after_the_op_closed_sends_no_abort(self, rig):
        # Half-duplex: an abort queued after the feed finished would stop the next op.
        rig.unit._conn.op_open = lambda op_id: False
        rig.unit._abort_quietly = MagicMock()
        rig.extruder.tool_start_state = True
        rig.later(1.0, lambda: rig.bmcu.complete_op(rig.bmcu.running_ops()[0]))
        result, _ = _feed(rig, rig.lane(), 0, True)
        assert result["ok"] is True
        rig.unit._abort_quietly.assert_not_called()

    def test_feed_timeout_is_given_to_the_firmware(self, make_rig):
        rig = make_rig(bowden=3000.0).start()
        rig.bmcu.auto_complete[proto.MSG_FEED_TO_CONTACT] = (proto.OP_STATE_DONE, 10)
        _feed(rig, rig.lane(), 0, False)
        # 3000 + 100 + 50 = 3150 mm at 60 mm/s: 3150 / 60 * 2 + 1.5 = 106.5 s
        assert _feed_frames(rig) == [(0, 3150.0, 70, 106500)]


class TestLoad:
    def test_without_sensor_success(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.follow = 1.0
        assert _load_lane(rig) is None
        assert _motions(rig) == [(0, proto.MOTION_BEFORE_ON_USE), (0, proto.MOTION_ON_USE)]
        assert rig.bmcu.channels[0].route == proto.ROUTE_LOADED
        rig.afc.move_e_pos.assert_called_once_with(72.0, 25.0, "tool stn", wait_tool=True)
        rig.lane().activate_toolhead_extruder.assert_called_once_with()

    def test_virtual_tool_start_counts_as_no_sensor(self, make_rig):
        rig = make_rig(tool_start="virtual").start()
        rig.follow = 1.0
        rig.extruder.tool_start_state = False
        assert _load_lane(rig) is None

    def test_ramming_tool_start_counts_as_no_sensor(self, make_rig):
        rig = make_rig(tool_start="buffer").start()
        rig.follow = 1.0
        assert _load_lane(rig) is None
        assert proto.MSG_ABORT_OP not in rig.types()

    def test_without_sensor_feed_without_contact_fails(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.bmcu.channels[0].contact_mm = None
        assert _load_lane(rig) == (
            "did not reach the extruder gears (BMCU feed ended: distance_limit).")
        assert _motions(rig) == []

    def test_without_sensor_failed_feed_fails(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.bmcu.channels[0].contact_mm = None
        rig.later(10.0, lambda: rig.bmcu.complete_op(rig.bmcu.running_ops()[0],
                                                     proto.OP_STATE_FAILED, 10))
        assert _load_lane(rig) == (
            "did not reach the extruder gears (BMCU feed ended: contact).")

    def test_without_sensor_follow_at_the_tolerance_passes(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.follow = 0.5
        assert _load_lane(rig) is None

    def test_without_sensor_follow_below_tolerance_fails(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.follow = 0.25
        assert _load_lane(rig) == (
            "encoder moved 18.0 mm while the extruder loaded 72.0 mm (needs 36.0 mm). "
            "The extruder did not take the filament.")
        assert rig.bmcu.channels[0].route == proto.ROUTE_UNCERTAIN

    def test_with_sensor_success(self, rig):
        rig.later(SENSOR_REACHED_S, lambda: setattr(rig.extruder, "tool_start_state", True))
        assert _load_lane(rig) is None
        assert _motions(rig) == [(0, proto.MOTION_BEFORE_ON_USE), (0, proto.MOTION_ON_USE)]

    def test_with_sensor_not_reached_fails(self, rig):
        rig.follow = 1.0
        assert _load_lane(rig) == (
            "did not reach the toolhead sensor (BMCU feed ended: contact).")
        assert _motions(rig) == []

    def test_with_sensor_lost_during_extruder_load_fails(self, rig):
        rig.later(SENSOR_REACHED_S, lambda: setattr(rig.extruder, "tool_start_state", True))
        rig.afc.move_e_pos.side_effect = lambda *a, **k: setattr(rig.extruder,
                                                                 "tool_start_state", False)
        assert _load_lane(rig) == (
            "left the toolhead sensor while the extruder loaded it.")
        assert rig.bmcu.frames(proto.MSG_MARK_LOADED) == []

    def test_real_hub_never_triggered_fails(self, make_rig):
        rig = make_rig(virtual=False, tool_start=None).start()
        rig.follow = 1.0
        assert _load_lane(rig) == (
            "was fed but hub sensor BMCU_1_hub never saw it. Check the path between the "
            "BMCU and the hub.")
        assert _motions(rig) == []

    def test_real_hub_triggered_passes(self, make_rig):
        rig = make_rig(virtual=False, tool_start=None).start()
        rig.follow = 1.0
        rig.hub.state = True
        assert _load_lane(rig) is None

    def test_zero_tool_stn(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.extruder.tool_stn = 0
        assert _load_lane(rig) is None
        rig.afc.move_e_pos.assert_called_once_with(0, 25.0, "tool stn", wait_tool=True)

    def test_tool_end_sensor_moves_count_toward_the_follow_check(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.extruder.tool_end = "PA2"
        extruded = []

        def extrude(amount, speed, log_string="", wait_tool=False):
            extruded.append(amount)
            rig.extruder.tool_end_state = True
            # the encoder follows only the tool_end moves, 10 mm
            if log_string == "Tool end":
                rig.bmcu.channels[0].meters += amount / 1000.0

        rig.afc.move_e_pos.side_effect = extrude
        # needs (10 + 72) * 0.5 = 41 mm of encoder travel
        assert _load_lane(rig) == (
            "encoder moved 10.0 mm while the extruder loaded 82.0 mm (needs 41.0 mm). "
            "The extruder did not take the filament.")
        assert extruded == [10.0, 72.0]

    def test_tool_end_sensor_never_reached_fails(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.follow = 1.0
        rig.extruder.tool_end = "PA2"
        assert _load_lane(rig) == (
            "did not reach the post-extruder sensor after 20 short moves.")
        assert rig.bmcu.frames(proto.MSG_MARK_LOADED) == []


class TestRunSequence:
    def test_success(self, rig):
        seen = []

        def sequence(lane, extruder, channel, generation):
            seen.append((lane, extruder, channel, generation, rig.unit._op_active))

        lane = rig.lane("bmcu2")
        assert rig.unit._run_sequence(sequence, "load", lane, rig.extruder, False) is True
        assert seen == [(lane, rig.extruder, 1, 1, True)]
        assert rig.unit._op_active is False
        assert rig.bmcu.received == []
        rig.afc.error.handle_lane_failure.assert_not_called()

    def test_a_reason_stops_and_is_reported(self, rig):
        lane = rig.lane()
        assert rig.unit._run_sequence(lambda *args: "jammed.", "unload", lane, rig.extruder,
                                      True) is False
        assert rig.types() == [proto.MSG_STOP_ALL]
        assert rig.failures() == [call(lane, "jammed.", pause=True)]
        assert rig.unit._op_active is False

    def test_an_exception_is_named_for_the_sequence(self, rig):
        lane = rig.lane()

        def sequence(*args):
            raise CommandError("Extrude below minimum temp")

        assert rig.unit._run_sequence(sequence, "unload", lane, rig.extruder, False) is False
        assert rig.types() == [proto.MSG_STOP_ALL]
        assert rig.failures() == [call(lane, "unload failed: Extrude below minimum temp",
                                       pause=False)]


class TestUnitLoadLane:
    def test_an_occupied_u1_head_is_seen_and_refused(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.extruder.fila_tool_start = SimpleNamespace(runout_buttun_state=True)
        lane = rig.lane()
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "cannot load: the toolhead sensor already sees filament. Clear the "
            "toolhead first.", pause=False)]
        assert rig.bmcu.frames(proto.MSG_FEED_TO_CONTACT) == []

    def test_toolhead_sensor_already_triggered_refuses_before_feeding(self, rig):
        rig.extruder.tool_start_state = True
        lane = rig.lane()
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "cannot load: the toolhead sensor already sees filament. Clear the "
            "toolhead first.", pause=False)]
        assert rig.bmcu.frames(proto.MSG_FEED_TO_CONTACT) == []
        assert lane.tool_loaded is False

    def test_contact_short_of_the_bowden_fails(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.follow = 1.0
        rig.later(1.0, lambda: rig.bmcu.complete_op(rig.bmcu.running_ops()[0],
                                                    proto.OP_STATE_DONE, 10, measured_mm=100.0))
        lane = rig.lane()
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "BMCU fed 100.0 mm, short of the 450.0 mm needed to reach the toolhead "
            "(feed ended: contact).", pause=False)]
        assert rig.bmcu.frames(proto.MSG_MARK_LOADED) == []

    def test_with_toolhead_sensor_aborts_feed_on_trigger(self, rig):
        lane = rig.lane()
        rig.later(SENSOR_REACHED_S, lambda: setattr(rig.extruder, "tool_start_state", True))
        assert rig.unit.unit_load_lane(lane, rig.extruder) is True
        assert rig.types()[0] == proto.MSG_FEED_TO_CONTACT
        assert rig.types().count(proto.MSG_ABORT_OP) == 1
        assert rig.bmcu.running_ops() == []
        assert lane.status == AFCLaneState.TOOL_LOADED
        assert lane.loaded_to_hub is True
        assert lane._load_state is True
        assert rig.unit._op_active is False
        rig.afc.save_vars.assert_called_once_with()
        rig.afc.error.handle_lane_failure.assert_not_called()

    def test_without_sensor_fails_when_encoder_does_not_follow(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.afc.function.in_print.return_value = True
        lane = rig.lane()
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "encoder moved 0.0 mm while the extruder loaded 72.0 mm (needs 36.0 mm). "
            "The extruder did not take the filament.", pause=True)]
        assert rig.types()[-1] == proto.MSG_STOP_ALL
        assert rig.bmcu.frames(proto.MSG_MARK_LOADED) == []
        assert lane.tool_loaded is False
        assert lane.status == AFCLaneState.NONE
        assert rig.unit._op_active is False

    def test_contact_never_reached_fails(self, make_rig):
        rig = make_rig(tool_start=None).start()
        # A slipping drive: no contact before the firmware's 33 s timeout.
        rig.bmcu.channels[0].contact_mm = None
        rig.bmcu.move_speed = 20.0
        lane = rig.lane()
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "did not reach the extruder gears (BMCU feed ended: timeout).", pause=False)]
        assert rig.types()[-1] == proto.MSG_STOP_ALL
        assert _motions(rig) == []

    def test_uncalibrated_lane_refused(self, rig):
        rig.afc.function.in_print.return_value = True
        rig.unit._status = rig.status(cal_valid_mask=0b1110)
        lane = rig.lane()
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.bmcu.received == []
        assert rig.failures() == [call(
            lane, "is not calibrated. Run AFC_BMCU_CALIBRATE UNIT=BMCU_1 LANE=bmcu1, then "
            "load it again.", pause=True)]

    @pytest.mark.parametrize("cause, reason", [
        ("unplug", UNPLUGGED),
        ("reboot", f"BMCU on {PORT} is not connected"),
    ])
    def test_link_lost_mid_feed_fails(self, rig, cause, reason):
        rig.afc.function.in_print.return_value = True
        rig.afc.function.is_printing.return_value = True
        lane = rig.lane()
        rig.later(2.0, getattr(rig.bmcu, cause))
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(lane, f"load failed: {reason}", pause=True)]
        assert len(rig.bmcu.frames(proto.MSG_FEED_TO_CONTACT)) == 1
        assert lane.tool_loaded is False
        assert lane.status == AFCLaneState.NONE
        assert rig.unit._op_active is False
        rig.reactor.advance(0)
        assert rig.errors() == [call("BMCU BMCU_1 disconnected during a print. "
                                     "Reconnect it, then resume.", pause=True)]
        rig.bmcu.received.clear()
        rig.bmcu.replug()
        rig.reactor.advance(5.0)
        assert rig.unit._conn.connected is True
        assert rig.bmcu.motion_frames() == []

    def test_not_connected(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        lane = rig.lane()
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(lane, "cannot load: BMCU BMCU_1 is not connected.",
                                       pause=False)]

    def test_gcode_error_in_the_extruder_move_stops_and_fails(self, rig):
        lane = rig.lane()
        rig.afc.function.in_print.return_value = True
        rig.later(SENSOR_REACHED_S, lambda: setattr(rig.extruder, "tool_start_state", True))
        rig.afc.move_e_pos.side_effect = CommandError("Extrude below minimum temp")
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.types()[-1] == proto.MSG_STOP_ALL
        assert rig.bmcu.channels[0].motion == proto.MOTION_IDLE
        assert rig.failures() == [call(lane, "load failed: Extrude below minimum temp",
                                       pause=True)]
        assert lane.tool_loaded is False
        assert rig.unit._op_active is False

    # Without a toolhead sensor the load is judged by the encoder, which a
    # reboot resets, so the reconnect must be caught before it is read.
    @pytest.mark.parametrize("tool_start", ["PA1", None])
    def test_reconnect_during_the_extruder_move_fails(self, make_rig, tool_start):
        rig = make_rig(tool_start=tool_start).start()
        lane = rig.lane()
        rig.follow = 1.0
        rig.later(SENSOR_REACHED_S, lambda: setattr(rig.extruder, "tool_start_state", True))
        rig.afc.move_e_pos.side_effect = rig.reconnect
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "load failed: the BMCU reconnected during the operation and lost its state",
            pause=False)]
        assert rig.bmcu.frames(proto.MSG_MARK_LOADED) == []
        assert lane.status != AFCLaneState.TOOL_LOADED

    def test_refused_mark_fails_the_load_before_following(self, rig):
        # The flash fails while the extruder takes the filament in, so the
        # firmware refuses MARK_LOADED (error 71) and the lane must not follow.
        lane = rig.lane()
        rig.later(SENSOR_REACHED_S, lambda: setattr(rig.extruder, "tool_start_state", True))
        rig.afc.move_e_pos.side_effect = lambda *a, **k: setattr(rig.bmcu, "nvm_ok", False)
        assert rig.unit.unit_load_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "load failed: BMCU error code=71 channel=0 details=0", pause=False)]
        assert _motions(rig) == [(0, proto.MOTION_BEFORE_ON_USE)]
        assert rig.types()[-1] == proto.MSG_STOP_ALL
        assert lane.tool_loaded is False

    def test_success_clears_the_runout_latch(self, rig):
        rig.unit._runout_dispatched.add("bmcu1")
        rig.later(SENSOR_REACHED_S, lambda: setattr(rig.extruder, "tool_start_state", True))
        assert rig.unit.unit_load_lane(rig.lane(), rig.extruder) is True
        assert rig.unit._runout_dispatched == set()


# Unload

def _tail_frames(rig):
    return [struct.unpack("<BfI", f.payload) for f in rig.bmcu.frames(proto.MSG_TAIL_RETRACT)]


class TestWaitBufferSettled:
    def test_already_inside(self, rig):
        start = rig.reactor.now
        assert rig.unit._wait_buffer_settled(0) is None
        assert rig.reactor.now == start

    @pytest.mark.parametrize("buffer_pct", [5, 95])
    def test_settles_after_a_while(self, rig, buffer_pct):
        rig.bmcu.channels[0].buffer_pct = buffer_pct
        rig.later(1.0, lambda: setattr(rig.bmcu.channels[0], "buffer_pct", 50))
        start = rig.reactor.now
        assert rig.unit._wait_buffer_settled(0) is None
        assert 1.0 <= rig.reactor.now - start <= 1.2

    @pytest.mark.parametrize("buffer_pct", [6, 94])
    def test_inside_limits(self, rig, buffer_pct):
        rig.bmcu.channels[0].buffer_pct = buffer_pct
        assert rig.unit._wait_buffer_settled(0) is None

    def test_never_settles(self, rig):
        rig.bmcu.channels[0].buffer_pct = 97
        start = rig.reactor.now
        assert rig.unit._wait_buffer_settled(0) == (
            "the buffer stayed at 97% before the pull back: the filament may be jammed.")
        assert rig.reactor.now - start == pytest.approx(4.0)


class TestPullBack:
    def test_the_extruder_retract_counts_toward_the_firmware_length(self, rig):
        rig.follow = 1.0
        _in_toolhead(rig.lane())
        rig.bmcu.channels[0].present = 1
        length = rig.unit._runtime_values()["retract_mm"]
        tool_stn_unload = rig.extruder.tool_stn_unload
        rig.later(2.0, lambda: rig.finish_pull_back(0, length - tool_stn_unload))
        assert _unload_lane(rig) == (
            f"pull back used its whole {length:.0f} mm without reaching the inlet sensor: "
            "the tip may still be inside the BMCU. Raise path_extra or clear the "
            "path by hand.")
        assert rig.bmcu.frames(proto.MSG_MARK_UNLOADED) == []

    def test_a_changed_bowden_reaches_the_board_before_the_pull(self, rig):
        rig.hub.afc_bowden_length = 1200.0
        rig.later(1.0, lambda: rig.finish_pull_back(0, 900.0))
        assert rig.unit._pull_back(0) is None
        sent = [f for f in rig.bmcu.received
                if f.msg_type in (proto.MSG_RUNTIME_SYNC, proto.MSG_SET_MOTION)]
        assert [f.msg_type for f in sent] == [proto.MSG_RUNTIME_SYNC, proto.MSG_SET_MOTION]
        assert _runtime_fields(sent[0].payload)[6] == pytest.approx(1.3)

    def test_an_unchanged_bowden_sends_no_runtime_sync(self, rig):
        rig.later(1.0, lambda: rig.finish_pull_back(0, 900.0))
        assert rig.unit._pull_back(0) is None
        assert rig.bmcu.frames(proto.MSG_RUNTIME_SYNC) == []

    def test_pull_back_that_used_its_whole_length_fails(self, rig):
        length = rig.unit._runtime_values()["retract_mm"]
        rig.later(1.0, lambda: rig.finish_pull_back(0, length))
        assert rig.unit._pull_back(0) == (
            f"pull back used its whole {length:.0f} mm without reaching the inlet sensor: "
            "the tip may still be inside the BMCU. Raise path_extra or clear the "
            "path by hand.")

    def test_pull_back_stopped_by_the_inlet_sensor_succeeds(self, rig):
        length = rig.unit._runtime_values()["retract_mm"]
        rig.later(1.0, lambda: rig.finish_pull_back(0, length - 10.0))
        assert rig.unit._pull_back(0) is None

    def test_pulls_back_until_idle(self, rig):
        rig.later(2.0, lambda: setattr(rig.bmcu.channels[1], "motion", proto.MOTION_IDLE))
        assert rig.unit._pull_back(1) is None
        assert _motions(rig) == [(1, proto.MOTION_PULL_BACK)]

    def test_pull_far_enough_passes(self, rig):
        rig.later(2.0, lambda: rig.finish_pull_back(1, 451.0))
        assert rig.unit._pull_back(1, 450.0) is None

    def test_short_pull_fails(self, rig):
        rig.later(2.0, lambda: rig.finish_pull_back(1, 449.0))
        assert rig.unit._pull_back(1, 450.0) == (
            "pull back moved 449.0 mm, short of the 450.0 mm it must pull to clear the path.")

    def test_times_out_at_the_feed_timeout_floor(self, make_rig):
        # retract 20 mm at 60 mm/s gives 20 / 60 * 2 + 1.5 = 2.17 s, under the 3 s floor
        rig = make_rig(bowden=20.0, values={"feed_timeout": 3.0,
                                            "path_extra": 0.0}).start()
        start = rig.reactor.now
        assert rig.unit._pull_back(1) == "pull back did not finish within 3s."
        assert rig.reactor.now - start == pytest.approx(3.0)

    def test_long_pull_back_gets_a_longer_timeout(self, rig):
        # retract 1000 mm at 60 mm/s gives 1000 / 60 * 2 + 1.5 = 34.83 s
        start = rig.reactor.now
        assert rig.unit._pull_back(1) == "pull back did not finish within 35s."
        assert rig.reactor.now - start == pytest.approx(34.8333, abs=0.01)

    def test_waits_for_the_buffer_before_pulling(self, rig):
        rig.bmcu.channels[1].buffer_pct = 98
        rig.later(1.0, lambda: setattr(rig.bmcu.channels[1], "buffer_pct", 50))
        rig.later(2.0, lambda: rig.finish_pull_back(1, 900.0))
        assert rig.unit._pull_back(1, 450.0) is None
        assert rig.bmcu.frames(proto.MSG_SET_MOTION)[0].time >= 101.5 + 1.0

    def test_buffer_never_leaving_an_extreme_fails_without_pulling(self, rig):
        rig.bmcu.channels[1].buffer_pct = 98
        assert rig.unit._pull_back(1) == (
            "the buffer stayed at 98% before the pull back: the filament may be jammed.")
        assert _motions(rig) == []

    def test_measures_from_the_given_start(self, rig):
        start_m = rig.bmcu.channels[1].meters + 0.3
        rig.later(1.0, lambda: rig.finish_pull_back(1, 200.0))
        assert rig.unit._pull_back(1, 450.0, start_m) is None


class TestRetractFromPath:
    def test_present_uses_pull_back(self, rig):
        rig.bmcu.channels[0].present = 1
        rig.later(1.0, lambda: rig.finish_pull_back(0, 900.0))
        assert rig.unit._retract_from_path(rig.lane(), 0) is None
        assert _motions(rig) == [(0, proto.MOTION_PULL_BACK)]
        assert rig.bmcu.frames(proto.MSG_TAIL_RETRACT) == []

    def test_present_pull_back_must_cover_half_the_bowden(self, make_rig):
        rig = make_rig(bowden=900.0).start()
        rig.bmcu.channels[0].present = 1
        rig.later(1.0, lambda: rig.finish_pull_back(0, 300.0))
        assert rig.unit._retract_from_path(rig.lane(), 0) == (
            "pull back moved 300.0 mm, short of the 450.0 mm it must pull to clear the path.")

    def test_inlet_empty_uses_tail_retract(self, make_rig):
        rig = make_rig(spools=(), bowden=700.0, values={"path_extra": 80.0}).start()
        rig.bmcu.auto_complete[proto.MSG_TAIL_RETRACT] = (proto.OP_STATE_DONE, 1)
        assert rig.unit._retract_from_path(rig.lane(), 0) is None
        # 780 mm at 60 mm/s: 780 / 60 * 2 + 1.5 = 27.5 s, under the 30 s floor
        assert _tail_frames(rig) == [(0, 780.0, 30000)]
        assert _motions(rig) == [(0, proto.MOTION_IDLE)]

    def test_tail_retract_capped(self, make_rig):
        rig = make_rig(spools=(), bowden=1990.0).start()
        rig.bmcu.auto_complete[proto.MSG_TAIL_RETRACT] = (proto.OP_STATE_DONE, 1)
        rig.unit._retract_from_path(rig.lane(), 0)
        assert _tail_frames(rig)[0][1] == 2000.0

    def test_tail_retract_floor(self, make_rig):
        rig = make_rig(spools=(), values={"path_extra": 0.0}).start()
        rig.lane().hub_obj = None
        rig.bmcu.auto_complete[proto.MSG_TAIL_RETRACT] = (proto.OP_STATE_DONE, 1)
        rig.unit._retract_from_path(rig.lane(), 0)
        assert _tail_frames(rig)[0][1] == 10.0

    def test_tail_retract_failure(self, make_rig):
        rig = make_rig(spools=()).start()
        rig.bmcu.channels[0].buffer_pct = 2
        assert rig.unit._retract_from_path(rig.lane(), 0) == (
            "tail retract failed (buffer_limit). Clear the bmcu1 path by hand before "
            "loading another lane.")

    def test_no_capability(self, make_rig):
        rig = make_rig(spools=(), caps=0).start()
        assert rig.unit._retract_from_path(rig.lane(), 0) == (
            "ran out and this BMCU firmware cannot retract the tail. Clear the bmcu1 path "
            "by hand before loading another lane.")
        assert rig.types() == [proto.MSG_GET_STATUS]


class TestCheckRetracted:
    def test_clean(self, rig):
        assert rig.unit._check_retracted(rig.lane(), rig.extruder, 0) is None

    def test_toolhead_sensor_still_triggered(self, rig):
        rig.extruder.tool_start_state = True
        assert rig.unit._check_retracted(rig.lane(), rig.extruder, 0) == (
            "retracted but the toolhead sensor still sees filament.")

    def test_without_a_toolhead_sensor_its_state_is_ignored(self, make_rig):
        rig = make_rig(tool_start="buffer").start()
        rig.extruder.tool_start_state = True
        assert rig.unit._check_retracted(rig.lane(), rig.extruder, 0) is None

    def test_post_extruder_sensor_still_triggered(self, make_rig):
        rig = make_rig(tool_start=None).start()
        rig.extruder.tool_end_state = True
        assert rig.unit._check_retracted(rig.lane(), rig.extruder, 0) == (
            "retracted but the toolhead sensor still sees filament.")

    @pytest.mark.parametrize("buffer_pct", [3, 97])
    def test_buffer_at_an_extreme(self, rig, buffer_pct):
        rig.bmcu.channels[0].buffer_pct = buffer_pct
        assert rig.unit._check_retracted(rig.lane(), rig.extruder, 0) == (
            f"retracted but the buffer ended at {buffer_pct}%: the filament may be jammed "
            "or slipping.")

    @pytest.mark.parametrize("buffer_pct", [4, 96])
    def test_buffer_inside_the_limits(self, rig, buffer_pct):
        rig.bmcu.channels[0].buffer_pct = buffer_pct
        assert rig.unit._check_retracted(rig.lane(), rig.extruder, 0) is None


class TestUnload:
    def test_marks_unloaded_after_a_clean_retract(self, rig):
        rig.bmcu.channels[0].present = 1
        rig.bmcu.channels[0].route = proto.ROUTE_LOADED
        rig.later(1.0, lambda: rig.finish_pull_back(0, 900.0))
        assert _unload_lane(rig) is None
        assert rig.bmcu.channels[0].route == proto.ROUTE_EMPTY

    def test_unclean_retract_is_not_marked(self, rig):
        rig.bmcu.channels[0].route = proto.ROUTE_LOADED
        rig.extruder.tool_start_state = True
        rig.sensor_stuck = True
        rig.later(1.0, lambda: rig.finish_pull_back(0, 900.0))
        assert _unload_lane(rig) == "retracted but the toolhead sensor still sees filament."
        assert rig.bmcu.frames(proto.MSG_MARK_UNLOADED) == []
        # The pull-back left the route UNCERTAIN, as the firmware does until a MARK.
        assert rig.bmcu.channels[0].route == proto.ROUTE_UNCERTAIN

    def test_reconnect_during_tip_forming_fails_before_the_retract(self, rig):
        rig.afc.do_tool_cut_tip_form.side_effect = rig.reconnect
        with pytest.raises(BMCUError, match="reconnected"):
            _unload_lane(rig)
        assert _motions(rig) == [(0, proto.MOTION_BEFORE_PULL_BACK)]
        assert rig.bmcu.frames(proto.MSG_TAIL_RETRACT) == []


class TestUnitUnloadLane:
    def test_runs_post_unload_macro_after_a_clean_unload(self, rig):
        rig.afc.post_unload_macro = "WIPE_NOZZLE"
        lane = self._loaded(rig)
        rig.later(2.0, lambda: rig.finish_pull_back(0, 900.0))
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is True
        rig.unit.gcode.run_script_from_command.assert_called_once_with("WIPE_NOZZLE")

    def test_failed_unload_skips_post_unload_macro(self, rig):
        rig.afc.post_unload_macro = "WIPE_NOZZLE"
        lane = self._loaded(rig)
        rig.sensor_stuck = True
        rig.later(2.0, lambda: rig.finish_pull_back(0, 900.0))
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is False
        rig.unit.gcode.run_script_from_command.assert_not_called()

    def _loaded(self, rig, present=1):
        lane = rig.lane()
        _in_toolhead(lane)
        lane.status = AFCLaneState.TOOLED
        rig.extruder.lane_loaded = "bmcu1"
        rig.bmcu.channels[0].present = present
        rig.bmcu.channels[0].route = proto.ROUTE_LOADED
        return lane

    def test_present_uses_pull_back(self, rig):
        lane = self._loaded(rig)
        rig.later(2.0, lambda: rig.finish_pull_back(0, 900.0))
        during = []
        rig.afc.do_tool_cut_tip_form.side_effect = lambda *a: during.append(lane.status)
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is True
        assert during == [AFCLaneState.TOOL_UNLOADING]
        assert _motions(rig) == [(0, proto.MOTION_BEFORE_PULL_BACK), (0, proto.MOTION_PULL_BACK)]
        rig.afc.do_tool_cut_tip_form.assert_called_once_with(lane, rig.extruder)
        rig.afc.move_e_pos.assert_called_once_with(-100.0, 30.0, "BMCU unload", wait_tool=True)
        assert rig.bmcu.channels[0].route == proto.ROUTE_EMPTY
        assert rig.afc.spool.set_active_spool.call_args_list == [call(None)]
        assert lane.tool_loaded is False
        assert lane.status == AFCLaneState.NONE
        assert lane.loaded_to_hub is True
        assert lane._load_state is False
        assert rig.unit._op_active is False
        rig.afc.save_vars.assert_called_once_with()

    def test_inlet_empty_uses_tail_retract_with_bowden_plus_extra(self, make_rig):
        rig = make_rig(bowden=900.0).start()
        lane = self._loaded(rig, present=0)
        rig.bmcu.auto_complete[proto.MSG_TAIL_RETRACT] = (proto.OP_STATE_DONE, 1)
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is True
        # 1000 mm at 60 mm/s: 1000 / 60 * 2 + 1.5 = 34.833 s
        assert _tail_frames(rig) == [(0, 1000.0, 34833)]
        assert _motions(rig) == [(0, proto.MOTION_BEFORE_PULL_BACK), (0, proto.MOTION_IDLE)]
        assert lane.loaded_to_hub is False
        assert lane.tool_loaded is False

    def test_real_hub_stays_blocked_pauses(self, make_rig):
        rig = make_rig(virtual=False).start()
        rig.afc.function.in_print.return_value = True
        lane = self._loaded(rig, present=0)
        rig.bmcu.auto_complete[proto.MSG_TAIL_RETRACT] = (proto.OP_STATE_DONE, 1)
        rig.hub.state = True
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "unloaded but the hub sensor still sees filament. Clear the path before "
            "loading another lane.", pause=True)]
        assert lane.tool_loaded is False
        assert rig.bmcu.channels[0].route == proto.ROUTE_EMPTY

    def test_real_hub_clear_succeeds(self, make_rig):
        # A real hub's lane keeps the inlet's state; only a virtual hub's is cleared.
        rig = make_rig(virtual=False).start()
        lane = self._loaded(rig, present=1)
        rig.later(1.0, lambda: rig.finish_pull_back(0, 900.0))
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is True
        assert lane._load_state is True
        rig.afc.error.handle_lane_failure.assert_not_called()

    def test_tail_retract_failure_pauses(self, rig):
        rig.afc.function.in_print.return_value = True
        lane = self._loaded(rig, present=0)
        # The pull reaches its target but the flash cannot save the empty route.
        rig.later(1.0, lambda: setattr(rig.bmcu, "nvm_ok", False))
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is False
        assert rig.bmcu.result(rig.bmcu.started[0])["reason"] == "nvm_write"
        assert rig.bmcu.channels[0].route == proto.ROUTE_UNCERTAIN
        assert rig.failures() == [call(
            lane, "tail retract failed (nvm_write). Clear the bmcu1 path by hand before "
            "loading another lane.", pause=True)]
        assert rig.types()[-1] == proto.MSG_STOP_ALL
        assert rig.bmcu.frames(proto.MSG_MARK_UNLOADED) == []
        assert lane.tool_loaded is True
        rig.afc.spool.set_active_spool.assert_not_called()
        assert rig.unit._op_active is False

    @pytest.mark.parametrize("cause, reason", [
        ("unplug", UNPLUGGED),
        ("reboot", f"BMCU on {PORT} is not connected"),
    ])
    def test_link_lost_mid_tail_retract_fails(self, rig, cause, reason):
        lane = self._loaded(rig, present=0)
        rig.later(2.0, getattr(rig.bmcu, cause))
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(lane, f"unload failed: {reason}", pause=False)]
        assert lane.tool_loaded is True
        assert len(rig.bmcu.frames(proto.MSG_TAIL_RETRACT)) == 1
        if cause == "unplug":
            rig.bmcu.replug()
        rig.bmcu.received.clear()
        rig.reactor.advance(5.0)
        assert rig.unit._conn.connected is True
        # AFC still has the lane in the toolhead, so the restore marks it loaded.
        assert rig.types() == HANDSHAKE + [proto.MSG_RUNTIME_SYNC, proto.MSG_STOP_ALL,
                                           proto.MSG_MARK_LOADED, proto.MSG_MARK_UNLOADED,
                                           proto.MSG_GET_STATUS]

    def test_no_tool_stn_unload_skips_extruder_retract(self, rig):
        lane = self._loaded(rig, present=0)
        rig.extruder.tool_stn_unload = 0
        # The tail retract pulls the filament off the toolhead sensor.
        rig.extruder.tool_start_state = False
        rig.bmcu.auto_complete[proto.MSG_TAIL_RETRACT] = (proto.OP_STATE_DONE, 1)
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is True
        rig.afc.move_e_pos.assert_not_called()

    def test_not_connected(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        lane = rig.lane()
        _in_toolhead(lane)
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(lane, "cannot unload: BMCU BMCU_1 is not connected.",
                                       pause=False)]
        rig.afc.do_tool_cut_tip_form.assert_not_called()

    def test_toolhead_sensor_still_triggered_keeps_the_tool_loaded(self, rig):
        rig.afc.function.in_print.return_value = True
        lane = self._loaded(rig)
        rig.sensor_stuck = True
        rig.later(2.0, lambda: rig.finish_pull_back(0, 900.0))
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "retracted but the toolhead sensor still sees filament.", pause=True)]
        assert lane.tool_loaded is True
        rig.afc.spool.set_active_spool.assert_not_called()
        assert rig.bmcu.frames(proto.MSG_MARK_UNLOADED) == []
        assert rig.types()[-1] == proto.MSG_STOP_ALL

    def test_short_pull_back_keeps_the_tool_loaded(self, rig):
        lane = self._loaded(rig)
        rig.later(2.0, lambda: rig.finish_pull_back(0, 100.0))
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "pull back moved 100.0 mm, short of the 450.0 mm it must pull to clear "
            "the path.", pause=False)]
        assert lane.tool_loaded is True

    def test_buffer_at_an_extreme_keeps_the_tool_loaded(self, rig):
        lane = self._loaded(rig, present=0)
        rig.bmcu.auto_complete[proto.MSG_TAIL_RETRACT] = (proto.OP_STATE_DONE, 1)
        rig.bmcu.channels[0].buffer_pct = 97
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is False
        assert rig.failures() == [call(
            lane, "retracted but the buffer ended at 97%: the filament may be jammed or "
            "slipping.", pause=False)]
        assert lane.tool_loaded is True

    def test_gcode_error_in_tip_forming_stops_and_fails(self, rig):
        lane = self._loaded(rig)
        rig.afc.do_tool_cut_tip_form.side_effect = CommandError("Extrude below minimum temp")
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is False
        assert rig.types()[-1] == proto.MSG_STOP_ALL
        assert rig.failures() == [call(lane, "unload failed: Extrude below minimum temp",
                                       pause=False)]
        assert lane.tool_loaded is True
        assert rig.unit._op_active is False

    def test_success_clears_the_runout_latch(self, rig):
        lane = self._loaded(rig, present=0)
        rig.unit._runout_dispatched.add("bmcu1")
        rig.bmcu.auto_complete[proto.MSG_TAIL_RETRACT] = (proto.OP_STATE_DONE, 1)
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is True
        assert rig.unit._runout_dispatched == set()

    def test_short_bowden_counts_the_retract_before_the_pull_back(self, make_rig):
        # bowden 200 mm needs 100 mm pulled; BEFORE_PULL_BACK follows the 80 mm
        # extruder retract, then PULL_BACK moves 40 mm.
        rig = make_rig(bowden=200.0).start()
        lane = self._loaded(rig)
        rig.follow = 1.0
        rig.later(2.0, lambda: rig.finish_pull_back(0, 40.0))
        rig.extruder.tool_stn_unload = 80.0
        assert rig.unit.unit_unload_lane(lane, rig.extruder) is True
        rig.afc.error.handle_lane_failure.assert_not_called()


# lane_move, reset, eject

class TestLaneMove:
    def test_forward_feeds_distance(self, rig):
        rig.bmcu.auto_complete[proto.MSG_FEED_DISTANCE] = (proto.OP_STATE_DONE, 1)
        rig.unit.lane_move(rig.lane("bmcu2"), 150.0, None)
        frames = rig.bmcu.frames(proto.MSG_FEED_DISTANCE)
        assert [struct.unpack("<BfBI", f.payload) for f in frames] == [(1, 150.0, 90, 30000)]
        rig.afc.error.AFC_error.assert_not_called()
        assert rig.unit._op_active is False

    def test_backward_tail_retracts(self, rig):
        rig.bmcu.auto_complete[proto.MSG_TAIL_RETRACT] = (proto.OP_STATE_DONE, 1)
        rig.unit.lane_move(rig.lane(), -250.0, None)
        assert rig.errors() == []
        assert _tail_frames(rig) == [(0, 250.0, 30000)]
        rig.afc.error.AFC_error.assert_not_called()

    @pytest.mark.parametrize("distance, refusal", [
        (4.9, "BMCU lanes move at least 5 mm, not 4.9 mm."),
        (-4.9, "BMCU lanes move at least 5 mm, not -4.9 mm."),
        (5000.5, "BMCU lanes feed at most 5000 mm in one move."),
        (-2000.5, "BMCU lanes retract at most 2000 mm in one move."),
    ])
    def test_distance_outside_the_firmware_limits_rejected(self, rig, distance, refusal):
        rig.unit.lane_move(rig.lane(), distance, None)
        assert rig.errors() == [call(f"bmcu1: {refusal}", pause=False)]
        assert rig.bmcu.received == []

    @pytest.mark.parametrize("distance, msg_type", [
        (5.0, proto.MSG_FEED_DISTANCE), (5000.0, proto.MSG_FEED_DISTANCE),
        (-5.0, proto.MSG_TAIL_RETRACT), (-2000.0, proto.MSG_TAIL_RETRACT),
    ])
    def test_moves_at_the_firmware_limits(self, rig, distance, msg_type):
        rig.bmcu.auto_complete[msg_type] = (proto.OP_STATE_DONE, 1)
        rig.unit.lane_move(rig.lane(), distance, None)
        assert rig.errors() == []
        frames = rig.bmcu.frames(msg_type)
        assert len(frames) == 1
        assert struct.unpack_from("<Bf", frames[0].payload)[1] == abs(distance)

    def test_retract_without_capability_rejected(self, make_rig):
        rig = make_rig(caps=0).start()
        rig.unit.lane_move(rig.lane(), -50.0, None)
        assert rig.errors() == [call("bmcu1: This BMCU firmware cannot retract a lane by "
                                     "distance.", pause=False)]
        assert rig.bmcu.received == []

    def test_forward_without_capability_allowed(self, make_rig):
        rig = make_rig(caps=0).start()
        rig.bmcu.auto_complete[proto.MSG_FEED_DISTANCE] = (proto.OP_STATE_DONE, 1)
        rig.unit.lane_move(rig.lane(), 50.0, None)
        rig.afc.error.AFC_error.assert_not_called()

    def test_not_connected(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.unit.lane_move(rig.lane(), 50.0, None)
        assert rig.errors() == [call("bmcu1: BMCU BMCU_1 is not connected.", pause=False)]

    def test_failed_move_reports_reason(self, rig):
        rig.bmcu.auto_complete[proto.MSG_FEED_DISTANCE] = (proto.OP_STATE_FAILED, 5)
        rig.unit.lane_move(rig.lane(), 50.0, None)
        assert rig.errors() == [call("bmcu1 move failed: buffer_limit", pause=False)]

    def test_unconfirmed_start_stops_every_motor_and_reports(self, rig):
        rig.bmcu.add_fault("drop", proto.MSG_FEED_DISTANCE)
        rig.unit.lane_move(rig.lane(), 500.0, None)
        feed_id = rig.bmcu.frames(proto.MSG_FEED_DISTANCE)[0].cmd_id
        assert rig.types() == [proto.MSG_FEED_DISTANCE, proto.MSG_ABORT_OP, proto.MSG_STOP_ALL]
        assert rig.unit._conn.connected is True
        assert rig.errors() == [call(
            f"bmcu1 move failed: BMCU request 0x19 cmd={feed_id} got no reply within 2s",
            pause=False)]
        assert rig.unit._op_active is False

    def test_long_move_gets_a_longer_timeout(self, rig):
        rig.bmcu.auto_complete[proto.MSG_FEED_DISTANCE] = (proto.OP_STATE_DONE, 1)
        rig.bmcu.auto_complete[proto.MSG_TAIL_RETRACT] = (proto.OP_STATE_DONE, 1)
        rig.unit.lane_move(rig.lane(), 3000.0, None)
        rig.unit.lane_move(rig.lane(), -1800.0, None)
        feed = struct.unpack("<BfBI", rig.bmcu.frames(proto.MSG_FEED_DISTANCE)[0].payload)
        # 3000 / 60 * 2 + 1.5 = 101.5 s and 1800 / 60 * 2 + 1.5 = 61.5 s
        assert feed[3] == 101500
        assert _tail_frames(rig)[0][2] == 61500


class TestGetLaneResetCommand:
    def test_lane_move_back(self, make_rig):
        rig = make_rig()
        assert rig.unit.get_lane_reset_command(rig.lane("bmcu2"), 350.0) == (
            "LANE_MOVE LANE=bmcu2 DISTANCE=-350.0")


class TestEjectLane:
    def test_present_pulls_back_and_marks_unloaded(self, rig):
        rig.bmcu.channels[0].present = 1
        rig.bmcu.channels[0].route = proto.ROUTE_LOADED
        rig.later(1.0, lambda: setattr(rig.bmcu.channels[0], "motion", proto.MOTION_IDLE))
        rig.unit.eject_lane(rig.lane())
        assert _motions(rig) == [(0, proto.MOTION_PULL_BACK)]
        assert rig.bmcu.channels[0].route == proto.ROUTE_EMPTY
        assert rig.logger.messages == []
        assert rig.unit._op_active is False

    def test_empty_lane_only_marks_unloaded(self, make_rig):
        rig = make_rig(spools=()).start()
        rig.unit.eject_lane(rig.lane())
        assert rig.types() == [proto.MSG_GET_STATUS, proto.MSG_MARK_UNLOADED]
        assert rig.bmcu.motion_frames() == []

    def test_pull_back_timeout_stops(self, make_rig):
        rig = make_rig(bowden=20.0, values={"feed_timeout": 2.0,
                                            "path_extra": 0.0}).start()
        rig.bmcu.channels[0].present = 1
        rig.unit.eject_lane(rig.lane())
        assert rig.types().count(proto.MSG_STOP_ALL) == 1
        assert rig.bmcu.frames(proto.MSG_MARK_UNLOADED) == []
        assert rig.errors() == [
            call("bmcu1 eject failed: pull back did not finish within 2s.", pause=False)]

    def test_link_error_logged(self, rig):
        rig.unit._conn.request = MagicMock(side_effect=BMCUError("gone"))
        rig.unit._conn.stop_all = MagicMock()
        rig.unit.eject_lane(rig.lane())
        rig.unit._conn.stop_all.assert_called_once_with()
        assert rig.errors() == [call("bmcu1 eject failed: gone", pause=False)]
        assert rig.unit._op_active is False

    def test_not_connected(self, make_rig):
        rig = make_rig()
        rig.unit.handle_connect()
        rig.unit.eject_lane(rig.lane())
        assert rig.logger.messages == [
            ("error", "Cannot eject bmcu1: BMCU BMCU_1 is not connected.")]


class TestCoreLoadUnloadPath:
    """AFC core's load_sequence and unload_sequence calling this unit's hooks."""

    def _core(self, rig):
        from tests.test_AFC import _make_afc

        core = _make_afc()
        core._check_extruder_temp = MagicMock(return_value=False)
        core.save_vars = MagicMock()
        core.post_unload_macro = None
        return core

    def test_load_then_unload_a_bmcu_lane(self, rig):
        lane = rig.lane()
        core = self._core(rig)
        rig.later(SENSOR_REACHED_S, lambda: setattr(rig.extruder, "tool_start_state", True))
        assert core.load_sequence(lane, rig.hub, rig.extruder) is True
        assert lane.tool_loaded is True
        assert lane.status == AFCLaneState.TOOLED
        assert rig.extruder.lane_loaded == "bmcu1"
        assert rig.bmcu.channels[0].route == proto.ROUTE_LOADED
        assert rig.bmcu.channels[0].motion == proto.MOTION_ON_USE
        lane.enable_buffer.assert_called_once_with(disable_fault=True)

        rig.extruder.tool_start_state = False
        rig.bmcu.channels[0].present = 1
        rig.later(2.0, lambda: rig.finish_pull_back(0, 900.0))
        assert core.unload_sequence(lane, rig.hub, rig.extruder) is True
        assert lane.tool_loaded is False
        lane.disable_buffer.assert_called_once_with()
        assert rig.extruder.lane_loaded is None
        assert rig.bmcu.channels[0].route == proto.ROUTE_EMPTY

    def test_failed_load_stops_core(self, make_rig):
        rig = make_rig(tool_start=None).start()
        lane = rig.lane()
        core = self._core(rig)
        rig.bmcu.auto_complete[proto.MSG_FEED_TO_CONTACT] = (proto.OP_STATE_FAILED, 2)
        assert core.load_sequence(lane, rig.hub, rig.extruder) is False
        assert lane.tool_loaded is False
        lane.enable_buffer.assert_not_called()


class TestLoadConfigPrefix:
    def test_builds_the_unit(self, make_rig):
        rig = make_rig()
        unit = load_config_prefix(rig.config)
        assert isinstance(unit, afcBMCU)
        assert unit.name == "BMCU_1"


# Module import guards

def _exec_afc_bmcu_with_blocked_dependency(blocked):
    """Run a throwaway copy of extras/AFC_BMCU.py with one dependency made
    unimportable (``sys.modules[name] = None``), restoring the real module
    object afterwards so other tests keep their bindings."""
    import importlib.util

    import extras
    import extras.AFC_BMCU as real_module
    probe = "extras.AFC_BMCU_import_guard_probe"
    original = sys.modules.get(blocked)
    sys.modules[blocked] = None
    # "from extras import X" also finds X as an attribute of the package.
    attr = blocked.split(".", 1)[1]
    package_attr = extras.__dict__.pop(attr, None)
    try:
        spec = importlib.util.spec_from_file_location(probe, real_module.__file__)
        fresh = importlib.util.module_from_spec(spec)
        sys.modules[probe] = fresh
        try:
            spec.loader.exec_module(fresh)
        finally:
            sys.modules.pop(probe, None)
    finally:
        sys.modules[blocked] = original
        if package_attr is not None:
            setattr(extras, attr, package_attr)


class TestModuleImportGuards:
    def test_afc_utils_failure(self):
        with pytest.raises(config_error) as raised:
            _exec_afc_bmcu_with_blocked_dependency("extras.AFC_utils")
        assert str(raised.value).startswith("Error when trying to import AFC_utils.ERROR_STR\n")

    @pytest.mark.parametrize("module,label", [
        ("extras.AFC_lane", "AFC_lane"),
        ("extras.AFC_unit", "AFC_unit"),
        ("extras.AFC_OpenAMS", "AFC_OpenAMS"),
        ("extras.AFC_BMCU_proto", "AFC_BMCU_proto"),
        ("extras.AFC_BMCU_link", "AFC_BMCU_link"),
    ])
    def test_dependency_failure(self, module, label):
        with pytest.raises(config_error) as raised:
            _exec_afc_bmcu_with_blocked_dependency(module)
        assert str(raised.value).startswith(
            f"Error trying to import {label}, please rerun install-afc.sh")
