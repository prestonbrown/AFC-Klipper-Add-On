# AFCProject Automated Filament Changer Software
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.

"""
AFC unit for a BMCU running BMCU-Klipper firmware: four DC-motor lanes with
encoders and buffers, driven over one serial link (USB, TTL or RS485).

AFC decides every load, unload and runout; the firmware runs the motors. The
lanes have no AFC pins: the inlet sensor each lane reports becomes its prep
state, and a virtual hub reads loaded while a lane is in the toolhead.
"""

from __future__ import annotations

import contextlib
import os
import traceback

from configparser import Error as config_error
from typing import TYPE_CHECKING, Any, Callable, Dict, Iterator, List, Optional, Set, Tuple

if TYPE_CHECKING:
    from configfile import ConfigWrapper
    from gcode import GCodeCommand
    from extras.AFC_extruder import AFCExtruder
    from extras.AFC_lane import AFCLane, SpeedMode

try: from extras.AFC_utils import ERROR_STR
except:
    trace = traceback.format_exc()
    err_str = f"Error when trying to import AFC_utils.ERROR_STR\n{trace}"
    raise config_error(err_str)

try: from extras.AFC_lane import AFCLaneState
except: raise config_error(ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc()))

try: from extras.AFC_unit import afcUnit
except: raise config_error(ERROR_STR.format(import_lib="AFC_unit", trace=traceback.format_exc()))

try: from extras.AFC_OpenAMS import _ams_box_logo, _ams_box_logo_error
except:
    err_str = ERROR_STR.format(import_lib="AFC_OpenAMS", trace=traceback.format_exc())
    raise config_error(err_str)

try: from extras import AFC_BMCU_proto as proto
except:
    err_str = ERROR_STR.format(import_lib="AFC_BMCU_proto", trace=traceback.format_exc())
    raise config_error(err_str)

try: from extras.AFC_BMCU_link import RECONNECT_MAX, BMCUConnection, BMCUError
except:
    err_str = ERROR_STR.format(import_lib="AFC_BMCU_link", trace=traceback.format_exc())
    raise config_error(err_str)

CHANNELS = 4
# A load feeds this far past the bowden length and path_extra before giving up on contact.
FEED_OVERSHOOT = 50.0
# Firmware limits for one operation's distance.
MIN_MOVE_MM = 5.0
MAX_FEED_MM = 5000.0
MAX_RETRACT_MM = 2000.0
# Buffer percentage that bounds a LANE_MOVE feed (BMCU-Klipper's default).
FEED_DISTANCE_PCT = 90
# Seconds the host waits for an operation beyond its own firmware timeout.
OP_GRACE = 5.0
# An operation's timeout is twice its travel time plus this margin, as the
# firmware's distance_timeout_ms computes it, never more than the firmware allows.
OP_TIMEOUT_MARGIN = 1.5
MAX_OP_TIMEOUT = 300.0
# Speeds the firmware accepts in RUNTIME_SYNC, mm/s.
MIN_SPEED = 10.0
MAX_SPEED = 120.0
# A runout must still read empty this long after the inlet drops.
RUNOUT_DEBOUNCE = 0.3
# After a retract the buffer must not sit at an extreme (BMCU-Klipper PARK_FAILED).
PARK_BUFFER_LOW = 3
PARK_BUFFER_HIGH = 97
# During a pull-back, a buffer held at an extreme for this many polls over at
# least JAM_MIN_TIME is a jam (BMCU-Klipper PARK_JAM).
JAM_BUFFER_LOW = 2
JAM_BUFFER_HIGH = 98
JAM_SAMPLES = 3
JAM_MIN_TIME = 0.4
# A feed to the toolhead, and a pull-back from it, must move at least this
# share of the bowden.
MIN_TRAVEL_SHARE = 0.5
# Buffer % that ends a feed whose arrival the toolhead sensor decides: high
# enough that a stiff bowden reaches the sensor before contact stops the feed.
SENSOR_FEED_PCT = 98
# A pull-back whose encoder travel comes this close to its length stopped on
# the length, not on the inlet sensor, so the tip is still inside the BMCU.
PULL_BACK_LENGTH_MARGIN = 2.0
# Short extruder moves tried to reach a post-extruder sensor, as AFC core does.
TOOL_END_ATTEMPTS = 20
# Before a pull-back the buffer must leave its extremes (BMCU-Klipper waits 4 s,
# polling every 0.15 s, for 5 < pct < 95).
SETTLE_BUFFER_LOW = 5
SETTLE_BUFFER_HIGH = 95
SETTLE_TIMEOUT = 4.0
SETTLE_POLL = 0.15
# RUNTIME_SYNC failures logged one by one; after that, only once the backoff caps.
SYNC_FAILURES_LOGGED = 5
# A calibration waits for a person to move the buffer by hand.
CALIBRATION_TIMEOUT = 300.0
# How often a load checks the toolhead and hub sensors, and an unload polls motion.
SENSOR_POLL = 0.05
MOTION_POLL = 0.25
# Firmware tuning the unit does not expose, as BMCU-Klipper ships it.
RUNTIME_DEFAULTS = {
    "load_pressure_pct": 82.0,
    "pull_speed_end": 12.0,
    "jam_timeout_ms": 20000.0,
    "before_pullback_pct": 40.0,
    "autoload_mm": 120.0,
}
SYSTEM_LED_RGB = (255, 255, 255)
# Shortest pull-back length RUNTIME_SYNC accepts.
PULL_BACK_MIN_MM = 10.0


class afcBMCU(afcUnit):
    """
    AFC unit driver for a BMCU (four lanes) over the BMCU-Klipper serial protocol.
    """

    def __init__(self, config: ConfigWrapper) -> None:
        """
        Read the unit's options and register its gcode command.

        :param config: Klipper config for the [AFC_BMCU <name>] section
        """
        super().__init__(config)
        self.type = config.get('type', 'BMCU')
        self.stepperless_drive: bool = True

        # /dev/serial/by-id/... path of the BMCU
        self.serial = config.get('serial')
        self.baud = config.getint('baud', 115200, minval=1)
        # True for RS485 wiring: resends and a receive-aware write gate
        self.half_duplex = config.getboolean('half_duplex', False)
        # Firmware feed and retract speeds, mm/s
        self.load_speed = config.getfloat('load_speed', 60.0, minval=MIN_SPEED,
                                          maxval=MAX_SPEED)
        self.pull_speed = config.getfloat('pull_speed', 60.0, minval=MIN_SPEED,
                                          maxval=MAX_SPEED)
        # Buffer % that means the filament reached the extruder gears
        self.contact_pct = config.getint('contact_pct', 70, minval=55, maxval=98)
        # Shortest timeout given to one feed or retract, in seconds; longer moves get more
        self.feed_timeout = config.getfloat('feed_timeout', 30.0, minval=1.0, maxval=300.0)
        # mm a feed, pull-back or tail retract may move beyond the bowden length: the
        # path inside the BMCU from the inlet sensor to its outlet, plus tool_stn
        self.path_extra = config.getfloat('path_extra', 100.0, minval=0.0)
        # Without a toolhead sensor: share of the extruder load the encoder must see
        self.follow_tolerance = config.getfloat('follow_tolerance', 0.5, minval=0.0,
                                                maxval=1.0)

        self._fileconfig = config.fileconfig
        self._conn: Optional[BMCUConnection] = None
        self._status: Optional[Dict[str, Any]] = None
        self._prev_present: Optional[List[int]] = None
        self._channels: Dict[str, int] = {}
        self._connecting = False
        self._op_active = False
        self._follow_pending = False
        # An inlet edge arrived during an operation and waits for the next STATUS.
        self._edges_held = False
        # Slot colors the board last accepted; None until sent on this connection.
        self._slots_sent: Optional[Tuple[Tuple[int, int, int, str], ...]] = None
        self._slots_pending = False
        # RUNTIME_SYNC values the board last accepted.
        self._synced_runtime: Dict[str, float] = {}
        # Lanes whose runout was handed to AFC; cleared when the inlet sees filament again.
        self._runout_dispatched: Set[str] = set()
        # Lanes with a runout confirmation scheduled.
        self._runout_pending: Set[str] = set()
        # True from a reconnect until its setup (RUNTIME_SYNC, stop, marks) finishes.
        self._restoring = False
        self._sync_failures = 0

        # Print start or resume: the printing lane may need to follow the extruder again.
        self.printer.register_event_handler("idle_timeout:printing", self._handle_printing)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        self.printer.register_event_handler("klippy:disconnect", self._handle_disconnect)
        self.gcode.register_mux_command('AFC_BMCU_CALIBRATE', "UNIT", self.name,
                                        self.cmd_AFC_BMCU_CALIBRATE,
                                        desc=self.cmd_AFC_BMCU_CALIBRATE_help)

    # Lifecycle

    def handle_connect(self) -> None:
        """
        Refuse a BMCU-Klipper [bmcu] section on the same port, map lanes to
        channels, and create the serial connection (opened at klippy:ready).
        """
        super().handle_connect()
        self._refuse_coinstalled_bmcu()
        self._channels = self._build_channel_map()
        self.logo = _ams_box_logo("BMCU", len(self.lanes), self.name)
        self.logo_error = _ams_box_logo_error("BMCU", len(self.lanes), self.name)
        # Pinless lanes default to a loaded load sensor; start them clear.
        for lane in self.lanes.values():
            lane._load_state = False
        self._conn = BMCUConnection(self.reactor, self.serial, self.baud, self.half_duplex,
                                    self.logger, self._sync_from_status, self._on_link)

    def _handle_shutdown(self) -> None:
        """
        klippy:shutdown (M112 or an error): stop every BMCU motor. The reactor
        cannot wait here, so STOP_ALL is written twice without waiting for a
        reply, then the link is closed: no resend, PING or reconnect follows,
        a request still waiting fails, and the firmware's host-silence failsafe
        backs up the stop.
        """
        if self._conn is not None:
            self._conn.stop_all_nowait()
            self._conn.stop_all_nowait()
            self._conn.close()

    def _handle_disconnect(self) -> None:
        """
        klippy:disconnect (RESTART, FIRMWARE_RESTART, exit): stop the motors and
        release the serial port, which is opened exclusively, so the next klippy
        instance can open it. The reactor has stopped, so nothing is waited for.
        """
        if self._conn is not None:
            self._conn.stop_all_nowait()
            self._conn.close()

    def _refuse_coinstalled_bmcu(self) -> None:
        """
        Raise a config error when a BMCU-Klipper [bmcu] section names this
        unit's serial port: two hosts driving one board fight over every motor.
        """
        own = os.path.realpath(self.serial)
        for section in self._fileconfig.sections():
            if (section != "bmcu"
                and not section.startswith("bmcu ")):
                continue
            for port in self._bmcu_section_ports(section):
                if os.path.realpath(port) == own:
                    error_str = (f"[AFC_BMCU {self.name}] and [{section}] both use "
                                 f"{self.serial}. Remove one: two hosts cannot drive one BMCU.")
                    raise config_error(error_str)

    def _bmcu_section_ports(self, section: str) -> List[str]:
        """
        Serial ports a BMCU-Klipper section names: its ``serial`` option and the
        port field of each ``devices`` line (``name,port[,uid]``).

        :param section: config section name
        :return List[str]: the ports, in config order
        """
        ports = []
        if self._fileconfig.has_option(section, "serial"):
            ports.append(self._fileconfig.get(section, "serial").strip())
        if self._fileconfig.has_option(section, "devices"):
            raw = self._fileconfig.get(section, "devices")
            for line in raw.replace(";", "\n").splitlines():
                parts = [part.strip() for part in line.split(",")]
                if len(parts) >= 2:
                    ports.append(parts[1])
        return ports

    def _build_channel_map(self) -> Dict[str, int]:
        """
        Map each lane to its 0-based channel from ``unit: <name>:<1-4>``, and
        refuse a lane with a custom load or unload command: core runs those
        instead of the unit's hooks, so the BMCU would never move.

        :return Dict[str, int]: lane name to channel
        """
        channels: Dict[str, int] = {}
        owners: Dict[int, str] = {}
        for lane_name, lane in self.lanes.items():
            for option in ("custom_load_cmd", "custom_unload_cmd"):
                if getattr(lane, option, None):
                    error_str = (f"BMCU {self.name}: lane {lane_name} sets {option}, which "
                                 "would replace the BMCU's own load and unload. Remove it.")
                    raise config_error(error_str)
            if (lane.index < 1
                or lane.index > CHANNELS):
                error_str = (f"BMCU {self.name}: lane {lane_name} has index {lane.index}; "
                             f"use unit: {self.name}:1 to {self.name}:{CHANNELS}.")
                raise config_error(error_str)
            channel = lane.index - 1
            if channel in owners:
                error_str = (f"BMCU {self.name}: lanes {owners[channel]} and {lane_name} "
                             f"both use channel {lane.index}.")
                raise config_error(error_str)
            owners[channel] = lane_name
            channels[lane_name] = channel
        return channels

    def handle_ready(self) -> None:
        """
        Sort lanes, then open the serial link from a reactor callback so the
        connect does not hold up klippy:ready.
        """
        super().handle_ready()
        self._connecting = True
        self.reactor.register_callback(self._deferred_connect)

    def _deferred_connect(self, eventtime: float) -> None:
        """
        Connect and send RUNTIME_SYNC. Lane state is marked in the firmware by
        PREP (system_Test), once AFC has restored its saved state.

        :param eventtime: reactor time
        """
        try:
            try:
                self._conn.connect()
            except BMCUError as e:
                self.logger.warning(f"BMCU {self.name} not connected: {e}")
                return
            if self._sync_or_drop():
                self._conn.mark_ready()
        finally:
            # PREP waits on this, so it must cover RUNTIME_SYNC too.
            self._connecting = False

    def _sync_or_drop(self) -> bool:
        """
        Send RUNTIME_SYNC. Without it the firmware holds every motor, so a
        failure drops the link and the reconnect path tries again.

        :return bool: True when the firmware took it
        """
        try:
            self._sync_runtime()
        except BMCUError as e:
            self._sync_failures += 1
            if self._sync_failures <= SYNC_FAILURES_LOGGED:
                self.logger.error(f"BMCU {self.name} RUNTIME_SYNC failed: {e}")
            elif self._conn.reconnect_delay >= RECONNECT_MAX:
                self.logger.error(f"BMCU {self.name} RUNTIME_SYNC failed "
                                  f"{self._sync_failures} times in a row, retrying every "
                                  f"{RECONNECT_MAX:g}s: {e}")
            else:
                self.logger.debug(f"BMCU {self.name} RUNTIME_SYNC failed: {e}")
            self._conn.drop(f"RUNTIME_SYNC failed: {e}")
            return False
        self._sync_failures = 0
        return True

    def _runtime_values(self) -> Dict[str, float]:
        """
        RUNTIME_SYNC values: the configured speeds, a pull-back length that
        covers the longest bowden plus path_extra, and firmware defaults.

        :return Dict[str, float]: every key pack_runtime_sync needs
        """
        longest = max((self._bowden_length(lane) for lane in self.lanes.values()), default=0.0)
        values = dict(RUNTIME_DEFAULTS)
        values["load_speed"] = self.load_speed
        values["pull_speed"] = self.pull_speed
        values["retract_mm"] = self._retract_length(longest)
        return values

    def _retract_length(self, bowden_mm: float) -> float:
        """
        How far a pull-back or tail retract may move: the bowden plus
        path_extra, inside the firmware's limits.

        :param bowden_mm: bowden length to cover
        :return float: mm
        """
        return min(MAX_RETRACT_MM, max(PULL_BACK_MIN_MM, bowden_mm + self.path_extra))

    def _sync_runtime(self) -> None:
        """
        Send RUNTIME_SYNC. The firmware holds every motor at zero until it has
        one, and standalone mode stays off so only AFC starts motion.
        """
        values = self._runtime_values()
        payload = proto.pack_runtime_sync(values, SYSTEM_LED_RGB)
        self._conn.request(proto.MSG_RUNTIME_SYNC, payload)
        self._synced_runtime = values

    def _reconcile_lane(self, lane: AFCLane) -> None:
        """
        Mark a lane in the firmware from AFC's state: loaded when AFC has it in
        the toolhead, unloaded otherwise, whatever route the firmware reports.
        Moves no motor.

        :param lane: lane to mark
        """
        self._mark(self._channels[lane.name], lane.tool_loaded)

    # Firmware callbacks (reactor fd/timer context: never block here)

    def _sync_from_status(self, status: Dict[str, Any]) -> None:
        """
        Apply a STATUS to the lanes: prep state from the inlet sensor, hub
        state, and inlet edges dispatched to AFC's lane handler. Runout is
        level-triggered: whenever the lane in the toolhead reads empty while
        printing and no operation is moving filament, a confirmation is
        scheduled, so a drop missed during a pause or a resume is still caught.

        :param status: parsed STATUS
        """
        previous = self._prev_present
        self._status = status
        busy = self._unit_busy()
        self._prev_present = list(status["present"])
        for lane in self.lanes.values():
            channel = self._channels[lane.name]
            present = bool(status["present"][channel])
            self._apply_inlet(lane, present)
            if present:
                self._runout_dispatched.discard(lane.name)
            elif self._may_have_run_out(lane, present):
                self._schedule_runout_check(lane)
            if (previous is None
                or bool(previous[channel]) == present
                or lane.tool_loaded):
                continue
            if busy:
                # Held back, not dropped: the STATUS asked for when the
                # operation ends dispatches it.
                self._prev_present[channel] = previous[channel]
                self._edges_held = True
                continue
            self.reactor.register_callback(
                lambda et, lane=lane, state=present: self._dispatch_load_runout(lane, et, state))
        self._maybe_follow(status)
        self._maybe_push_slots()

    def _may_have_run_out(self, lane: AFCLane, present: bool) -> bool:
        """
        Whether an inlet reading can be a runout: it is empty on the printing
        lane in the toolhead, and no operation is moving filament.

        :param lane: lane the reading is for
        :param present: True when the inlet sees filament
        :return bool: True when a runout is possible
        """
        return (not present
                and lane.tool_loaded
                and not self._unit_busy()
                and self.check_runout(lane))

    def _schedule_runout_check(self, lane: AFCLane) -> None:
        """
        Confirm a runout RUNOUT_DEBOUNCE from now, unless one is already handed
        over or scheduled for the lane.

        :param lane: lane whose inlet reads empty
        """
        if (lane.name in self._runout_dispatched
            or lane.name in self._runout_pending):
            return
        self._runout_pending.add(lane.name)
        self.reactor.register_callback(lambda et, lane=lane: self._confirm_runout(lane),
                                       self.reactor.monotonic() + RUNOUT_DEBOUNCE)

    def _confirm_runout(self, lane: AFCLane) -> None:
        """
        Hand a runout to AFC once a fresh STATUS still shows the printing
        lane's inlet empty (reactor callback). Each runout is handed over once.

        :param lane: lane whose inlet emptied
        """
        self._runout_pending.discard(lane.name)
        if lane.name in self._runout_dispatched:
            return
        try:
            status = self._refresh_status()
        except BMCUError as e:
            self.logger.debug(f"BMCU {self.name} runout check failed: {e}")
            return
        if not self._may_have_run_out(lane, bool(status["present"][self._channels[lane.name]])):
            return
        self._runout_dispatched.add(lane.name)
        self._dispatch_load_runout(lane, self.reactor.monotonic(), False)

    def _apply_inlet(self, lane: AFCLane, present: bool) -> None:
        """
        Set a lane's AFC sensor state from its inlet sensor. The inlet is the
        lane's prep sensor and says whether it can feed (loaded_to_hub). A
        virtual hub reads loaded only while the lane is in the toolhead; with
        a real hub sensor the lane's load state follows the inlet.

        :param lane: lane to update
        :param present: True when the inlet sees filament
        """
        lane.prep_state = present
        lane.loaded_to_hub = present or lane.tool_loaded
        if self._is_virtual_hub(lane):
            lane._load_state = lane.tool_loaded
        else:
            lane._load_state = present

    def _unit_busy(self) -> bool:
        """
        Whether filament is moving under an operation: one this unit started,
        or one the firmware reports running (a tail retract shows its motion
        as idle, so motion alone cannot tell).

        :return bool: True while an operation runs
        """
        return (self._op_active
                or self._restoring
                or (self._status is not None
                    and self._status["active_op_state"] == proto.OP_STATE_RUNNING))

    def _maybe_follow(self, status: Dict[str, Any]) -> None:
        """
        Put the printing lane back in ON_USE when it sits idle with filament
        present, as after a reconnect or a Klipper restart, so the firmware
        follows the extruder again once the print runs.

        :param status: parsed STATUS
        """
        lane = self.lanes.get(self.afc.current)
        if (lane is None
            or self._follow_pending
            or self._unit_busy()
            or not lane.tool_loaded
            or not self.afc.function.is_printing()
            or self._toolhead_disagrees(lane) is not None):
            return
        channel = self._channels[lane.name]
        if (status["motion"][channel] != proto.MOTION_IDLE
            or not status["present"][channel]):
            return
        self._follow_pending = True
        self.reactor.register_callback(lambda et: self._resume_follow(channel))

    def _lane_slots(self) -> Tuple[Tuple[int, int, int, str], ...]:
        """
        Each channel's lane color and material, for the BMCU's channel LEDs.
        A channel without a lane, or a lane without a valid #RRGGBB color,
        is black.

        :return Tuple[Tuple[int, int, int, str], ...]: (red, green, blue, name) per channel
        """
        slots = [(0, 0, 0, "")] * CHANNELS
        for lane_name, channel in self._channels.items():
            lane = self.lanes[lane_name]
            hex_color = (lane.color or "").lstrip("#")
            try:
                rgb = bytes.fromhex(hex_color) if len(hex_color) == 6 else b"\0\0\0"
            except ValueError:
                rgb = b"\0\0\0"
            slots[channel] = (rgb[0], rgb[1], rgb[2], getattr(lane, "material", "") or "")
        return tuple(slots)

    def _maybe_push_slots(self) -> None:
        """
        Schedule SET_SLOTS when the lanes' colors differ from what the board
        last accepted, as after a color change or a reconnect (the board keeps
        them in RAM only).
        """
        if (self._slots_pending
            or not self._conn.caps & proto.CAP_VOLATILE_SLOTS
            or self._lane_slots() == self._slots_sent):
            return
        self._slots_pending = True
        self.reactor.register_callback(self._push_slots)

    def _push_slots(self, eventtime: float) -> None:
        """
        Send the lanes' colors with SET_SLOTS (reactor callback). A failure is
        logged at debug level and retried on the next STATUS.

        :param eventtime: reactor time
        """
        slots = self._lane_slots()
        try:
            self._conn.request(proto.MSG_SET_SLOTS, proto.pack_slots(slots))
            self._slots_sent = slots
        except BMCUError as e:
            self.logger.debug(f"BMCU {self.name} slot colors not sent: {e}")
        finally:
            self._slots_pending = False

    def _handle_printing(self, print_time: float) -> None:
        """
        The toolhead went busy (a print started or resumed). An idle board
        reports STATUS only every 30 s, so ask for one now and let
        _maybe_follow see it.

        :param print_time: print time of the transition
        """
        # A resume retries a runout whose earlier handling did not unload the lane.
        self._runout_dispatched.clear()
        self.reactor.register_callback(self._refresh_follow)

    def _refresh_follow(self, eventtime: float) -> None:
        """
        Request a STATUS so an idle printing lane resumes ON_USE, and hand over
        a runout that happened while the print was paused (reactor callback).

        :param eventtime: reactor time
        """
        status = self._refresh_quietly(eventtime)
        if status is None:
            return
        lane = self.lanes.get(self.afc.current)
        if (lane is not None
            and lane.tool_loaded
            and not status["present"][self._channels[lane.name]]):
            self._schedule_runout_check(lane)

    def _resume_follow(self, channel: int) -> None:
        """
        Send SET_MOTION ON_USE for the printing lane (reactor callback),
        unless an operation started since it was scheduled.

        :param channel: channel to follow the extruder
        """
        try:
            if not self._unit_busy():
                self._set_motion(channel, proto.MOTION_ON_USE)
        except BMCUError as e:
            self.logger.warning(f"BMCU {self.name} could not resume following the "
                                f"extruder: {e}")
        finally:
            self._follow_pending = False

    def _dispatch_load_runout(self, lane: AFCLane, eventtime: float, load_state: bool) -> None:
        """
        Run a lane's load/runout handler, reporting a failure as an AFC error
        (pausing when in a print) instead of losing it in the reactor.

        :param lane: lane whose inlet sensor changed
        :param eventtime: reactor time of the change
        :param load_state: True when filament is present at the inlet
        """
        what = 'load' if load_state else 'runout'
        try:
            lane.handle_load_runout(eventtime, load_state)
        except Exception as e:
            # Let a later STATUS retry it.
            self._runout_dispatched.discard(lane.name)
            self.logger.error(f"{lane.name}: {what} handling failed: {e}",
                              traceback=traceback.format_exc())
            self.afc.error.AFC_error(f"Lane {lane.name} {what} handling failed: {e}\n"
                                     "Filament may still be loaded in the toolhead, check the "
                                     "toolhead before resuming.",
                                     pause=self.afc.function.in_print())

    def _on_link(self, online: bool) -> None:
        """
        Link state change from the connection. The handling blocks or pauses,
        so it runs from a reactor callback.

        :param online: True when the link came back, False when it dropped
        """
        if online:
            # Unit operations refuse to start until the restore has finished.
            self._restoring = True
        handler = self._handle_link_restored if online else self._handle_link_lost
        self.reactor.register_callback(handler)

    def _handle_link_lost(self, eventtime: float) -> None:
        """
        Pause a running print when the link drops; nothing re-arms on its own.

        :param eventtime: reactor time
        """
        self._prev_present = None
        self._slots_sent = None
        if self.afc.function.is_printing():
            self.afc.error.AFC_error(f"BMCU {self.name} disconnected during a print. "
                                     "Reconnect it, then resume.", pause=True)
        else:
            self.logger.warning(f"BMCU {self.name} disconnected.")

    def _handle_link_restored(self, eventtime: float) -> None:
        """
        After a reconnect: RUNTIME_SYNC, stop every lane, and re-mark lanes from
        AFC's state. A lane follows the extruder again only once printing runs.
        Unit operations stay refused until this finishes, and the reconnect
        backoff resets only when all of it succeeded.

        :param eventtime: reactor time
        """
        try:
            if not self._sync_or_drop():
                return
            try:
                self._conn.stop_all()
                for lane in self.lanes.values():
                    self._reconcile_lane(lane)
                status = self._refresh_status()
            except BMCUError as e:
                # Dropping holds the backoff and retries the whole restore.
                self.logger.warning(f"BMCU {self.name} reconnect setup failed: {e}")
                self._conn.drop(f"reconnect setup failed: {e}")
                return
            for lane in self.lanes.values():
                self._show_lane(lane, status)
            self._conn.mark_ready()
            self.logger.info(f"BMCU {self.name} reconnected, every lane is idle.")
        finally:
            self._restoring = False

    # Helpers

    def _is_virtual_hub(self, lane: AFCLane) -> bool:
        """
        Whether the lane's hub is a virtual (pinless) hub.

        :param lane: lane to check
        :return bool: True for a virtual hub
        """
        hub = lane.hub_obj
        return hub is not None and hub.is_virtual_pin()

    def _real_hub(self, lane: AFCLane) -> Optional[Any]:
        """
        The lane's hub when it has a real sensor.

        :param lane: lane to check
        :return Optional[Any]: the hub, or None for a virtual or missing hub
        """
        hub = lane.hub_obj
        if (hub is None
            or hub.is_virtual_pin()):
            return None
        return hub

    def _bowden_length(self, lane: AFCLane) -> float:
        """
        Distance from the lane's hub to the toolhead.

        :param lane: lane to measure
        :return float: the hub's afc_bowden_length, or the lane's dist_hub when
            the hub uses dist_hub, 0 without a hub
        """
        hub = lane.hub_obj
        if hub is None:
            return 0.0
        if getattr(hub, "use_dist_hub", False):
            return float(lane.dist_hub)
        return float(hub.afc_bowden_length)

    def _connected(self) -> bool:
        """
        Whether the serial link is up.

        :return bool: True while connected
        """
        return self._conn is not None and self._conn.connected

    def _calibrated(self, channel: int) -> bool:
        """
        Whether the firmware holds a calibration for the channel.

        :param channel: channel 0-3
        :return bool: True when the last STATUS marks it calibrated
        """
        return (self._status is not None
                and bool(self._status["calibration_valid_mask"] & (1 << channel)))

    def _calibrate_command(self, lane: AFCLane) -> str:
        """
        The gcode a user runs to calibrate a lane.

        :param lane: lane to calibrate
        :return str: the AFC_BMCU_CALIBRATE command line
        """
        return f"AFC_BMCU_CALIBRATE UNIT={self.name} LANE={lane.name}"

    def _op_timeout(self, millimeters: float, speed: float) -> float:
        """
        Timeout for a move: twice its travel time plus OP_TIMEOUT_MARGIN, as the
        firmware's distance_timeout_ms does, at least feed_timeout and at most
        MAX_OP_TIMEOUT.

        :param millimeters: distance of the move
        :param speed: speed of the move in mm/s
        :return float: seconds
        """
        estimate = millimeters / speed * 2.0 + OP_TIMEOUT_MARGIN
        return min(MAX_OP_TIMEOUT, max(self.feed_timeout, estimate))

    def _same_session(self, generation: int) -> None:
        """
        Raise when the board was reconnected since an operation started: its
        motion state and route marks were reset, so the operation cannot go on.

        :param generation: connection generation when the operation started
        """
        if self._conn.generation != generation:
            error_str = "the BMCU reconnected during the operation and lost its state"
            raise BMCUError(error_str)

    @contextlib.contextmanager
    def _operation(self) -> Iterator[int]:
        """
        Mark a unit operation as running, so inlet edges are not taken as
        runout, and give it the connection generation to check against. An
        operation cannot start while a reconnect's setup is still running.

        :return Iterator[int]: the connection generation at the start
        """
        if self._restoring:
            error_str = "the BMCU is still being set up after a reconnect"
            raise BMCUError(error_str)
        self._op_active = True
        try:
            yield self._conn.generation
        finally:
            self._op_active = False
            if self._edges_held:
                self._edges_held = False
                self.reactor.register_callback(self._refresh_quietly)

    def _mark(self, channel: int, loaded: bool) -> None:
        """
        MARK_LOADED or MARK_UNLOADED for one channel. Moves no motor.

        :param channel: channel 0-3
        :param loaded: True for MARK_LOADED
        """
        msg_type = proto.MSG_MARK_LOADED if loaded else proto.MSG_MARK_UNLOADED
        self._conn.request(msg_type, proto.pack_channel(channel))

    def _timeout_ms(self, seconds: float) -> int:
        """
        Convert a timeout to the firmware's milliseconds.

        :param seconds: operation timeout in seconds
        :return int: the timeout in ms
        """
        return int(round(seconds * 1000.0))

    def _refresh_status(self) -> Dict[str, Any]:
        """
        Ask for a STATUS and return the reply itself, parsed. The reply also
        reaches _sync_from_status like any other.

        :return Dict[str, Any]: the parsed STATUS
        """
        _reply_type, payload = self._conn.request(proto.MSG_GET_STATUS)
        try:
            return proto.parse_status(payload)
        except ValueError as e:
            error_str = f"BMCU sent a bad STATUS: {e}"
            raise BMCUError(error_str)

    def _refresh_quietly(self, eventtime: float) -> Optional[Dict[str, Any]]:
        """
        Ask for a STATUS from a reactor callback without waiting for the next
        periodic one, as for inlet edges held back during an operation. A
        failure is logged at debug level.

        :param eventtime: reactor time
        :return Optional[Dict[str, Any]]: the parsed STATUS, or None when not
            connected or the request failed
        """
        if not self._connected():
            return None
        try:
            return self._refresh_status()
        except BMCUError as e:
            self.logger.debug(f"BMCU {self.name} status refresh failed: {e}")
            return None

    def _set_motion(self, channel: int, motion: int) -> None:
        """
        Send SET_MOTION for one channel.

        :param channel: channel 0-3
        :param motion: MOTION_* state
        """
        self._conn.request(proto.MSG_SET_MOTION, proto.pack_set_motion(channel, motion))

    def _run_op(self, msg_type: int, payload: bytes, timeout: float) -> Dict[str, Any]:
        """
        Start a firmware operation and wait for its result.

        :param msg_type: operation message id
        :param payload: operation payload
        :param timeout: seconds the firmware was given
        :return Dict[str, Any]: the parsed OP_RESULT
        """
        op_id = self._conn.start_op(msg_type, payload)
        return self._conn.wait_op(op_id, timeout + OP_GRACE)

    def _stop_after_failure(self) -> None:
        """
        STOP_ALL after a failed operation; every channel goes idle. A dead link
        has nothing to stop.
        """
        try:
            self._conn.stop_all()
        except BMCUError as e:
            self.logger.debug(f"BMCU {self.name} stop after failure: {e}")

    def _abort_quietly(self, op_id: int) -> None:
        """
        ABORT_OP from a reactor callback, only while the operation is still
        open: an abort sent after it finished would stop the next one. A
        failure is logged at debug level.

        :param op_id: the operation to stop
        """
        if not self._conn.op_open(op_id):
            return
        try:
            self._conn.abort_op()
        except BMCUError as e:
            self.logger.debug(f"BMCU {self.name} abort failed: {e}")

    def _wait_motion_idle(self, channel: int, timeout: float) -> Optional[str]:
        """
        Poll STATUS until the channel's motion is idle. A buffer held at an
        extreme for JAM_SAMPLES polls over JAM_MIN_TIME is a jam.

        :param channel: channel 0-3
        :param timeout: seconds to wait
        :return Optional[str]: None once idle, else why it stopped waiting
        """
        deadline = self.reactor.monotonic() + timeout
        extreme_since = 0.0
        extreme_samples = 0
        while True:
            status = self._refresh_status()
            if status["motion"][channel] == proto.MOTION_IDLE:
                return None
            now = self.reactor.monotonic()
            buffer_pct = status["buffer_pct"][channel]
            if (buffer_pct <= JAM_BUFFER_LOW
                or buffer_pct >= JAM_BUFFER_HIGH):
                if extreme_samples == 0:
                    extreme_since = now
                extreme_samples += 1
                if (extreme_samples >= JAM_SAMPLES
                    and now - extreme_since >= JAM_MIN_TIME):
                    return f"jammed: the buffer stayed at {buffer_pct}% during the pull back."
            else:
                extreme_samples = 0
            if now >= deadline:
                return f"pull back did not finish within {timeout:.0f}s."
            self.reactor.pause(min(deadline, now + MOTION_POLL))

    # AFC unit interface

    def check_runout(self, cur_lane: AFCLane) -> bool:
        """
        Runout acts only on the lane in the toolhead while printing.

        :param cur_lane: lane whose inlet emptied
        :return bool: True when AFC's runout handling should run
        """
        return (cur_lane.name == self.afc.current
                and bool(self.afc.function.is_printing()))

    def handle_runout(self, lane: AFCLane) -> bool:
        """
        Without the firmware's tail retract, a runout tail cannot be pulled
        out of the path, so pause instead of letting AFC load another lane.
        The person clearing the path by hand takes the lane out of the
        toolhead, so AFC and the firmware mark it unloaded now; otherwise the
        same runout would pause the print again on resume.

        :param lane: lane that ran out
        :return bool: True when handled here (paused), False to let AFC's
            infinite spool or pause handling run
        """
        if self._conn.caps & proto.CAP_TAIL_RETRACT:
            return False
        lane.set_tool_unloaded(normal_toolchange=True)
        self.lane_not_ready(lane)
        self.afc.save_vars()
        try:
            self._mark(self._channels[lane.name], False)
        except BMCUError as e:
            self.logger.debug(f"BMCU {self.name} runout unmark failed: {e}")
        self.afc.error.AFC_error(f"{lane.name} ran out. This BMCU firmware cannot retract the "
                                 f"tail: clear the {lane.name} path by hand, load a lane, then "
                                 "resume.", pause=True)
        return True

    def get_status(self, eventtime: Optional[float] = None) -> Dict[str, Any]:
        """
        Unit status plus the link and per-lane firmware state, for screens
        and diagnostics. Board values are None while disconnected.

        :param eventtime: reactor time
        :return Dict[str, Any]: base unit status, ``connected``, ``half_duplex``,
            ``firmware``, ``tail_retract``, ``last_error`` (why the link last
            dropped or failed to connect, "" when it did not), ``fault_flags``
            (the board's controller fault bits), ``nvm_fault`` and ``bmcu_lanes``
            (per lane: channel, buffer_pct, calibrated, route, inlet, motion,
            encoder_ok)
        """
        response = super().get_status(eventtime)
        hello = self._conn.hello if self._conn is not None else None
        caps = self._conn.caps if self._conn is not None else 0
        response["connected"] = self._connected()
        response["half_duplex"] = self.half_duplex
        response["firmware"] = hello["firmware"] if hello is not None else None
        response["tail_retract"] = bool(caps & proto.CAP_TAIL_RETRACT)
        response["last_error"] = self._conn.last_error if self._conn is not None else ""
        # Values from before a disconnect are not the board's state now.
        status = self._status if self._connected() else None
        response["fault_flags"] = status["controller_fault_flags"] if status else None
        response["nvm_fault"] = status["nvm_fault"] if status else None
        lanes: Dict[str, Dict[str, Any]] = {}
        for lane_name, channel in self._channels.items():
            if status is None:
                board = dict.fromkeys(("buffer_pct", "calibrated", "route", "inlet", "motion",
                                       "encoder_ok"))
            else:
                board = {
                    "buffer_pct": status["buffer_pct"][channel],
                    "calibrated": self._calibrated(channel),
                    "route": status["route_state_name"][channel],
                    "inlet": bool(status["present"][channel]),
                    "motion": proto.MOTION_NAMES.get(status["motion"][channel], "UNKNOWN"),
                    "encoder_ok": bool(status["encoder_io_mask"] & (1 << channel)),
                }
            lanes[lane_name] = {"channel": channel + 1, **board}
        response["bmcu_lanes"] = lanes
        return response

    def _wait_first_connect(self) -> None:
        """
        Let the connect started at klippy:ready finish before PREP reads the unit.
        """
        while self._connecting:
            self.reactor.pause(self.reactor.monotonic() + 0.1)

    def system_Test(self, cur_lane: AFCLane, delay: float, assignTcmd: bool,
                    enable_movement: bool) -> bool:
        """
        PREP check for one lane: marks it in the firmware from AFC's restored
        state and sets prep and LED state from the firmware. Moves no motor.

        :param cur_lane: lane to check
        :param delay: unused
        :param assignTcmd: True to assign the lane's T command
        :param enable_movement: unused, a BMCU check never moves filament
        :return bool: True when the lane is usable
        """
        self._wait_first_connect()
        try:
            if not self._connected():
                error_str = f"BMCU on {self.serial} is not connected"
                raise BMCUError(error_str)
            self._reconcile_lane(cur_lane)
            status = self._refresh_status()
        except BMCUError as e:
            self.logger.debug(f"BMCU {self.name} PREP: {e}")
            if e.code is None:
                msg = '<span class=error--text>BMCU NOT CONNECTED</span>'
            else:
                # The firmware answered and refused, e.g. error 71 when its flash
                # cannot save the lane's route: not a link problem.
                msg = f'<span class=error--text>BMCU REFUSED: {e}</span>'
            succeeded = False
            self.lane_fault(cur_lane)
        else:
            succeeded, msg = self._show_lane(cur_lane, status)

        if assignTcmd:
            self.afc.function.TcmdAssign(cur_lane)
        cur_lane.send_lane_data()
        self.logger.info(f'{cur_lane.name} tool cmd: {cur_lane.map_to_string():3} {msg}')
        cur_lane.set_afc_prep_done()
        return succeeded

    def _show_lane(self, lane: AFCLane, status: Dict[str, Any]) -> Tuple[bool, str]:
        """
        Set a lane's sensor state, AFC status and LEDs from a STATUS, as PREP
        and a reconnect do.

        :param lane: lane to show
        :param status: parsed STATUS
        :return Tuple[bool, str]: whether the lane is usable, and its PREP message
        """
        channel = self._channels[lane.name]
        present = bool(status["present"][channel])
        self._apply_inlet(lane, present)
        if not self._calibrated(channel):
            self.lane_fault(lane)
            return False, ('<span class=error--text>NOT CALIBRATED, run '
                           f'{self._calibrate_command(lane)}</span>')
        if lane.tool_loaded:
            reason = self._toolhead_disagrees(lane)
            if reason is not None:
                self.lane_fault(lane)
                return False, (f'<span class=error--text>{reason} Check the toolhead, then '
                               'run TOOL_UNLOAD or UNSET_LANE_LOADED.</span>')
            lane.status = AFCLaneState.TOOLED
            if self.afc.current == lane.name:
                self.afc.spool.set_active_spool(lane.spool_id)
                self.lane_tool_loaded(lane)
            else:
                self.lane_tool_loaded_idle(lane)
            return True, "<span class=success--text>LOCKED AND LOADED</span>"
        if present:
            lane.status = AFCLaneState.LOADED
            self.lane_loaded(lane)
            self.lane_illuminate_spool(lane)
            return True, "<span class=success--text>LOCKED AND LOADED</span>"
        self.lane_not_ready(lane)
        return True, "<span class=success--text>EMPTY READY FOR SPOOL</span>"

    def _toolhead_disagrees(self, lane: AFCLane) -> Optional[str]:
        """
        Why the toolhead does not back AFC's record that a lane is loaded in
        it: the extruder names another lane, or its toolhead sensor is empty.
        Without a toolhead sensor the record cannot be checked.

        :param lane: lane AFC has as tool_loaded
        :return Optional[str]: None when they agree, else the disagreement
        """
        extruder = lane.extruder_obj
        if extruder.lane_loaded != lane.name:
            return f"AFC has {lane.name} loaded but its extruder has {extruder.lane_loaded}."
        if (self._has_toolhead_sensor(extruder)
            and not lane.get_toolhead_pre_sensor_state()
            and not extruder.tool_end_state):
            return f"AFC has {lane.name} loaded but the toolhead sensor is empty."
        return None

    cmd_AFC_BMCU_CALIBRATE_help = "Calibrate one BMCU lane's buffer"
    def cmd_AFC_BMCU_CALIBRATE(self, gcmd: GCodeCommand) -> None:
        """
        Calibrate a lane's buffer sensor. The lane must be empty, and a person
        moves the lane's buffer by hand while it runs. The calibration is
        stored in the BMCU's flash.

        Usage
        -------
        `AFC_BMCU_CALIBRATE UNIT=<unit> LANE=<lane>`

        Example
        -------
        ```
        AFC_BMCU_CALIBRATE UNIT=BMCU_1 LANE=bmcu1
        ```
        """
        lane_name = gcmd.get('LANE')
        lane = self.lanes.get(lane_name)
        refusal = None
        if lane is None:
            refusal = f"{lane_name} is not a lane of BMCU {self.name}."
        elif self.afc.function.is_printing():
            refusal = "Cannot calibrate a BMCU lane while printing."
        elif not self._connected():
            refusal = f"BMCU {self.name} is not connected."
        elif (lane.tool_loaded
              or lane.prep_state):
            refusal = f"Remove the filament from {lane.name} before calibrating it."
        if refusal is not None:
            self.afc.error.AFC_error(refusal, pause=False)
            return

        channel = self._channels[lane.name]
        self.logger.info(f"Calibrating {lane.name}: push its buffer all the way in, let it "
                         "center, pull it all the way out, let it center.")
        try:
            with self._operation():
                # The firmware refuses a calibration while any channel moves or
                # this one's route reads loaded or uncertain; the lane is empty.
                self._conn.stop_all()
                self._mark(channel, False)
                result = self._run_op(proto.MSG_CAL_AUTO_START,
                                      proto.pack_cal_auto_start(channel), CALIBRATION_TIMEOUT)
        except BMCUError as e:
            self._stop_after_failure()
            self.afc.error.AFC_error(f"{lane.name} calibration failed: {e}", pause=False)
            return
        if result["ok"]:
            self.logger.info(f"{lane.name} calibrated.")
        else:
            self.afc.error.AFC_error(f"{lane.name} calibration failed: {result['reason']}",
                                     pause=False)

    # Load, unload, runout

    def _feed_to_contact(self, cur_lane: AFCLane, channel: int,
                         has_sensor: bool) -> Tuple[Dict[str, Any], bool]:
        """
        FEED_TO_CONTACT toward the toolhead, aborted early when the toolhead
        sensor triggers. A timer watches the sensors and the abort is sent from
        a reactor callback, so the wait keeps polling for a lost result.

        :param cur_lane: lane to feed
        :param channel: its channel
        :param has_sensor: True when the extruder has a toolhead sensor
        :return Tuple[Dict[str, Any], bool]: the OP_RESULT, and whether a real
            hub sensor saw filament during the feed
        """
        hub = self._real_hub(cur_lane)
        watch = {"hub_seen": False, "stopping": False, "op_id": 0}

        def _tick(eventtime: float) -> float:
            """
            Sensor watch timer: note the hub sensor, and stop the feed once the
            toolhead sensor triggers while the operation is still open.

            :param eventtime: reactor time
            :return float: next waketime
            """
            if (hub is not None
                and hub.state):
                watch["hub_seen"] = True
            if (has_sensor
                and not watch["stopping"]
                and self._conn.op_open(watch["op_id"])
                and cur_lane.get_toolhead_pre_sensor_state()):
                watch["stopping"] = True
                self.reactor.register_callback(
                    lambda et: self._abort_quietly(watch["op_id"]))
            return eventtime + SENSOR_POLL

        feed_mm = min(MAX_FEED_MM,
                      self._bowden_length(cur_lane) + self.path_extra + FEED_OVERSHOOT)
        timeout = self._op_timeout(feed_mm, self.load_speed)
        feed_pct = SENSOR_FEED_PCT if has_sensor else self.contact_pct
        payload = proto.pack_feed(channel, feed_mm, feed_pct, self._timeout_ms(timeout))
        watch["op_id"] = self._conn.start_op(proto.MSG_FEED_TO_CONTACT, payload)
        timer = self.reactor.register_timer(_tick, self.reactor.NOW)
        try:
            result = self._conn.wait_op(watch["op_id"], timeout + OP_GRACE)
        finally:
            self.reactor.unregister_timer(timer)
        hub_seen = watch["hub_seen"] or (hub is not None and bool(hub.state))
        return result, hub_seen

    def _has_toolhead_sensor(self, cur_extruder: AFCExtruder) -> bool:
        """
        Whether the extruder has a toolhead sensor the unit can judge a load by:
        a pre-extruder switch or a U1 head motion sensor. "buffer" (ramming)
        means the BMCU's own buffer: BMCU lanes have no AFC buffer, so such a
        load is judged like one with no sensor.

        :param cur_extruder: extruder to check
        :return bool: True for a real toolhead sensor
        """
        return (cur_extruder.has_toolhead_sensor
                and cur_extruder.tool_start != "buffer")

    def _extrude_to_tool_end(self, cur_lane: AFCLane,
                             cur_extruder: AFCExtruder) -> Tuple[Optional[str], float]:
        """
        With a post-extruder sensor, extrude in short moves until it triggers,
        as AFC core does before the tool_stn move.

        :param cur_lane: lane being loaded
        :param cur_extruder: extruder it loads into
        :return Tuple[Optional[str], float]: None or why it failed, and the mm extruded
        """
        extruded = 0.0
        attempts = 0
        while (cur_extruder.tool_end
               and not cur_extruder.tool_end_state):
            attempts += 1
            if attempts > TOOL_END_ATTEMPTS:
                return ("did not reach the post-extruder sensor after "
                        f"{TOOL_END_ATTEMPTS} short moves."), extruded
            self.afc.move_e_pos(cur_lane.short_move_dis, cur_extruder.tool_load_speed,
                                "Tool end", wait_tool=True)
            extruded += cur_lane.short_move_dis
        return None, extruded

    def _load(self, cur_lane: AFCLane, cur_extruder: AFCExtruder, channel: int,
              generation: int) -> Optional[str]:
        """
        The load sequence: feed to contact, hold pressure while the extruder
        takes the filament in, verify, then follow the extruder.

        :param cur_lane: lane to load
        :param cur_extruder: extruder it loads into
        :param channel: its channel
        :param generation: connection generation of the load
        :return Optional[str]: None on success, else why it failed
        """
        has_sensor = self._has_toolhead_sensor(cur_extruder)
        if (has_sensor
            and cur_lane.get_toolhead_pre_sensor_state()):
            return ("cannot load: the toolhead sensor already sees filament. Clear the "
                    "toolhead first.")
        result, hub_seen = self._feed_to_contact(cur_lane, channel, has_sensor)
        hub = self._real_hub(cur_lane)
        min_feed_mm = self._bowden_length(cur_lane) * MIN_TRAVEL_SHARE
        if result["measured_mm"] < min_feed_mm:
            return (f"BMCU fed {result['measured_mm']:.1f} mm, short of the "
                    f"{min_feed_mm:.1f} mm needed to reach the toolhead (feed ended: "
                    f"{result['reason']}).")
        if has_sensor:
            if not cur_lane.get_toolhead_pre_sensor_state():
                return (f"did not reach the toolhead sensor (BMCU feed ended: "
                        f"{result['reason']}).")
        elif not result["ok"]:
            # A contact feed ends DONE only on contact; running out of length fails.
            return f"did not reach the extruder gears (BMCU feed ended: {result['reason']})."
        if (hub is not None
            and not hub_seen):
            return (f"was fed but hub sensor {hub.name} never saw it. Check the path "
                    "between the BMCU and the hub.")

        self._set_motion(channel, proto.MOTION_BEFORE_ON_USE)
        start_m = self._refresh_status()["meters"][channel]
        cur_lane.activate_toolhead_extruder()
        reason, extruded = self._extrude_to_tool_end(cur_lane, cur_extruder)
        if reason is not None:
            return reason
        tool_stn = cur_extruder.tool_stn or 0
        self.afc.move_e_pos(tool_stn, cur_extruder.tool_load_speed, "tool stn", wait_tool=True)
        extruded += tool_stn
        # The extruder moves let the reactor run long enough for a reconnect,
        # which resets the encoder and the route marks.
        self._same_session(generation)
        if has_sensor:
            if not cur_lane.get_toolhead_pre_sensor_state():
                return "left the toolhead sensor while the extruder loaded it."
        else:
            moved_mm = (self._refresh_status()["meters"][channel] - start_m) * 1000.0
            needed_mm = extruded * self.follow_tolerance
            if moved_mm < needed_mm:
                return (f"encoder moved {moved_mm:.1f} mm while the extruder loaded "
                        f"{extruded:.1f} mm (needs {needed_mm:.1f} mm). The extruder "
                        "did not take the filament.")

        self._mark(channel, True)
        self._set_motion(channel, proto.MOTION_ON_USE)
        return None

    def _run_sequence(self, sequence: Callable[[AFCLane, AFCExtruder, int, int], Optional[str]],
                      what: str, cur_lane: AFCLane, cur_extruder: AFCExtruder,
                      pause: bool) -> bool:
        """
        Run `_load` or `_unload` as one unit operation. A failure, including an
        exception such as a gcode error from an extruder move, stops every
        motor and is reported through handle_lane_failure.

        :param sequence: `_load` or `_unload`
        :param what: "load" or "unload", for an exception's message
        :param cur_lane: lane to move
        :param cur_extruder: its extruder
        :param pause: True to pause the print on failure
        :return bool: True when the sequence succeeded
        """
        channel = self._channels[cur_lane.name]
        try:
            with self._operation() as generation:
                reason = sequence(cur_lane, cur_extruder, channel, generation)
        except Exception as e:
            reason = f"{what} failed: {e}"
        if reason is None:
            return True
        self._stop_after_failure()
        self.afc.error.handle_lane_failure(cur_lane, reason, pause=pause)
        return False

    def unit_load_lane(self, cur_lane: AFCLane, cur_extruder: AFCExtruder) -> bool:
        """
        Load a lane into the toolhead (AFC.load_sequence's unit hook). An
        uncalibrated lane is refused before any frame is sent. A failed load
        leaves AFC's tool unloaded (see _run_sequence).

        :param cur_lane: lane to load
        :param cur_extruder: extruder it loads into
        :return bool: True when loaded
        """
        channel = self._channels[cur_lane.name]
        pause = self.afc.function.in_print()
        if not self._connected():
            self.afc.error.handle_lane_failure(cur_lane, f"cannot load: BMCU {self.name} "
                                               "is not connected.", pause=pause)
            return False
        if not self._calibrated(channel):
            self.afc.error.handle_lane_failure(
                cur_lane, f"is not calibrated. Run {self._calibrate_command(cur_lane)}, "
                "then load it again.", pause=pause)
            return False
        if not self._run_sequence(self._load, "load", cur_lane, cur_extruder, pause):
            return False

        if self._is_virtual_hub(cur_lane):
            cur_lane._load_state = True
        cur_lane.status = AFCLaneState.TOOL_LOADED
        self.afc.save_vars()
        return True

    def _wait_buffer_settled(self, channel: int) -> Optional[str]:
        """
        Wait up to SETTLE_TIMEOUT for the buffer to leave its extremes, as it
        can sit at one right after tip forming, which the pull-back jam check
        would take for a jam.

        :param channel: channel 0-3
        :return Optional[str]: None once inside the limits, else why not
        """
        deadline = self.reactor.monotonic() + SETTLE_TIMEOUT
        while True:
            buffer_pct = self._refresh_status()["buffer_pct"][channel]
            if SETTLE_BUFFER_LOW < buffer_pct < SETTLE_BUFFER_HIGH:
                return None
            now = self.reactor.monotonic()
            if now >= deadline:
                return (f"the buffer stayed at {buffer_pct}% before the pull back: the "
                        "filament may be jammed.")
            self.reactor.pause(min(deadline, now + SETTLE_POLL))

    def _pull_back(self, channel: int, min_pull_mm: Optional[float] = None,
                   start_m: Optional[float] = None) -> Optional[str]:
        """
        PULL_BACK: the firmware retracts until the inlet sensor clears (or its
        configured length), then parks the filament at the sensor. It starts
        once the buffer has left its extremes. After BEFORE_PULL_BACK the
        firmware counts its length from where BEFORE_PULL_BACK began, so the
        caller passes that position as start_m.

        :param channel: channel 0-3
        :param min_pull_mm: when given, the encoder must have pulled at least this far
        :param start_m: encoder position the firmware counts the pull from, now when None
        :return Optional[str]: None once the channel is idle, else why it failed
        """
        if start_m is None:
            start_m = self._refresh_status()["meters"][channel]
        reason = self._wait_buffer_settled(channel)
        if reason is not None:
            return reason
        # The firmware's length comes from the last RUNTIME_SYNC; a bowden
        # changed since then (SET_BOWDEN_LENGTH) must reach it first.
        if self._runtime_values() != self._synced_runtime:
            self._sync_runtime()
        length_mm = self._synced_runtime["retract_mm"]
        timeout = self._op_timeout(length_mm, self.pull_speed)
        self._set_motion(channel, proto.MOTION_PULL_BACK)
        reason = self._wait_motion_idle(channel, timeout)
        if reason is not None:
            return reason
        end_m = self._refresh_status()["meters"][channel]
        # The firmware ends a pull-back that used its whole length as quietly as
        # one that reached the inlet sensor.
        pulled_mm = (start_m - end_m) * 1000.0
        if pulled_mm >= length_mm - PULL_BACK_LENGTH_MARGIN:
            return (f"pull back used its whole {length_mm:.0f} mm without reaching the inlet "
                    "sensor: the tip may still be inside the BMCU. Raise path_extra "
                    "or clear the path by hand.")
        if (min_pull_mm is not None
            and pulled_mm < min_pull_mm):
            return (f"pull back moved {pulled_mm:.1f} mm, short of the {min_pull_mm:.1f} mm "
                    "it must pull to clear the path.")
        return None

    def _retract_from_path(self, cur_lane: AFCLane, channel: int,
                           start_m: Optional[float] = None) -> Optional[str]:
        """
        Pull the lane's filament back out of the path: PULL_BACK while the
        inlet still sees filament, else a tail retract of the bowden length
        plus path_extra.

        :param cur_lane: lane to retract
        :param channel: its channel
        :param start_m: encoder position before the unload began, for the
            pull-back distance check
        :return Optional[str]: None on success, else why it failed
        """
        if self._refresh_status()["present"][channel]:
            return self._pull_back(channel,
                                   self._bowden_length(cur_lane) * MIN_TRAVEL_SHARE,
                                   start_m)
        clear_path = f"Clear the {cur_lane.name} path by hand before loading another lane."
        if not self._conn.caps & proto.CAP_TAIL_RETRACT:
            return f"ran out and this BMCU firmware cannot retract the tail. {clear_path}"
        retract_mm = self._retract_length(self._bowden_length(cur_lane))
        timeout = self._op_timeout(retract_mm, self.pull_speed)
        # The firmware refuses a tail retract on a channel that is not idle.
        self._set_motion(channel, proto.MOTION_IDLE)
        payload = proto.pack_tail_retract(channel, retract_mm, self._timeout_ms(timeout))
        result = self._run_op(proto.MSG_TAIL_RETRACT, payload, timeout)
        if not result["ok"]:
            return f"tail retract failed ({result['reason']}). {clear_path}"
        return None

    def _check_retracted(self, cur_lane: AFCLane, cur_extruder: AFCExtruder,
                         channel: int) -> Optional[str]:
        """
        Proof the filament left the toolhead and the BMCU parked cleanly: no
        toolhead sensor still triggered, and the buffer not at an extreme
        (BMCU-Klipper's PARK_FAILED limits).

        :param cur_lane: lane that was retracted
        :param cur_extruder: its extruder
        :param channel: its channel
        :return Optional[str]: None when clean, else why not
        """
        if ((self._has_toolhead_sensor(cur_extruder)
             and cur_lane.get_toolhead_pre_sensor_state())
            or cur_extruder.tool_end_state):
            return "retracted but the toolhead sensor still sees filament."
        buffer_pct = self._refresh_status()["buffer_pct"][channel]
        if (buffer_pct <= PARK_BUFFER_LOW
            or buffer_pct >= PARK_BUFFER_HIGH):
            return (f"retracted but the buffer ended at {buffer_pct}%: the filament may be "
                    "jammed or slipping.")
        return None

    def _unload(self, cur_lane: AFCLane, cur_extruder: AFCExtruder, channel: int,
                generation: int) -> Optional[str]:
        """
        The unload sequence: relieve the buffer, cut or form the tip, retract
        out of the extruder, pull the filament back, prove it left, mark it.

        :param cur_lane: lane to unload
        :param cur_extruder: extruder it is loaded in
        :param channel: its channel
        :param generation: connection generation of the unload
        :return Optional[str]: None on success, else why it failed
        """
        # BEFORE_PULL_BACK follows the tip-forming and extruder retracts, so the
        # pull is measured from before them.
        start_m = self._refresh_status()["meters"][channel]
        self._set_motion(channel, proto.MOTION_BEFORE_PULL_BACK)
        self.afc.do_tool_cut_tip_form(cur_lane, cur_extruder)
        if cur_extruder.tool_stn_unload > 0:
            self.afc.move_e_pos(cur_extruder.tool_stn_unload * -1,
                                cur_extruder.tool_unload_speed, "BMCU unload", wait_tool=True)
        self._same_session(generation)
        reason = self._retract_from_path(cur_lane, channel, start_m)
        if reason is None:
            reason = self._check_retracted(cur_lane, cur_extruder, channel)
        if reason is None:
            # The retract's polls pause the reactor, so a reconnect can land in them.
            self._same_session(generation)
            self._mark(channel, False)
        return reason

    def unit_unload_lane(self, cur_lane: AFCLane, cur_extruder: AFCExtruder) -> bool:
        """
        Unload a lane from the toolhead (AFC.unload_sequence's unit hook). A
        failed unload leaves AFC's tool loaded (see _run_sequence). A hub
        sensor still blocked after a clean unload is reported too, with the
        lane already unloaded.

        :param cur_lane: lane to unload
        :param cur_extruder: extruder it is loaded in
        :return bool: True when unloaded and the path is clear
        """
        channel = self._channels[cur_lane.name]
        pause = self.afc.function.in_print()
        if not self._connected():
            self.afc.error.handle_lane_failure(cur_lane, f"cannot unload: BMCU {self.name} "
                                               "is not connected.", pause=pause)
            return False

        cur_lane.status = AFCLaneState.TOOL_UNLOADING
        if not self._run_sequence(self._unload, "unload", cur_lane, cur_extruder, pause):
            return False

        if self.afc.post_unload_macro is not None:
            self.gcode.run_script_from_command(self.afc.post_unload_macro)
        self._runout_dispatched.discard(cur_lane.name)
        cur_lane.set_tool_unloaded(normal_toolchange=True)
        present = bool(self._status["present"][channel])
        cur_lane.loaded_to_hub = present
        if self._is_virtual_hub(cur_lane):
            cur_lane._load_state = False
        self.afc.save_vars()
        hub = self._real_hub(cur_lane)
        if (hub is not None
            and hub.state):
            self.afc.error.handle_lane_failure(cur_lane, "unloaded but the hub sensor still "
                                               "sees filament. Clear the path before loading "
                                               "another lane.", pause=pause)
            return False
        return True

    def lane_move(self, cur_lane: AFCLane, distance: float, speed_mode: SpeedMode) -> None:
        """
        Move a lane's filament with the firmware: FEED_DISTANCE forward, a tail
        retract backward. The firmware moves at least 5 mm.

        :param cur_lane: lane to move
        :param distance: mm, positive feeds, negative retracts
        :param speed_mode: unused, the firmware uses its configured speeds
        """
        channel = self._channels[cur_lane.name]
        refusal = None
        if not self._connected():
            refusal = f"BMCU {self.name} is not connected."
        elif abs(distance) < MIN_MOVE_MM:
            refusal = f"BMCU lanes move at least {MIN_MOVE_MM:g} mm, not {distance:g} mm."
        elif distance > MAX_FEED_MM:
            refusal = f"BMCU lanes feed at most {MAX_FEED_MM:g} mm in one move."
        elif -distance > MAX_RETRACT_MM:
            refusal = f"BMCU lanes retract at most {MAX_RETRACT_MM:g} mm in one move."
        elif (distance < 0
              and not self._conn.caps & proto.CAP_TAIL_RETRACT):
            refusal = "This BMCU firmware cannot retract a lane by distance."
        if refusal is not None:
            self.afc.error.AFC_error(f"{cur_lane.name}: {refusal}", pause=False)
            return

        if distance > 0:
            msg_type = proto.MSG_FEED_DISTANCE
            timeout = self._op_timeout(distance, self.load_speed)
            payload = proto.pack_feed(channel, distance, FEED_DISTANCE_PCT,
                                      self._timeout_ms(timeout))
        else:
            msg_type = proto.MSG_TAIL_RETRACT
            timeout = self._op_timeout(-distance, self.pull_speed)
            payload = proto.pack_tail_retract(channel, -distance, self._timeout_ms(timeout))
        try:
            with self._operation():
                result = self._run_op(msg_type, payload, timeout)
        except BMCUError as e:
            self._stop_after_failure()
            self.afc.error.AFC_error(f"{cur_lane.name} move failed: {e}", pause=False)
            return
        if not result["ok"]:
            self.afc.error.AFC_error(f"{cur_lane.name} move failed: {result['reason']}",
                                     pause=False)

    def get_lane_reset_command(self, lane: AFCLane, dis: float) -> str:
        """
        Gcode that retracts a lane for AFC_RESET.

        :param lane: lane to reset
        :param dis: mm to retract
        :return str: a LANE_MOVE with the negative distance
        """
        return f"LANE_MOVE LANE={lane.name} DISTANCE=-{dis}"

    def eject_lane(self, lane: AFCLane) -> None:
        """
        Pull a lane's filament back to the BMCU's inlet (PULL_BACK) when the
        inlet sees it, then mark the lane unloaded in the firmware.

        :param lane: lane to eject
        """
        channel = self._channels[lane.name]
        if not self._connected():
            self.logger.error(f"Cannot eject {lane.name}: BMCU {self.name} is not connected.")
            return
        try:
            with self._operation():
                reason = None
                if self._refresh_status()["present"][channel]:
                    reason = self._pull_back(channel)
                if reason is None:
                    self._mark(channel, False)
        except BMCUError as e:
            reason = str(e)
        if reason is not None:
            self._stop_after_failure()
            self.afc.error.AFC_error(f"{lane.name} eject failed: {reason}", pause=False)


def load_config_prefix(config: ConfigWrapper) -> afcBMCU:
    """
    Klipper config entry point for [AFC_BMCU <name>] sections.

    :param config: Klipper config for the section
    :return afcBMCU: the unit
    """
    return afcBMCU(config)
