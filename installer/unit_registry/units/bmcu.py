# AFCProject Automated Filament Changer
#
# Copyright (C) 2026 AFCProject
#
# This file may be distributed under the terms of the GNU GPLv3 license.
"""BMCU."""
from __future__ import annotations

from typing import List

from installer.config_editor.edits import read_tool_start_pin, replace_text
from installer.platform_core.console import Console, Prompter
from installer.platform_core.context import Context
from installer.platform_core.fsops import safe_copy
from installer.unit_registry.base import UnitType
from installer.unit_registry.options import NameOption, UnitOption
from installer.unit_registry.selection import UnitSelection


class Bmcu(UnitType):
    """BMCU on BMCU-Klipper firmware. Its lanes use no AFC buffer, so it has no buffer target."""

    display_name = "BMCU"
    key = "BMCU"
    buffer_selectable = False
    additional_name_prefix = "BMCU"
    additional_buffer_supported = False
    ramming_hardware = "the BMCU's own buffer"
    # The BMCU judges a sensorless load by its own buffer and encoder; AFC's "buffer"
    # pin needs an [AFC_buffer] the BMCU does not have.
    ramming_pin = None

    def copy_files(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> None:
        """
        Copy the hardware file and the fixed-name BMCU unit file.

        :param ctx: installer context
        :param console: where messages go
        :param prompter: used when a destination file already exists
        :param sel: the user's unit choices
        """
        templates = ctx.afc_path / "templates"
        cfg = ctx.afc_config_dir
        hardware = templates / "AFC_Hardware-AFC.cfg"
        safe_copy(ctx, console, prompter, hardware, cfg / "AFC_Hardware.cfg")
        safe_copy(ctx, console, prompter, templates / "AFC_BMCU_1.cfg", cfg / "AFC_BMCU_1.cfg")

    def install_additional(
        self, ctx: Context, console: Console, prompter: Prompter, sel: UnitSelection
    ) -> None:
        """
        Copy the BMCU unit file under its new name, renaming BMCU_1 and the lanes
        (lane names are global in AFC, so a second unit cannot reuse bmcu1..bmcu4).

        :param ctx: installer context
        :param console: where messages go
        :param prompter: used when a destination file already exists
        :param sel: the user's unit choices
        """
        name = sel.unit_name
        unit_file = ctx.afc_config_dir / f"AFC_{name}.cfg"
        template = ctx.afc_path / "templates" / "AFC_BMCU_1.cfg"
        safe_copy(ctx, console, prompter, template, unit_file)
        replace_text(unit_file, "BMCU_1", name)
        replace_text(unit_file, "[AFC_lane bmcu", f"[AFC_lane {name}_lane")
        hardware = ctx.afc_config_dir / "AFC_Hardware.cfg"
        if (hardware.exists()
                and read_tool_start_pin(hardware) == "buffer"):
            console.print_msg(
                "WARNING",
                f"[AFC_extruder extruder] uses pin_tool_start: buffer, which needs an AFC "
                f"buffer the BMCU does not have, so Klipper will not start with {name} on that "
                f"extruder. Set 'extruder:' in AFC_{name}.cfg to an extruder with a toolhead "
                "sensor or with no pin_tool_start.",
            )

    def message(self, ctx: Context, sel: UnitSelection) -> str:
        """
        Point the user at the serial path and the per-lane calibration.

        :param ctx: installer context
        :param sel: the user's unit choices
        :return str: the message
        """
        unit_file = f"{ctx.afc_config_dir}/AFC_BMCU_1.cfg"
        return (
            f"\n- Enter your BMCU's serial path in the {unit_file} file, and set "
            "half_duplex: True for RS485 wiring.\n\n"
            "- The BMCU must run BMCU-Klipper firmware. Calibrate each lane once with "
            "AFC_BMCU_CALIBRATE.\n  "
        )

    def additional_options(self) -> List[UnitOption]:
        """
        The name option of the additional-unit screen.

        :return List[UnitOption]: the options
        """
        return [NameOption("BMCU Name", additional_prompt=True)]

    def on_select(self, sel: UnitSelection) -> None:
        """
        BMCU lanes use no AFC buffer.

        :param sel: the user's unit choices, ``buffer_type`` is cleared
        """
        sel.buffer_type = "None"

    def after_sensor_change(self, sel: UnitSelection) -> str:
        """
        Keep the buffer at None whatever the sensor choice. Ramming uses the BMCU's
        own buffer: the load ends at the firmware's contact report.

        :param sel: the user's unit choices, ``buffer_type`` is set to None
        :return str: the status message
        """
        sel.buffer_type = "None"
        if sel.toolhead_sensor == "Sensor":
            return "Using toolhead sensor"
        return f"Using ramming with {self.ramming_hardware}"
