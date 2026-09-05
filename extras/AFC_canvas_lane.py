from __future__ import annotations

import traceback
from typing import Any, Optional

import configfile

CONFIG_ERROR = getattr(configfile, "error", Exception)

try:
    from extras.AFC_utils import ERROR_STR
except Exception:
    raise CONFIG_ERROR(
        "Error when trying to import AFC_utils.ERROR_STR\n{}".format(
            traceback.format_exc()
        )
    )

try:
    from extras.AFC_lane import (
        AFCHomingPoints,
        AFCLane,
        AFCMoveWarning,
    )
except Exception:
    raise CONFIG_ERROR(
        ERROR_STR.format(import_lib="AFC_lane", trace=traceback.format_exc())
    )


class AFCCanvasLane(AFCLane):
    cmd_AFC_CANVAS_TOOL_LOAD_help = "CANVAS-specific tool load for a lane"
    cmd_AFC_CANVAS_TOOL_UNLOAD_help = "CANVAS-specific tool unload for a lane"
    DEFAULT_ODOMETER_POLL_INTERVAL = 0.05
    DEFAULT_ODOMETER_LOAD_THRESHOLD = 3
    DEFAULT_EXTRUDER_FEED_CHUNK = 1.0
    DEFAULT_LOAD_TO_TOOLHEAD_TIMEOUT = 60.0
    DEFAULT_EXTRUDER_FEED_TIMEOUT = 10.0
    DEFAULT_EJECT_EXTRA_DISTANCE = 200.0
    DEFAULT_EJECT_CLEAR_DISTANCE = 30.0
    DEFAULT_EJECT_STALL_TIMEOUT = 3.0
    DEFAULT_PREP_CHECK_DISTANCE = 10.0
    DEFAULT_LOAD_ATTEMPTS = 3
    DEFAULT_LOAD_RECOVERY_RETRACT_DISTANCE = 30.0
    DEFAULT_CUTTER_WAIT_TIME = 3.0
    GPIO_PIN_MIN_TIME = 2
    LED_PWM_CYCLE_TIME = 0.01

    def __init__(self, config):
        super().__init__(config)
        self.supports_lane_unload = True
        self.drv8833_object_name = config.get("drv8833", None)
        if self.drv8833_object_name is None:
            raise CONFIG_ERROR(
                "drv8833_object must be configured in [{}]".format(config.get_name())
            )

        self.canvas_motor = self.printer.lookup_object("drv8833 " + self.drv8833_object_name, None)
        if self.canvas_motor is None:
            raise CONFIG_ERROR(
                "Unable to find CANVAS motor object '{}' for lane {}".format(
                    self.drv8833_object_name, self.name
                )
            )

        pins = self.printer.lookup_object("pins")
        self.red_led_pin = self._setup_led_pin(pins, config.get("led_red_pin", None))
        self.white_led_pin = self._setup_led_pin(pins, config.get("led_white_pin", None))

        self.odometer_pin = config.get("odometer_pin")
        self.odometer_mm_per_pulse = config.getfloat("odometer_resolution", minval=0.0)
        self.odometer_poll_interval = config.getfloat(
            "odometer_poll_interval", self.DEFAULT_ODOMETER_POLL_INTERVAL, minval=0.01
        )
        self.odometer_load_threshold = config.getint(
            "odometer_load_threshold", self.DEFAULT_ODOMETER_LOAD_THRESHOLD, minval=0
        )
        self.load_to_toolhead_timeout = config.getfloat(
            "load_to_toolhead_timeout", self.DEFAULT_LOAD_TO_TOOLHEAD_TIMEOUT, minval=0.1
        )
        self.extruder_feed_timeout = config.getfloat(
            "extruder_feed_timeout", self.DEFAULT_EXTRUDER_FEED_TIMEOUT, minval=0.1
        )
        self.load_attempts = config.getint(
            "load_attempts", self.DEFAULT_LOAD_ATTEMPTS, minval=1
        )
        self.load_recovery_retract_distance = config.getfloat(
            "load_recovery_retract_distance",
            self.DEFAULT_LOAD_RECOVERY_RETRACT_DISTANCE,
            minval=0.0,
        )
        # Lane eject (LANE_UNLOAD): retract until the lane's entry sensor clears, then
        # a little further so the drive gear lets go of the filament. The spool
        # holder's spring rewinder takes up what it can of the returned filament.
        self.eject_speed = config.getfloat("eject_speed", None, above=0.0)
        self.eject_max_distance = config.getfloat("eject_max_distance", None, above=0.0)
        self.eject_clear_distance = config.getfloat(
            "eject_clear_distance", self.DEFAULT_EJECT_CLEAR_DISTANCE, minval=0.0
        )
        self.eject_stall_timeout = config.getfloat(
            "eject_stall_timeout", self.DEFAULT_EJECT_STALL_TIMEOUT, above=0.0
        )
        # Distance to move filament back and forth during PREP to verify it moves, 0 disables
        self.prep_check_distance = config.getfloat(
            "prep_check_distance", self.DEFAULT_PREP_CHECK_DISTANCE, minval=0.0
        )
        self.odometer_count = 0
        self.last_odometer_eventtime = None
        self._last_odometer_state = False
        if self.odometer_pin is not None:
            buttons = self.printer.load_object(config, "buttons")
            buttons.register_buttons([self.odometer_pin], self.odometer_callback)

        self.disengage_distance = config.getfloat("disengage_distance", 1.0)

        if self.custom_load_cmd is None:
            self.custom_load_cmd = "AFC_CANVAS_TOOL_LOAD LANE={}".format(self.name)
        if self.custom_unload_cmd is None:
            self.custom_unload_cmd = "AFC_CANVAS_TOOL_UNLOAD LANE={}".format(self.name)

        self.afc.gcode.register_mux_command(
            "AFC_CANVAS_TOOL_LOAD",
            "LANE",
            self.name,
            self.cmd_AFC_CANVAS_TOOL_LOAD,
            desc=self.cmd_AFC_CANVAS_TOOL_LOAD_help,
        )
        self.afc.gcode.register_mux_command(
            "AFC_CANVAS_TOOL_UNLOAD",
            "LANE",
            self.name,
            self.cmd_AFC_CANVAS_TOOL_UNLOAD,
            desc=self.cmd_AFC_CANVAS_TOOL_UNLOAD_help,
        )

        self._register_frontend_compat_aliases()
        self._load_state = False

    def _setup_led_pin(
        self,
        pins: Any,
        pin_name: Optional[str],
    ) -> Optional[Any]:
        """
        Configure a CANVAS LED pin as a PWM output.

        :param pins: Klipper pin manager
        :param pin_name: MCU pin name, or None when the LED is not configured
        :return Any: configured PWM pin, or None when no pin was supplied
        """
        if pin_name is None:
            return None
        pin = pins.setup_pin("pwm", pin_name)
        pin.setup_cycle_time(self.LED_PWM_CYCLE_TIME, False)
        pin.setup_start_value(0.0, 0.0)
        pin.setup_max_duration(0.0)
        pin.last_set_time = 0.0
        return pin

    def _set_gpio_pin(self, pin: Optional[Any], value: float) -> None:
        """
        Set a CANVAS LED PWM duty cycle.

        :param pin: configured PWM pin, or None when the LED is unavailable
        :param value: PWM duty cycle between zero and one
        """
        if pin is None:
            return

        mcu = pin.get_mcu()
        print_time = (
            mcu.estimated_print_time(self.reactor.monotonic())
            + mcu.min_schedule_time()
            + 0.1
        )
        print_time = max(print_time, pin.last_set_time + 0.2)
        pin.last_set_time = print_time
        pin.set_pwm(print_time, value)

    def _register_frontend_compat_aliases(self) -> None:
        for alias in (f"AFC_lane {self.name}", f"AFC_stepper {self.name}"):
            if alias == self.fullname:
                continue

            existing = self.printer.lookup_object(alias, None)
            if existing not in (None, self):
                continue

            add_object = getattr(self.printer, "add_object", None)
            if callable(add_object):
                try:
                    add_object(alias, self)
                    continue
                except Exception:
                    pass

            for registry_name in ("_objects", "objects"):
                registry = getattr(self.printer, registry_name, None)
                if isinstance(registry, dict):
                    registry.setdefault(alias, self)
                    break

    def move(self, distance, speed, accel, assist_active=False, wait_for_completion=True):
        self.unit_obj.select_lane(self)
        if distance == 0:
            return
        self.canvas_motor.drv8833_move(
            speed, distance, wait_for_completion=wait_for_completion
        )

    def do_enable(self, enable):
        if not enable:
            self.canvas_motor.drv8833_set_speed(0.0)

    def set_extruder_assist(self, extruder_speed, speed_offset):
        direction = 1.0 if extruder_speed >= 0 else -1.0
        assist_speed = max(abs(extruder_speed) - max(speed_offset, 0.0), 0.0)
        self.canvas_motor.drv8833_set_speed(direction * assist_speed)

    def _canvas_endstop_state(self, endstop):
        if endstop in (
            AFCHomingPoints.HUB,
            AFCHomingPoints.TOOL,
            AFCHomingPoints.TOOL_START,
            self.hub_endstop_name,
            self.tool_endstop_name,
        ):
            return bool(self.get_toolhead_pre_sensor_state())
        if endstop in (AFCHomingPoints.LOAD, self.load_es):
            return bool(self.raw_load_state)
        if endstop == AFCHomingPoints.BUFFER:
            return bool(getattr(self.buffer_obj, "advance_state", False))
        if endstop == AFCHomingPoints.BUFFER_TRAIL:
            return bool(getattr(self.buffer_obj, "trailing_state", False))
        return False

    def _canvas_move_until(self, endstop, distance, speed):
        desired_state = True if distance >= 0 else False
        remaining = abs(distance)
        moved = 0.0
        chunk = max(getattr(self, "short_move_dis", 1.0), 1.0)
        if self._canvas_endstop_state(endstop) == desired_state:
            return True, 0.0, AFCMoveWarning.NONE

        while remaining > 0:
            step = min(chunk, remaining)
            self.move(step if distance >= 0 else -step, speed, self.short_moves_accel, False)
            moved += step
            remaining -= step
            if self._canvas_endstop_state(endstop) == desired_state:
                return True, moved, AFCMoveWarning.NONE

        return False, moved, AFCMoveWarning.WARN

    def move_to(
        self,
        distance,
        speed_mode,
        endstop=AFCHomingPoints.NONE,
        assist_active=None,
        use_homing=True,
    ):
        if not use_homing:
            self.move_advanced(distance, speed_mode)
            return True, 0, AFCMoveWarning.NONE

        speed_data = self.get_speed_accel(speed_mode)
        speed = speed_data[0] if isinstance(speed_data, tuple) else speed_data
        return self._canvas_move_until(endstop, distance, speed)

    def apply_canvas_led(self, color_string: str) -> None:
        """
        Apply the red and white color channels as PWM duty cycles.

        :param color_string: comma-separated RGBW LED color values
        """
        channels = [item.strip() for item in str(color_string).split(",")]
        while len(channels) < 4:
            channels.append("0")
        red_value = max(0.0, min(float(channels[0]), 1.0))
        white_value = max(0.0, min(float(channels[3]), 1.0))
        self._set_gpio_pin(self.red_led_pin, red_value)
        self._set_gpio_pin(self.white_led_pin, white_value)

    def disengage_motors(self, direction):
        if self.disengage_distance <= 0:
            self.do_enable(False)
            return
        self.reactor.pause(self.reactor.monotonic() + 0.5)
        release_distance = -abs(self.disengage_distance) if direction >= 0 else abs(self.disengage_distance)
        self.move(release_distance, 30.0, self.short_moves_accel, False)
        self.do_enable(False)

    def odometer_callback(self, eventtime, state):
        state = bool(state)
        if state and not self._last_odometer_state:
            self.odometer_count += 1
            self.last_odometer_eventtime = eventtime
        self._last_odometer_state = state

    def reset_odometer(self):
        self.odometer_count = 0
        self.last_odometer_eventtime = None
        self._last_odometer_state = False

    def get_odometer_distance(self):
        if self.odometer_mm_per_pulse is None:
            return 0.0
        return self.odometer_count * self.odometer_mm_per_pulse

    def move_with_odometer(self, distance, speed, stop_condition=None):
        target_distance = abs(distance)
        if target_distance == 0:
            return 0.0

        signed_speed = abs(speed) if distance >= 0 else -abs(speed)
        moved = 0.0
        timeout = (target_distance / max(abs(speed), 1.0)) * 5.0 + 1.0
        poll_interval = max(
            getattr(
                self,
                "odometer_poll_interval",
                self.DEFAULT_ODOMETER_POLL_INTERVAL,
            ),
            0.01,
        )
        deadline = self.reactor.monotonic() + timeout

        self.reset_odometer()
        self.unit_obj.select_lane(self)

        try:
            self.canvas_motor.drv8833_set_speed(signed_speed)
            while moved < target_distance:
                if stop_condition is not None and stop_condition():
                    break
                moved = self.get_odometer_distance()
                if moved >= target_distance:
                    break
                now = self.reactor.monotonic()
                if now >= deadline:
                    raise TimeoutError(
                        "Timed out waiting for odometer movement on {}".format(
                            self.name
                        )
                    )
                self.reactor.pause(now + poll_interval)
                moved = self.get_odometer_distance()
        finally:
            self.canvas_motor.drv8833_set_speed(0.0)

        return min(moved, target_distance)

    def retract_until_prep_clear(self, speed, max_distance, stall_timeout):
        """
        Retract filament until the lane's entry (prep) sensor no longer sees it.

        The odometer only guards the move: it stops the motor when the filament
        stops moving for ``stall_timeout`` seconds, or after ``max_distance``
        without the sensor clearing. Once the filament tip has passed the
        odometer wheel the count stops, so the stall timeout has to be longer
        than the time it takes to cover the wheel-to-sensor distance.

        :param speed: Retract speed in mm/s
        :param max_distance: Give up after this much odometer travel
        :param stall_timeout: Give up when the odometer stops counting for this long
        :return tuple: (sensor cleared, odometer distance moved)
        """
        poll_interval = max(self.odometer_poll_interval, 0.01)
        self.reset_odometer()
        self.unit_obj.select_lane(self)
        moved = 0.0
        last_count = 0
        now = self.reactor.monotonic()
        last_progress = now
        try:
            self.canvas_motor.drv8833_set_speed(-abs(speed))
            while bool(self.prep_state):
                now = self.reactor.pause(now + poll_interval)
                moved = self.get_odometer_distance()
                if self.odometer_count != last_count:
                    last_count = self.odometer_count
                    last_progress = now
                elif now - last_progress >= stall_timeout:
                    return False, moved
                if moved >= max_distance:
                    return False, moved
        finally:
            self.canvas_motor.drv8833_set_speed(0.0)
        return True, moved

    def _run_cutter_macro(self) -> None:
        """
        Run the configured filament cutter macro.
        """
        if self.afc.tool_cut:
            self.extruder_obj.estats.increase_cut_total()
            self.afc.gcode.run_script_from_command(
                f"{self.afc.tool_cut_cmd} EXTRUDER={self.extruder_obj.name}"
            )

    def _run_post_cut_macros(self) -> None:
        """
        Run the configured parking and tip-forming macros once.
        """
        if self.afc.tool_cut:
            if self.afc.park:
                self.afc.gcode.run_script_from_command(
                    f"{self.afc.park_cmd} EXTRUDER={self.extruder_obj.name}"
                )

        if self.afc.form_tip:
            if self.afc.park:
                self.afc.gcode.run_script_from_command(
                    f"{self.afc.park_cmd} EXTRUDER={self.extruder_obj.name}"
                )
            if self.afc.form_tip_cmd == "AFC":
                self.printer.lookup_object("AFC_form_tip").tip_form()
            else:
                self.afc.gcode.run_script_from_command(self.afc.form_tip_cmd)

    def _assist_extruder_move(self, distance, speed, speed_offset, label):
        try:
            self.set_extruder_assist(speed if distance >= 0 else -speed, speed_offset)
            self.afc.move_e_pos(distance, speed, label, wait_tool=True)
        finally:
            self.canvas_motor.drv8833_set_speed(0.0)

    def _move_canvas_with_extruder_feed(
        self, distance, canvas_speed, extruder_speed, label
    ):
        if distance == 0:
            return

        feed_direction = 1.0 if distance >= 0 else -1.0
        feed_chunk = self.DEFAULT_EXTRUDER_FEED_CHUNK * feed_direction
        deadline = self.reactor.monotonic() + self.extruder_feed_timeout

        try:
            self.move(
                distance,
                canvas_speed,
                self.short_moves_accel,
                wait_for_completion=False,
            )
            while self.canvas_motor.active:
                if self.reactor.monotonic() >= deadline:
                    raise TimeoutError(
                        "CANVAS motor did not complete the assisted extruder move within "
                        f"{self.extruder_feed_timeout:g} seconds"
                    )
                self.afc.move_e_pos(feed_chunk, extruder_speed, label, wait_tool=True)
        finally:
            self.canvas_motor.drv8833_set_speed(0.0)

    def _cutter_sensor_engaged(self):
        return bool(getattr(self.unit_obj, "cutter_sensor_state", False))

    def cmd_AFC_CANVAS_TOOL_LOAD(self, gcmd):
        self.select_lane()
        self.afc._check_extruder_temp(self)

        if self._cutter_sensor_engaged():
            self.afc.error.handle_lane_failure(self, "CANVAS tool load failed: cutter sensor is engaged.")
            return

        if self.afc.park:
            self.afc.gcode.run_script_from_command(
                "{} EXTRUDER={}".format(self.afc.park_cmd, self.extruder_obj.name)
            )

        for load_attempt in range(self.load_attempts):
            self.logger.info(f"Attempting CANVAS tool load for {self.name}, try {load_attempt+1}/{self.load_attempts}")
            if self.get_toolhead_pre_sensor_state():
                return
            
            start = self.reactor.monotonic()
            self.canvas_motor.drv8833_set_speed(self.short_moves_speed)
            toolhead_sensor_load_fail = False
            while not self.get_toolhead_pre_sensor_state():
                now = self.reactor.monotonic()
                if now - start > self.load_to_toolhead_timeout:
                    self.canvas_motor.drv8833_set_speed(0.0)
                    self.logger.warning(f"CANVAS tool load timed out after {self.load_to_toolhead_timeout:g}s while moving filament to the extruder (load_to_toolhead_timeout). Attempting recovery.")
                    toolhead_sensor_load_fail = True

                    try:
                        self.move_with_odometer(
                            -self.load_recovery_retract_distance,
                            self.long_moves_speed,
                        )
                    except:
                        self.logger.warning("CANVAS tool load recovery failed while retracting filament to the hub using the odometer.")

                    break
                    
                self.reactor.pause(now + 0.005)

            if toolhead_sensor_load_fail:
                continue
            
            self.canvas_motor.drv8833_set_speed(0.0)
            self.loaded_to_hub = True

            if self.hub_obj and self.hub_obj.afc_bowden_length > 0:
                try:
                    self._move_canvas_with_extruder_feed(
                        self.hub_obj.afc_bowden_length + load_attempt,
                        self.short_moves_speed,
                        self.extruder_obj.tool_load_speed,
                        "CANVAS tool load extra move",
                    )
                except TimeoutError:
                    self.logger.warning(f"CANVAS tool load timed out after {self.extruder_feed_timeout:g}s while moving filament from the hub to the extruder (extruder_feed_timeout). Attempting recovery.")
                    self.disengage_motors(1.0)
                    self.afc.move_e_pos(-(self.hub_obj.afc_bowden_length + load_attempt), self.extruder_obj.tool_load_speed, "CANVAS tool load recovery retract", wait_tool=True)

                    try:
                        self.move_with_odometer(
                            -self.load_recovery_retract_distance,
                            self.long_moves_speed,
                        )
                    except:
                        self.logger.warning("CANVAS tool load recovery failed while retracting filament to the hub using the odometer.")

                    continue

            self.reset_odometer()

            if self.extruder_obj.tool_stn > 0:
                self.afc.move_e_pos(self.extruder_obj.tool_stn, self.extruder_obj.tool_load_speed, "CANVAS tool load", wait_tool=True)

            if self.odometer_count <= self.odometer_load_threshold:
                self.logger.warning(f"Odometer count after load is {self.odometer_count}, which may mean the filament was not grabbed by the extruder. Attempting to unload.")
                self.disengage_motors(1.0) # The motors would be locked at this point, so disengage them before trying to unload
                if self.extruder_obj.tool_stn_unload > 0:
                    self.afc.move_e_pos(-self.extruder_obj.tool_stn_unload, self.extruder_obj.tool_unload_speed, "CANVAS tool unload", wait_tool=True)

                if self.hub_obj and self.hub_obj.afc_unload_bowden_length > 0:
                    try:
                        self.move_with_odometer(
                            -self.hub_obj.afc_unload_bowden_length,
                            self.long_moves_speed,
                        )
                    except TimeoutError:
                        self.logger.warning("CANVAS tool load recovery failed while retracting filament to the hub using the odometer.")

                self.disengage_motors(-1.0)
                continue
            
            self.disengage_motors(1.0)
            return # Success

        # Failure case
        self.afc.error.handle_lane_failure(self, f"CANVAS tool load failed after {self.load_attempts} attempts.")

    def run_cutter_sequence(self) -> bool:
        """
        Retry the cutter macro until the cutter sensor disengages.

        :return bool: True when cutting is disabled or the sensor disengages
        """
        if not self.afc.tool_cut:
            return True

        for attempt in range(self.load_attempts):
            attempt_number = attempt + 1
            self.logger.info(
                f"Attempting cutting filament, try {attempt_number}/{self.load_attempts}"
            )
            self._run_cutter_macro()
            start = self.reactor.monotonic()
            timed_out = False

            while self._cutter_sensor_engaged():
                now = self.reactor.monotonic()
                if now - start >= self.DEFAULT_CUTTER_WAIT_TIME:
                    self.logger.warning(
                        "Cutter sensor did not disengage after cutting filament. Trying again."
                    )
                    timed_out = True
                    break
                self.reactor.pause(now + 0.005)

            if timed_out:
                continue

            return True

        return False

    def cmd_AFC_CANVAS_TOOL_UNLOAD(self, gcmd: Any) -> None:
        """
        Unload filament from a CANVAS lane.

        Usage
        -------
        `AFC_CANVAS_TOOL_UNLOAD LANE=<lane>`

        Example
        -------
        ```
        AFC_CANVAS_TOOL_UNLOAD LANE=lane1
        ```
        """
        self.select_lane()
        self.afc._check_extruder_temp(self)
        self.disable_buffer()
        self.unit_obj.lane_unloading(self)

        if self._cutter_sensor_engaged():
            error_message = (
                "CANVAS tool unload failed: cutter sensor is engaged before cutting."
            )
            self.afc.error.handle_lane_failure(self, error_message)
            return

        if not self.run_cutter_sequence():
            error_message = (
                "CANVAS tool unload failed: cutter sensor did not disengage after cutting."
            )
            self.afc.error.handle_lane_failure(self, error_message)
            return

        self._run_post_cut_macros()

        for attempt in range(self.load_attempts):
            self.logger.info(f"Attempting CANVAS tool unload for {self.name}, try {attempt+1}/{self.load_attempts}")
            failed_operation = "toolhead gear retract"
            try:
                if self.extruder_obj.tool_stn_unload > 0:
                    self.afc.move_e_pos(
                        -self.extruder_obj.tool_stn_unload,
                        self.extruder_obj.tool_unload_speed,
                        "CANVAS tool unload",
                        wait_tool=True,
                    )

                failed_operation = "Canvas-unit retract"
                if self.hub_obj and self.hub_obj.afc_unload_bowden_length > 0:
                    self.move_with_odometer(
                        -self.hub_obj.afc_unload_bowden_length,
                        self.long_moves_speed,
                    )

                failed_operation = "check toolhead sensor"
                if self.get_toolhead_pre_sensor_state():
                    raise TimeoutError(
                        "CANVAS tool unload failed to clear the shared toolhead sensor after extruder retract"
                    )

                self.loaded_to_hub = False
                self.disengage_motors(-1.0)
                return  # Success
            except TimeoutError as error:
                self.logger.warning(f"CANVAS tool unload failed during {failed_operation}: {error}.")

        # Failure case
        self.afc.error.handle_lane_failure(self, f"CANVAS tool unload failed after {self.load_attempts} attempts.")

    def get_status(self, eventtime=None, save_to_file=False):
        response = super().get_status(eventtime=eventtime, save_to_file=save_to_file)
        if not response:
            return response
        response["canvas_motor"] = self.drv8833_object_name
        response["odometer_count"] = self.odometer_count
        response["odometer_distance"] = self.get_odometer_distance()
        return response

def load_config_prefix(config):
    return AFCCanvasLane(config)
