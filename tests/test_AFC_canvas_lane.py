from __future__ import annotations

from unittest.mock import MagicMock, call

import pytest

from extras.AFC_canvas_lane import AFCCanvasLane
from extras.AFC_lane import AFCHomingPoints, AFCLane, AFCMoveWarning, SpeedMode
from tests.conftest import MockAFC, MockConfig, MockLogger, MockPrinter, MockReactor


def _make_canvas_lane(name="lane1"):
    lane = AFCCanvasLane.__new__(AFCCanvasLane)
    afc = MockAFC()
    afc.tool_cut = True
    afc.park = True
    afc.form_tip = False
    afc.tool_cut_cmd = "AFC_CUT"
    afc.park_cmd = "AFC_PARK"
    afc.form_tip_cmd = "AFC"
    afc.tool_max_unload_attempts = 4
    afc._check_extruder_temp = MagicMock(return_value=False)
    afc.move_e_pos = MagicMock()
    afc.error.handle_lane_failure = MagicMock()
    reactor = MockReactor()
    printer = MockPrinter(afc=afc)

    lane.printer = printer
    lane.afc = afc
    lane.logger = MockLogger()
    lane.reactor = reactor
    lane.name = name
    lane.unit_obj = MagicMock()
    lane.unit_obj.select_lane = MagicMock()
    lane.unit_obj.move_to_hub = MagicMock(return_value=(True, 10.0, AFCMoveWarning.NONE))
    lane.unit_obj.lane_unloading = MagicMock()
    lane.unit_obj.cutter_sensor_state = False
    lane.hub_obj = None
    lane.canvas_motor = MagicMock()
    lane.red_led_pin = MagicMock()
    lane.white_led_pin = MagicMock()
    lane.short_move_dis = 5.0
    lane.short_moves_speed = 20.0
    lane.short_moves_accel = 400.0
    lane.long_moves_speed = 80.0
    lane.long_moves_accel = 400.0
    lane.tool_load_sync_speed_offset = 4.0
    lane.tool_unload_sync_speed_offset = 6.0
    lane.tool_unload_lane_extra_distance = 7.0
    lane.tool_unload_lane_extra_speed = 18.0
    lane.disengage_distance = 2.0
    lane.dist_hub = 100.0
    lane.loaded_to_hub = False
    lane._load_state = True
    lane.buffer_obj = MagicMock()
    lane.endstops = {}
    lane.only_lane = True
    lane.extruder_obj = MagicMock()
    lane.extruder_obj.name = "extruder"
    lane.extruder_obj.tool_load_speed = 25.0
    lane.extruder_obj.tool_unload_speed = 20.0
    lane.extruder_obj.tool_stn = 72.0
    lane.extruder_obj.tool_stn_unload = 40.0
    lane.extruder_obj.tool_sensor_after_extruder = 12.0
    lane.extruder_obj.tool_end = None
    lane.extruder_obj.tool_end_state = False
    lane.extruder_obj.estats = MagicMock()
    lane.get_speed_accel = MagicMock(side_effect=lambda mode: (30.0, 300.0))
    lane.move_advanced = MagicMock()
    lane.disable_buffer = MagicMock()
    lane.select_lane = MagicMock()
    lane.do_enable = AFCCanvasLane.do_enable.__get__(lane, AFCCanvasLane)
    lane._last_odometer_state = False
    lane.odometer_count = 0
    lane.last_odometer_eventtime = None
    lane.odometer_poll_interval = AFCCanvasLane.DEFAULT_ODOMETER_POLL_INTERVAL
    lane.odometer_load_threshold = AFCCanvasLane.DEFAULT_ODOMETER_LOAD_THRESHOLD
    lane.load_to_toolhead_timeout = AFCCanvasLane.DEFAULT_LOAD_TO_TOOLHEAD_TIMEOUT
    lane.extruder_feed_timeout = AFCCanvasLane.DEFAULT_EXTRUDER_FEED_TIMEOUT
    lane.load_attempts = AFCCanvasLane.DEFAULT_LOAD_ATTEMPTS
    lane.load_recovery_retract_distance = AFCCanvasLane.DEFAULT_LOAD_RECOVERY_RETRACT_DISTANCE
    lane.odometer_mm_per_pulse = 0.5
    lane.drv8833_object_name = "drv8833 lane1"
    lane.connect_done = True
    lane.unit = "CANVAS_1"
    lane.hub = "hub"
    lane.extruder_name = "extruder"
    lane.buffer_name = None
    lane.index = 1
    lane.map = "T0"
    lane.prep_state = True
    lane.tool_loaded = False
    lane._material = None
    lane.remember_spool = False
    lane.spool_id = None
    lane.color = None
    lane.weight = 0
    lane.extruder_temp = 0
    lane.bed_temp = 0
    lane.runout_lane = None
    lane.status = "Loaded"
    lane.td1_data = {}
    lane._selector_state = None
    lane.get_toolhead_pre_sensor_state = MagicMock(return_value=False)
    lane.buffer_status = MagicMock(return_value=None)
    lane._set_gpio_pin = AFCCanvasLane._set_gpio_pin.__get__(lane, AFCCanvasLane)
    return lane


def _make_configured_canvas_lane(monkeypatch, **config_values):
    afc = MockAFC()
    printer = MockPrinter(afc=afc)
    printer._objects["drv8833 motor"] = MagicMock()
    pins = MagicMock()
    pins.setup_pin.side_effect = [MagicMock(), MagicMock()]
    printer._objects["pins"] = pins

    def mock_lane_init(lane, config):
        lane.printer = printer
        lane.afc = afc
        lane.logger = afc.logger
        lane.reactor = printer.get_reactor()
        lane.name = "lane1"
        lane.fullname = "AFC_canvas_lane lane1"
        lane.custom_load_cmd = None
        lane.custom_unload_cmd = None

    monkeypatch.setattr(AFCLane, "__init__", mock_lane_init)
    values = {
        "drv8833": "motor",
        "odometer_pin": "PA0",
        "odometer_resolution": 0.5,
    }
    values.update(config_values)
    config = MockConfig(name="AFC_canvas_lane lane1", printer=printer, values=values)
    return AFCCanvasLane(config)


def _make_initialized_canvas_lane(
    monkeypatch: pytest.MonkeyPatch,
) -> AFCCanvasLane:
    lane = _make_configured_canvas_lane(monkeypatch)
    lane.afc.tool_cut = True
    lane.afc.park = True
    lane.afc.form_tip = False
    lane.afc.tool_cut_cmd = "AFC_CUT"
    lane.afc.park_cmd = "AFC_PARK"
    lane.afc.form_tip_cmd = "AFC"
    lane.afc._check_extruder_temp = MagicMock(return_value=False)
    lane.afc.move_e_pos = MagicMock()
    lane.unit_obj = MagicMock()
    lane.unit_obj.cutter_sensor_state = False
    lane.unit_obj.lane_unloading = MagicMock()
    lane.extruder_obj = MagicMock()
    lane.extruder_obj.name = "extruder"
    lane.extruder_obj.tool_stn_unload = 40.0
    lane.extruder_obj.tool_unload_speed = 20.0
    lane.hub_obj = None
    lane.loaded_to_hub = True
    lane.select_lane = MagicMock()
    lane.disable_buffer = MagicMock()
    lane.disengage_motors = MagicMock()
    lane.get_toolhead_pre_sensor_state = MagicMock(return_value=False)
    return lane


def test_odometer_load_threshold_defaults_to_three(monkeypatch):
    lane = _make_configured_canvas_lane(monkeypatch)

    assert lane.odometer_load_threshold == 3


def test_odometer_load_threshold_is_configurable(monkeypatch):
    lane = _make_configured_canvas_lane(monkeypatch, odometer_load_threshold=7)

    assert lane.odometer_load_threshold == 7


def test_canvas_load_timeouts_have_expected_defaults(monkeypatch):
    lane = _make_configured_canvas_lane(monkeypatch)

    assert lane.load_to_toolhead_timeout == 30.0
    assert lane.extruder_feed_timeout == 10.0


def test_canvas_load_timeouts_are_configurable(monkeypatch):
    lane = _make_configured_canvas_lane(
        monkeypatch,
        load_to_toolhead_timeout=45.0,
        extruder_feed_timeout=15.0,
    )

    assert lane.load_to_toolhead_timeout == 45.0
    assert lane.extruder_feed_timeout == 15.0


def test_canvas_load_recovery_defaults(monkeypatch):
    lane = _make_configured_canvas_lane(monkeypatch)

    assert lane.load_attempts == 3
    assert lane.load_recovery_retract_distance == 30.0


def test_canvas_load_recovery_is_configurable(monkeypatch):
    lane = _make_configured_canvas_lane(
        monkeypatch,
        load_attempts=5,
        load_recovery_retract_distance=18.5,
    )

    assert lane.load_attempts == 5
    assert lane.load_recovery_retract_distance == 18.5


def test_move_translates_signed_direction():
    lane = _make_canvas_lane()

    AFCCanvasLane.move(lane, -12.0, 30.0, 400.0)

    lane.unit_obj.select_lane.assert_called_once_with(lane)
    lane.canvas_motor.drv8833_move.assert_called_once_with(
        30.0, -12.0, wait_for_completion=True
    )


def test_canvas_feed_keeps_extruder_moving_until_canvas_completes():
    lane = _make_canvas_lane()
    lane.canvas_motor.active = True

    def move_e_side_effect(*args, **kwargs):
        if lane.afc.move_e_pos.call_count >= 3:
            lane.canvas_motor.active = False

    lane.afc.move_e_pos.side_effect = move_e_side_effect

    AFCCanvasLane._move_canvas_with_extruder_feed(
        lane, 12.0, 20.0, 25.0, "CANVAS tool load extra move"
    )

    lane.canvas_motor.drv8833_move.assert_called_once_with(
        20.0, 12.0, wait_for_completion=False
    )
    assert lane.afc.move_e_pos.call_args_list == [
        call(1.0, 25.0, "CANVAS tool load extra move", wait_tool=True),
        call(1.0, 25.0, "CANVAS tool load extra move", wait_tool=True),
        call(1.0, 25.0, "CANVAS tool load extra move", wait_tool=True),
    ]
    lane.canvas_motor.drv8833_set_speed.assert_called_once_with(0.0)


def test_do_enable_false_stops_motor():
    lane = _make_canvas_lane()

    lane.do_enable(False)

    lane.canvas_motor.drv8833_set_speed.assert_called_once_with(0.0)


def test_set_extruder_assist_applies_signed_offset():
    lane = _make_canvas_lane()

    AFCCanvasLane.set_extruder_assist(lane, -25.0, 5.0)

    lane.canvas_motor.drv8833_set_speed.assert_called_once_with(-20.0)


def test_move_to_without_homing_returns_success():
    lane = _make_canvas_lane()

    success, moved, warn = AFCCanvasLane.move_to(lane, 30.0, SpeedMode.LONG, use_homing=False)

    assert success is True
    assert moved == 0
    assert warn == AFCMoveWarning.NONE
    lane.move_advanced.assert_called_once_with(30.0, SpeedMode.LONG)


def test_move_to_with_homing_stops_on_sensor():
    lane = _make_canvas_lane()
    lane.get_toolhead_pre_sensor_state = MagicMock(side_effect=[False, True])
    lane.move = MagicMock()

    success, moved, warn = AFCCanvasLane.move_to(
        lane, 20.0, SpeedMode.SHORT, endstop=AFCHomingPoints.TOOL, use_homing=True
    )

    assert success is True
    assert moved == 5.0
    assert warn == AFCMoveWarning.NONE


def test_odometer_counts_active_edges_only():
    lane = _make_canvas_lane()

    AFCCanvasLane.odometer_callback(lane, 1.0, True)
    AFCCanvasLane.odometer_callback(lane, 2.0, True)
    AFCCanvasLane.odometer_callback(lane, 3.0, False)
    AFCCanvasLane.odometer_callback(lane, 4.0, True)

    assert lane.odometer_count == 2
    assert lane.get_odometer_distance() == 1.0


def test_move_with_odometer_uses_set_speed_and_stops_at_target():
    lane = _make_canvas_lane()
    lane.reset_odometer = MagicMock()
    lane.unit_obj.select_lane = MagicMock()
    lane.get_odometer_distance = MagicMock(side_effect=[0.0, 0.5, 1.0, 1.5, 2.0])
    lane.reactor.monotonic = MagicMock(side_effect=[0.0, 0.0, 0.01, 0.01, 0.02, 0.02, 0.03])
    lane.reactor.pause = MagicMock()

    moved = AFCCanvasLane.move_with_odometer(lane, 2.0, 25.0)

    assert moved == 2.0
    lane.reset_odometer.assert_called_once_with()
    lane.unit_obj.select_lane.assert_called_once_with(lane)
    assert lane.canvas_motor.drv8833_set_speed.call_args_list[0] == call(25.0)
    assert lane.canvas_motor.drv8833_set_speed.call_args_list[-1] == call(0.0)


def test_move_with_odometer_stops_early_when_condition_is_met():
    lane = _make_canvas_lane()
    lane.reset_odometer = MagicMock()
    lane.unit_obj.select_lane = MagicMock()
    lane.get_odometer_distance = MagicMock(return_value=0.5)
    lane.reactor.monotonic = MagicMock(return_value=0.0)
    lane.reactor.pause = MagicMock()
    stop_condition = MagicMock(side_effect=[False, True])

    moved = AFCCanvasLane.move_with_odometer(
        lane, 3.0, 20.0, stop_condition=stop_condition
    )

    assert moved == 0.5
    assert lane.canvas_motor.drv8833_set_speed.call_args_list[0] == call(20.0)
    assert lane.canvas_motor.drv8833_set_speed.call_args_list[-1] == call(0.0)


class TestAFCCanvasLaneSetupLedPin:
    def test_missing_pin_is_not_configured(self, monkeypatch):
        lane = _make_configured_canvas_lane(monkeypatch)
        pins = lane.printer.lookup_object("pins")

        assert lane.red_led_pin is None
        assert lane.white_led_pin is None
        pins.setup_pin.assert_not_called()

    def test_pin_is_configured_for_software_pwm(self, monkeypatch):
        lane = _make_configured_canvas_lane(monkeypatch, led_red_pin="PA1")
        pins = lane.printer.lookup_object("pins")

        pins.setup_pin.assert_called_once_with("pwm", "PA1")
        lane.red_led_pin.setup_cycle_time.assert_called_once_with(0.01, False)
        lane.red_led_pin.setup_start_value.assert_called_once_with(0.0, 0.0)
        lane.red_led_pin.setup_max_duration.assert_called_once_with(0.0)
        assert lane.red_led_pin.last_set_time == 0.0
        assert lane.white_led_pin is None


class TestAFCCanvasLaneSetGpioPin:
    def test_missing_pin_returns_without_scheduling(self, monkeypatch):
        lane = _make_configured_canvas_lane(monkeypatch)
        lane.reactor.monotonic = MagicMock()

        lane._set_gpio_pin(None, 0.5)

        lane.reactor.monotonic.assert_not_called()

    def test_sets_fractional_pwm_at_mcu_schedule_time(self, monkeypatch):
        lane = _make_configured_canvas_lane(monkeypatch, led_red_pin="PA1")
        pin = lane.red_led_pin
        lane.reactor.monotonic = MagicMock(return_value=10.0)
        pin.get_mcu.return_value.estimated_print_time.return_value = 4.0
        pin.get_mcu.return_value.min_schedule_time.return_value = 0.05

        lane._set_gpio_pin(pin, 0.35)

        pin.set_pwm.assert_called_once()
        print_time, value = pin.set_pwm.call_args.args
        assert print_time == pytest.approx(4.15)
        assert value == 0.35
        assert pin.last_set_time == pytest.approx(4.15)

    def test_respects_previous_pin_schedule_time(self, monkeypatch):
        lane = _make_configured_canvas_lane(monkeypatch, led_red_pin="PA1")
        pin = lane.red_led_pin
        pin.last_set_time = 6.0
        pin.get_mcu.return_value.estimated_print_time.return_value = 4.0
        pin.get_mcu.return_value.min_schedule_time.return_value = 0.05

        lane._set_gpio_pin(pin, 0.75)

        pin.set_pwm.assert_called_once_with(6.2, 0.75)
        assert pin.last_set_time == 6.2


class TestAFCCanvasLaneApplyCanvasLed:
    def test_maps_fractional_red_and_white_channels_to_pwm(self, monkeypatch):
        lane = _make_configured_canvas_lane(
            monkeypatch,
            led_red_pin="PA1",
            led_white_pin="PA2",
        )
        lane._set_gpio_pin = MagicMock()

        lane.apply_canvas_led("0.25,0,0,0.7")

        assert lane._set_gpio_pin.call_args_list == [
            call(lane.red_led_pin, 0.25),
            call(lane.white_led_pin, 0.7),
        ]

    def test_missing_white_channel_defaults_to_off(self, monkeypatch):
        lane = _make_configured_canvas_lane(
            monkeypatch,
            led_red_pin="PA1",
            led_white_pin="PA2",
        )
        lane._set_gpio_pin = MagicMock()

        lane.apply_canvas_led("0.4")

        assert lane._set_gpio_pin.call_args_list == [
            call(lane.red_led_pin, 0.4),
            call(lane.white_led_pin, 0.0),
        ]

    def test_channel_values_are_clamped_to_pwm_range(self, monkeypatch):
        lane = _make_configured_canvas_lane(
            monkeypatch,
            led_red_pin="PA1",
            led_white_pin="PA2",
        )
        lane._set_gpio_pin = MagicMock()

        lane.apply_canvas_led("1.5,0,0,-0.2")

        assert lane._set_gpio_pin.call_args_list == [
            call(lane.red_led_pin, 1.0),
            call(lane.white_led_pin, 0.0),
        ]


class TestAFCCanvasLaneCmdAfcCanvasToolLoad:
    def test_runs_extruder_move_after_sensor_triggers(self):
        lane = _make_canvas_lane()
        lane.get_toolhead_pre_sensor_state = MagicMock(
            side_effect=[False, False, True]
        )

        def record_odometer_pulses(
            distance,
            speed,
            label,
            wait_tool=True,
        ) -> None:
            lane.odometer_count = lane.odometer_load_threshold + 1

        lane.afc.move_e_pos.side_effect = record_odometer_pulses

        AFCCanvasLane.cmd_AFC_CANVAS_TOOL_LOAD(lane, MagicMock())

        assert lane.get_toolhead_pre_sensor_state.call_count == 3
        assert lane.afc.gcode.run_script_from_command.call_args_list == [
            call("AFC_PARK EXTRUDER=extruder")
        ]
        assert lane.afc.move_e_pos.call_args_list == [
            call(72.0, 25.0, "CANVAS tool load", wait_tool=True)
        ]
        assert lane.canvas_motor.drv8833_set_speed.call_args_list == [
            call(20.0),
            call(0.0),
            call(0.0),
        ]
        assert lane.odometer_count == 4
        assert lane.loaded_to_hub is True
        assert lane.logger.messages == [
            ("info", "Attempting CANVAS tool load for lane1, try 1/3")
        ]
        lane.afc.error.handle_lane_failure.assert_not_called()

    def test_stops_when_cutter_sensor_is_engaged(self):
        lane = _make_canvas_lane()
        lane.unit_obj.cutter_sensor_state = True

        AFCCanvasLane.cmd_AFC_CANVAS_TOOL_LOAD(lane, MagicMock())

        lane.get_toolhead_pre_sensor_state.assert_not_called()
        lane.afc.gcode.run_script_from_command.assert_not_called()
        lane.afc.move_e_pos.assert_not_called()
        lane.canvas_motor.drv8833_set_speed.assert_not_called()
        lane.afc.error.handle_lane_failure.assert_called_once_with(
            lane,
            "CANVAS tool load failed: cutter sensor is engaged.",
        )
        assert lane.loaded_to_hub is False
        assert lane.logger.messages == []


class TestAFCCanvasLaneRunCutterMacro:
    def test_runs_configured_cutter(self, monkeypatch):
        lane = _make_initialized_canvas_lane(monkeypatch)

        lane._run_cutter_macro()

        lane.extruder_obj.estats.increase_cut_total.assert_called_once_with()
        assert lane.afc.gcode.run_script_from_command.call_args_list == [
            call("AFC_CUT EXTRUDER=extruder")
        ]
        assert lane.logger.messages == []

    def test_skips_cutter_when_disabled(self, monkeypatch):
        lane = _make_initialized_canvas_lane(monkeypatch)
        lane.afc.tool_cut = False

        lane._run_cutter_macro()

        lane.extruder_obj.estats.increase_cut_total.assert_not_called()
        lane.afc.gcode.run_script_from_command.assert_not_called()
        assert lane.logger.messages == []


class TestAFCCanvasLaneRunPostCutMacros:
    def test_parks_after_cut_and_runs_builtin_tip_form(self, monkeypatch):
        lane = _make_initialized_canvas_lane(monkeypatch)
        lane.afc.form_tip = True
        form_tip = MagicMock()
        lane.printer._objects["AFC_form_tip"] = form_tip

        lane._run_post_cut_macros()

        assert lane.afc.gcode.run_script_from_command.call_args_list == [
            call("AFC_PARK EXTRUDER=extruder"),
            call("AFC_PARK EXTRUDER=extruder"),
        ]
        form_tip.tip_form.assert_called_once_with()
        assert lane.logger.messages == []

    def test_runs_custom_tip_form_without_parking(self, monkeypatch):
        lane = _make_initialized_canvas_lane(monkeypatch)
        lane.afc.tool_cut = False
        lane.afc.park = False
        lane.afc.form_tip = True
        lane.afc.form_tip_cmd = "CUSTOM_FORM_TIP"

        lane._run_post_cut_macros()

        assert lane.afc.gcode.run_script_from_command.call_args_list == [
            call("CUSTOM_FORM_TIP")
        ]
        assert lane.logger.messages == []

    def test_skips_disabled_post_cut_macros(self, monkeypatch):
        lane = _make_initialized_canvas_lane(monkeypatch)
        lane.afc.park = False
        lane.afc.form_tip = False

        lane._run_post_cut_macros()

        lane.afc.gcode.run_script_from_command.assert_not_called()
        assert lane.logger.messages == []


class TestAFCCanvasLaneRunCutterSequence:
    def test_succeeds_without_cutting_when_cutter_is_disabled(self, monkeypatch):
        lane = _make_initialized_canvas_lane(monkeypatch)
        lane.afc.tool_cut = False
        lane._run_cutter_macro = MagicMock()

        result = lane.run_cutter_sequence()

        assert result is True
        lane._run_cutter_macro.assert_not_called()
        assert lane.logger.messages == []

    def test_polls_from_current_time_until_sensor_disengages(self, monkeypatch):
        lane = _make_initialized_canvas_lane(monkeypatch)
        lane._run_cutter_macro = MagicMock()
        lane._cutter_sensor_engaged = MagicMock(side_effect=[True, False])
        lane.reactor.monotonic = MagicMock(side_effect=[10.0, 10.25])
        lane.reactor.pause = MagicMock()

        result = lane.run_cutter_sequence()

        assert result is True
        lane._run_cutter_macro.assert_called_once_with()
        lane.reactor.pause.assert_called_once_with(10.255)
        assert lane.logger.messages == [
            ("info", "Attempting cutting filament, try 1/3")
        ]

    def test_retries_after_timeout_then_succeeds(self, monkeypatch):
        lane = _make_initialized_canvas_lane(monkeypatch)
        lane.load_attempts = 2
        lane._run_cutter_macro = MagicMock()
        lane._cutter_sensor_engaged = MagicMock(side_effect=[True, False])
        lane.reactor.monotonic = MagicMock(side_effect=[20.0, 23.0, 24.0])
        lane.reactor.pause = MagicMock()

        result = lane.run_cutter_sequence()

        assert result is True
        assert lane._run_cutter_macro.call_count == 2
        lane.reactor.pause.assert_not_called()
        assert lane.logger.messages == [
            ("info", "Attempting cutting filament, try 1/2"),
            (
                "warning",
                "Cutter sensor did not disengage after cutting filament. "
                "Trying again.",
            ),
            ("info", "Attempting cutting filament, try 2/2"),
        ]

    def test_returns_false_after_all_attempts_time_out(self, monkeypatch):
        lane = _make_initialized_canvas_lane(monkeypatch)
        lane.load_attempts = 2
        lane._run_cutter_macro = MagicMock()
        lane._cutter_sensor_engaged = MagicMock(return_value=True)
        lane.reactor.monotonic = MagicMock(side_effect=[0.0, 3.0, 4.0, 7.0])
        lane.reactor.pause = MagicMock()

        result = lane.run_cutter_sequence()

        assert result is False
        assert lane._run_cutter_macro.call_count == 2
        lane.reactor.pause.assert_not_called()
        assert lane.logger.messages == [
            ("info", "Attempting cutting filament, try 1/2"),
            (
                "warning",
                "Cutter sensor did not disengage after cutting filament. "
                "Trying again.",
            ),
            ("info", "Attempting cutting filament, try 2/2"),
            (
                "warning",
                "Cutter sensor did not disengage after cutting filament. "
                "Trying again.",
            ),
        ]


class TestAFCCanvasLaneCmdAfcCanvasToolUnload:
    def test_runs_macros_then_retracts(self):
        lane = _make_canvas_lane()
        lane.get_toolhead_pre_sensor_state = MagicMock(return_value=False)
        lane.disengage_motors = MagicMock()

        AFCCanvasLane.cmd_AFC_CANVAS_TOOL_UNLOAD(lane, MagicMock())

        assert lane.afc.gcode.run_script_from_command.call_args_list == [
            call("AFC_CUT EXTRUDER=extruder"),
            call("AFC_PARK EXTRUDER=extruder"),
        ]
        assert lane.afc.move_e_pos.call_args_list == [
            call(-40.0, 20.0, "CANVAS tool unload", wait_tool=True)
        ]
        lane.disengage_motors.assert_called_once_with(-1.0)
        assert lane.loaded_to_hub is False
        assert lane.logger.messages == [
            ("info", "Attempting cutting filament, try 1/3"),
            ("info", "Attempting CANVAS tool unload for lane1, try 1/3")
        ]
        lane.afc.error.handle_lane_failure.assert_not_called()

    def test_stops_when_cutter_sensor_is_engaged(self, monkeypatch):
        lane = _make_initialized_canvas_lane(monkeypatch)
        lane.unit_obj.cutter_sensor_state = True
        lane.run_cutter_sequence = MagicMock()
        lane._run_post_cut_macros = MagicMock()

        lane.cmd_AFC_CANVAS_TOOL_UNLOAD(MagicMock())

        lane.select_lane.assert_called_once_with()
        lane.afc._check_extruder_temp.assert_called_once_with(lane)
        lane.disable_buffer.assert_called_once_with()
        lane.unit_obj.lane_unloading.assert_called_once_with(lane)
        lane.run_cutter_sequence.assert_not_called()
        lane._run_post_cut_macros.assert_not_called()
        lane.afc.move_e_pos.assert_not_called()
        lane.afc.error.handle_lane_failure.assert_called_once_with(
            lane,
            "CANVAS tool unload failed: cutter sensor is engaged before cutting.",
        )
        assert lane.loaded_to_hub is True
        assert lane.logger.messages == []

    def test_stops_when_cutter_retries_fail(self, monkeypatch):
        lane = _make_initialized_canvas_lane(monkeypatch)
        lane.run_cutter_sequence = MagicMock(return_value=False)
        lane._run_post_cut_macros = MagicMock()

        lane.cmd_AFC_CANVAS_TOOL_UNLOAD(MagicMock())

        lane.select_lane.assert_called_once_with()
        lane.afc._check_extruder_temp.assert_called_once_with(lane)
        lane.disable_buffer.assert_called_once_with()
        lane.unit_obj.lane_unloading.assert_called_once_with(lane)
        lane.run_cutter_sequence.assert_called_once_with()
        lane._run_post_cut_macros.assert_not_called()
        lane.afc.move_e_pos.assert_not_called()
        lane.afc.error.handle_lane_failure.assert_called_once_with(
            lane,
            "CANVAS tool unload failed: cutter sensor did not disengage after cutting.",
        )
        assert lane.loaded_to_hub is True
        assert lane.logger.messages == []
