from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from extras.AFC_canvas import afcCanvas
from extras.AFC_functions import afcFunction
from extras.AFC_lane import AFCMoveWarning, MoveDirection, SpeedMode, AFCHomingPoints
from tests.conftest import MockAFC, MockConfig, MockLogger, MockPrinter, MockReactor


def _make_canvas_unit(name="CANVAS_1"):
    unit = afcCanvas.__new__(afcCanvas)
    afc = MockAFC()
    afc.error.pause_resume = MagicMock()
    afc.function.is_printing = MagicMock(return_value=False)
    reactor = MockReactor()
    afc.reactor = reactor
    printer = MockPrinter(afc=afc)

    unit.printer = printer
    unit.afc = afc
    unit.function = afc.function
    unit.logger = MockLogger()
    unit.reactor = reactor
    unit.name = name
    unit.type = "canvas"
    unit.prep_distance = 15.0
    unit.prep_speed = 25.0
    unit.short_move_dis = 5.0
    unit.short_moves_speed = 25.0
    unit.short_moves_accel = 400.0
    unit.tool_load_sync_speed_offset = 4.0
    unit.tool_unload_sync_speed_offset = 6.0
    unit.tool_unload_lane_extra_distance = 7.0
    unit.tool_unload_lane_extra_speed = 12.0
    unit.cutter_sensor_state = False
    unit.enable_9v = None
    unit.enable_24v = None
    return unit


def _make_lane(sensor_states=None):
    lane = MagicMock()
    lane.prep_state = True
    lane.short_move_dis = 4.0
    lane.short_moves_speed = 20.0
    lane.short_moves_accel = 300.0
    lane.led_not_ready = "1,0,0,0"
    lane.led_ready = "0,1,0,0"
    lane.led_loading = "0,0,1,0"
    lane.led_unloading = "1,1,0,0"
    lane.led_tool_loaded = "0,0,1,0"
    lane.led_tool_loaded_idle = "0,0,0,1"
    lane.led_tool_unloaded = "1,0,0,0"
    lane.extruder_obj = MagicMock()
    lane.apply_canvas_led = MagicMock()
    lane.get_speed_accel = MagicMock(return_value=(30.0, 300.0))
    lane.move = MagicMock()
    if sensor_states is None:
        sensor_states = [False]
    lane.get_toolhead_pre_sensor_state = MagicMock(side_effect=sensor_states)
    return lane


def test_prep_load_uses_configured_distance_and_speed():
    unit = _make_canvas_unit()
    lane = _make_lane()
    lane.move_with_odometer = MagicMock()

    unit.prep_load(lane)

    args, kwargs = lane.move_with_odometer.call_args
    assert args[:2] == (15.0, 25.0)
    assert callable(kwargs["stop_condition"])


def test_prep_post_load_stops_motor():
    unit = _make_canvas_unit()
    lane = _make_lane()
    lane.disengage_motors = MagicMock()

    unit.prep_post_load(lane)

    lane.apply_canvas_led.assert_called_once_with(unit.afc.led_ready)
    lane.disengage_motors.assert_called_once_with(1.0)
    assert lane._load_state is True


def test_led_callbacks_use_canvas_led_outputs():
    unit = _make_canvas_unit()
    lane = _make_lane()

    unit.lane_tool_loaded(lane)
    unit.lane_tool_unloaded(lane)

    lane.apply_canvas_led.assert_any_call(unit.afc.led_tool_loaded)
    lane.apply_canvas_led.assert_any_call(unit.afc.led_ready)
    lane.extruder_obj.set_status_led.assert_any_call(unit.afc.led_tool_loaded)
    lane.extruder_obj.set_status_led.assert_any_call(unit.afc.led_tool_unloaded)


def test_cutter_callback_updates_sensor_state():
    unit = _make_canvas_unit()

    unit.cutter_callback(10.0, True)
    assert unit.cutter_sensor_state is True

    unit.cutter_callback(11.0, False)
    assert unit.cutter_sensor_state is False


def test_lane_unloaded_turns_led_off_and_clears_load_state():
    unit = _make_canvas_unit()
    lane = _make_lane()
    lane._load_state = True

    unit.lane_unloaded(lane)

    lane.apply_canvas_led.assert_called_once_with(unit.afc.led_off)
    assert lane._load_state is False


def test_move_to_hub_succeeds_when_sensor_triggers():
    unit = _make_canvas_unit()
    lane = _make_lane(sensor_states=[False, True])

    success, moved, warn = unit.move_to_hub(
        lane, 10.0, MoveDirection.POS, True, speed_mode=SpeedMode.LONG
    )

    assert success is True
    assert moved == 4.0
    assert warn == AFCMoveWarning.NONE


def test_move_to_hub_warns_when_sensor_never_triggers():
    unit = _make_canvas_unit()
    lane = _make_lane(sensor_states=[False, False, False])

    success, moved, warn = unit.move_to_hub(
        lane, 8.0, MoveDirection.POS, True, speed_mode=SpeedMode.LONG
    )

    assert success is False
    assert moved == 8.0
    assert warn == AFCMoveWarning.WARN


def test_system_test_seeds_load_state_from_pressed_prep_endstop():
    unit = _make_canvas_unit()
    lane = _make_lane()
    lane.prep_state = False
    lane._load_state = False
    lane._afc_prep_done = False
    lane.name = "lane1"
    lane.map = "T0"
    lane.tool_loaded = False
    lane.send_lane_data = MagicMock()
    lane.do_enable = MagicMock()
    lane.set_afc_prep_done = MagicMock()
    lane.endstops = {
        AFCHomingPoints.PREP: {"endstop": MagicMock(query_endstop=MagicMock(return_value=True))}
    }

    unit.system_Test(lane, delay=0.0, assignTcmd=False, enable_movement=False)

    assert lane.prep_state is True
    assert lane._load_state is True
    lane.apply_canvas_led.assert_called_with(unit.afc.led_ready)
    lane.set_afc_prep_done.assert_called_once_with()


def test_system_test_falls_back_to_callback_state_without_prep_endstop():
    unit = _make_canvas_unit()
    lane = _make_lane()
    lane.prep_state = False
    lane._load_state = True
    lane._afc_prep_done = False
    lane.name = "lane1"
    lane.map = "T0"
    lane.tool_loaded = False
    lane.send_lane_data = MagicMock()
    lane.do_enable = MagicMock()
    lane.set_afc_prep_done = MagicMock()
    lane.endstops = {}

    unit.system_Test(lane, delay=0.0, assignTcmd=False, enable_movement=False)

    assert lane.prep_state is False
    assert lane._load_state is False
    lane.apply_canvas_led.assert_called_with(unit.afc.led_off)


@pytest.fixture
def canvas_unit() -> afcCanvas:
    afc = MockAFC()
    printer = MockPrinter(afc=afc)
    config = MockConfig(name="AFC_canvas CANVAS_1", printer=printer)
    return afcCanvas(config)


@pytest.fixture
def unsupported_prompt_messages() -> list[tuple[str, str]]:
    return [
        ("raw", "// action:prompt_begin CANVAS_1 Calibration"),
        ("raw", "// action:prompt_text Canvas does not support calibration."),
        ("raw", "// action:prompt_footer_button Cancel|RESPOND TYPE=command MSG=action:prompt_end"
         "|warning"),
        ("raw", "// action:prompt_show"),
    ]


class TestAFCCanvasCmdUnitCalibration:
    @pytest.mark.parametrize("has_lanes", [False, True])
    def test_shows_unsupported_prompt(
        self, canvas_unit: afcCanvas, unsupported_prompt_messages: list[tuple[str, str]],
        has_lanes: bool,
    ) -> None:
        if has_lanes:
            canvas_unit.lanes["lane1"] = MagicMock()

        canvas_unit.cmd_UNIT_CALIBRATION(MagicMock())

        assert canvas_unit.logger.messages == unsupported_prompt_messages
        canvas_unit.afc.gcode.run_script_from_command.assert_not_called()

    @pytest.mark.parametrize("command", [
        "UNIT_CALIBRATION", "UNIT_LANE_CALIBRATION", "UNIT_BOW_CALIBRATION",
        "AFC_UNIT_TD_ONE_CALIBRATION",
    ])
    def test_registered_command_shows_unsupported_prompt(
        self, canvas_unit: afcCanvas, unsupported_prompt_messages: list[tuple[str, str]],
        command: str,
    ) -> None:
        canvas_unit.gcode.register_mux_command = MagicMock()
        canvas_unit.afc.td1_defined = True
        canvas_unit.handle_connect()
        canvas_unit.handle_moonraker_connect()
        assert canvas_unit.logo == '<span class=success--text>CANVAS Ready</span>\n'
        assert canvas_unit.logo_error == '<span class=error--text>CANVAS Not Ready</span>\n'
        assert canvas_unit.afc.units == {"CANVAS_1": canvas_unit}
        callbacks = {
            call.args[0]: call.args[3]
            for call in canvas_unit.gcode.register_mux_command.call_args_list
        }

        callbacks[command](MagicMock())

        assert canvas_unit.logger.messages == unsupported_prompt_messages
        canvas_unit.afc.gcode.run_script_from_command.assert_not_called()


class TestAFCCanvasCmdUnitLaneCalibration:
    def test_shows_unsupported_prompt(
        self, canvas_unit: afcCanvas, unsupported_prompt_messages: list[tuple[str, str]],
    ) -> None:
        canvas_unit.cmd_UNIT_LANE_CALIBRATION(MagicMock())

        assert canvas_unit.logger.messages == unsupported_prompt_messages
        canvas_unit.afc.gcode.run_script_from_command.assert_not_called()


class TestAFCCanvasCmdUnitBowCalibration:
    def test_shows_unsupported_prompt(
        self, canvas_unit: afcCanvas, unsupported_prompt_messages: list[tuple[str, str]],
    ) -> None:
        canvas_unit.cmd_UNIT_BOW_CALIBRATION(MagicMock())

        assert canvas_unit.logger.messages == unsupported_prompt_messages
        canvas_unit.afc.gcode.run_script_from_command.assert_not_called()


class TestAFCCanvasCmdAFCUnitTDOneCalibration:
    def test_shows_unsupported_prompt(
        self, canvas_unit: afcCanvas, unsupported_prompt_messages: list[tuple[str, str]],
    ) -> None:
        canvas_unit.cmd_AFC_UNIT_TD_ONE_CALIBRATION(MagicMock())

        assert canvas_unit.logger.messages == unsupported_prompt_messages
        canvas_unit.afc.gcode.run_script_from_command.assert_not_called()


class TestAFCCanvasCalibrateLane:
    def test_rejects_calibration_without_touching_lane(self, canvas_unit: afcCanvas) -> None:
        lane = MagicMock()

        result = canvas_unit.calibrate_lane(lane, 5.0)

        assert result == (False, "Canvas does not support calibration.", 0.0)
        assert lane.mock_calls == []
        assert canvas_unit.logger.messages == []
        canvas_unit.afc.gcode.run_script_from_command.assert_not_called()

    @pytest.mark.parametrize("direct_hub", [False, True])
    @pytest.mark.parametrize("lanes,unit", [("lane1", None), ("all", None), ("all", "CANVAS_1")])
    def test_shared_calibration_displays_failure_without_moving_filament(
        self, canvas_unit: afcCanvas, direct_hub: bool, lanes: str, unit: str | None,
    ) -> None:
        config = MockConfig(name="AFC_functions", printer=canvas_unit.printer)
        config.access_tracking = {}
        functions = afcFunction(config)
        functions.handle_connect()
        lane = MagicMock()
        lane.name = "lane1"
        lane.unit_obj = canvas_unit
        lane.extruder_obj.is_standalone.return_value = False
        lane.load_state = True
        lane.prep_state = True
        lane.is_direct_hub.return_value = direct_hub
        canvas_unit.lanes["lane1"] = lane
        canvas_unit.afc.lanes["lane1"] = lane
        canvas_unit.afc.units["CANVAS_1"] = canvas_unit

        result = functions._lane_calibration(lanes, unit, 5.0, 25.0)

        assert result == (False, [], [])
        assert canvas_unit.logger.messages == [
            ("info", "Starting AFC distance Calibrations"),
            ("raw", "// action:prompt_begin AFC Calibration Failed"),
            ("raw", "// action:prompt_text AFC Calibration Failed for lane1. "),
            ("raw", "// action:prompt_text Fail message: Canvas does not support calibration."),
            ("raw", "// action:prompt_footer_button Cancel|RESPOND TYPE=command "
             "MSG=action:prompt_end|warning"),
            ("raw", "// action:prompt_show"),
        ]
        canvas_unit.afc.error.AFC_error.assert_called_once_with(
            "Canvas does not support calibration.", False,
        )
        lane.move.assert_not_called()
        lane.move_advanced.assert_not_called()
        lane.move_with_odometer.assert_not_called()
        canvas_unit.afc.gcode.run_script_from_command.assert_not_called()


class TestAFCCanvasCalibrateBowden:
    def test_rejects_calibration_without_touching_lane(self, canvas_unit: afcCanvas) -> None:
        lane = MagicMock()

        result = canvas_unit.calibrate_bowden(lane, 25.0, 5.0)

        assert result == (False, "Canvas does not support calibration.", 0.0)
        assert lane.mock_calls == []
        assert canvas_unit.logger.messages == []
        canvas_unit.afc.gcode.run_script_from_command.assert_not_called()


class TestAFCCanvasCalibrateTD1:
    def test_rejects_calibration_without_touching_lane(self, canvas_unit: afcCanvas) -> None:
        lane = MagicMock()

        result = canvas_unit.calibrate_td1(lane, 25.0, 5.0)

        assert result == (False, "Canvas does not support calibration.", 0.0)
        assert lane.mock_calls == []
        assert canvas_unit.logger.messages == []
        canvas_unit.afc.gcode.run_script_from_command.assert_not_called()
