#  The ROS remote project (PUI package)
#  Remote PUI handler. Stores and manages button/sw states and callbacks.
#  Also handles setting LED states.
#  Copyright 2024 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2024.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https: www.gnu.org/licenses/>.

import ros_remote_pui.ros_main
import subprocess
import re
from asyncio import Future
from PySide6.QtCore import QTimer, QObject, Signal, Slot, QProcess
from ros_remote_hid.srv import SetLedState, GetLedStates
from ros_remote_gui.main_window import get_main_window
from ros_remote_gui.ros_main import get_ros_node as get_gui_ros_node
from ros_remote_pui.config import ProgramConfig, RpiIoConfig
from datetime import datetime
from std_msgs.msg import Empty, Bool
from std_srvs.srv import SetBool


# ROS signals
class RosSignals(QObject):
    left_l_kd2_btn_press_sig = Signal()
    left_r_kd2_btn_press_sig = Signal()
    left_l_green_btn_press_sig = Signal()
    left_r_green_btn_press_sig = Signal()


# Remote state
class RemoteState:
    key_sw_en: bool
    left_top_sw_en: bool
    left_mid_a_sw_en: bool
    left_mid_b_sw_en: bool
    left_mid_c_sw_en: bool
    e_stop_sw_en: bool
    right_sw_en: bool
    right_kd2_en: bool
    potentiometer_val: int
    joystick_vals: list[int]
    last_joystick_pub: datetime
    max_linear_velocity_mps: float
    max_angular_velocity_rps: float
    left_green_kd2_btn_en: bool
    left_red_kd2_btn_en: bool
    left_green_left_btn_en: bool
    left_green_right_btn_en: bool

    def __init__(self):
        self._ros_signals = RosSignals()
        self._ros_signals.left_l_kd2_btn_press_sig.connect(self.left_l_kd2_btn_press)
        self._ros_signals.left_r_kd2_btn_press_sig.connect(self.left_r_kd2_btn_press)
        self._ros_signals.left_l_green_btn_press_sig.connect(self.left_l_green_btn_press)
        self._ros_signals.left_r_green_btn_press_sig.connect(self.left_r_green_btn_press)

        self.key_sw_en = False          # Lock/unlock remote
        self.left_top_sw_en = False     # Not assigned
        self.left_mid_a_sw_en = False   # Enable/disable camera LEDs (all full-on/full-off)
        self.left_mid_b_sw_en = False   # Not assigned
        self.left_mid_c_sw_en = False   # Not assigned
        self.e_stop_sw_en = False       # Motor controller enable
        self.right_sw_en = False        # Joystick input override/avg with nav select
        self.right_kd2_en = False       # Joystick enable

        self.left_green_kd2_btn_en = False    # Not assigned
        self.left_red_kd2_btn_en = False      # Not assigned
        self.left_green_left_btn_en = False   # UI - previous page
        self.left_green_right_btn_en = False  # UI - next page

        # Get the ID of the touchscreen from Xinput.
        # This will be used when locking/unlocking the remote.
        self._touchscreen_id = None
        xinput_resp = subprocess.run(["xinput", "list"], stdout=subprocess.PIPE, text=True)
        id_matches = re.search(r"touchscreen.*id=(\d+)", xinput_resp.stdout, re.IGNORECASE)
        if id_matches: self._touchscreen_id = id_matches.group(1)

        # Will be True when there is an active call to the motor controller enable service in-progress.
        self._call_to_mtr_ctrl_en_in_progress = False

        # LED-related state
        self._power_led_set = False
        self._mtr_ctrl_last_state = 0   # 0: one or more not enabled, 1: all enabled, 2: data stale
        self._last_battery_led_state = False

        self._sw_state_act_tmr = QTimer()
        self._sw_state_act_tmr.timeout.connect(self._sw_state_act_tmr_call)
        self._sw_state_act_tmr.start(ProgramConfig.SW_ACT_TIMER_INTERVAL_MS)

        from ros_remote_gui.main import qt_app
        qt_app.aboutToQuit.connect(self._set_all_leds_off)

    def _sw_state_act_tmr_call(self) -> None:
        if ros_remote_pui.ros_main.is_ros_node_initialized():
            # Lock/unlock remote
            if self.key_sw_en and get_main_window().isEnabled():
                get_main_window().setEnabled(False)
                if self._touchscreen_id: QProcess.startDetached("/bin/xinput", ["disable", self._touchscreen_id])
                self._make_set_led_request(3, 3, 65535)
            elif (not self.key_sw_en) and (not get_main_window().isEnabled()):
                get_main_window().setEnabled(True)
                if self._touchscreen_id: QProcess.startDetached("/bin/xinput", ["enable", self._touchscreen_id])
                self._make_set_led_request(3, 0, 0)

            # LED states
            if not self._power_led_set:
                self._power_led_set = self._make_set_led_request(4, 0, 32000)

            if get_main_window().power_tab_ui_handler.batt_voltage < ProgramConfig.BATT_WARN_LED_TRIG_VOLT and not self._last_battery_led_state:
                self._last_battery_led_state = self._make_set_led_request(6, 2, 65535)
            elif get_main_window().power_tab_ui_handler.batt_voltage > ProgramConfig.BATT_WARN_LED_TRIG_VOLT and self._last_battery_led_state:
                self._last_battery_led_state = not self._make_set_led_request(6, 0, 0)

            # Motor controller LED
            current_mtr_ctrl_state = 1

            if get_main_window().motor_tab_ui_handler.right_data_stale or get_main_window().motor_tab_ui_handler.left_data_stale:
                current_mtr_ctrl_state = 3
            else:
                if get_main_window().motor_tab_ui_handler.left_ctrl_enabled and get_main_window().motor_tab_ui_handler.right_ctrl_enabled:
                    current_mtr_ctrl_state = 2
                elif (not get_main_window().motor_tab_ui_handler.left_ctrl_enabled) and (not get_main_window().motor_tab_ui_handler.right_ctrl_enabled):
                    current_mtr_ctrl_state = 0

            if current_mtr_ctrl_state != self._mtr_ctrl_last_state:
                success = False

                match current_mtr_ctrl_state:
                    case 0:
                        success = self._make_set_led_request(1, 0, 0)
                    case 1:
                        success = self._make_set_led_request(1, 1, 32000)
                    case 2:
                        success = self._make_set_led_request(1, 0, 32000)
                    case 3:
                        success = self._make_set_led_request(1, 2, 32000)

                if success:
                    self._mtr_ctrl_last_state = current_mtr_ctrl_state

        # Enable/disable camera LEDs (all full-on/full-off)
        if not self.key_sw_en:
            # TODO: Improve the logic of this.
            if (not self.left_mid_a_sw_en) and get_main_window().ui.camLedsBrightnessSlider.value() > 0:
                get_main_window().ui.camLed1Check.setChecked(True)
                get_main_window().ui.camLed2Check.setChecked(True)
                get_main_window().ui.camLed3Check.setChecked(True)
                get_main_window().ui.camLed4Check.setChecked(True)
                get_main_window().ui.camLedsBrightnessSlider.setValue(0)
            elif self.left_mid_a_sw_en and get_main_window().ui.camLedsBrightnessSlider.value() == 0:
                get_main_window().ui.camLed1Check.setChecked(True)
                get_main_window().ui.camLed2Check.setChecked(True)
                get_main_window().ui.camLed3Check.setChecked(True)
                get_main_window().ui.camLed4Check.setChecked(True)
                get_main_window().ui.camLedsBrightnessSlider.setValue(100)

        # Motor controller enable (NO REMOTE LOCK CHECK)
        # TODO: This could result in the motor controller enable service being called over and over again.
        if (not self.e_stop_sw_en) and (get_main_window().motor_tab_ui_handler.left_ctrl_enabled or get_main_window().motor_tab_ui_handler.right_ctrl_enabled):
            self._enable_mtr_ctrl(False)
        elif self.e_stop_sw_en and (not get_main_window().motor_tab_ui_handler.left_ctrl_enabled or not get_main_window().motor_tab_ui_handler.right_ctrl_enabled):
            self._enable_mtr_ctrl(True)

    # BUTTON NOT ASSIGNED
    @Slot()
    def left_l_kd2_btn_press(self) -> None:
        if self._get_led_state(9)[1] == 0:
            self._make_set_led_request(9, 0, 65535)
        else:
            self._make_set_led_request(9, 0, 0)

    # BUTTON NOT ASSIGNED
    @Slot()
    def left_r_kd2_btn_press(self) -> None:
        if self._get_led_state(10)[1] == 0:
            self._make_set_led_request(10, 0, 65535)
        else:
            self._make_set_led_request(10, 0, 0)

    # UI - PREVIOUS PAGE
    @Slot()
    def left_l_green_btn_press(self) -> None:
        if not self.key_sw_en:
            current_index = get_main_window().ui.pages.currentIndex()
            next_index = (current_index - 1) % get_main_window().ui.pages.count()
            get_main_window().ui.pages.setCurrentIndex(next_index)

    # UI - NEXT PAGE
    @Slot()
    def left_r_green_btn_press(self) -> None:
        if not self.key_sw_en:
            current_index = get_main_window().ui.pages.currentIndex()
            next_index = (current_index + 1) % get_main_window().ui.pages.count()
            get_main_window().ui.pages.setCurrentIndex(next_index)

    @staticmethod
    def _led_set_request_done_call(future: Future) -> None:
        if future.exception() or (not future.result()):
            ros_remote_pui.ros_main.get_ros_node().get_logger().error("Set LED states service call failure!")

    def _make_set_led_request(self, led_num: int, mode: int, pwm_out: int) -> bool:
        if ros_remote_pui.ros_main.get_ros_node().set_led_state_srvcl.service_is_ready():
            req = SetLedState.Request()
            req.index = led_num
            req.led_mode = mode
            req.pwm_output = pwm_out

            future = ros_remote_pui.ros_main.get_ros_node().set_led_state_srvcl.call_async(req)
            future.add_done_callback(self._led_set_request_done_call)
            return True
        else:
            ros_remote_pui.ros_main.get_ros_node().get_logger().error("Set LED states service unavailable!")
        return False

    @Slot()
    def _set_all_leds_off(self) -> bool:
        for i in range(ProgramConfig.PICO_NUM_LEDS):
            if not self._make_set_led_request(i, 0, 0):
                return False
        return True

    @staticmethod
    def _get_led_state(led_num: int, timeout_s = ProgramConfig.LED_SRVCL_TIMEOUT_S) -> tuple[int, int]:
        if ros_remote_pui.ros_main.get_ros_node().get_led_states_srvcl.service_is_ready():
            req = GetLedStates.Request()
            res = ros_remote_pui.ros_main.get_ros_node().srv_call_with_timeout(ros_remote_pui.ros_main.get_ros_node().get_led_states_srvcl, req, timeout_s)

            if res: return res.led_mode[led_num], res.pwm_output[led_num]
            ros_remote_pui.ros_main.get_ros_node().get_logger().error("Get LED states service timed-out!")
        else:
            ros_remote_pui.ros_main.get_ros_node().get_logger().error("Get LED states service unavailable!")
        return 0, 0

    def _enable_mtr_ctrl_done_call(self, future: Future) -> None:
        self._call_to_mtr_ctrl_en_in_progress = False

        if future.exception() or not future.result():
            ros_remote_pui.ros_main.get_ros_node().get_logger().error("Motor controller enable/disable service call failed!")
        elif future.result() and not future.result().success:
            ros_remote_pui.ros_main.get_ros_node().get_logger().error(f"Motor controller enable/disable service call failed: {future.result().message}")

    def _enable_mtr_ctrl(self, enable: bool) -> bool:
        if not self._call_to_mtr_ctrl_en_in_progress:
            if get_gui_ros_node().mtr_ctrl_enable_srvcl.service_is_ready():
                req = SetBool.Request()
                req.data = enable
                future = get_gui_ros_node().mtr_ctrl_enable_srvcl.call_async(req)
                future.add_done_callback(self._enable_mtr_ctrl_done_call)
                self._call_to_mtr_ctrl_en_in_progress = True
                return True
            else:
                ros_remote_pui.ros_main.get_ros_node().get_logger().error("Motor controller enable/disable service unavailable!")
        return False


# RemoteState instance
_remote_state: RemoteState | None = None
def get_remote_state() -> RemoteState | None:
    return _remote_state


# Raspberry Pi IO handler
from gpiozero import Button
class RpiIoHandler:
    def __init__(self):
        self.toggle_sw_a = Button(RpiIoConfig.TOGGLE_SW_A_PIN, pull_up=True)
        self.toggle_sw_b = Button(RpiIoConfig.TOGGLE_SW_B_PIN, pull_up=True)
        self.toggle_sw_c = Button(RpiIoConfig.TOGGLE_SW_C_PIN, pull_up=True)
        self.toggle_sw_a.when_activated = self._set_toggle_a_state
        self.toggle_sw_b.when_activated = self._set_toggle_b_state
        self.toggle_sw_c.when_activated = self._set_toggle_c_state
        self.toggle_sw_a.when_deactivated = self._set_toggle_a_state
        self.toggle_sw_b.when_deactivated = self._set_toggle_b_state
        self.toggle_sw_c.when_deactivated = self._set_toggle_c_state

    def _set_toggle_a_state(self):
        _remote_state.left_mid_a_sw_en = self.toggle_sw_a.is_active

    def _set_toggle_b_state(self):
        _remote_state.left_mid_b_sw_en = self.toggle_sw_b.is_active

    def _set_toggle_c_state(self):
        _remote_state.left_mid_c_sw_en = self.toggle_sw_c.is_active


# Initialize the remote state and RPi IO handler
from ros_remote_pui.encoder_handler import EncoderNavHandler
_io_handler: RpiIoHandler | None = None
_encoder_nav_handler: EncoderNavHandler | None = None

def init_remote_state() -> None:
    global _remote_state, _io_handler, _encoder_nav_handler
    if _remote_state is None:
        _remote_state = RemoteState()
        _io_handler = RpiIoHandler()
        _encoder_nav_handler = EncoderNavHandler()
