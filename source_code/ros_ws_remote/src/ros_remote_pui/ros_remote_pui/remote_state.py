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

from asyncio import Future
from PySide6.QtCore import QTimer, QObject, Signal, Slot
from remote_pico_coms.srv import SetLedStates, GetLedStates
from ros_remote_gui.main_window import get_main_window
from ros_remote_pui.config import ProgramConfig
from datetime import datetime
import ros_remote_pui.ros_main


# Button signals
class ButtonSignals(QObject):
    left_l_kd2_btn_press_sig = Signal()
    left_r_kd2_btn_press_sig = Signal()
    left_red_btn_press_sig = Signal()
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
    last_joystick_recv: datetime

    def __init__(self):
        self._btn_signals = ButtonSignals()
        self._btn_signals.left_l_kd2_btn_press_sig.connect(self.left_l_kd2_btn_press)
        self._btn_signals.left_r_kd2_btn_press_sig.connect(self.left_r_kd2_btn_press)
        self._btn_signals.left_red_btn_press_sig.connect(self.left_red_btn_press)
        self._btn_signals.left_l_green_btn_press_sig.connect(self.left_l_green_btn_press)
        self._btn_signals.left_r_green_btn_press_sig.connect(self.left_r_green_btn_press)

        self.key_sw_en = False          # Lock/unlock remote
        self.left_top_sw_en = False     # Not assigned
        self.left_mid_a_sw_en = False   # Enable/disable camera LEDs (all full-on/full-off)
        self.left_mid_b_sw_en = False   # Not assigned
        self.left_mid_c_sw_en = False   # Not assigned
        self.e_stop_sw_en = False       # Motor controller enable
        self.right_sw_en = False        # Joystick input override/avg with nav select
        self.right_kd2_en = False       # Joystick enable
        self.potentiometer_val = 0      # Robot max. linear velocity

        self.joystick_vals = [0, 0]
        self.last_joystick_recv = datetime.now()

        self._sw_state_act_tmr = QTimer()
        self._sw_state_act_tmr.timeout.connect(self._sw_state_act_tmr_call)
        self._sw_state_act_tmr.start(ProgramConfig.SW_ACT_TIMER_INTERVAL_MS)

    def _sw_state_act_tmr_call(self) -> None:
        pass

    # BUTTON NOT ASSIGNED
    @Slot()
    def left_l_kd2_btn_press(self) -> None:
        if self._get_led_state(9)[1] == 0:
            self._set_led_state(9, 0, 65535)
        else:
            self._set_led_state(9, 0, 0)

    # BUTTON NOT ASSIGNED
    @Slot()
    def left_r_kd2_btn_press(self) -> None:
        if self._get_led_state(10)[1] == 0:
            self._set_led_state(10, 0, 65535)
        else:
            self._set_led_state(10, 0, 0)

    # BUTTON NOT ASSIGNED
    @Slot()
    def left_red_btn_press(self) -> None:
        pass

    # UI - PREVIOUS PAGE
    @Slot()
    def left_l_green_btn_press(self) -> None:
        current_index = get_main_window().ui.pages.currentIndex()
        next_index = (current_index - 1) % get_main_window().ui.pages.count()
        get_main_window().ui.pages.setCurrentIndex(next_index)

    # UI - NEXT PAGE
    @Slot()
    def left_r_green_btn_press(self) -> None:
        current_index = get_main_window().ui.pages.currentIndex()
        next_index = (current_index + 1) % get_main_window().ui.pages.count()
        get_main_window().ui.pages.setCurrentIndex(next_index)

    @staticmethod
    def _led_set_request_done_call(future: Future):
        if future.exception() or (not future.result()) or (not future.result().success):
            ros_remote_pui.ros_main.get_ros_node().get_logger().error("Set LED states service call failure!")

    def _make_set_led_request(self, mask: list[int], modes: list[int], pwm_vals: list[int]) -> bool:
        if ros_remote_pui.ros_main.get_ros_node().set_led_states_srvcl.service_is_ready():
            req = SetLedStates.Request()
            req.set_state_mask = mask
            req.led_modes = modes
            req.pwm_outputs = pwm_vals

            future = ros_remote_pui.ros_main.get_ros_node().set_led_states_srvcl.call_async(req)
            future.add_done_callback(self._led_set_request_done_call)
            return True
        else:
            ros_remote_pui.ros_main.get_ros_node().get_logger().error("Set LED states service unavailable!")
        return False

    def _set_led_state(self, led_num: int, mode: int, pwm_out: int) -> bool:
        mask = [False] * ProgramConfig.PICO_NUM_LEDS
        modes = [0] * ProgramConfig.PICO_NUM_LEDS
        pwm_vals = [0] * ProgramConfig.PICO_NUM_LEDS

        mask[led_num] = True
        modes[led_num] = mode
        pwm_vals[led_num] = pwm_out

        return self._make_set_led_request(mask, modes, pwm_vals)

    def _set_all_leds_off(self) -> bool:
        mask = [True] * ProgramConfig.PICO_NUM_LEDS
        modes = [0] * ProgramConfig.PICO_NUM_LEDS
        pwm_vals = [0] * ProgramConfig.PICO_NUM_LEDS

        return self._make_set_led_request(mask, modes, pwm_vals)

    def _set_all_leds_on(self) -> bool:
        mask = [True] * ProgramConfig.PICO_NUM_LEDS
        modes = [0] * ProgramConfig.PICO_NUM_LEDS
        pwm_vals = [65535] * ProgramConfig.PICO_NUM_LEDS

        return self._make_set_led_request(mask, modes, pwm_vals)

    @staticmethod
    def _get_led_state(led_num: int, timeout_s = ProgramConfig.LED_SRVCL_TIMEOUT_S) -> tuple[int, int]:
        if ros_remote_pui.ros_main.get_ros_node().get_led_states_srvcl.service_is_ready():
            req = GetLedStates.Request()
            res = ros_remote_pui.ros_main.get_ros_node().srv_call_with_timeout(ros_remote_pui.ros_main.get_ros_node().get_led_states_srvcl, req, timeout_s)

            if res:
                return res.led_modes[led_num], res.pwm_outputs[led_num]
            ros_remote_pui.ros_main.get_ros_node().get_logger().error("Get LED states service timed-out!")
        else:
            ros_remote_pui.ros_main.get_ros_node().get_logger().error("Get LED states service unavailable!")
        return 0, 0


# RemoteState instance
_remote_state = RemoteState()

def get_remote_state() -> RemoteState:
    global _remote_state
    return _remote_state

# Raspberry Pi IO handler & encoder nav handler
from ros_remote_pui.rpi_io_handler import RpiIoHandler
from ros_remote_pui.encoder_handler import EncoderNavHandler
_rpi_io_handler = RpiIoHandler()
_encoder_handler = EncoderNavHandler()