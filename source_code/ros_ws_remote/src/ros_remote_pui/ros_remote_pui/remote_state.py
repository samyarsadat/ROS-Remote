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
from geometry_msgs.msg import Twist
from remote_pico_coms.srv import SetLedStates, GetLedStates
from ros_remote_gui.main import qt_app
from ros_remote_gui.main_window import get_main_window
from ros_remote_gui.ros_main import get_ros_node as get_gui_ros_node
from ros_remote_gui.ros_main import is_ros_node_initialized as is_gui_node_initialized
from ros_remote_gui.utils.gui_utils import get_msg_box_helper
from ros_remote_pui.config import ProgramConfig
from datetime import datetime
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
from numpy import interp


# Button signals
class RosSignals(QObject):
    left_l_kd2_btn_press_sig = Signal()
    left_r_kd2_btn_press_sig = Signal()
    left_red_btn_press_sig = Signal()
    left_l_green_btn_press_sig = Signal()
    left_r_green_btn_press_sig = Signal()
    joystick_msg_recv_sig = Signal()


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
    max_linear_velocity_mps: float
    max_angular_velocity_rps: float

    def __init__(self):
        self._ros_signals = RosSignals()
        self._ros_signals.left_l_kd2_btn_press_sig.connect(self.left_l_kd2_btn_press)
        self._ros_signals.left_r_kd2_btn_press_sig.connect(self.left_r_kd2_btn_press)
        self._ros_signals.left_red_btn_press_sig.connect(self.left_red_btn_press)
        self._ros_signals.left_l_green_btn_press_sig.connect(self.left_l_green_btn_press)
        self._ros_signals.left_r_green_btn_press_sig.connect(self.left_r_green_btn_press)
        self._ros_signals.joystick_msg_recv_sig.connect(self._publish_joystick)

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

        self.max_linear_velocity_mps = 0.0
        self.max_angular_velocity_rps = 0.0

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
        self._last_joystick_en_state = False

        self._sw_state_act_tmr = QTimer()
        self._sw_state_act_tmr.timeout.connect(self._sw_state_act_tmr_call)
        self._sw_state_act_tmr.start(ProgramConfig.SW_ACT_TIMER_INTERVAL_MS)

        self._led_state_act_tmr = QTimer()
        self._led_state_act_tmr.timeout.connect(self._led_state_act_tmr_call)
        self._led_state_act_tmr.start(ProgramConfig.LED_STATE_SET_TIMER_INTERVAL_MS)

        qt_app.aboutToQuit.connect(self._set_all_leds_off)

    def _sw_state_act_tmr_call(self) -> None:
        if ros_remote_pui.ros_main.is_ros_node_initialized():
            # Lock/unlock remote
            if (not self.key_sw_en) and get_main_window().isEnabled():
                get_main_window().setEnabled(False)
                if self._touchscreen_id: QProcess.startDetached("/bin/xinput", ["disable", self._touchscreen_id])
                self._set_led_state(3, 3, 65535)
            elif self.key_sw_en and (not get_main_window().isEnabled()):
                get_main_window().setEnabled(True)
                if self._touchscreen_id: QProcess.startDetached("/bin/xinput", ["enable", self._touchscreen_id])
                self._set_led_state(3, 0, 0)

        if is_gui_node_initialized():
            # Enable/disable camera LEDs (all full-on/full-off)
            # TODO: Improve the logic of this.
            if self.key_sw_en:
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

        # Calculate max. linear & angular velocities based on potentiometer value
        self.max_linear_velocity_mps = interp(self.potentiometer_val, [0, 1024], [0, ProgramConfig.MAX_LINEAR_VEL_MPS])
        self.max_angular_velocity_rps = interp(self.potentiometer_val, [0, 1024], [0, ProgramConfig.MAX_ANGULAR_VEL_RPS])

    def _led_state_act_tmr_call(self) -> None:
        if ros_remote_pui.ros_main.is_ros_node_initialized():
            if not self._power_led_set:
                self._power_led_set = self._set_led_state(4, 0, 32000)
            if self.right_kd2_en != self._last_joystick_en_state:
                self._last_joystick_en_state = self.right_kd2_en
                self._set_led_state(0, 2, 65535 if self.right_kd2_en else 0)

    @Slot()
    def _publish_joystick(self) -> None:
        if self.right_kd2_en:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = interp(self.joystick_vals[0], [-512, 512], [-self.max_linear_velocity_mps, self.max_linear_velocity_mps])
            cmd_vel_msg.angular.z = interp(self.joystick_vals[1], [-512, 512], [-self.max_angular_velocity_rps, self.max_angular_velocity_rps])
            get_gui_ros_node().cmd_vel_pub.publish(cmd_vel_msg)

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

    # EMERGENCY STOP
    @Slot()
    def left_red_btn_press(self) -> None:
        get_gui_ros_node().emergency_stop_pub.publish(Empty())
        ros_remote_pui.ros_main.get_ros_node().get_logger().warn("Emergency stop command published!")
        get_msg_box_helper().show_msg_box_sig.emit("warn", "Emergency Stop", "Emergency stop has been requested.")

    # UI - PREVIOUS PAGE
    @Slot()
    def left_l_green_btn_press(self) -> None:
        if self.key_sw_en:
            current_index = get_main_window().ui.pages.currentIndex()
            next_index = (current_index - 1) % get_main_window().ui.pages.count()
            get_main_window().ui.pages.setCurrentIndex(next_index)

    # UI - NEXT PAGE
    @Slot()
    def left_r_green_btn_press(self) -> None:
        if self.key_sw_en:
            current_index = get_main_window().ui.pages.currentIndex()
            next_index = (current_index + 1) % get_main_window().ui.pages.count()
            get_main_window().ui.pages.setCurrentIndex(next_index)

    @staticmethod
    def _led_set_request_done_call(future: Future) -> None:
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

    @Slot()
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
_remote_state = RemoteState()

def get_remote_state() -> RemoteState:
    global _remote_state
    return _remote_state

# Raspberry Pi IO handler & encoder nav handler
from ros_remote_pui.rpi_io_handler import RpiIoHandler
from ros_remote_pui.encoder_handler import EncoderNavHandler
_rpi_io_handler = RpiIoHandler()
_encoder_handler = EncoderNavHandler()