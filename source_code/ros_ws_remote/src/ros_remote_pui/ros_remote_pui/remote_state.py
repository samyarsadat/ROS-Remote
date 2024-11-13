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

from PySide6.QtCore import QTimer, QObject, Signal, Slot
from ros_remote_pui.config import ProgramConfig
from datetime import datetime


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

    @Slot()
    def left_l_kd2_btn_press(self) -> None:
        print("left_l_kd2_btn_press")

    @Slot()
    def left_r_kd2_btn_press(self) -> None:
        print("left_r_kd2_btn_press")

    @Slot()
    def left_red_btn_press(self) -> None:
        print("left_red_btn_press")

    @Slot()
    def left_l_green_btn_press(self) -> None:
        print("left_l_green_btn_press")

    @Slot()
    def left_r_green_btn_press(self) -> None:
        print("left_r_green_btn_press")


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