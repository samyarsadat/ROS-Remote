#  The ROS remote project (PUI package)
#  Raspberry Pi IO handler for the rotary encoder and 3 toggle switches.
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

from gpiozero import RotaryEncoder, Button
from ros_remote_pui.config import RpiIoConfig
import ros_remote_pui.remote_state


class RpiIoHandler:
    def __init__(self):
        self.rotary_enc = RotaryEncoder(RpiIoConfig.ENCODER_PIN_A, RpiIoConfig.ENCODER_PIN_B)
        self.rotary_enc.when_rotated_clockwise = self._enc_rotate_call_cw
        self.rotary_enc.when_rotated_counter_clockwise = self._enc_rotate_call_ccw
        self.rotary_btn = Button(RpiIoConfig.ENCODER_BTN_PIN, pull_up=False, bounce_time=RpiIoConfig.BUTTON_DEBOUNCE_TIME_S)
        self.rotary_btn.when_activated = self._enc_btn_call

        self.toggle_sw_a = Button(RpiIoConfig.TOGGLE_SW_A_PIN, pull_up=True, bounce_time=RpiIoConfig.BUTTON_DEBOUNCE_TIME_S)
        self.toggle_sw_b = Button(RpiIoConfig.TOGGLE_SW_B_PIN, pull_up=True, bounce_time=RpiIoConfig.BUTTON_DEBOUNCE_TIME_S)
        self.toggle_sw_c = Button(RpiIoConfig.TOGGLE_SW_C_PIN, pull_up=True, bounce_time=RpiIoConfig.BUTTON_DEBOUNCE_TIME_S)
        self.toggle_sw_a.when_activated = self._set_toggle_a_state
        self.toggle_sw_b.when_activated = self._set_toggle_b_state
        self.toggle_sw_c.when_activated = self._set_toggle_c_state

    def _set_toggle_a_state(self):
        ros_remote_pui.remote_state.get_remote_state().left_mid_a_sw_en = self.toggle_sw_a.is_active

    def _set_toggle_b_state(self):
        ros_remote_pui.remote_state.get_remote_state().left_mid_b_sw_en = self.toggle_sw_b.is_active

    def _set_toggle_c_state(self):
        ros_remote_pui.remote_state.get_remote_state().left_mid_c_sw_en = self.toggle_sw_c.is_active

    def _enc_rotate_call_cw(self):
        ros_remote_pui.remote_state.get_encoder_handler().rot_enc_sig(direction=True)

    def _enc_rotate_call_ccw(self):
        ros_remote_pui.remote_state.get_encoder_handler().rot_enc_sig(direction=False)

    def _enc_btn_call(self):
        ros_remote_pui.remote_state.get_encoder_handler().rot_enc_btn_press()
