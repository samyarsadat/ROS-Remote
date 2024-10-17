#  The ROS remote project (PUI package)
#  Remote button & switch state storage and callbacks
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

    def left_l_kd2_btn_press(self):
        print("left_l_kd2_btn_press")

    def left_r_kd2_btn_press(self):
        print("left_r_kd2_btn_press")

    def left_red_btn_press(self):
        print("left_red_btn_press")

    def left_l_green_btn_press(self):
        print("left_l_green_btn_press")

    def left_r_green_btn_press(self):
        print("left_r_green_btn_press")

    def left_rot_enc_btn_press(self):
        print("left_rot_enc_btn_press")

    def left_rot_enc_sig(self):
        print("left_rot_enc_sig")


# RemoteState instance
_remote_state = RemoteState()

def get_remote_state() -> RemoteState:
    global _remote_state
    return _remote_state
