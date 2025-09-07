#  The ROS remote project (PUI package)
#  Program configuration
#  Copyright 2024-2025 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2024-2025.
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

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, LivelinessPolicy


class ProgramConfig:
    SW_ACT_TIMER_INTERVAL_MS = 250
    ENCODER_HIGHLIGHT_TIMEOUT_MS = 4000
    PICO_NUM_LEDS = 11
    LED_SRVCL_TIMEOUT_S = 4
    BATT_WARN_LED_TRIG_VOLT = 9.5
    BATT_WARN_POPUP_TRIG_VOLT = 9

class RpiIoConfig:
    BUTTON_DEBOUNCE_TIME_S = 0.25
    ENCODER_PIN_A = 27
    ENCODER_PIN_B = 22
    ENCODER_BTN_PIN = 17
    TOGGLE_SW_A_PIN = 2
    TOGGLE_SW_B_PIN = 3
    TOGGLE_SW_C_PIN = 4


# ---- ROS Config ----
class RosConfig:
    NODE_NAME = "remote_pui_node"
    NODE_NAMESPACE = ""
    QOS_BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)
    QOS_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)

class RosNames:
    # From remote Raspberry Pi Pico
    JOYSTICK_TOPIC = "joy"
    GET_LED_STATES_SRV = "get_led_states"
    SET_LED_STATES_SRV = "set_led_state"
