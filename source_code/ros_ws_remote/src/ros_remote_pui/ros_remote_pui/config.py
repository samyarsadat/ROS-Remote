#  The ROS remote project (PUI package)
#  Program configuration
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

PROGRAM_VERSION = "2024.11.12"
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, LivelinessPolicy


# ---- Program Info ----
class ProgramInfoConfig:
    VERSION = PROGRAM_VERSION
    VERSION_DATE = "2024/11/12 @ 7:31 UTC"


class ProgramConfig:
    THREADS_LIVELINESS_CHECK_INTERVAL_S = 2
    SW_ACT_TIMER_INTERVAL_MS = 250
    ENCODER_HIGHLIGHT_TIMEOUT_MS = 4000
    PICO_NUM_LEDS = 11
    LED_SRVCL_TIMEOUT_S = 4
    MAX_LINEAR_VEL_MPS = 0.628   # Meters per second (theoretical max for robot)
    MAX_ANGULAR_VEL_RPS = 8      # Radians per second (TODO: Adjust this later)
    CMD_VEL_SAFETY_TIMEOUT_MS = 800


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
    EXECUTOR_DOMAIN_ID = 75
    EXECUTOR_TIMEOUT = 0.05         # 50ms
    EXECUTOR_SHUTDOWN_TIMEOUT = 5   # 5s
    QOS_BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)
    QOS_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)
    THREAD_NAME = "ros_thread"


class RosNames:
    # From remote Raspberry Pi Pico
    BUTTON_STATES_TOPIC = "inputs/buttons"
    SWITCH_STATES_TOPIC = "inputs/switches"
    JOYSTICK_STATE_TOPIC = "inputs/joystick"
    POTENTIOMETER_STATE_TOPIC = "inputs/potentiometer"
    GET_JOYSTICK_CONFIG_SRV = "inputs/joystick/get_config"
    SET_JOYSTICK_CONFIG_SRV = "inputs/joystick/set_config"
    GET_LED_STATES_SRV = "outputs/leds/get_states"
    SET_LED_STATES_SRV = "outputs/leds/set_states"
    RUN_SELFTEST_SRV = "self_test/pico"
