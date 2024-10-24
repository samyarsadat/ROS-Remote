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

PROGRAM_VERSION = "2024.10.16"
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, LivelinessPolicy


# ---- Program Info ----
class ProgramInfoConfig:
    VERSION = PROGRAM_VERSION
    VERSION_DATE = "2024/10/16 @ 15:19 UTC"


class ProgramConfig:
    THREADS_LIVELINESS_CHECK_INTERVAL_S = 2
    BUTTON_DEBOUNCE_TIME_MS = 350


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
