#  The ROS remote project (GUI package)
#  Empty package init file
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

PROGRAM_VERSION = "2024.8.8"
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, LivelinessPolicy


# ---- Program Info ----
class ProgramInfoConfig:
    VERSION = PROGRAM_VERSION
    VERSION_DATE = "2024/08/08 @ 12:15PM UTC"


class ProgramConfig:
    THREADS_LIVELINESS_CHECK_INTERVAL_S = 2
    UI_DATA_UPDATE_INTERVAL_MS = 100
    UI_VIEWPORT_UPDATE_INTERVAL_MS = 50


# ---- ROS Config ----
class RosConfig:
    NODE_NAME = "remote_gui_node"
    NODE_NAMESPACE = ""
    EXECUTOR_DOMAIN_ID = None
    EXECUTOR_TIMEOUT = 0.05         # 50ms
    EXECUTOR_SHUTDOWN_TIMEOUT = 5   # 5s
    QOS_BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)
    QOS_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)
    THREAD_NAME = "robot_ros_thread"


class RosNames:
    ROBOT_NAMESPACE = "ros_robot"

    DIAGNOSTICS_TOPIC = f"{ROBOT_NAMESPACE}/diagnostics"
    ENABLE_RELAY_SRV = f"{ROBOT_NAMESPACE}/enable/set_relay"
    BATTERY_INFO_TOPIC = f"{ROBOT_NAMESPACE}/electrical/battery_state"
    ENCODER_ODOM_TOPIC = f"{ROBOT_NAMESPACE}/pos_data/encoder_odom"
    SET_CAMERA_LEDS_SRV = f"{ROBOT_NAMESPACE}/lights/camera/set"
    GET_CAMERA_LEDS_SRV = f"{ROBOT_NAMESPACE}/lights/camera/get"
    IMU_SENS_TOPIC = f"{ROBOT_NAMESPACE}/pos_data/imu"
    ENV_TEMP_SENS_TOPIC = f"{ROBOT_NAMESPACE}/env_sensors/temp"
    ENV_HUMIDITY_SENS_TOPIC = f"{ROBOT_NAMESPACE}/env_sensors/humidity"
    EMERGENCY_STOP_TOPIC = f"{ROBOT_NAMESPACE}/emergency_stop"
    PICO_A_CPU_TEMP_TOPIC = f"{ROBOT_NAMESPACE}/sys_temps/processor/pico_a"
    PICO_B_CPU_TEMP_TOPIC = f"{ROBOT_NAMESPACE}/sys_temps/processor/pico_b"
    PICO_A_SELFTEST_SRV = f"{ROBOT_NAMESPACE}/self_test/run_pico_a_routines"
    PICO_B_SELFTEST_SRV = f"{ROBOT_NAMESPACE}/self_test/run_pico_b_routines"
    PICO_A_CALIBRATE_SRV = f"{ROBOT_NAMESPACE}/calibrate/run_pico_a_routines"
    ENABLE_EMITTERS_SRV = f"{ROBOT_NAMESPACE}/enable/set_emitters"
    GET_EMITTERS_ENABLED_SRV = f"{ROBOT_NAMESPACE}/enable/get_emitters"
    LEFT_MTR_CTRL_TOPIC = f"{ROBOT_NAMESPACE}/motors/controllers/left_state"
    RIGHT_MTR_CTRL_TOPIC = f"{ROBOT_NAMESPACE}/motors/controllers/right_state"
    MTR_CTRL_ENABLE_SRV = f"{ROBOT_NAMESPACE}/motors/controllers/enable"
    MTR_CTRL_SET_PID_SRV = f"{ROBOT_NAMESPACE}/motors/controllers/set_pid"

    MICRO_SWITCH_TOPIC_BASE = ROBOT_NAMESPACE + "/range_sens/micro_switch/{}"
    MICRO_SWITCH_TOPIC_NAMES = ["front_right", "front_left", "back_right", "back_left"]

    ULTRASONIC_SENS_TOPIC_BASE = ROBOT_NAMESPACE + "/range_sens/ultrasonic/{}"
    ULTRASONIC_SENS_TOPIC_NAMES = ["front", "back", "right", "left"]

    CLIFF_SENS_TOPIC_BASE = ROBOT_NAMESPACE + "/range_base/cliff/{}"
    CLIFF_SENS_TOPIC_NAMES = ["front_1", "front_2", "front_3", "front_4", "back_1", "back_2", "back_3", "back_4"]


class RosFrameIds:
    CLIFF_SENS_BASE_FRAME_ID = "cliff_sens_{}"
    ULTRASONIC_SENS_BASE_FRAME_ID = "ultrasonic_sens_{}"
    MICRO_SW_SENS_BASE_FRAME_ID = "micro_switch_{}"
