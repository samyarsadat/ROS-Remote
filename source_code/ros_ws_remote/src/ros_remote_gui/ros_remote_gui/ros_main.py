#  The ROS remote project (GUI package)
#  ROS setup, node, and executor
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

import rclpy
from datetime import datetime
from copy import deepcopy
from geometry_msgs.msg import Twist
from rclpy import Context
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_remote_gui.config import RosConfig, RosFrameIds, RosNames
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.srv import SelfTest
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ros_remote_gui.main_window import get_main_window
from ros_remote_gui.utils.utils import quat_msg_to_euler
from sensor_msgs.msg import BatteryState, Imu, Temperature, RelativeHumidity, Range, CompressedImage, Image
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
from ros_robot_msgs.srv import SetCameraLeds, GetCameraLeds, SetPidTunings, RunCalibrationsA, GetBool
from ros_robot_msgs.msg import MotorCtrlState


# ---- ROS Node ----
class RosNode(Node):
    def __init__(self, context: Context):
        super().__init__(node_name=RosConfig.NODE_NAME, namespace=RosConfig.NODE_NAMESPACE, context=context)
        self.get_logger().info("Creating publishers, subscribers, and services clients...")

        self._reentrant_cb_group = ReentrantCallbackGroup()
        self._selftest_calib_cb_group = MutuallyExclusiveCallbackGroup()
        self._emer_stop_cb_group = MutuallyExclusiveCallbackGroup()
        self._viewport_cb_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.create_publisher(Twist, RosNames.CMD_VEL_TOPIC, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group)

        # Viewport camera & camera overlay
        self.front_camera_comp_sub = self.create_subscription(CompressedImage, RosNames.CAMERA_FEED_TOPIC, self.front_camera_comp_callback, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._viewport_cb_group)
        self.front_overlay_sub = self.create_subscription(CompressedImage, RosNames.CAMERA_OVERLAY_TOPIC, self.front_overlay_callback, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._viewport_cb_group)

        self.diagnostics_sub = self.create_subscription(DiagnosticStatus, RosNames.DIAGNOSTICS_TOPIC, self.diagnostics_callback, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group)
        self.ping_driver_srvcl = self.create_client(GetBool, RosNames.PING_DRIVER_TOPIC, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group)
        self.enable_relay_srvcl = self.create_client(SetBool, RosNames.ENABLE_RELAY_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.battery_info_sub = self.create_subscription(BatteryState, RosNames.BATTERY_INFO_TOPIC, self.battery_info_callback, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.encoder_odom_sub = self.create_subscription(Odometry, RosNames.ENCODER_ODOM_TOPIC, self.encoder_odom_callback, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group)
        self.set_camera_leds_srvcl = self.create_client(SetCameraLeds, RosNames.SET_CAMERA_LEDS_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.get_camera_leds_srvcl = self.create_client(GetCameraLeds, RosNames.GET_CAMERA_LEDS_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.imu_sens_sub = self.create_subscription(Imu, RosNames.IMU_SENS_TOPIC, self.imu_sens_callback, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group)
        self.env_temp_sens_sub = self.create_subscription(Temperature, RosNames.ENV_TEMP_SENS_TOPIC, self.env_temp_sens_callback, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.env_humidity_sens_sub = self.create_subscription(RelativeHumidity, RosNames.ENV_HUMIDITY_SENS_TOPIC, self.env_humidity_sens_callback, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.emergency_stop_pub = self.create_publisher(Empty, RosNames.EMERGENCY_STOP_TOPIC, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._emer_stop_cb_group)
        self.pico_a_cpu_temp_sub = self.create_subscription(Temperature, RosNames.PICO_A_CPU_TEMP_TOPIC, self.pico_a_cpu_temp_callback, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.pico_b_cpu_temp_sub = self.create_subscription(Temperature, RosNames.PICO_B_CPU_TEMP_TOPIC, self.pico_b_cpu_temp_callback, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.pico_a_selftest_srvcl = self.create_client(SelfTest, RosNames.PICO_A_SELFTEST_SRV, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._selftest_calib_cb_group)
        self.pico_b_selftest_srvcl = self.create_client(SelfTest, RosNames.PICO_B_SELFTEST_SRV, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._selftest_calib_cb_group)
        self.pico_a_calibrate_srvcl = self.create_client(RunCalibrationsA, RosNames.PICO_A_CALIBRATE_SRV, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._selftest_calib_cb_group)
        self.enable_emitters_srvcl = self.create_client(SetBool, RosNames.ENABLE_EMITTERS_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.get_emitters_enabled_srvcl = self.create_client(GetBool, RosNames.GET_EMITTERS_ENABLED_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.left_mtr_ctrl_sub = self.create_subscription(MotorCtrlState, RosNames.LEFT_MTR_CTRL_TOPIC, self.left_mtr_ctrl_callback, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.right_mtr_ctrl_sub = self.create_subscription(MotorCtrlState, RosNames.RIGHT_MTR_CTRL_TOPIC, self.right_mtr_ctrl_callback, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.mtr_ctrl_enable_srvcl = self.create_client(SetBool, RosNames.MTR_CTRL_ENABLE_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.mtr_ctrl_set_pid_srvcl = self.create_client(SetPidTunings, RosNames.MTR_CTRL_SET_PID_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.micro_switch_subs = []
        self.ultrasonic_sens_subs = []
        self.cliff_sens_subs = []

        for i in range(0, 4):
            t_name = RosNames.MICRO_SWITCH_TOPIC_BASE.format(RosNames.MICRO_SWITCH_TOPIC_NAMES[i])
            self.micro_switch_subs.append(self.create_subscription(Range, t_name, self.micro_switch_callback, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group))

        for i in range(0, 4):
            t_name = RosNames.ULTRASONIC_SENS_TOPIC_BASE.format(RosNames.ULTRASONIC_SENS_TOPIC_NAMES[i])
            self.ultrasonic_sens_subs.append(self.create_subscription(Range, t_name, self.ultrasonic_sens_callback, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group))

        for i in range(0, 8):
            t_name = RosNames.CLIFF_SENS_TOPIC_BASE.format(RosNames.CLIFF_SENS_TOPIC_NAMES[i])
            self.cliff_sens_subs.append(self.create_subscription(Range, t_name, self.cliff_sens_callback, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group))

    @staticmethod
    def diagnostics_callback(msg: DiagnosticStatus) -> None:
        get_main_window().diag_tab_ui_handler.rcv_diag_msg_sig.emit(msg)

    @staticmethod
    def front_camera_comp_callback(msg: CompressedImage):
        get_main_window().main_tab_ui_handler.viewport_thread.ros_image_cam = msg

    @staticmethod
    def front_overlay_callback(msg: CompressedImage):
        get_main_window().main_tab_ui_handler.viewport_thread.ros_image_overlay = msg

    @staticmethod
    def battery_info_callback(msg: BatteryState) -> None:
        rounded_voltage = round(msg.voltage, 2)
        rounded_current = round(msg.current, 3)
        get_main_window().sensors_tab_ui_handler.batt_voltage = rounded_voltage
        get_main_window().sensors_tab_ui_handler.batt_current = rounded_current
        get_main_window().power_tab_ui_handler.batt_voltage = rounded_voltage
        get_main_window().power_tab_ui_handler.batt_current = rounded_current
        get_main_window().main_tab_ui_handler.batt_voltage = rounded_voltage

    @staticmethod
    def encoder_odom_callback(msg: Odometry) -> None:
        orientation_euler = quat_msg_to_euler(msg.pose.pose.orientation)
        get_main_window().motor_tab_ui_handler.orientation_yaw = orientation_euler.z
        get_main_window().motor_tab_ui_handler.position_x = msg.pose.pose.position.x
        get_main_window().motor_tab_ui_handler.position_y = msg.pose.pose.position.y
        get_main_window().motor_tab_ui_handler.linear_velocity = msg.twist.twist.linear.x
        get_main_window().motor_tab_ui_handler.angular_velocity = msg.twist.twist.angular.z

    @staticmethod
    def imu_sens_callback(msg: Imu) -> None:
        get_main_window().sensors_tab_ui_handler.imu_accels = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        get_main_window().sensors_tab_ui_handler.imu_gyros = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        get_main_window().sensors_tab_ui_handler.imu_comps = [msg.orientation.x, msg.orientation.y, msg.orientation.z]

    @staticmethod
    def env_temp_sens_callback(msg: Temperature) -> None:
        get_main_window().sensors_tab_ui_handler.env_dht_temp = msg.temperature

    @staticmethod
    def env_humidity_sens_callback(msg: RelativeHumidity) -> None:
        get_main_window().sensors_tab_ui_handler.env_dht_humidity = msg.relative_humidity

    @staticmethod
    def pico_a_cpu_temp_callback(msg: Temperature) -> None:
        get_main_window().sensors_tab_ui_handler.env_pico_a_temp = msg.temperature

    @staticmethod
    def pico_b_cpu_temp_callback(msg: Temperature) -> None:
        get_main_window().sensors_tab_ui_handler.env_pico_b_temp = msg.temperature

    @staticmethod
    def left_mtr_ctrl_callback(msg: MotorCtrlState) -> None:
        get_main_window().motor_tab_ui_handler.left_ctrl_enabled = msg.controller_enabled
        get_main_window().motor_tab_ui_handler.rpm_measured_left = deepcopy(msg.measured_rpms)
        get_main_window().motor_tab_ui_handler.rpm_target_left = msg.target_rpm
        get_main_window().motor_tab_ui_handler.left_ctrl_pid_out = msg.pid_output
        get_main_window().motor_tab_ui_handler.left_ctrl_pid_tunings = deepcopy(msg.pid_tunings)
        get_main_window().motor_tab_ui_handler.encoder_pulse_ctrs_left = deepcopy(msg.total_enc_counts)
        get_main_window().motor_tab_ui_handler.last_left_data_rcv = datetime.now()

    @staticmethod
    def right_mtr_ctrl_callback(msg: MotorCtrlState) -> None:
        get_main_window().motor_tab_ui_handler.right_ctrl_enabled = msg.controller_enabled
        get_main_window().motor_tab_ui_handler.rpm_measured_right = deepcopy(msg.measured_rpms)
        get_main_window().motor_tab_ui_handler.rpm_target_right = msg.target_rpm
        get_main_window().motor_tab_ui_handler.right_ctrl_pid_out = msg.pid_output
        get_main_window().motor_tab_ui_handler.right_ctrl_pid_tunings = deepcopy(msg.pid_tunings)
        get_main_window().motor_tab_ui_handler.encoder_pulse_ctrs_right = deepcopy(msg.total_enc_counts)
        get_main_window().motor_tab_ui_handler.last_right_data_rcv = datetime.now()

    @staticmethod
    def micro_switch_callback(msg: Range) -> None:
        if msg.header.frame_id == RosFrameIds.MICRO_SW_SENS_BASE_FRAME_ID.format(RosNames.MICRO_SWITCH_TOPIC_NAMES[0]):
            get_main_window().sensors_tab_ui_handler.micro_sw_states[0] = msg.range
        elif msg.header.frame_id == RosFrameIds.MICRO_SW_SENS_BASE_FRAME_ID.format(RosNames.MICRO_SWITCH_TOPIC_NAMES[1]):
            get_main_window().sensors_tab_ui_handler.micro_sw_states[1] = msg.range
        elif msg.header.frame_id == RosFrameIds.MICRO_SW_SENS_BASE_FRAME_ID.format(RosNames.MICRO_SWITCH_TOPIC_NAMES[2]):
            get_main_window().sensors_tab_ui_handler.micro_sw_states[2] = msg.range
        elif msg.header.frame_id == RosFrameIds.MICRO_SW_SENS_BASE_FRAME_ID.format(RosNames.MICRO_SWITCH_TOPIC_NAMES[3]):
            get_main_window().sensors_tab_ui_handler.micro_sw_states[3] = msg.range
        else:
            get_ros_node().get_logger().error("Received micro switch data for unknown sensor.")

    @staticmethod
    def ultrasonic_sens_callback(msg: Range) -> None:
        if msg.header.frame_id == RosFrameIds.ULTRASONIC_SENS_BASE_FRAME_ID.format(RosNames.ULTRASONIC_SENS_TOPIC_NAMES[0]):
            get_main_window().sensors_tab_ui_handler.ultra_dists[0] = msg.range
        elif msg.header.frame_id == RosFrameIds.ULTRASONIC_SENS_BASE_FRAME_ID.format(RosNames.ULTRASONIC_SENS_TOPIC_NAMES[1]):
            get_main_window().sensors_tab_ui_handler.ultra_dists[1] = msg.range
        elif msg.header.frame_id == RosFrameIds.ULTRASONIC_SENS_BASE_FRAME_ID.format(RosNames.ULTRASONIC_SENS_TOPIC_NAMES[2]):
            get_main_window().sensors_tab_ui_handler.ultra_dists[2] = msg.range
        elif msg.header.frame_id == RosFrameIds.ULTRASONIC_SENS_BASE_FRAME_ID.format(RosNames.ULTRASONIC_SENS_TOPIC_NAMES[3]):
            get_main_window().sensors_tab_ui_handler.ultra_dists[3] = msg.range
        else:
            get_ros_node().get_logger().error("Received ultrasonic sensor data for unknown sensor.")

    @staticmethod
    def cliff_sens_callback(msg: Range) -> None:
        if msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[0]):
            get_main_window().sensors_tab_ui_handler.cliff_front_states[0] = msg.range
        elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[1]):
            get_main_window().sensors_tab_ui_handler.cliff_front_states[1] = msg.range
        elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[2]):
            get_main_window().sensors_tab_ui_handler.cliff_front_states[2] = msg.range
        elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[3]):
            get_main_window().sensors_tab_ui_handler.cliff_front_states[3] = msg.range
        elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[4]):
            get_main_window().sensors_tab_ui_handler.cliff_back_states[0] = msg.range
        elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[5]):
            get_main_window().sensors_tab_ui_handler.cliff_back_states[1] = msg.range
        elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[6]):
            get_main_window().sensors_tab_ui_handler.cliff_back_states[2] = msg.range
        elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[7]):
            get_main_window().sensors_tab_ui_handler.cliff_back_states[3] = msg.range
        else:
            get_ros_node().get_logger().error("Received cliff sensor data for unknown sensor.")


# ---- Node object ----
_gui_ros_node = RosNode


# ---- Executor ----
def is_ros_node_initialized() -> bool:
    global _gui_ros_node
    return isinstance(_gui_ros_node, RosNode)

def get_ros_node() -> RosNode:
    global _gui_ros_node
    return _gui_ros_node

def ros_executor_thread(stop_thread):
    try:
        internal_context = rclpy.Context()
        rclpy.init(context=internal_context, domain_id=RosConfig.EXECUTOR_DOMAIN_ID)

        global _gui_ros_node
        _gui_ros_node = RosNode(context=internal_context)
        _gui_ros_node.get_logger().info("Node initialized!")

        executor = MultiThreadedExecutor(context=internal_context)
        executor.add_node(_gui_ros_node)
        _gui_ros_node.get_logger().info("Executor initialized!")

        _gui_ros_node.get_logger().info("Starting the executor...")

        while not stop_thread():
            executor.spin_once(timeout_sec=RosConfig.EXECUTOR_TIMEOUT)

        # Cleanup
        _gui_ros_node.get_logger().warn("ROS thread stopping. Shutting the executor down.")
        executor.shutdown(timeout_sec=RosConfig.EXECUTOR_SHUTDOWN_TIMEOUT)
        rclpy.shutdown(context=internal_context)

    except ExternalShutdownException:
        if is_ros_node_initialized():
            get_ros_node().get_logger().warn("External shutdown exception!")
