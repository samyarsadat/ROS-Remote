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

import threading
import rclpy
import ros_remote_gui.main
from typing import Union
from rclpy import Context
from rclpy.client import Client, SrvTypeRequest, SrvTypeResponse
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_remote_gui.config import RosConfig, RosFrameIds, RosNames
from ros_remote_gui.main_window import get_main_window
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.srv import SelfTest
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import BatteryState, Imu, Temperature, RelativeHumidity, Range, CompressedImage, Image
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
from ros_robot_msgs.srv import SetCameraLeds, GetCameraLeds, SetPidTunings, RunCalibrationsA, GetBool
from ros_robot_msgs.msg import MotorCtrlState
from ros_remote_gui.utils.gui_utils import generate_indicator_stylesheet


# ---- ROS Node ----
class RosNode(Node):
    # Client.call() implementation with timeout.
    def srv_call_with_timeout(self, client: Client, request: SrvTypeRequest, timeout_s: int) -> Union[SrvTypeResponse, None]:
        event = threading.Event()

        def unblock(ftr):
            event.set()

        future = client.call_async(request)
        future.add_done_callback(unblock)

        if not future.done():
            if not event.wait(float(timeout_s)):
                self.get_logger().error(f"Service call failure [{client.srv_name}]: timed out! Cancelling.")
                future.cancel()
                return None

        if future.exception() is not None:
            raise future.exception()

        return future.result()

    def __init__(self, context: Context):
        super().__init__(node_name=RosConfig.NODE_NAME, namespace=RosConfig.NODE_NAMESPACE, context=context)
        self.get_logger().info("Creating publishers, subscribers, and services servers...")

        self._reentrant_cb_group = ReentrantCallbackGroup()
        self._selftest_calib_cb_group = MutuallyExclusiveCallbackGroup()
        self._emer_stop_cb_group = MutuallyExclusiveCallbackGroup()
        self._viewport_cb_group = ReentrantCallbackGroup()

        # Viewport camera & camera overlay
        self.front_camera_comp_sub = self.create_subscription(CompressedImage, RosNames.CAMERA_FEED_TOPIC, self.front_camera_comp_callback, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._viewport_cb_group)
        self.front_overlay_comp_sub = self.create_subscription(Image, RosNames.CAMERA_OVERLAY_TOPIC, self.front_overlay_comp_callback, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._viewport_cb_group)

        self.diagnostics_sub = self.create_subscription(DiagnosticStatus, RosNames.DIAGNOSTICS_TOPIC, self.diagnostics_callback, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._reentrant_cb_group)
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

    def diagnostics_callback(self, msg: DiagnosticStatus) -> None:
        pass

    @staticmethod
    def front_camera_comp_callback(msg: CompressedImage):
        ros_remote_gui.main.main_tab_ui_handler.viewport_thread.ros_image_cam = msg

    @staticmethod
    def front_overlay_comp_callback(msg: CompressedImage):
        ros_remote_gui.main.main_tab_ui_handler.viewport_thread.ros_image_overlay = msg

    @staticmethod
    def battery_info_callback(msg: BatteryState) -> None:
        rounded_voltage = round(msg.voltage, 2)
        rounded_current = round(msg.current, 3)

        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.sensorsTab.objectName():
            get_main_window().ui.batterySensCurrentValue.display(rounded_current)
            get_main_window().ui.batterySensVoltageValue.display(rounded_voltage)

        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.powerTab.objectName():
            get_main_window().ui.batteryVoltageValue.display(rounded_voltage)
            get_main_window().ui.batteryCurrentValue.display(rounded_current)

        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.mainTab.objectName():
            get_main_window().ui.teleBatteryValue.setText(str(rounded_voltage))

    @staticmethod
    def encoder_odom_callback(msg: Odometry) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.motorCtrlTab.objectName():
            get_main_window().ui.encOdomPosXValue.setText(str(round(msg.pose.pose.position.x, 3)))
            get_main_window().ui.encOdomPosYValue.setText(str(round(msg.pose.pose.position.y, 3)))
            get_main_window().ui.encOdomYawValue.setText(str(round(msg.pose.pose.orientation.z, 1)))
            get_main_window().ui.encOdomLinVelValue.setText(str(round(msg.twist.twist.linear.x, 2)))
            get_main_window().ui.encOdomAngVelValue.setText(str(round(msg.twist.twist.angular.z, 2)))

    @staticmethod
    def imu_sens_callback(msg: Imu) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.sensorsTab.objectName():
            get_main_window().ui.imuSensAccelXValue.setText(str(round(msg.linear_acceleration.x, 2)))
            get_main_window().ui.imuSensAccelYValue.setText(str(round(msg.linear_acceleration.y, 2)))
            get_main_window().ui.imuSensAccelZValue.setText(str(round(msg.linear_acceleration.z, 2)))
            get_main_window().ui.imuSensGyroXValue.setText(str(round(msg.angular_velocity.x, 2)))
            get_main_window().ui.imuSensGyroYValue.setText(str(round(msg.angular_velocity.y, 2)))
            get_main_window().ui.imuSensGyroZValue.setText(str(round(msg.angular_velocity.z, 2)))
            get_main_window().ui.imuSensCompXValue.setText(str(round(msg.orientation.x, 1)))
            get_main_window().ui.imuSensCompYValue.setText(str(round(msg.orientation.y, 1)))
            get_main_window().ui.imuSensCompZValue.setText(str(round(msg.orientation.z, 1)))

    @staticmethod
    def env_temp_sens_callback(msg: Temperature) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.sensorsTab.objectName():
            get_main_window().ui.envSensTempValue.setText(str(round(msg.temperature, 2)))

    @staticmethod
    def env_humidity_sens_callback(msg: RelativeHumidity) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.sensorsTab.objectName():
            get_main_window().ui.envSensHumValue.setText(str(round(msg.relative_humidity, 1)))

    @staticmethod
    def pico_a_cpu_temp_callback(msg: Temperature) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.sensorsTab.objectName():
            get_main_window().ui.envSensPicoATempValue.setText(str(round(msg.temperature, 2)))

    @staticmethod
    def pico_b_cpu_temp_callback(msg: Temperature) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.sensorsTab.objectName():
            get_main_window().ui.envSensPicoBTempValue.setText(str(round(msg.temperature, 2)))

    @staticmethod
    def left_mtr_ctrl_callback(msg: MotorCtrlState) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.motorCtrlTab.objectName():
            get_main_window().ui.mtrCtrlEnableRIndicator.setStyleSheet(generate_indicator_stylesheet(msg.controller_enabled, "green"))
            get_main_window().ui.rpmMeasuredFRValue.setText(str(round(msg.measured_rpms[0], 1)))
            get_main_window().ui.rpmMeasuredBRValue.setText(str(round(msg.measured_rpms[1], 1)))
            get_main_window().ui.rpmTargetFRValue.setText(str(round(msg.target_rpm, 1)))
            get_main_window().ui.rpmTargetBRValue.setText(str(round(msg.target_rpm, 1)))
            get_main_window().ui.rightPidOutValue.setText(str(msg.pid_output))
            get_main_window().ui.rightPidPValue.setText(str(round(msg.pid_tunings[0], 2)))
            get_main_window().ui.rightPidIValue.setText(str(round(msg.pid_tunings[1], 2)))
            get_main_window().ui.rightPidDValue.setText(str(round(msg.pid_tunings[2], 2)))
            get_main_window().ui.encPulseCtrsFRValue.setText(str(msg.total_enc_counts[0]))
            get_main_window().ui.encPulseCtrsBRValue.setText(str(msg.total_enc_counts[1]))

    @staticmethod
    def right_mtr_ctrl_callback(msg: MotorCtrlState) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.motorCtrlTab.objectName():
            get_main_window().ui.mtrCtrlEnableLIndicator.setStyleSheet(generate_indicator_stylesheet(msg.controller_enabled, "green"))
            get_main_window().ui.rpmMeasuredFLValue.setText(str(round(msg.measured_rpms[0], 1)))
            get_main_window().ui.rpmMeasuredBLValue.setText(str(round(msg.measured_rpms[1], 1)))
            get_main_window().ui.rpmTargetFLValue.setText(str(round(msg.target_rpm, 1)))
            get_main_window().ui.rpmTargetBLValue.setText(str(round(msg.target_rpm, 1)))
            get_main_window().ui.leftPidOutValue.setText(str(msg.pid_output))
            get_main_window().ui.leftPidPValue.setText(str(round(msg.pid_tunings[0], 2)))
            get_main_window().ui.leftPidIValue.setText(str(round(msg.pid_tunings[1], 2)))
            get_main_window().ui.leftPidDValue.setText(str(round(msg.pid_tunings[2], 2)))
            get_main_window().ui.encPulseCtrsFLValue.setText(str(msg.total_enc_counts[0]))
            get_main_window().ui.encPulseCtrsBLValue.setText(str(msg.total_enc_counts[1]))

    @staticmethod
    def micro_switch_callback(msg: Range) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.sensorsTab.objectName():
            if msg.header.frame_id == RosFrameIds.MICRO_SW_SENS_BASE_FRAME_ID.format(RosNames.MICRO_SWITCH_TOPIC_NAMES[0]):
                get_main_window().ui.microswSensIndicatorFR.setStyleSheet(generate_indicator_stylesheet(msg.range == float("-inf")))
            elif msg.header.frame_id == RosFrameIds.MICRO_SW_SENS_BASE_FRAME_ID.format(RosNames.MICRO_SWITCH_TOPIC_NAMES[1]):
                get_main_window().ui.microswSensIndicatorFL.setStyleSheet(generate_indicator_stylesheet(msg.range == float("-inf")))
            elif msg.header.frame_id == RosFrameIds.MICRO_SW_SENS_BASE_FRAME_ID.format(RosNames.MICRO_SWITCH_TOPIC_NAMES[2]):
                get_main_window().ui.microswSensIndicatorBR.setStyleSheet(generate_indicator_stylesheet(msg.range == float("-inf")))
            elif msg.header.frame_id == RosFrameIds.MICRO_SW_SENS_BASE_FRAME_ID.format(RosNames.MICRO_SWITCH_TOPIC_NAMES[3]):
                get_main_window().ui.microswSensIndicatorBL.setStyleSheet(generate_indicator_stylesheet(msg.range == float("-inf")))
            else:
                get_ros_node().get_logger().error("Received micro switch data for unknown sensor.")

    @staticmethod
    def ultrasonic_sens_callback(msg: Range) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.sensorsTab.objectName():
            if msg.header.frame_id == RosFrameIds.ULTRASONIC_SENS_BASE_FRAME_ID.format(RosNames.ULTRASONIC_SENS_TOPIC_NAMES[0]):
                get_main_window().ui.ultrasonicSensValueF.setText(str(round(msg.range, 2)))
            elif msg.header.frame_id == RosFrameIds.ULTRASONIC_SENS_BASE_FRAME_ID.format(RosNames.ULTRASONIC_SENS_TOPIC_NAMES[1]):
                get_main_window().ui.ultrasonicSensValueB.setText(str(round(msg.range, 2)))
            elif msg.header.frame_id == RosFrameIds.ULTRASONIC_SENS_BASE_FRAME_ID.format(RosNames.ULTRASONIC_SENS_TOPIC_NAMES[2]):
                get_main_window().ui.ultrasonicSensValueR.setText(str(round(msg.range, 2)))
            elif msg.header.frame_id == RosFrameIds.ULTRASONIC_SENS_BASE_FRAME_ID.format(RosNames.ULTRASONIC_SENS_TOPIC_NAMES[3]):
                get_main_window().ui.ultrasonicSensValueL.setText(str(round(msg.range, 2)))
            else:
                get_ros_node().get_logger().error("Received ultrasonic sensor data for unknown sensor.")

    @staticmethod
    def cliff_sens_callback(msg: Range) -> None:
        if get_main_window().ui.pages.currentWidget().objectName() == get_main_window().ui.sensorsTab.objectName():
            if msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[0]):
                get_main_window().ui.cliffSensIndicatorF1.setStyleSheet(generate_indicator_stylesheet(msg.range == float("inf")))
            elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[1]):
                get_main_window().ui.cliffSensIndicatorF2.setStyleSheet(generate_indicator_stylesheet(msg.range == float("inf")))
            elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[2]):
                get_main_window().ui.cliffSensIndicatorF3.setStyleSheet(generate_indicator_stylesheet(msg.range == float("inf")))
            elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[3]):
                get_main_window().ui.cliffSensIndicatorF4.setStyleSheet(generate_indicator_stylesheet(msg.range == float("inf")))
            elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[4]):
                get_main_window().ui.cliffSensIndicatorB1.setStyleSheet(generate_indicator_stylesheet(msg.range == float("inf")))
            elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[5]):
                get_main_window().ui.cliffSensIndicatorB2.setStyleSheet(generate_indicator_stylesheet(msg.range == float("inf")))
            elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[6]):
                get_main_window().ui.cliffSensIndicatorB3.setStyleSheet(generate_indicator_stylesheet(msg.range == float("inf")))
            elif msg.header.frame_id == RosFrameIds.CLIFF_SENS_BASE_FRAME_ID.format(RosNames.CLIFF_SENS_TOPIC_NAMES[7]):
                get_main_window().ui.cliffSensIndicatorB4.setStyleSheet(generate_indicator_stylesheet(msg.range == float("inf")))
            else:
                get_ros_node().get_logger().error("Received cliff sensor data for unknown sensor.")


# ---- Node object ----
gui_ros_node = RosNode


# ---- Executor ----
def is_ros_node_initialized() -> bool:
    global gui_ros_node
    return isinstance(gui_ros_node, RosNode)

def get_ros_node() -> RosNode:
    global gui_ros_node
    return gui_ros_node

def ros_executor_thread(stop_thread):
    try:
        internal_context = rclpy.Context()
        rclpy.init(context=internal_context, domain_id=RosConfig.EXECUTOR_DOMAIN_ID)

        global gui_ros_node
        gui_ros_node = RosNode(context=internal_context)
        gui_ros_node.get_logger().info("Node initialized!")

        executor = MultiThreadedExecutor(context=internal_context)
        executor.add_node(gui_ros_node)
        gui_ros_node.get_logger().info("Executor initialized!")

        gui_ros_node.get_logger().info("Starting the executor...")

        while not stop_thread():
            executor.spin_once(timeout_sec=RosConfig.EXECUTOR_TIMEOUT)

        # Cleanup
        gui_ros_node.get_logger().warn("ROS thread stopping. Shutting the executor down.")
        executor.shutdown(timeout_sec=RosConfig.EXECUTOR_SHUTDOWN_TIMEOUT)
        rclpy.shutdown(context=internal_context)

    except ExternalShutdownException:
        if is_ros_node_initialized():
            get_ros_node().get_logger().warn("External shutdown exception!")
