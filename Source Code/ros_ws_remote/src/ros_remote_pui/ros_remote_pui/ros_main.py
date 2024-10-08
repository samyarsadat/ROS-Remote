#  The ROS remote project (PUI package)
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
from rclpy import Context
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from remote_pico_coms.msg import ButtonStates, SwitchStates, JoystickState, PotentiometerState
from remote_pico_coms.srv import GetJoystickConfig, SetJoystickConfig, GetLedStates, SetLedStates
from ros_remote_pui.config import RosConfig, RosFrameIds, RosNames
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.srv import SelfTest
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import BatteryState, Imu, Temperature, RelativeHumidity, Range, CompressedImage, Image
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
from ros_robot_msgs.srv import SetCameraLeds, GetCameraLeds, SetPidTunings, RunCalibrationsA, GetBool
from ros_robot_msgs.msg import MotorCtrlState


# ---- ROS Node ----
class RosNode(Node):
    def __init__(self, context: Context):
        super().__init__(node_name=RosConfig.NODE_NAME, namespace=RosConfig.NODE_NAMESPACE, context=context)
        self.get_logger().info("Creating publishers, subscribers, and services servers...")

        self._reentrant_cb_group = ReentrantCallbackGroup()
        self._selftest_cb_group = MutuallyExclusiveCallbackGroup()

        self._button_states_sub = self.create_subscription(ButtonStates, RosNames.BUTTON_STATES_TOPIC, None, RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._switch_states_sub = self.create_subscription(SwitchStates, RosNames.SWITCH_STATES_TOPIC, None, RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._joystick_states_sub = self.create_subscription(JoystickState, RosNames.JOYSTICK_STATE_TOPIC, None, RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._potentiometer_states_sub = self.create_subscription(PotentiometerState, RosNames.POTENTIOMETER_STATE_TOPIC, None, RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.get_joystick_config_srvcl = self.create_client(GetJoystickConfig, RosNames.GET_JOYSTICK_CONFIG_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.set_joystick_config_srvcl = self.create_client(SetJoystickConfig, RosNames.SET_JOYSTICK_CONFIG_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.get_led_states_srvcl = self.create_client(GetLedStates, RosNames.GET_LED_STATES_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.set_led_states_srvcl = self.create_client(SetLedStates, RosNames.SET_LED_STATES_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.run_selftest_srvcl = self.create_client(SelfTest, RosNames.RUN_SELFTEST_SRV, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._selftest_cb_group)


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
