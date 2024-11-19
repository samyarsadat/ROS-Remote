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

import threading
import rclpy
from typing import Union
from datetime import datetime
from rclpy import Context
from rclpy.client import SrvTypeRequest, Client, SrvTypeResponse
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from remote_pico_coms.msg import ButtonStates, SwitchStates, JoystickState, PotentiometerState
from remote_pico_coms.srv import GetJoystickConfig, SetJoystickConfig, GetLedStates, SetLedStates
from ros_remote_pui.config import RosConfig, RosNames
from diagnostic_msgs.srv import SelfTest
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from ros_remote_pui.remote_state import get_remote_state


# ---- Global variables ----
_stop_ros_thread = False
_ros_thread = None


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
        self.get_logger().info("Creating subscribers, and services clients...")

        self._reentrant_cb_group = ReentrantCallbackGroup()
        self._selftest_cb_group = MutuallyExclusiveCallbackGroup()

        self._button_states_sub = self.create_subscription(ButtonStates, RosNames.BUTTON_STATES_TOPIC, self._button_states_call, RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._switch_states_sub = self.create_subscription(SwitchStates, RosNames.SWITCH_STATES_TOPIC, self._switch_states_call, RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._joystick_state_sub = self.create_subscription(JoystickState, RosNames.JOYSTICK_STATE_TOPIC, self._joystick_state_call, RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self._potentiometer_state_sub = self.create_subscription(PotentiometerState, RosNames.POTENTIOMETER_STATE_TOPIC, self._potentiometer_state_call, RosConfig.QOS_BEST_EFFORT, callback_group=self._reentrant_cb_group)
        self.get_joystick_config_srvcl = self.create_client(GetJoystickConfig, RosNames.GET_JOYSTICK_CONFIG_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.set_joystick_config_srvcl = self.create_client(SetJoystickConfig, RosNames.SET_JOYSTICK_CONFIG_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.get_led_states_srvcl = self.create_client(GetLedStates, RosNames.GET_LED_STATES_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.set_led_states_srvcl = self.create_client(SetLedStates, RosNames.SET_LED_STATES_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.run_selftest_srvcl = self.create_client(SelfTest, RosNames.RUN_SELFTEST_SRV, qos_profile=RosConfig.QOS_RELIABLE, callback_group=self._selftest_cb_group)

    def _button_states_call(self, msg: ButtonStates) -> None:
        if msg.left_red_btn:
            get_remote_state()._ros_signals.left_red_btn_press_sig.emit()
        elif msg.left_red_kd2_btn:
            get_remote_state()._ros_signals.left_l_kd2_btn_press_sig.emit()
        elif msg.left_green_kd2_btn:
            get_remote_state()._ros_signals.left_r_kd2_btn_press_sig.emit()
        elif msg.left_green_left_btn:
            get_remote_state()._ros_signals.left_l_green_btn_press_sig.emit()
        elif msg.left_green_right_btn:
            get_remote_state()._ros_signals.left_r_green_btn_press_sig.emit()

    def _switch_states_call(self, msg: SwitchStates) -> None:
        get_remote_state().key_sw_en = msg.left_key_sw
        get_remote_state().right_sw_en = msg.right_top_toggle_sw
        get_remote_state().e_stop_sw_en = msg.right_e_stop_btn
        get_remote_state().right_kd2_en = msg.right_kd2_btn
        get_remote_state().left_top_sw_en = msg.left_top_toggle_sw

    def _joystick_state_call(self, msg: JoystickState) -> None:
        get_remote_state().joystick_vals = [msg.joystick_x_axis_reading, msg.joystick_y_axis_reading]
        get_remote_state()._ros_signals.joystick_msg_recv_sig.emit()

    def _potentiometer_state_call(self, msg: PotentiometerState) -> None:
        get_remote_state().potentiometer_val = msg.potentiometer_reading


# ---- Node object ----
_pui_ros_node = RosNode


# ---- Executor ----
def is_ros_node_initialized() -> bool:
    global _pui_ros_node
    return isinstance(_pui_ros_node, RosNode)

def get_ros_node() -> RosNode:
    global _pui_ros_node
    return _pui_ros_node

def _ros_executor_thread(stop_thread):
    try:
        internal_context = rclpy.Context()
        rclpy.init(context=internal_context, domain_id=RosConfig.EXECUTOR_DOMAIN_ID)

        global _pui_ros_node
        _pui_ros_node = RosNode(context=internal_context)
        _pui_ros_node.get_logger().info("Node initialized!")

        executor = MultiThreadedExecutor(context=internal_context)
        executor.add_node(_pui_ros_node)
        _pui_ros_node.get_logger().info("Executor initialized!")

        _pui_ros_node.get_logger().info("Starting the executor...")

        while not stop_thread():
            executor.spin_once(timeout_sec=RosConfig.EXECUTOR_TIMEOUT)

        # Cleanup
        _pui_ros_node.get_logger().warn("ROS thread stopping. Shutting the executor down.")
        executor.shutdown(timeout_sec=RosConfig.EXECUTOR_SHUTDOWN_TIMEOUT)
        rclpy.shutdown(context=internal_context)

    except ExternalShutdownException:
        if is_ros_node_initialized():
            get_ros_node().get_logger().warn("External shutdown exception!")


# ROS thread control
class RemotePuiThread:
    @staticmethod
    def start_thread() -> None:
        global _stop_ros_thread
        global _ros_thread

        if _ros_thread is None:
            _ros_thread = threading.Thread(target=_ros_executor_thread, args=(lambda: _stop_ros_thread, ), name=RosConfig.THREAD_NAME)
            _ros_thread.start()

    @staticmethod
    def stop_thread(await_thread_termination: bool) -> None:
        global _stop_ros_thread
        global _ros_thread

        if not _ros_thread is None:
            if is_ros_node_initialized():
                get_ros_node().get_logger().warn("PUI thread stop request received!")

            _stop_ros_thread = True
            if await_thread_termination: _ros_thread.join()

    @staticmethod
    def is_alive() -> bool:
        global _ros_thread

        if _ros_thread:
            return _ros_thread.is_alive()
        return False
