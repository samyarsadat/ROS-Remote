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
from typing import Union
from rclpy.client import SrvTypeRequest, Client, SrvTypeResponse
from rclpy.node import Node
from rclpy.context import Context
from ros_remote_hid.srv import SetLedState, GetLedStates
from ros_remote_pui.config import RosConfig, RosNames
from sensor_msgs.msg import Joy


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
        self._joystick_pub = self.create_subscription(Joy, RosNames.JOYSTICK_TOPIC, self._joystick_msg_call, qos_profile=RosConfig.QOS_BEST_EFFORT)
        self.get_led_states_srvcl = self.create_client(GetLedStates, RosNames.GET_LED_STATES_SRV, qos_profile=RosConfig.QOS_RELIABLE)
        self.set_led_state_srvcl = self.create_client(SetLedState, RosNames.SET_LED_STATES_SRV, qos_profile=RosConfig.QOS_RELIABLE)

    def _joystick_msg_call(self, msg: Joy) -> None:
        from ros_remote_pui.remote_state import get_remote_state
        remote_state = get_remote_state()

        if msg.buttons[0] == 1 and not remote_state.left_green_right_btn_en:
            remote_state.left_green_right_btn_en = True
            remote_state._ros_signals.left_r_green_btn_press_sig.emit()
        else:
            remote_state.left_green_right_btn_en = msg.buttons[0] == 1

        # Skip button[1]. It's used as the axis swap button in firmware.

        if msg.buttons[2] == 1 and not remote_state.left_green_kd2_btn_en:
            remote_state.left_green_kd2_btn_en = True
            remote_state._ros_signals.left_l_kd2_btn_press_sig.emit()
        else:
            remote_state.left_green_kd2_btn_en = msg.buttons[2] == 1

        if msg.buttons[3] == 1 and not remote_state.left_red_kd2_btn_en:
            remote_state.left_red_kd2_btn_en = True
            remote_state._ros_signals.left_r_kd2_btn_press_sig.emit()
        else:
            remote_state.left_red_kd2_btn_en = msg.buttons[3] == 1

        if msg.buttons[4] == 1 and not remote_state.left_green_left_btn_en:
            remote_state.left_green_left_btn_en = True
            remote_state._ros_signals.left_l_green_btn_press_sig.emit()
        else:
            remote_state.left_green_left_btn_en = msg.buttons[4] == 1

        remote_state.key_sw_en = msg.buttons[5] == 1
        remote_state.left_top_sw_en = msg.buttons[6] == 1
        remote_state.e_stop_sw_en = msg.buttons[7] == 1
        remote_state.right_kd2_en = msg.buttons[8] == 1
        remote_state.right_sw_en = msg.buttons[9] == 1
        remote_state.potentiometer_val = int(msg.axes[3] * 100)


# ---- Node object ----
_pui_ros_node: RosNode | None = None


# ---- Executor ----
def is_ros_node_initialized() -> bool:
    return _pui_ros_node is not None

def get_ros_node() -> RosNode | None:
    return _pui_ros_node

def init_ros_node(context: Context) -> RosNode:
    global _pui_ros_node
    if not is_ros_node_initialized():
        _pui_ros_node = RosNode(context=context)
        _pui_ros_node.get_logger().info("Node initialized!")
    return _pui_ros_node
