#  The ROS remote project (Test Camera Publisher)
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
from rclpy import Context
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_remote_gui.config import RosConfig, RosNames
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import CompressedImage, Image


# ---- ROS Node ----
class RosNode(Node):
    def __init__(self, context: Context):
        super().__init__(node_name=RosConfig.NODE_NAME, namespace=RosConfig.NODE_NAMESPACE, context=context)
        self.get_logger().info("Creating publishers...")
        self._reent_cb_group = ReentrantCallbackGroup()

        # Viewport camera & camera overlay
        self.front_camera_comp_pub = self.create_publisher(CompressedImage, RosNames.CAMERA_FEED_TOPIC, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reent_cb_group)
        self.front_overlay_comp_pub = self.create_publisher(Image, RosNames.CAMERA_OVERLAY_TOPIC, qos_profile=RosConfig.QOS_BEST_EFFORT, callback_group=self._reent_cb_group)


# ---- Node object ----
ros_node = RosNode


# ---- Executor ----
def is_ros_node_initialized() -> bool:
    global ros_node
    return isinstance(ros_node, RosNode)

def get_ros_node() -> RosNode:
    global ros_node
    return ros_node

def ros_executor_thread(stop_thread):
    try:
        internal_context = rclpy.Context()
        rclpy.init(context=internal_context, domain_id=RosConfig.EXECUTOR_DOMAIN_ID)

        global ros_node
        ros_node = RosNode(context=internal_context)
        ros_node.get_logger().info("Node initialized!")

        executor = MultiThreadedExecutor(context=internal_context)
        executor.add_node(ros_node)
        ros_node.get_logger().info("Executor initialized!")

        ros_node.get_logger().info("Starting the executor...")

        while not stop_thread():
            executor.spin_once(timeout_sec=RosConfig.EXECUTOR_TIMEOUT)

        # Cleanup
        ros_node.get_logger().warn("ROS thread stopping. Shutting the executor down.")
        executor.shutdown(timeout_sec=RosConfig.EXECUTOR_SHUTDOWN_TIMEOUT)
        rclpy.shutdown(context=internal_context)

    except ExternalShutdownException:
        if is_ros_node_initialized():
            get_ros_node().get_logger().warn("External shutdown exception!")
