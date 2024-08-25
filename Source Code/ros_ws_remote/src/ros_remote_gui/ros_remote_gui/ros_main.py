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
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_remote_gui.config import RosConfig


# ---- ROS Node ----
class RosNode(Node):
    def __init__(self):
        super().__init__(node_name=RosConfig.NODE_NAME, namespace=RosConfig.NODE_NAMESPACE)


# ---- Node object ----
gui_ros_node: RosNode


# ---- Executor ----
def ros_executor_thread(stop_thread):
    try:
        rclpy.init(domain_id=RosConfig.EXECUTOR_DOMAIN_ID)
        global gui_ros_node
        gui_ros_node = RosNode()
        executor = MultiThreadedExecutor()
        executor.add_node(gui_ros_node)

        while not stop_thread():
            executor.spin_once(timeout_sec=RosConfig.EXECUTOR_TIMEOUT)

        # Cleanup
        executor.shutdown(timeout_sec=RosConfig.EXECUTOR_SHUTDOWN_TIMEOUT)

    except ExternalShutdownException:
        # TODO: LOGGER
        pass
