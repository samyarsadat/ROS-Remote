#  The ROS remote project (GUI package)
#  Full GUI launch description
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

import launch
import launch_ros.actions
from launch.actions import RegisterEventHandler, LogInfo, DeclareLaunchArgument, EmitEvent
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_dev_arg = DeclareLaunchArgument("serial_device", default_value="/dev/ttyAMA0")
    launch_gui = launch_ros.actions.Node(package="ros_remote_gui", executable="remote_gui_node", name="remote_gui")
    launch_agent = launch_ros.actions.Node(package="micro_ros_agent", executable="micro_ros_agent", name="micro_ros_agent",
                                           arguments=["serial", "--dev", LaunchConfiguration("serial_device")])

    return launch.LaunchDescription([
        serial_dev_arg,
        launch_agent,
        RegisterEventHandler(
            OnProcessStart(
                target_action=launch_agent,
                on_start=[
                    LogInfo(msg="MicroROS agent started, starting remote GUI."),
                    launch_gui
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=launch_gui,
                on_exit=[
                    LogInfo(msg="GUI exited, shutting down."),
                    EmitEvent(event=Shutdown(reason="GUI node exited."))
                ]
            )
        ),
    ])