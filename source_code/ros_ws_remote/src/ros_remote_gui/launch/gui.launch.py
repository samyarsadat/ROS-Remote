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

from launch_ros.actions import Node, PushROSNamespace
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent, IncludeLaunchDescription
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_gui = Node(
        package="ros_remote_gui", 
        executable="remote_gui_node"
    )

    ros_remote_pui_pkg = FindPackageShare("ros_remote_pui")
    launch_pui_drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_remote_pui_pkg, "launch", "pui.launch.py"])
        )
    )

    return LaunchDescription([
        PushROSNamespace("remote"),
        launch_pui_drivers,
        launch_gui,
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