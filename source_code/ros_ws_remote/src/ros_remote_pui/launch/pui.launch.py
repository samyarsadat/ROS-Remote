#  The ROS remote project (PUI package)
#  Joystick and LED driver launch file
#  Copyright 2025 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2025.
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
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
package_name = "ros_remote_pui"


def generate_launch_description():
    drv_config = "remote_drvs_conf.yaml"
    drv_config_path = PathJoinSubstitution([FindPackageShare(package_name), "config", drv_config])

    joystick_dev_arg = DeclareLaunchArgument("joystick_device", default_value="/dev/input/js0")
    joystick_dev = LaunchConfiguration("joystick_device")

    launch_joy_linux = Node(
        package="joy_linux",
        executable="joy_linux_node",
        parameters=[
            drv_config_path,
            {"dev": joystick_dev}
        ],
        output="screen"
    )

    launch_led_driver = Node(
        package="ros_remote_hid",
        executable="led_interface_node",
        parameters=[
            drv_config_path
        ],
        output="screen"
    )

    launch_joy_filter = Node(
        package="ros_remote_joy",
        executable="joy_filter_node",
        parameters=[
            drv_config_path
        ],
        remappings=[
            ("cmd_vel_joy", "cmd_vel")
        ],
        output="screen"
    )

    return launch.LaunchDescription([
        joystick_dev_arg,
        launch_joy_linux,
        launch_led_driver,
        launch_joy_filter
    ])