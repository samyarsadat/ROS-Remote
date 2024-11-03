#  The ROS remote project (Test Camera Publisher)
#  Main file - everything is ran from here
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
from time import sleep, time_ns
from cv_bridge import CvBridge
from test_camera_publisher.ros_main import ros_executor_thread, is_ros_node_initialized, get_ros_node
from test_camera_publisher.config import RosConfig, ProgramConfig
from test_camera_publisher.pattern_gen import get_camera_image, get_camera_overlay


# ---- Run the program ----
def main():
    # Start the ROS thread
    stop_ros_thread = False
    ros_thread = threading.Thread(target=ros_executor_thread, args=(lambda: stop_ros_thread, ), name=RosConfig.THREAD_NAME)
    ros_thread.start()

    # Check ROS thread liveliness
    def ros_liveliness_check() -> None:
        if not ros_thread.is_alive():
            if is_ros_node_initialized():
                get_ros_node().get_logger().fatal("The ROS thread has died! Terminating program.")
            else:
                print("The ROS thread has died! Terminating program.")
            exit(1)

    try:
        img_bridge = CvBridge()

        while True:
            frame_start_time = time_ns()
            ros_liveliness_check()

            if is_ros_node_initialized():
                cam_img, width, height = get_camera_image("FRONT CAMERA")
                get_ros_node().front_camera_comp_pub.publish(img_bridge.cv2_to_compressed_imgmsg(cam_img, "jpg"))
                get_ros_node().front_overlay_comp_pub.publish(img_bridge.cv2_to_imgmsg(get_camera_overlay(width, height), "rgba8"))

            # Framerate limiting
            sleep_time_ns = (1000000000 / ProgramConfig.TARGET_PUBLISH_FPS) - (time_ns() - frame_start_time)
            sleep((sleep_time_ns / 1000000000) if sleep_time_ns > 0 else 0)
    except KeyboardInterrupt:
        if is_ros_node_initialized():
            get_ros_node().get_logger().info("Keyboard interrupt!")

    # Shutdown
    stop_ros_thread = True
    ros_thread.join()

    if is_ros_node_initialized():
        get_ros_node().get_logger().info(f"Program exiting...")
