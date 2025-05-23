#  The ROS remote project (Test Camera Publisher)
#  Empty package init file
#  Copyright 2024-2025 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2024-2025.
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

PROGRAM_VERSION = "2025.1.18"
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, LivelinessPolicy
import cv2


# ---- Program Info ----
class ProgramInfoConfig:
    VERSION = PROGRAM_VERSION
    VERSION_DATE = "2025-01-18 @ 16:17 UTC"


class ProgramConfig:
    BASE_TEST_PATTERN = "test_images/pm5644.png"   # Options: pm5544.png, pm5644.png (widescreen)
    PM5644_MODE = True
    ENABLE_MOVING_SQUARES = True
    TARGET_PUBLISH_FPS = 24
    TARGET_WIDTH = 800
    TARGET_HEIGHT = 450
    CAMERA_NAME = "Front Camera"
    CV_TEXT_FONT = cv2.FONT_HERSHEY_SIMPLEX


# ---- ROS Config ----
class RosConfig:
    NODE_NAME = "test_image_publisher_node"
    NODE_NAMESPACE = ""
    EXECUTOR_DOMAIN_ID = None
    EXECUTOR_TIMEOUT = 0.05         # 50ms
    EXECUTOR_SHUTDOWN_TIMEOUT = 5   # 5s
    QOS_BEST_EFFORT = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)
    QOS_RELIABLE = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1, liveliness=LivelinessPolicy.AUTOMATIC)
    THREAD_NAME = "ros_thread"


class RosNames:
    ROBOT_NAMESPACE = "ros_robot"
    CAMERA_FEED_TOPIC = f"{ROBOT_NAMESPACE}/cameras/front/image_raw/compressed"
    CAMERA_OVERLAY_TOPIC = f"{ROBOT_NAMESPACE}/cameras/front_overlay/image"
