#  The ROS remote project (GUI package)
#  Empty package init file
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

PROGRAM_VERSION = "2024.8.8"


# ---- Program Info ----
class ProgramInfoConfig:
    VERSION = PROGRAM_VERSION
    VERSION_DATE = "2024/08/08 @ 12:15PM UTC"


# ---- ROS Config ----
class RosConfig:
    NODE_NAME = "remote_gui_node"
    NODE_NAMESPACE = ""
    EXECUTOR_DOMAIN_ID = None
    EXECUTOR_TIMEOUT = 0.05         # 50ms
    EXECUTOR_SHUTDOWN_TIMEOUT = 5   # 5s


class RosNames:
    # Main tab
    SET_CAMERA_LED_SRV_NAME = ""
