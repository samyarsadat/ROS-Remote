#  The ROS remote project (GUI package)
#  General utility functions
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


from geometry_msgs.msg import Quaternion, Vector3
from math import atan2, copysign, asin



# ---- Quaternion to Euler (radians) ----
def quat_msg_to_euler(quat: Quaternion) -> Vector3:
    angles = Vector3()

    sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z)
    cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)
    angles.x = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (quat.w * quat.y - quat.z * quat.x)
    sinp = 1.0 if sinp > 1.0 else sinp
    sinp = -1.0 if sinp < -1.0 else sinp
    angles.y = asin(sinp)

    siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
    angles.z = atan2(siny_cosp, cosy_cosp)

    return angles