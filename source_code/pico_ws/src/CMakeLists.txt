#  The ROS remote project - Firmware CMake config
#  (NOTE: THIS IS NOT THE TOP-LEVEL CMAKE FILE!)
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


# Pico firmware
add_subdirectory(pico)

# Library options
set(PICO_LOG_FREERTOS              1)
set(BRIDGE_EXEC_INTERVAL           250)
set(DIAG_PUBLISHER_DISABLE_PUBLISH 1)
set(MAX_EXECUTORS                  1)
set(MAX_PUBLISHERS                 5)
set(MAX_SERVICES                   5)
set(MAX_SUBSCRIBERS                0)
set(DIAG_FIRMWARE_HARDWARE_ID      pico-fw-${PROJECT_NAME}-v${PROJECT_VERSION})

# Custom library subdirectories
add_subdirectory(lib)