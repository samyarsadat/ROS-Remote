#  The ROS remote project - Top Level CMake config
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


# Set minimum required version of CMake
cmake_minimum_required(VERSION 3.12)

# Set Pico Board and Pico Platform
set(PICO_PLATFORM rp2040)
set(PICO_BOARD pico)

# Include build functions from the Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Set project name and C/C++ standards
string(TIMESTAMP DATEVER "%Y.%m.%d" UTC)
project(ROS_Remote VERSION ${DATEVER} LANGUAGES C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

# Initialize the Pico SDK
pico_sdk_init()

# Include the FreeRTOS Kernel
set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/libfreertos/FreeRTOS-Kernel" CACHE STRING "FreeRTOS Kernel Path" FORCE)
include(FreeRTOS_Kernel_import.cmake)

# FreeRTOS config
set(FREERTOS_CONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/libfreertos/Config" CACHE STRING "FreeRTOS Config Path" FORCE)
include_directories("${FREERTOS_CONFIG_DIR}") 

# Add our source directory
add_subdirectory(src)

# C/C++ flags
set(FLAGS "-mthumb -ffunction-sections -fdata-sections")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${FLAGS}")

message("Configuration completed! (${PROJECT_NAME} - v${PROJECT_VERSION})")