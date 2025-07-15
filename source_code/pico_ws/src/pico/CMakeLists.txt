#  The ROS remote project - Pico firmware CMake config 
#  (NOTE: THIS IS NOT THE TOP-LEVEL CMAKE FILE!)
#  Copyright 2024-2025 Samyar Sadat Akhavi.
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


# Target name
set(TARGET_NAME "${PROJECT_NAME}_Pico")
message("Configuring Pico firmware build... (${TARGET_NAME})")

# Add source files
add_executable(${TARGET_NAME} main.cpp
                              posix.c
                              io/buttons.cpp
                              io/joystick.cpp
                              io/leds.cpp
                              uros/uros_init.cpp
                              uros/service_handlers.cpp
                              uros/sensor_publishers.cpp
                              selftest/leds.cpp)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${TARGET_NAME})

# Link to libraries
target_link_libraries(${TARGET_NAME} pico_stdlib
                                     hardware_pwm
                                     hardware_adc
                                     pico_stdio_usb
                                     pico_stdio_uart
                                     cmsis_core
                                     FreeRTOS-Kernel
                                     FreeRTOS-Kernel-Heap4
                                     micro_ros
                                     utils_lib
                                     uros_utils_lib
                                     freertos_agent_lib
                                     uros_freertos_abstract_lib
                                     pico_log_lib
                                     cpp_freertos_alloc_lib
)

# Link to include directories
target_include_directories(${TARGET_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Target-specific compile definitions
target_compile_definitions(${TARGET_NAME} PRIVATE
    PICO_RP2040_B0_SUPPORTED=0
    PICO_STACK_SIZE=0x1000
    PICO_USE_STACK_GUARDS=0
    PICO_USE_FASTEST_SUPPORTED_CLOCK=1
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=16
    PICO_DEFAULT_UART_RX_PIN=17
    PICO_DEFAULT_UART_BAUD_RATE=921600
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
    AGENT_STATE_MACHINE_EXEC_INTERVAL_MS=250
    MAX_EXECUTORS=1
    MAX_PUBLISHERS=5
    MAX_SERVICES=5
    MAX_SUBSCRIBERS=0
)

# PicoTool binary information
pico_set_program_name(${TARGET_NAME} ${TARGET_NAME})
pico_set_program_version(${TARGET_NAME} ${PROJECT_VERSION})
pico_set_program_description(${TARGET_NAME} "The ROS Remote Project - Main Pico Firmware")

# Enable STDIO over USB and UART
pico_enable_stdio_usb(${TARGET_NAME} 1)
pico_enable_stdio_uart(${TARGET_NAME} 1)

# Enable all/pedantic warnings
target_compile_options(${TARGET_NAME} PRIVATE -Wall -Wextra -Wpedantic)