cmake_minimum_required(VERSION 3.12)
project(ros_remote_hid LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
    add_link_options("-Wl,-z,relro,-z,now")
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(HIDAPI REQUIRED hidapi-hidraw)

# Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SetLedState.srv"
    "srv/GetLedStates.srv"
    DEPENDENCIES builtin_interfaces
)

# Node target
set(TARGET_NAME led_interface_component)
add_library(${TARGET_NAME} SHARED
    src/led_interface_node.cpp
    src/hid_helper.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_include_directories(${TARGET_NAME} PUBLIC ${HIDAPI_INCLUDE_DIRS})
target_link_directories(${TARGET_NAME} PRIVATE ${HIDAPI_LIBRARY_DIRS})
target_link_libraries(${TARGET_NAME} PRIVATE
    ${HIDAPI_LIBRARIES}
    ${cpp_typesupport_target}
    ${diagnostic_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${std_srvs_TARGETS}
    diagnostic_updater::diagnostic_updater
    rclcpp::rclcpp
    rclcpp_components::component
)
add_dependencies(${TARGET_NAME} ${cpp_typesupport_target})
rclcpp_components_register_node(${TARGET_NAME}
    PLUGIN "LEDInterfaceNode"
    EXECUTABLE led_interface_node
)

ament_export_targets(export_${TARGET_NAME})
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

install(TARGETS ${TARGET_NAME}
        EXPORT export_${TARGET_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
    set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_cpplint REQUIRED)
    find_package(ament_cmake_clang_format REQUIRED)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()