cmake_minimum_required(VERSION 3.8)
project(ros_remote_joy)
set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
    add_link_options("-Wl,-z,relro,-z,now")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_remote_hid REQUIRED)

# Node target
set(TARGET_NAME joy_filter_component)
add_library(${TARGET_NAME} SHARED 
    src/joy_filter_node.cpp
)

target_link_libraries(${TARGET_NAME} PRIVATE
    rclcpp::rclcpp
    rclcpp_components::component
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${ros_remote_hid_TARGETS}
)
rclcpp_components_register_node(${TARGET_NAME}
    PLUGIN "JoyFilterNode"
    EXECUTABLE joy_filter_node
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
