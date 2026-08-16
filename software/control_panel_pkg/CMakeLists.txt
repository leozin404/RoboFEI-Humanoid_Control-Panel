cmake_minimum_required(VERSION 3.8)
project(control_panel_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

include_directories(include)

add_executable(usb_monitor_node src/usb_monitor_node.cpp)
ament_target_dependencies(usb_monitor_node rclcpp std_msgs sensor_msgs)

add_executable(system_status_node src/system_status_node.cpp)
ament_target_dependencies(system_status_node rclcpp std_msgs custom_interfaces)

add_executable(panel_teleop_node src/panel_teleop_node.cpp)
ament_target_dependencies(panel_teleop_node rclcpp std_msgs custom_interfaces)

install(TARGETS
  usb_monitor_node
  system_status_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS
  usb_monitor_node
  system_status_node
  panel_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

