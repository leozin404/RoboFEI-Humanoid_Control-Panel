cmake_minimum_required(VERSION 3.8)
project(stm_communication_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

# LibSerial does not always ship a CMake config package on every distro;
# pkg-config is the more portable lookup path (works with the Ubuntu
# libserial-dev package used on the NUC).
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

include_directories(
  include
  ${LIBSERIAL_INCLUDE_DIRS}
)

add_executable(stm_communication_node
  src/stm_communication_node.cpp
)

ament_target_dependencies(stm_communication_node
  rclcpp
  custom_interfaces
)

target_link_libraries(stm_communication_node
  ${LIBSERIAL_LIBRARIES}
)

target_compile_options(stm_communication_node PUBLIC ${LIBSERIAL_CFLAGS_OTHER})

install(TARGETS
  stm_communication_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

