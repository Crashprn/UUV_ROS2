cmake_minimum_required(VERSION 3.8)
project(cpp_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uuv_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(serial/include)
set(serial_SRCS 
  serial/src/serial.cc serial/include/serial/serial.h
  serial/include/serial/v8stdint.h serial/src/impl/list_ports/list_ports_linux.cc
)

list(APPEND serial_SRCS serial/src/impl/unix.cc)

add_library(serial ${serial_SRCS})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(arduino_interface src/arduino_interface.cpp)
ament_target_dependencies(
  arduino_interface
  rclcpp
  uuv_interfaces
  sensor_msgs
)
target_link_libraries(arduino_interface serial)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    arduino_interface
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
