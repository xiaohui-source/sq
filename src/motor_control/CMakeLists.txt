cmake_minimum_required(VERSION 3.16)
project(motor_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

# Hardware Interface Library
add_library(motor_hw_interface SHARED
  src/CANDriver.cpp
  src/MockDriver.cpp
  src/motor_control_hardware_interface.cpp
)
target_include_directories(motor_hw_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(motor_hw_interface
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libcontrolcan.so
)
ament_target_dependencies(motor_hw_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Install
install(TARGETS motor_hw_interface
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES motor_control_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(
  hardware_interface
  motor_control_plugins.xml
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_include_directories(include)
ament_export_libraries(motor_hw_interface)
ament_export_dependencies(rclcpp hardware_interface pluginlib)

ament_package()
