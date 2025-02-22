cmake_minimum_required(VERSION 3.8)
project(ros2_control_arduino_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  ros2_control_arduino_hw
  SHARED
  src/robo_cart_arduino_hw.cpp
)
target_include_directories(
  ros2_control_arduino_hw
  PUBLIC
  include
)
ament_target_dependencies(
  ros2_control_arduino_hw
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
# prevent pluginlib from using boost
target_compile_definitions(ros2_control_arduino_hw PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

target_link_libraries(ros2_control_arduino_hw serial)

pluginlib_export_plugin_description_file(
  hardware_interface ros2_control_arduino_hw.xml)

install(
  TARGETS
  ros2_control_arduino_hw
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ros2_control_arduino_hw
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
