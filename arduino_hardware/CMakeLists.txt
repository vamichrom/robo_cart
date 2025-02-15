cmake_minimum_required(VERSION 3.8)
project(arduino_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  rclcpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)
add_library(arduino_hardware SHARED src/arduino_hardware.cpp)
target_link_libraries(arduino_hardware PRIVATE ${SERIAL} ${CMAKE_THREAD_LIBS_INIT})
ament_target_dependencies(arduino_hardware hardware_interface rclcpp)

install(TARGETS arduino_hardware
        DESTINATION lib/${PROJECT_NAME})

pluginlib_export_plugin_description_file(arduino_hardware plugins.xml)

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

ament_package()
