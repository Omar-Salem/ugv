cmake_minimum_required(VERSION 3.8)
project(ugv_chassis_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ugv_interfaces REQUIRED)

set(HW_INTERFACE_LIB "ugv_chassis_firmware")

add_library(
  ${HW_INTERFACE_LIB}
  SHARED
  src/ugv_chassis_hardware.cpp
)
target_include_directories(
  ${HW_INTERFACE_LIB}
  PUBLIC
  include
)
ament_target_dependencies(
  ${HW_INTERFACE_LIB}
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  ugv_interfaces
)
# prevent pluginlib from using boost
target_compile_definitions(${HW_INTERFACE_LIB} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ${HW_INTERFACE_LIB}.xml)

install(
  TARGETS
  ${HW_INTERFACE_LIB}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_ugv_chassis_hardware test/test_ugv_chassis_hardware.cpp)
  target_include_directories(test_ugv_chassis_hardware PRIVATE include)
  ament_target_dependencies(
    test_ugv_chassis_hardware
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${HW_INTERFACE_LIB}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
