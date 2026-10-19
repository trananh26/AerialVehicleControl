cmake_minimum_required(VERSION 3.16)
project(uwb_gz_plugin VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- ROS 2 build system ---------------------------------------------------
find_package(ament_cmake REQUIRED)

# --- Dependencies ---------------------------------------------------------
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# --- Library --------------------------------------------------------------
set(SOURCES
  src/UwbRanging.cc
  src/UwbSystem.cc
)

set(HEADERS
  include/uwb_gz_plugin/UwbRanging.hh
  include/uwb_gz_plugin/UwbSystem.hh
)

add_library(uwb_gz_plugin SHARED ${SOURCES} ${HEADERS})
target_include_directories(uwb_gz_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/uwb_gz_plugin>
)
target_link_libraries(uwb_gz_plugin PUBLIC
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
  Eigen3::Eigen
)

# --- Install --------------------------------------------------------------
install(TARGETS uwb_gz_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib/uwb_gz_plugin
  RUNTIME DESTINATION bin
)

# Standalone ROS 2 executable that bridges gz topics to ROS topics.
add_executable(uwb_gz_to_ros_bridge src/ros_gz_bridge_node.cc)
target_link_libraries(uwb_gz_to_ros_bridge
  rclcpp::rclcpp
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
ament_target_dependencies(uwb_gz_to_ros_bridge
  rclcpp
  geometry_msgs
  ros_gz_interfaces
)
install(TARGETS uwb_gz_to_ros_bridge RUNTIME DESTINATION lib/${PROJECT_NAME})

# Register the plugin directory with Gazebo Harmonic so it can be found
# automatically after `source install/setup.bash`. ament_cmake_gz would do
# this via the <gz-plugin2> export tag, but that helper is not installed in
# this workspace, so we register the hook manually.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/uwb_gz_plugin.dsv.in")
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/uwb_gz_plugin.sh.in")

install(DIRECTORY include/
  DESTINATION include/uwb_gz_plugin
)

# Install SDF/models so they are discoverable by Gazebo (GZ_SIM_RESOURCE_PATH).
install(DIRECTORY models worlds
  DESTINATION share/uwb_gz_plugin
)

ament_package()
