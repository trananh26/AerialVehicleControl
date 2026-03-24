cmake_minimum_required(VERSION 3.5)

project(ros_gz_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# TODO(CH3): Deprecated. Remove on tock.
if("$ENV{GZ_VERSION}" STREQUAL "" AND NOT "$ENV{IGNITION_VERSION}" STREQUAL "")
  message(DEPRECATION "Environment variable [IGNITION_VERSION] is deprecated. Use [GZ_VERSION] instead.")
  set(ENV{GZ_VERSION} $ENV{IGNITION_VERSION})
endif()

# Edifice
if("$ENV{GZ_VERSION}" STREQUAL "edifice")
  find_package(ignition-transport10 REQUIRED)
  find_package(ignition-msgs7 REQUIRED)

  set(GZ_TARGET_PREFIX ignition)
  set(GZ_MSGS_VER ${ignition-msgs7_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${ignition-transport10_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Edifice")
# Garden
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(gz-transport12 REQUIRED)
  find_package(gz-msgs9 REQUIRED)

  set(GZ_TARGET_PREFIX gz)
  set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Garden")
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)

  set(GZ_TARGET_PREFIX gz)
  set(GZ_MSGS_VER ${gz-msgs10_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Harmonic")
# Default to Fortress
else()
  find_package(ignition-transport11 REQUIRED)
  find_package(ignition-msgs8 REQUIRED)

  set(GZ_TARGET_PREFIX ignition)
  set(GZ_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

  message(STATUS "Compiling against Ignition Fortress")
endif()

set(GZ_MSGS_VERSION_MAJOR ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_MAJOR})
set(GZ_MSGS_VERSION_MINOR ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_MINOR})
set(GZ_MSGS_VERSION_PATCH ${${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}_VERSION_PATCH})
set(GZ_MSGS_VERSION_FULL ${GZ_MSGS_VERSION_MAJOR}.${GZ_MSGS_VERSION_MINOR}.${GZ_MSGS_VERSION_PATCH})

set(BRIDGE_MESSAGE_TYPES
  builtin_interfaces
  actuator_msgs
  geometry_msgs
  gps_msgs
  nav_msgs
  rcl_interfaces
  ros_gz_interfaces
  rosgraph_msgs
  sensor_msgs
  std_msgs
  tf2_msgs
  trajectory_msgs
  vision_msgs
)

set(generated_path "${CMAKE_BINARY_DIR}/generated")
set(generated_files "${generated_path}/get_factory.cpp")
list(APPEND generated_files "${generated_path}/get_mappings.cpp")
set(convert_files "src/convert/utils.cpp")

foreach(package_name ${BRIDGE_MESSAGE_TYPES})
  find_package(${package_name} QUIET REQUIRED)
  message(STATUS "Found ${package_name}: ${${package_name}_VERSION} (${${package_name}_DIR})")

  list(APPEND generated_files "${generated_path}/factories/${package_name}.cpp")
  list(APPEND generated_files "${generated_path}/factories/${package_name}.hpp")

  list(APPEND convert_files "src/convert/${package_name}.cpp")
endforeach()

set(target_dependencies
  "bin/ros_gz_bridge_generate_factories"
  "resource/get_factory.cpp.em"
  "resource/get_mappings.cpp.em"
  "resource/pkg_factories.cpp.em"
  "resource/pkg_factories.hpp.em"
  "ros_gz_bridge/__init__.py"
  "ros_gz_bridge/mappings.py")

find_package(Python3 REQUIRED COMPONENTS Interpreter)

add_custom_command(
  OUTPUT ${generated_files}
  COMMAND Python3::Interpreter
  ARGS bin/ros_gz_bridge_generate_factories
    --output-path "${generated_path}" --template-dir resource
    --gz-msgs-ver "${GZ_MSGS_VERSION_FULL}"
  DEPENDS ${target_dependencies}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating factories for interface types")

set(bridge_lib
  ros_gz_bridge
)

add_library(${bridge_lib}
  SHARED
  src/bridge_config.cpp
  src/bridge_handle.cpp
  src/bridge_handle_ros_to_gz.cpp
  src/bridge_handle_gz_to_ros.cpp
  src/factory_interface.cpp
  src/service_factory_interface.cpp
  src/service_factories/ros_gz_interfaces.cpp
  src/ros_gz_bridge.cpp
  ${convert_files}
  ${generated_files}
)

target_link_libraries(${bridge_lib}
  ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
)

ament_target_dependencies(${bridge_lib}
  rclcpp
  rclcpp_components
  yaml_cpp_vendor
  ${BRIDGE_MESSAGE_TYPES}
)

target_include_directories(${bridge_lib}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>"
)

target_link_libraries(${bridge_lib}
  ${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER}::core
  ${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER}::core
)

rclcpp_components_register_node(
  ${bridge_lib}
  PLUGIN ros_gz_bridge::RosGzBridge
  EXECUTABLE bridge_node)

install(TARGETS ${bridge_lib} EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

set(bridge_executables
  parameter_bridge
  static_bridge
  bridge_types
)

foreach(bridge ${bridge_executables})
  add_executable(${bridge}
    src/${bridge}.cpp
  )
  target_link_libraries(${bridge}
    ${bridge_lib}
  )
  ament_target_dependencies(${bridge}
    "rclcpp"
    ${BRIDGE_MESSAGE_TYPES}
  )
  install(TARGETS ${bridge}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${bridge_lib})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

# specific order: dependents before dependencies
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(${GZ_TARGET_PREFIX}-msgs${GZ_MSGS_VER})
ament_export_dependencies(${GZ_TARGET_PREFIX}-transport${GZ_TRANSPORT_VER})
ament_export_dependencies(yaml_cpp_vendor)
ament_export_dependencies(${BRIDGE_MESSAGE_TYPES})

ament_package()
