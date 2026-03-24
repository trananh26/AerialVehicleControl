cmake_minimum_required(VERSION 3.5)

project(sdformat_urdf)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rcutils REQUIRED)
find_package(urdfdom_headers 1.0.6 REQUIRED)
find_package(urdf_parser_plugin REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)

# Default to Fortress
set(SDF_VER 12)

# If the user didn't specify a GZ distribution, pick the one matching the ROS distribution according to REP 2000
if(NOT DEFINED ENV{GZ_VERSION} AND DEFINED ENV{ROS_DISTRO})
  if("$ENV{ROS_DISTRO}" STREQUAL "humble")
    set(ENV{GZ_VERSION} "fortress")
  endif()
endif()

# Find libsdformat matching the picked GZ distribution
if("$ENV{GZ_VERSION}" STREQUAL "fortress")
  find_package(sdformat12 REQUIRED)
  set(SDF_VER ${sdformat12_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Fortress (libSDFormat 12)")
elseif("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(sdformat13 REQUIRED)
  set(SDF_VER ${sdformat13_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Garden (libSDFormat 13)")
elseif("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(sdformat14 REQUIRED)
  set(SDF_VER ${sdformat14_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Harmonic (libSDFormat 14)")
# No GZ distribution specified, find any version of libsdformat we can
else()
  foreach(major RANGE 13 9)
    find_package(sdformat${major} QUIET)
    if(sdformat${major}_FOUND)
      # Next `find_package` call will be a noop
      set(SDF_VER ${major})
      message(STATUS "Compiling against libSDFormat ${major}")
      break()
    endif()
  endforeach()
endif()

# Add sdformat_urdf shared library
add_library(sdformat_urdf SHARED
  src/sdformat_urdf.cpp
)
target_link_libraries(sdformat_urdf PUBLIC
    sdformat${SDF_VER}::sdformat${SDF_VER}
  urdfdom_headers::urdfdom_headers
)
target_link_libraries(sdformat_urdf PRIVATE
  rcutils::rcutils
)
target_include_directories(sdformat_urdf
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

# Add sdformat_urdf_plugin module library
# not actually a MODULE as workaround for
# https://github.com/ros/pluginlib/issues/200
add_library(sdformat_urdf_plugin SHARED
  src/sdformat_urdf_plugin.cpp
)
target_link_libraries(sdformat_urdf_plugin PRIVATE
  pluginlib::pluginlib
  rcutils::rcutils
  sdformat_urdf
  tinyxml2::tinyxml2
  urdf_parser_plugin::urdf_parser_plugin
)

ament_export_dependencies(pluginlib)
ament_export_dependencies(rcutils)
ament_export_dependencies(sdformat${SDF_VER})
ament_export_dependencies(tinyxml2)
ament_export_dependencies(urdf_parser_plugin)
ament_export_dependencies(urdfdom_headers)

install(TARGETS sdformat_urdf EXPORT sdformat_urdf-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS sdformat_urdf_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(sdformat_urdf-export HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(urdf_parser_plugin "sdformat_urdf_plugin.xml")

ament_package()
