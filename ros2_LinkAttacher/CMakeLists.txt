cmake_minimum_required(VERSION 3.10)
project(ros2_linkattacher)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(linkattacher_msgs REQUIRED)

# Gazebo Harmonic (gz-sim 8) is vendored under /opt/ros/jazzy/opt/gz_sim_vendor/
# gz_sim_vendor must be found first so it extends CMAKE_PREFIX_PATH
find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
set(GZ_SIM_TARGET gz-sim8::gz-sim8)

add_library(gz_link_attacher_plugin SHARED
  src/gz_link_attacher.cpp)

target_include_directories(gz_link_attacher_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_link_libraries(gz_link_attacher_plugin
  ${GZ_SIM_TARGET})

# gz_plugin_vendor doesn't expose INTERFACE_INCLUDE_DIRECTORIES on its cmake
# targets, so add the include path manually so GZ_ADD_PLUGIN macro is found.
target_include_directories(gz_link_attacher_plugin PRIVATE
  /opt/ros/jazzy/opt/gz_plugin_vendor/include/gz/plugin2)

ament_target_dependencies(gz_link_attacher_plugin
  rclcpp
  linkattacher_msgs)

install(TARGETS gz_link_attacher_plugin
  LIBRARY DESTINATION lib)

ament_package()
