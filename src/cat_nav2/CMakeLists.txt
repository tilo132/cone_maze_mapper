cmake_minimum_required(VERSION 3.8)
project(cat_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

#
# CAT_NAV2 LAYERS
#

set(layer_lib_name ${PROJECT_NAME}_layers)

find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cat_msgs_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${layer_lib_name} SHARED
  src/wall_layer.cpp
)
include_directories(include)

install(
  TARGETS ${layer_lib_name}
  DESTINATION lib
)

pluginlib_export_plugin_description_file(
  nav2_costmap_2d
  layers/wall_layer.xml
)
set(
  dep_pkgs
  rclcpp
  nav2_costmap_2d
  nav2_util
  pluginlib
  cat_msgs_ros
  geometry_msgs
  tf2
  tf2_ros
)
ament_target_dependencies(${layer_lib_name} ${dep_pkgs})

#
# NAV2 PLANNER
#

find_package(nav2_common REQUIRED)
find_package(nav2_theta_star_planner REQUIRED)
find_package(nav2_smoother REQUIRED)

#
# WallServer
#

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)

nav2_package()

include_directories(
  include
)

set(executable_name wall_server)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2_ros
  nav2_costmap_2d
  pluginlib
  nav2_core
)

add_library(${library_name} SHARED
  src/wall_server.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "cat_nav2::WallServer")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

#
# GENERAL
#

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

#install(
#  DIRECTORY rviz/
#  DESTINATION share/${PROJECT_NAME}/rviz
#)

install(
  DIRECTORY layers/
  DESTINATION share/${PROJECT_NAME}/layers
)

install(
  DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#
# WallLayer
#

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
