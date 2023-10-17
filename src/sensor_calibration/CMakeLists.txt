cmake_minimum_required(VERSION 3.8)
project(sensor_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED common io)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV 4 REQUIRED aruco)
if(OpenCV_arcuo_FOUND)
  add_definitions("-DHAVE_OPENCV_ARUCO")
endif()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(lidar_camera_calibration src/lidar_camera_calibration.cpp)
target_include_directories(lidar_camera_calibration PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_compile_features(lidar_camera_calibration PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(lidar_camera_calibration
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS lidar_camera_calibration
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_target_dependencies(lidar_camera_calibration
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  pcl_conversions
  geometry_msgs
  visualization_msgs
)

ament_package()
