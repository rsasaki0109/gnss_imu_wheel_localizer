cmake_minimum_required(VERSION 3.14)
project(gnss_imu_wheel_localizer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(include)

add_library(gnss_imu_wheel_localizer_component SHARED
  src/extended_kalman_filter.cpp
  src/gnss_imu_wheel_localizer_node.cpp
)

target_include_directories(gnss_imu_wheel_localizer_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gnss_imu_wheel_localizer_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

target_link_libraries(gnss_imu_wheel_localizer_component
  Eigen3::Eigen
  GeographicLib::GeographicLib
)

rclcpp_components_register_nodes(gnss_imu_wheel_localizer_component
  "gnss_imu_wheel_localizer::GnssImuWheelLocalizerNode")

add_executable(gnss_imu_wheel_localizer_node
  src/main.cpp
)

target_link_libraries(gnss_imu_wheel_localizer_node
  gnss_imu_wheel_localizer_component
)

ament_target_dependencies(gnss_imu_wheel_localizer_node
  rclcpp
)

install(TARGETS
  gnss_imu_wheel_localizer_component
  gnss_imu_wheel_localizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
