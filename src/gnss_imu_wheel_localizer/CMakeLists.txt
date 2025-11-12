cmake_minimum_required(VERSION 3.14)
project(gnss_imu_wheel_localizer)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
if(EXISTS "/usr/share/cmake/geographiclib/FindGeographicLib.cmake")
  list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
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
    ${GeographicLib_INCLUDE_DIRS}
)

ament_target_dependencies(gnss_imu_wheel_localizer_component
  rclcpp
  rclcpp_components
  sensor_msgs
  nav_msgs
  geometry_msgs
  autoware_vehicle_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

target_link_libraries(gnss_imu_wheel_localizer_component
  Eigen3::Eigen
  ${GeographicLib_LIBRARIES}
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
  autoware_vehicle_msgs
)

install(TARGETS
  gnss_imu_wheel_localizer_component
  gnss_imu_wheel_localizer_node
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_extended_kalman_filter
    test/extended_kalman_filter_test.cpp)
  if(TARGET test_extended_kalman_filter)
    target_link_libraries(test_extended_kalman_filter
      gnss_imu_wheel_localizer_component)
    ament_target_dependencies(test_extended_kalman_filter
      rclcpp)
  endif()

  ament_add_gtest(test_gnss_imu_wheel_localizer_node
    test/gnss_imu_wheel_localizer_node_test.cpp)
  if(TARGET test_gnss_imu_wheel_localizer_node)
    target_link_libraries(test_gnss_imu_wheel_localizer_node
      gnss_imu_wheel_localizer_component)
    ament_target_dependencies(test_gnss_imu_wheel_localizer_node
      rclcpp)
  endif()
endif()

ament_package()
