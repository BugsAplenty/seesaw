cmake_minimum_required(VERSION 3.8)
project(seesaw_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(udp_reader src/udp_reader.cpp)
ament_target_dependencies(udp_reader
  rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs 
  tf2 tf2_ros tf2_geometry_msgs
)

target_include_directories(udp_reader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS udp_reader
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  launch/seesaw_visualize.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
