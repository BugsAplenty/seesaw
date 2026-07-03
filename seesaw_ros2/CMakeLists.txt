cmake_minimum_required(VERSION 3.8)
project(seesaw_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(udp_reader src/udp_reader.cpp)
target_compile_features(udp_reader PUBLIC cxx_std_17)

ament_target_dependencies(udp_reader
  rclcpp
  sensor_msgs
)

target_link_libraries(udp_reader Eigen3::Eigen)

install(TARGETS
  udp_reader
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scan_to_cloud.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
