cmake_minimum_required(VERSION 3.8)
project(flight_perception_cpp CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(min_depth src/min_depth.cpp)
ament_target_dependencies(min_depth rclcpp sensor_msgs std_msgs)

add_executable(optical_flow_logger src/optical_flow_logger.cpp)
ament_target_dependencies(optical_flow_logger rclcpp px4_msgs)

install(TARGETS min_depth optical_flow_logger DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
