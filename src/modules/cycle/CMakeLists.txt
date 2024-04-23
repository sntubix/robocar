cmake_minimum_required(VERSION 3.22)
project(cycle)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

install(
    DIRECTORY include/
    DESTINATION include/
)

ament_export_dependencies(
    rclcpp
    std_msgs
)
ament_export_include_directories(
    include
)
ament_package()