cmake_minimum_required(VERSION 3.22)
project(common)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(cycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(msg_interfaces REQUIRED)

include_directories(
    include
)

install(
    DIRECTORY include/
    DESTINATION include/
)

ament_export_dependencies(
    cycle
    sensor_msgs
    msg_interfaces
)
ament_export_include_directories(
    include
)
ament_package()