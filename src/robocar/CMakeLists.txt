cmake_minimum_required(VERSION 3.22)
project(robocar)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenCL REQUIRED)
find_library(ZeroMQ zmq REQUIRED)
find_package(SDL2 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(libserial libserial REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)

find_package(oscc REQUIRED)
find_package(ouster REQUIRED)
find_package(linefit_ground_segmentation REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robocar_msgs REQUIRED)

include_directories(
    include
)

add_executable(robocar
    src/main.cc
    src/drivers/actuator/actuator_component.cc
    src/drivers/alexa/alexa_component.cc
    src/drivers/camera/camera_component.cc
    src/drivers/gnss/gsof_component.cc
    src/drivers/joystick/joystick_component.cc
    src/drivers/lidar/ouster_component.cc
    src/tod/tod_component.cc
    src/control/control_component.cc
    src/localization/localization_component.cc
    src/logging/logging_component.cc
    src/map/map_component.cc
    src/map/mapping_component.cc
    src/perception/lidar_perception_component.cc
    src/planning/planning_component.cc
    src/planning/mppi/mppi.cc
    src/simulation/sim_vehicle_component.cc
    src/simulation/sim_objects_component.cc
    src/simulation/sim_objects_agent.cc
    src/vehicle/vehicle_component.cc
    include/robocar/visualization/dashboard_component.h
    src/visualization/dashboard_component.cc
    src/visualization/visualization_component.cc
)
target_link_libraries(robocar
    ${libserial_LIBRARIES}
    zmq
)

install(
    TARGETS robocar
    DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(robocar
    rclcpp
    Eigen3
    PCL
    OpenCV
    OpenCL
    SDL2
    Qt5
    #
    cv_bridge
    pcl_conversions
    tf2
    tf2_ros
    rviz2
    rviz_common
    #
    oscc
    ouster
    linefit_ground_segmentation
    #
    std_msgs
    sensor_msgs
    visualization_msgs
    geometry_msgs
    robocar_msgs
)
ament_package()