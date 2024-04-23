cmake_minimum_required(VERSION 3.22)
project(visualization)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_common REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

include_directories(
    include
)

add_library(${PROJECT_NAME}
    src/visualization_component.cc
    include/visualization/dashboard_component.h
    src/dashboard_component.cc
)

install(
    DIRECTORY include/
    DESTINATION include/
)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
)

ament_target_dependencies(${PROJECT_NAME}
    common
    visualization_msgs
    geometry_msgs
    cv_bridge
    tf2
    tf2_ros
    rviz2
    rviz_common
    Qt5
)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    common
    visualization_msgs
    geometry_msgs
    cv_bridge
    tf2
    tf2_ros
    rviz2
    rviz_common
    Qt5
)
ament_package()