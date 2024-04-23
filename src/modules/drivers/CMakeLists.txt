cmake_minimum_required(VERSION 3.22)
project(drivers)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(oscc REQUIRED)
find_package(ouster REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(SDL2 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig)
pkg_check_modules(libserial libserial REQUIRED)
pkg_check_modules(libgps libgps REQUIRED)

include_directories(
    include
)

add_library(${PROJECT_NAME}
    src/actuator/actuator_component.cc
    src/alexa/alexa_component.cc
    src/camera/camera_component.cc
    src/gnss/gsof_component.cc
    src/joystick/joystick_component.cc
    src/lidar/ouster_component.cc
)
target_link_libraries(${PROJECT_NAME}
    ${libserial_LIBRARIES}
    ${libgps_LIBRARIES}
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
    oscc
    ouster
    pcl_conversions
    cv_bridge
    SDL2
    PCL
    OpenCV
)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    common
    oscc
    ouster
    pcl_conversions
    cv_bridge
    SDL2
    PCL
    OpenCV
)
ament_export_include_directories(
    ${libserial_INCLUDE_DIRS}
    ${libgps_INCLUDE_DIRS}
)
ament_package()