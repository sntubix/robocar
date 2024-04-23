cmake_minimum_required(VERSION 3.22)
project(startup)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(drivers REQUIRED)
find_package(vehicle REQUIRED)
find_package(control REQUIRED)
find_package(localization REQUIRED)
find_package(map REQUIRED)
find_package(perception REQUIRED)
find_package(planning REQUIRED)
find_package(visualization REQUIRED)
find_package(logging REQUIRED)
find_package(simulation REQUIRED)

add_executable(robocar
    src/main.cc
)

install(
    TARGETS robocar
    DESTINATION lib/${PROJECT_NAME}
)
install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(robocar
    common
    drivers
    vehicle
    control
    localization
    map
    perception
    planning
    visualization
    logging
    simulation
)
ament_package()