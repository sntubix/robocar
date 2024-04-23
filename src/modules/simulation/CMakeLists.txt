cmake_minimum_required(VERSION 3.22)
project(simulation)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    include
    /usr/include/eigen3
    /usr/local/include/eigen3
)

add_library(${PROJECT_NAME}
    src/simulation/sim_vehicle_component.cc
    src/simulation/sim_objects_component.cc
    src/simulation/sim_objects_agent.cc
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
    Eigen3
)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    common
    Eigen3
)
ament_package()