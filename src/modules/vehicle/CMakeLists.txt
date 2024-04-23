cmake_minimum_required(VERSION 3.22)
project(vehicle)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)

include_directories(
    include
)

add_library(${PROJECT_NAME}
    src/vehicle/vehicle_component.cc
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
)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    common
)
ament_package()