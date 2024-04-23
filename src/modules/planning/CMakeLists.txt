cmake_minimum_required(VERSION 3.22)
project(planning)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(OpenCL REQUIRED)

include_directories(
    include
    ${OpenCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/planning/planning_component.cc
    src/planning/mppi/mppi.cc
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
    OpenCL
)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    common
    OpenCL
)
ament_package()