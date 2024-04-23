cmake_minimum_required(VERSION 3.22)
project(linefit_ground_segmentation)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
find_package(Eigen3 REQUIRED)

include_directories(
    include
)

add_library(${PROJECT_NAME}
    src/ground_segmentation.cc
    src/segment.cc
    src/bin.cc
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

ament_target_dependencies(${PROJECT_NAME} PCL Eigen3)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(PCL Eigen3)
ament_package()