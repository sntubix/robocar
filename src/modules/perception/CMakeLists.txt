cmake_minimum_required(VERSION 3.22)
project(perception)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(common REQUIRED)
find_package(linefit_ground_segmentation REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    include
)

add_library(${PROJECT_NAME}
    src/lidar_component.cc
)
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
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
    linefit_ground_segmentation
    Eigen3
    PCL
    pcl_conversions
    OpenCV
)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    common
    linefit_ground_segmentation
    Eigen3
    PCL
    pcl_conversions
    OpenCV
)
ament_package()