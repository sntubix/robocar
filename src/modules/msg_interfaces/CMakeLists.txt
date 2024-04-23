cmake_minimum_required(VERSION 3.22)
project(msg_interfaces)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "msg/ActCmd.msg"
    "msg/ActStatus.msg"
    "msg/ActToggle.msg"
    "msg/AdToggle.msg"
    "msg/GNSS.msg"
    "msg/Localization.msg"
    "msg/LogEntry.msg"
    "msg/Mapping.msg"
    "msg/MapWaypoint.msg"
    "msg/Object2d.msg"
    "msg/Object3d.msg"
    "msg/Objects2d.msg"
    "msg/Objects3d.msg"
    "msg/ObjectsCircles.msg"
    "msg/Path.msg"
    "msg/Planning.msg"
    "msg/TargetSpeed.msg"
    "msg/TrafficLight.msg"
    "msg/Trajectory.msg"
    "msg/Vector3d.msg"
    "msg/Vehicle.msg"
    "msg/Waypoint.msg"
    DEPENDENCIES builtin_interfaces std_msgs
)

ament_package()