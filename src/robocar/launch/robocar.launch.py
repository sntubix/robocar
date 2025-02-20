from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    robocar = Node(
        package="robocar",
        executable="robocar",
        arguments=["src/robocar/config/robocar.json"]
    )

    tfl_detector = Node(
        package="tfl_detector",
        executable="tfl_detector",
        arguments=["src/tfl_detector/best_m.pt"]
    )

    return LaunchDescription([
        robocar,
        tfl_detector
    ])
