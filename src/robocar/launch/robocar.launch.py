import json
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    with open("src/robocar/config/robocar.json", 'r') as file:
        launch_group = json.load(file)['robocar']['global']['launch_group']

    nodes.append(
        Node(
            package="robocar",
            executable="robocar",
            arguments=["src/robocar/config/robocar.json"]
        )
    )

    if launch_group == "default":
        nodes.append(
            Node(
                package="robocar_tfl_detector",
                executable="robocar_tfl_detector",
                arguments=["src/robocar_tfl_detector/best_m.pt"]
            )
        )

    if launch_group == "tod":
        nodes.append(
            Node(
                package="robocar_tod_stream",
                executable="robocar_tod_stream"
            )
        )

    return LaunchDescription(nodes)
