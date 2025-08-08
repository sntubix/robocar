import json
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = []

    conf_filename = "src/robocar/config/robocar.json" 
    with open(conf_filename, 'r') as file:
        launch_group = json.load(file)['robocar']['global']['launch_group']

    nodes.append(
        Node(
            package="robocar",
            executable="robocar",
            arguments=[conf_filename]
        )
    )

    if launch_group == "default":
        nodes.append(
            Node(
                package="robocar_tfl_detector",
                prefix=['/venv/bin/python3'],
                executable="robocar_tfl_detector",
                arguments=["src/robocar_tfl_detector/best_m.pt"]
            )
        )

    return LaunchDescription(nodes)
