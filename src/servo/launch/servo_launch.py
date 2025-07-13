from launch import LaunchDescription
from launch_ros.actions import Node

from constants.constants import SERVOS


def generate_launch_description():
    nodes = []
    
    for servo in SERVOS:
        nodes.append(
            Node(
                package='servo',
                executable='servo',
                name=f'servo_{servo["id"]}',
                parameters=[servo]
            )
        )

    return LaunchDescription(nodes)
