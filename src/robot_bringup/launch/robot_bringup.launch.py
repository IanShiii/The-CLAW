from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():
    robot_description_path = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'urdf',
        'robot.urdf'
    ])

    robot_description = Command(['cat ', robot_description_path])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('robot_description'),
        'rviz',
        'rviz_config.rviz'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='arm_controller',
            executable='arm_controller',
            name='arm_controller',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
        )
    ])
