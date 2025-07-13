from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('servo'),
                'launch',
                'servo_launch.py'
            ])
        )
    )

    return LaunchDescription([
        servo_launch
    ])
