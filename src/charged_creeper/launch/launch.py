from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_dir = FindPackageShare('charged_creeper').find('charged_creeper')

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_dir, 'launch', 'controller_launch.py'])
        ])
    )

    return LaunchDescription([
        controller_launch
    ])
