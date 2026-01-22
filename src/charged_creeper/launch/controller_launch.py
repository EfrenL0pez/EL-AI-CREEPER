from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='charged_creeper',
            executable='keyboard_node',
            name='keyboard_node',
            output='screen',
        ),
        Node(
            package='charged_creeper',
            executable='communication_node',
            name='communication_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baudrate': 115200
            }]
        ),
    ])
