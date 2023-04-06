from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nodes_py',
            executable='ArduinoInterface',
            name="arduino_interface",
            parameters=[
                {
                    'uuv_name': 'uuv1',
                 }
            ]
        ),
    ])