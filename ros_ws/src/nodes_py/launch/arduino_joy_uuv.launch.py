from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nodes_py',
            executable='arduino_interface',
            name="arduino_interface"
        ),
        Node(
            package="joy_linux",
            executable='joy_linux_node',
            name="joy_node",
            parameters=[
                {'default_trig_val': True}
            ]
        )
    ])