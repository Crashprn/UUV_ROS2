from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="joy",
            executable='joy_node',
            name="joy_node",
            parameters=[
                {'default_trig_val': True,
                 "autorepeat_rate": 100.0,
                }
            ]
        )
    ])