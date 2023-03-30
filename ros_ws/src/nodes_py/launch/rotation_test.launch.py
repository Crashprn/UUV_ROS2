from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nodes_py',
            executable='RotationTester',
            name="rotation_tester",
            parameters=[
                {'uuv_name': 'uuv1',
                 'pwm_val': 1,
                 'run_time': 5,
                 }
            ]
        ),

    ])