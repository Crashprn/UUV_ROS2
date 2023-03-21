from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    cpp_nodes_path = get_package_share_directory('cpp_nodes')
    default_rviz_config_path = cpp_nodes_path + "/rviz/uuv_display.rviz"
    print(default_rviz_config_path)
    rviz_arg = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=str(default_rviz_config_path),
        description='Full path to the RVIZ config file to use'
    )
    return LaunchDescription([
        rviz_arg,
        Node(
            package='cpp_nodes',
            executable='uuv_broadcaster',
            name="PoseBroadcaster",
            parameters=[
                {'uuv_name': 'uuv1'}
            ]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration('rviz_config_file')]
        )
    ])