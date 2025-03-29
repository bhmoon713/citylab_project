from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to your RViz config file
    rviz_config_path = os.path.join(
        get_package_share_directory('robot_patrol'),
        'rviz',
        'patrol.rviz'
    )

    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='robot_patrol_node',
            name='robot_patrol_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
