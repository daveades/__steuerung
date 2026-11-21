from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = Path(get_package_share_directory('dofbot_description'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Open the simulated arm in RViz'),
        Node(
            package='dofbot_sim',
            executable='simulated_arm',
            output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package_path / 'launch' / 'display.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()),
    ])
