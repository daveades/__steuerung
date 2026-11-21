from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = Path(get_package_share_directory('dofbot_description'))
    robot_description = (package_path / 'urdf' / 'dofbot.urdf').read_text()

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Open the robot model in RViz'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),
        Node(
            package='dofbot_description',
            executable='grasp_reference_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(package_path / 'rviz' / 'robot.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'),
    ])
