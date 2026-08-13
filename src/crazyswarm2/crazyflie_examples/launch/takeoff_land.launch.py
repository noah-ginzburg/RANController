import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    example_node = Node(
        package='crazyflie_examples',
        executable='goto_node',
        name='goto_node',
        parameters=[{
        }]
    )

    return LaunchDescription([
        example_node
    ])
