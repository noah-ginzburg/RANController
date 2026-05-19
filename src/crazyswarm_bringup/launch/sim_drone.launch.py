import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_crazyswarm2 = get_package_share_directory('crazyflie')

    crazyswarm2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_crazyswarm2, 'launch', 'launch.py')
        ),
        launch_arguments={
            'backend': 'sim',
            'mocap': 'False',
            'rviz': 'True',
            'gui': 'False',
            'rviz_config_file': os.path.join(
                get_package_share_directory('crazyswarm_bringup'), #modified to crazyswarm_bringup from crazyflie
                'rviz',
                'config.rviz'),
        }.items()
    )

    return LaunchDescription([
        crazyswarm2,
    ])
