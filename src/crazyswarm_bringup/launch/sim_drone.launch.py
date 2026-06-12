import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def launch_controllers(context, *args, **kwargs):
    names_str = LaunchConfiguration('drone_names').perform(context)
    ran_str = LaunchConfiguration('ran_drones').perform(context)
    drone_names = [name.strip() for name in names_str.split(',') if name.strip()]
    ran_drones = [name.strip() for name in ran_str.split(',') if name.strip()]

    actions = []
    for name in drone_names:
        actions.append(Node(
            package='crazyflie_controller',
            executable='controller_server',
            name=f'controller_server_{name}',
            output='screen',
            parameters=[{'drone_name': name}, {'use_sim_odom': True}],
        ))
        if name in ran_drones:
            actions.append(Node(
                package='spherical_ran',
                executable='spherical_RAN_server',
                name=f'spherical_RAN_server_{name}',
                output='screen',
                parameters=[{'drone_name': name}, {'all_drones': drone_names}],
            ))

    return actions


def generate_launch_description():
    drone_names_arg = DeclareLaunchArgument('drone_names', default_value='cf01,cf02,cf03')
    ran_drones_arg = DeclareLaunchArgument('ran_drones', default_value='cf01')

    pkg_crazyswarm2 = get_package_share_directory('crazyflie')

    crazyswarm2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_crazyswarm2, 'launch', 'launch.py')
        ),
        launch_arguments={
            'backend': 'sim',
            'mocap': 'False',
            'rviz': 'False',
            'gui': 'False',
            'rviz_config_file': os.path.join(
                get_package_share_directory('crazyswarm_bringup'),
                'rviz',
                'config.rviz'),
        }.items()
    )

    crazyflie_controllers = OpaqueFunction(function=launch_controllers)

    return LaunchDescription([
        drone_names_arg,
        ran_drones_arg,
        crazyswarm2,
        TimerAction(period=3.0, actions=[crazyflie_controllers]),
    ])
