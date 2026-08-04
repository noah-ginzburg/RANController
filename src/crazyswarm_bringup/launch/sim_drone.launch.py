import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def _load_delta_z(drone_names):
    # Per-drone vertical offset added on top of controller_server's launch_height
    # at takeoff (see delta_z in crazyflies.yaml -- the icosphere target drones'
    # node z, keeping cf01/cf02/cf03 spread across the equilateral triangle).
    crazyflies_yaml = os.path.join(
        get_package_share_directory('crazyflie'), 'config', 'crazyflies.yaml')
    with open(crazyflies_yaml, 'r') as f:
        robots = yaml.safe_load(f)['robots']
    return {name: float(robots.get(name, {}).get('delta_z', 0.0)) for name in drone_names}


def launch_controllers(context, *args, **kwargs):
    names_str = LaunchConfiguration('drone_names').perform(context)
    ran_str = LaunchConfiguration('ran_drones').perform(context)
    target_names_str = LaunchConfiguration('target_names').perform(context)
    target_qualities_str = LaunchConfiguration('target_qualities').perform(context)
    real = LaunchConfiguration('real').perform(context) == 'true'
    drone_names = [name.strip() for name in names_str.split(',') if name.strip()]
    ran_drones = [name.strip() for name in ran_str.split(',') if name.strip()]
    target_names = [name.strip() for name in target_names_str.split(',') if name.strip()]
    target_qualities = [float(q.strip()) for q in target_qualities_str.split(',') if q.strip()]
    hover_speed = LaunchConfiguration('hover_speed_real').perform(context) if real else LaunchConfiguration('hover_speed_sim').perform(context)
    launch_height = float(LaunchConfiguration('launch_height').perform(context))
    delta_z_by_name = _load_delta_z(drone_names)

    actions = []
    for name in drone_names:
        actions.append(Node(
            package='crazyflie_controller',
            executable='controller_server',
            name=f'controller_server_{name}',
            output='screen',
            parameters=[{'drone_name': name}, {'hover_speed': float(hover_speed)}, {'real': real},
                        {'launch_height': launch_height}, {'delta_z': delta_z_by_name[name]}],
        ))
        if name in ran_drones:
            actions.append(Node(
                package='spherical_ran',
                executable='spherical_RAN_server_lloyd',
                name=f'spherical_RAN_server_{name}',
                output='screen',
                parameters=[{'drone_name': name}, {'all_drones': drone_names}, {'target_names': target_names}, {'target_qualities': target_qualities}],
            ))

    return actions


def generate_launch_description():
    real_arg = DeclareLaunchArgument('real', default_value='false')
    hover_speed_sim_arg = DeclareLaunchArgument('hover_speed_sim', default_value='0.1725')
    hover_speed_real_arg = DeclareLaunchArgument('hover_speed_real', default_value='0.0')
    # Base height every drone takes off to; per-drone delta_z (from crazyflies.yaml)
    # is added on top of this for the icosphere target drones.
    launch_height_arg = DeclareLaunchArgument('launch_height', default_value='1.5')
    # Must match the drones marked `enabled: true` in crazyflies.yaml — the sim
    # server only creates takeoff/land/arm services for enabled drones, and a
    # controller for a missing drone blocks forever in wait_for_service.
    drone_names_arg = DeclareLaunchArgument('drone_names', default_value='cf09,cf01,cf02,cf03')
    ran_drones_arg = DeclareLaunchArgument('ran_drones', default_value='cf09')
    target_names_arg = DeclareLaunchArgument('target_names', default_value='cf01,cf02,cf03')
    target_qualities_arg = DeclareLaunchArgument('target_qualities', default_value='20.0,20.0,20.0')

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
                get_package_share_directory('crazyswarm_bringup'),
                'rviz',
                '3_targets.rviz'),
        }.items()
    )

    crazyflie_controllers = OpaqueFunction(function=launch_controllers)

    return LaunchDescription([
        # On Ctrl+C, launch sends SIGINT, then SIGTERM after sigterm_timeout, then
        # an uncatchable SIGKILL sigkill_timeout later. The crazyflie_server ignores
        # SIGINT, so with the 5s/10s defaults it lingers ~10s and gets relaunched as
        # a duplicate. Shrink the window so a single Ctrl+C reliably kills it. (NOTE:
        # in Humble launch, a *second* Ctrl+C is ignored — escalation is timer-based.)
        SetLaunchConfiguration('sigterm_timeout', '2'),
        SetLaunchConfiguration('sigkill_timeout', '2'),
        real_arg,
        hover_speed_sim_arg,
        hover_speed_real_arg,
        launch_height_arg,
        drone_names_arg,
        ran_drones_arg,
        target_names_arg,
        target_qualities_arg, 
        crazyswarm2,
        TimerAction(period=3.0, actions=[crazyflie_controllers]),
    ])
