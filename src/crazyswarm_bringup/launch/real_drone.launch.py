import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

# Per-drone launch (takeoff) heights in meters. cf01/cf02/cf03 sit on the
# floor at the x,y of their icosphere triangle node (see crazyflies.yaml);
# takeoff brings each one up to its node's z. The unit icosphere is centered
# SPHERE_CENTER_Z above the floor, so height = SPHERE_CENTER_Z + node z.
SPHERE_CENTER_Z = 1.0
LAUNCH_HEIGHTS = {
    'cf01': SPHERE_CENTER_Z - 0.500000,  # icosphere node 12
    'cf02': SPHERE_CENTER_Z + 0.809017,  # icosphere node 21
    'cf03': SPHERE_CENTER_Z - 0.309017,  # icosphere node 35
}
DEFAULT_LAUNCH_HEIGHT = 0.5  # any drone not listed above (e.g. cf09)


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

    actions = []
    if LaunchConfiguration('record').perform(context) == 'true':
        # Humble's rosbag2 doesn't allow mixing explicit topic names with
        # --regex, so everything goes into one pattern.
        topic_regex = '(' + '|'.join([f'/{n}/.*' for n in drone_names] + ['/poses']) + ')$'
        bag_dir = os.path.expanduser('~/biodrone/bags')
        os.makedirs(bag_dir, exist_ok=True)
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-e', topic_regex, '-o',
                 os.path.join(bag_dir, datetime.now().strftime('flight_%Y%m%d_%H%M%S'))],
            output='screen',
        ))
    if real:
        actions.append(Node(
            package='vicon_receiver',
            executable='vicon_bridge.py',
            output='screen',
            # send_orientation=True requires firmware with the fixed external-
            # attitude update (2026.04+); on old firmware it destabilizes the EKF
            # at yaw far from 0. Set False to fall back to position-only fusion.
            # 2026-07-17: True — new airframe confirmed flashed with 2026.04.
            # Position-only fusion was the root cause of the day's runaways:
            # EKF yaw is gyro-only, drifts with every spin, and persists between
            # takeoff attempts -> control frame rotates -> lateral runaway/tumble
            # (bags 151857, 152128). Full pose fusion corrects yaw continuously.
            parameters=[{'all_drones': drone_names}, {'send_orientation': True}],
        ))
    for name in drone_names:
        actions.append(Node(
            package='crazyflie_controller',
            executable='controller_server',
            name=f'controller_server_{name}',
            output='screen',
            parameters=[{'drone_name': name}, {'hover_speed': float(hover_speed)}, {'real': real},
                        {'launch_height': float(LAUNCH_HEIGHTS.get(name, DEFAULT_LAUNCH_HEIGHT))}],
        ))
        if name in ran_drones:
            actions.append(Node(
                package='spherical_ran',
                executable='spherical_RAN_server',
                name=f'spherical_RAN_server_{name}',
                output='screen',
                parameters=[{'drone_name': name}, {'all_drones': drone_names}, {'target_names': target_names}, {'target_qualities': target_qualities}],
            ))

    return actions


def generate_launch_description():
    real_arg = DeclareLaunchArgument('real', default_value='true')
    hover_speed_sim_arg = DeclareLaunchArgument('hover_speed_sim', default_value='0.0')
    hover_speed_real_arg = DeclareLaunchArgument('hover_speed_real', default_value='0.0')
    # drone_names_arg = DeclareLaunchArgument('drone_names', default_value='cf01,cf02,cf03')
    drone_names_arg = DeclareLaunchArgument('drone_names', default_value='cf09')
    ran_drones_arg = DeclareLaunchArgument('ran_drones', default_value='cf09')
    target_names_arg = DeclareLaunchArgument('target_names', default_value='cf02,cf03')
    target_qualities_arg = DeclareLaunchArgument('target_qualities', default_value='20.0,20.0')
    record_arg = DeclareLaunchArgument('record', default_value='true')

    pkg_crazyswarm2 = get_package_share_directory('crazyflie')

    crazyswarm2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_crazyswarm2, 'launch', 'launch.py')
        ),
        launch_arguments={
            'backend': 'cflib',
            'mocap': 'False',
            'rviz': 'False',
            'gui': 'False',
            # Don't start crazyswarm2's gamepad teleop (joy_node + teleop node). We
            # drive via teleop_twist_keyboard -> /cmd_vel -> our controller instead,
            # and an extra commander on the drone is a safety/conflict hazard.
            'teleop': 'False',
            'rviz_config_file': os.path.join(
                get_package_share_directory('crazyswarm_bringup'),
                'rviz',
                'config.rviz'),
        }.items()
    )

    crazyflie_controllers = OpaqueFunction(function=launch_controllers)

    return LaunchDescription([
        # On Ctrl+C, launch sends SIGINT, then SIGTERM after sigterm_timeout, then an
        # uncatchable SIGKILL sigkill_timeout later. The crazyflie_server ignores
        # SIGINT, so with the 5s/10s defaults it lingers ~10s and gets relaunched as a
        # duplicate. sigterm_timeout is kept ABOVE the controller's ~3s landing
        # (DURATION) so a hardware land isn't cut short; sigkill then guarantees death
        # ~6s after Ctrl+C. (NOTE: in Humble launch a *second* Ctrl+C is ignored —
        # escalation is timer-based, not triggered by the second press.)
        SetLaunchConfiguration('sigterm_timeout', '4'),
        SetLaunchConfiguration('sigkill_timeout', '2'),
        real_arg,
        hover_speed_sim_arg,
        hover_speed_real_arg,
        drone_names_arg,
        ran_drones_arg,
        target_names_arg,
        target_qualities_arg,
        record_arg,
        crazyswarm2,
        TimerAction(period=3.0, actions=[crazyflie_controllers]),
    ])
