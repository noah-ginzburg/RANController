import os
import yaml
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def _load_static_targets(path, override_str):
    # (names, qualities) of the enabled targets in a static_targets.yaml, in
    # file order -- the names so the RAN server can be told which TF frames to
    # look for, the qualities so it weights them. An override of one value
    # covers every target; empty keeps the yaml's own `quality`.
    with open(path, 'r') as f:
        content = yaml.safe_load(f)
    names, qualities = [], []
    for name, cfg in content['static_targets'].items():
        if cfg.get('enabled', True):
            names.append(str(name))
            qualities.append(float(cfg.get('quality', 20.0)))

    override = [float(q.strip()) for q in override_str.split(',') if q.strip()]
    if len(override) == 1:
        override = override * len(names)
    if override and len(override) != len(names):
        raise RuntimeError(
            f'static_target_qualities has {len(override)} values but there are '
            f'{len(names)} enabled static targets ({", ".join(names)})')
    return names, (override or qualities)


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

    # Static (non-flying) targets: nothing simulated, just a `mocap -> <name>`
    # TF frame each, which is what spherical_RAN_server_lloyd looks up -- so the
    # RAN server treats them as targets purely by having them in `all_drones`.
    # use_static_targets:=false runs none of it, leaving the target drones as
    # the targets exactly as before.
    static_targets_yaml = LaunchConfiguration('static_targets_yaml').perform(context)
    static_qualities_str = LaunchConfiguration('static_target_qualities').perform(context)
    use_static_targets = LaunchConfiguration('use_static_targets').perform(context) == 'true'
    static_names, static_qualities = (
        _load_static_targets(static_targets_yaml, static_qualities_str)
        if use_static_targets else ([], []))

    target_names = target_names + static_names
    target_qualities = target_qualities + static_qualities
    if len(target_names) != len(target_qualities):
        raise RuntimeError(
            f'target_names has {len(target_names)} entries but target_qualities has '
            f'{len(target_qualities)} -- they are zipped into a per-target quality map')
    ran_targets = drone_names + static_names

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
    if static_names:
        actions.append(Node(
            package='crazyswarm_bringup',
            executable='static_target_server.py',
            name='static_target_server',
            output='screen',
            # Qualities go to the server too, so its marker labels show what the
            # RAN server is actually using rather than the yaml value.
            parameters=[{'targets_yaml': static_targets_yaml},
                        {'quality_overrides': static_qualities}],
        ))

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
            # An empty list has no inferrable parameter type, and launch rejects
            # it ("got '()' of type tuple") -- so leave the pair out entirely and
            # let the RAN server's own declare_parameter defaults stand. Hit by
            # both use_static_targets:=false and :=true with nothing enabled.
            ran_params = [{'drone_name': name}, {'all_drones': ran_targets}]
            if target_names and target_qualities:
                ran_params += [{'target_names': target_names},
                               {'target_qualities': target_qualities}]
            actions.append(Node(
                package='spherical_ran',
                executable='spherical_RAN_server_lloyd',
                name=f'spherical_RAN_server_{name}',
                output='screen',
                parameters=ran_params,
            ))
        # One GUI per drone, outside the ran_drones check so it always runs.
        actions.append(Node(
            package='crazyflie_debug_gui',
            executable='crazyflie_debug_gui',
            name=f'{name}_debug_gui',
            output='screen',
            parameters=[{'drone_name': name}],
        ))

    return actions


def generate_launch_description():
    real_arg = DeclareLaunchArgument('real', default_value='true')
    #initial hover speed guess, pid takes over in sim
    hover_speed_sim_arg = DeclareLaunchArgument('hover_speed_sim', default_value='0.0')
    hover_speed_real_arg = DeclareLaunchArgument('hover_speed_real', default_value='0.0')
    # Base height every drone takes off to; per-drone delta_z (from crazyflies.yaml)
    # is added on top of this for the icosphere target drones.
    launch_height_arg = DeclareLaunchArgument('launch_height', default_value='0.5')
    # Must match the drones marked `enabled: true` in crazyflies.yaml — the sim
    # server only creates takeoff/land/arm services for enabled drones, and a
    # controller for a missing drone blocks forever in wait_for_service.
    drone_names_arg = DeclareLaunchArgument('drone_names', default_value='cf09')
    ran_drones_arg = DeclareLaunchArgument('ran_drones', default_value='cf09')
    target_names_arg = DeclareLaunchArgument('target_names', default_value='')
    target_qualities_arg = DeclareLaunchArgument('target_qualities', default_value='')
    record_arg = DeclareLaunchArgument('record', default_value='true')
    # The real launch used to run headless; the static-target workflow needs RViz
    # for the marker drag handles, so it's on by default and switchable from here.
    # Capital True/False -- crazyswarm2 gates rviz on LaunchConfigurationEquals,
    # which is an exact string match, so 'true' would silently never start it.
    rviz_arg = DeclareLaunchArgument('rviz', default_value='True')

    pkg_bringup = get_package_share_directory('crazyswarm_bringup')

    # The targets file is always wired up; use_static_targets decides whether
    # anything reads it. Point it elsewhere to try a layout without editing the
    # shipped config.
    static_targets_yaml_arg = DeclareLaunchArgument(
        'static_targets_yaml',
        default_value=os.path.join(pkg_bringup, 'config', 'static_targets.yaml'))
    use_static_targets_arg = DeclareLaunchArgument('use_static_targets', default_value='false')
    # Overrides the yaml's per-target `quality`, positionally.
    static_target_qualities_arg = DeclareLaunchArgument(
        'static_target_qualities', default_value='')

    # 3_targets.rviz shows the cf01/02/03 RobotModels, which don't exist when
    # the targets are static -- so default to the config that draws the target
    # markers and their drag handles instead.
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PythonExpression([
            "'", os.path.join(pkg_bringup, 'rviz', 'static_targets.rviz'),
            "' if '", LaunchConfiguration('use_static_targets'), "' == 'true' else '",
            os.path.join(pkg_bringup, 'rviz', '3_targets.rviz'), "'"]))

    pkg_crazyswarm2 = get_package_share_directory('crazyflie')

    crazyswarm2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_crazyswarm2, 'launch', 'launch.py')
        ),
        launch_arguments={
            'backend': 'cflib',
            # vicon_bridge.py publishes /poses itself, so crazyswarm2's own
            # motion_capture_tracking node stays off.
            'mocap': 'False',
            'rviz': LaunchConfiguration('rviz'),
            'gui': 'False',
            # Don't start crazyswarm2's gamepad teleop (joy_node + teleop node). We
            # drive via teleop_twist_keyboard -> /cmd_vel -> our controller instead,
            # and an extra commander on the drone is a safety/conflict hazard.
            'teleop': 'False',
            'rviz_config_file': LaunchConfiguration('rviz_config_file'),
        }.items()
    )

    crazyflie_controllers = OpaqueFunction(function=launch_controllers)

    return LaunchDescription([
        # On Ctrl+C, launch sends SIGINT, then SIGTERM after sigterm_timeout, then
        # an uncatchable SIGKILL sigkill_timeout later. The crazyflie_server ignores
        # SIGINT, so with the 5s/10s defaults it lingers ~10s and gets relaunched as
        # a duplicate. sigterm_timeout MUST stay above the controller's
        # LAND_DURATION (5s): SIGINT starts the land, and both crazyflie_server and
        # vicon_bridge (which ignores SIGINT for this reason) have to survive the
        # whole descent or the drone lands with no mocap and no radio link.
        # 7s + 2s sigkill => guaranteed death ~9s after Ctrl+C.
        # (NOTE: in Humble launch a *second* Ctrl+C is ignored — escalation is
        # timer-based, not triggered by the second press.)
        SetLaunchConfiguration('sigterm_timeout', '7'),
        SetLaunchConfiguration('sigkill_timeout', '2'),
        real_arg,
        hover_speed_sim_arg,
        hover_speed_real_arg,
        launch_height_arg,
        drone_names_arg,
        ran_drones_arg,
        target_names_arg,
        target_qualities_arg,
        record_arg,
        rviz_arg,
        static_targets_yaml_arg,
        use_static_targets_arg,
        static_target_qualities_arg,
        rviz_config_arg,
        crazyswarm2,
        TimerAction(period=3.0, actions=[crazyflie_controllers]),
    ])
