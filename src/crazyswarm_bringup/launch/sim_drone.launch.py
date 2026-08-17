import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def _load_launch_args(section):
    # Defaults for the launch arguments, from config/launch_args.yaml. `common`
    # applies everywhere and the named section is layered on top of it. Values
    # come back as strings because that is what DeclareLaunchArgument's
    # default_value takes, and what the launch files compare against.
    path = os.path.join(
        get_package_share_directory('crazyswarm_bringup'), 'config', 'launch_args.yaml')
    with open(path, 'r') as f:
        content = yaml.safe_load(f)

    merged = dict(content.get('common', {}))
    merged.update(content.get(section, {}))
    # A bare yaml `true` becomes Python True, which str()s to 'True' and would
    # then fail the launch files' `== 'true'` checks. Fold bools down to the
    # lowercase form the comparisons expect.
    return {k: ('true' if v else 'false') if isinstance(v, bool) else str(v)
            for k, v in merged.items()}


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

    # Tuning parameters, one file per node type. These go first in each node's
    # parameters list, so the per-drone values computed below still override
    # them; launch applies the list in order and the last write wins.
    controller_params = LaunchConfiguration('controller_params').perform(context)
    ran_params = LaunchConfiguration('ran_params').perform(context)

    # Static (non-flying) targets: nothing simulated, just a `mocap -> <name>`
    # TF frame each, which is what spherical_RAN_server looks up -- so the
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
    ran_targets = drone_names + static_names

    actions = []
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
            parameters=[controller_params,
                        {'drone_name': name}, {'hover_speed': float(hover_speed)}, {'real': real},
                        {'launch_height': launch_height}, {'delta_z': delta_z_by_name[name]}],
        ))
        if name in ran_drones:
            actions.append(Node(
                package='spherical_ran',
                executable='spherical_RAN_server',
                name=f'spherical_RAN_server_{name}',
                output='screen',
                parameters=[ran_params,
                            {'drone_name': name}, {'all_drones': ran_targets}, {'target_names': target_names}, {'target_qualities': target_qualities}],
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
    # Every default below comes from config/launch_args.yaml, so the values live
    # in one editable place instead of being spread through this file. Each is
    # still overridable on the command line as usual.
    args = _load_launch_args('sim')

    real_arg = DeclareLaunchArgument('real', default_value=args['real'])
    hover_speed_sim_arg = DeclareLaunchArgument('hover_speed_sim', default_value=args['hover_speed_sim'])
    hover_speed_real_arg = DeclareLaunchArgument('hover_speed_real', default_value=args['hover_speed_real'])
    launch_height_arg = DeclareLaunchArgument('launch_height', default_value=args['launch_height'])
    drone_names_arg = DeclareLaunchArgument('drone_names', default_value=args['drone_names'])
    ran_drones_arg = DeclareLaunchArgument('ran_drones', default_value=args['ran_drones'])
    target_names_arg = DeclareLaunchArgument('target_names', default_value=args['target_names'])
    target_qualities_arg = DeclareLaunchArgument('target_qualities', default_value=args['target_qualities'])

    pkg_bringup = get_package_share_directory('crazyswarm_bringup')

    # The targets file is always wired up; use_static_targets decides whether
    # anything reads it. Point it elsewhere to try a layout without editing the
    # shipped config.
    static_targets_yaml_arg = DeclareLaunchArgument(
        'static_targets_yaml',
        default_value=os.path.join(pkg_bringup, 'config', 'static_targets.yaml'))
    use_static_targets_arg = DeclareLaunchArgument(
        'use_static_targets', default_value=args['use_static_targets'])
    static_target_qualities_arg = DeclareLaunchArgument(
        'static_target_qualities', default_value=args['static_target_qualities'])

    # Tuning parameters for the two nodes that have anything worth tuning. Point
    # these somewhere else to try out a set of gains without editing the configs
    # that ship with the package.
    controller_params_arg = DeclareLaunchArgument(
        'controller_params',
        default_value=os.path.join(pkg_bringup, 'config', 'controller_params.yaml'))
    ran_params_arg = DeclareLaunchArgument(
        'ran_params',
        default_value=os.path.join(pkg_bringup, 'config', 'ran_params.yaml'))

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
            'backend': 'sim',
            'mocap': 'False',
            'rviz': 'True',
            'gui': 'False',
            'rviz_config_file': LaunchConfiguration('rviz_config_file'),
        }.items()
    )

    crazyflie_controllers = OpaqueFunction(function=launch_controllers)

    return LaunchDescription([
        # Shutdown timing, explained in config/launch_args.yaml. Note that in
        # Humble a *second* Ctrl+C is ignored: the escalation is timer-based,
        # not triggered by pressing it again.
        SetLaunchConfiguration('sigterm_timeout', args['sigterm_timeout']),
        SetLaunchConfiguration('sigkill_timeout', args['sigkill_timeout']),
        real_arg,
        hover_speed_sim_arg,
        hover_speed_real_arg,
        launch_height_arg,
        drone_names_arg,
        ran_drones_arg,
        target_names_arg,
        target_qualities_arg,
        static_targets_yaml_arg,
        use_static_targets_arg,
        static_target_qualities_arg,
        controller_params_arg,
        ran_params_arg,
        rviz_config_arg,
        crazyswarm2,
        TimerAction(period=3.0, actions=[crazyflie_controllers]),
    ])
