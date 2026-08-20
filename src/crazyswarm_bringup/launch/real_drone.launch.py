import os
import yaml
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction, SetLaunchConfiguration, PushLaunchConfigurations, PopLaunchConfigurations
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


# The teleop window, and how it dies on Ctrl+C.
#
# `gnome-terminal` is a thin client: the window is created by
# gnome-terminal-server, which is nobody's child of ours. Signalling the client
# therefore closes nothing -- the window, its shell and teleop_twist_keyboard
# all sail through launch's SIGINT and keep publishing /cmd_vel at a drone that
# is trying to land. So the process launch owns is this wrapper, and the window
# is taken down from the inside: the wrapper holds a sentinel file for as long
# as it lives, and a watchdog in the window kills the terminal's process group
# within half a second of that file going away.
#
# The `& wait` at the end matters. A non-interactive bash defers its traps until
# the current foreground command returns, and `gnome-terminal --wait` doesn't
# return until the window closes -- which is exactly what the trap is supposed
# to cause. Backgrounding it and waiting lets the signal land immediately.
TELEOP_WRAPPER = r"""
sentinel=$(mktemp /tmp/cf_teleop.XXXXXX)
trap 'rm -f "$sentinel"' EXIT
trap 'rm -f "$sentinel"; exit 0' INT TERM
gnome-terminal --wait --title='cmd_vel teleop' -- bash -c '
    source /opt/ros/humble/setup.bash
    if [ -f "$1" ]; then source "$1"; fi
    ( while [ -e "$0" ]; do sleep 0.5; done; kill -TERM 0 ) &
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -p speed:="$2" -p turn:="$3"
' "$sentinel" "$1" "$2" "$3" &
wait $!
"""


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
    # There is no hover speed to configure on this stack: set_speeds only
    # applies one in sim, so the node gets 0.0 and the thrust comes from the
    # onboard commander. The sim knob lives in sim_drone.launch.py.
    hover_speed = 0.0
    launch_height = float(LaunchConfiguration('launch_height').perform(context))
    delta_z_by_name = _load_delta_z(drone_names)

    # Tuning parameters, one file per node type. These go first in each node's
    # parameters list, so the per-drone values computed below still override
    # them; launch applies the list in order and the last write wins.
    controller_params = LaunchConfiguration('controller_params').perform(context)
    ran_params = LaunchConfiguration('ran_params').perform(context)
    # Whether the RAN server publishes <drone>/desired_heading at startup.
    # Independent of `teleop` -- see the declarations in
    # generate_launch_description() for why those two came apart. The debug GUI
    # can flip this at runtime on <drone>/ran_enabled either way.
    ran_enabled = LaunchConfiguration('ran_enabled').perform(context) == 'true'
    auto_launch = LaunchConfiguration('auto_launch').perform(context) == 'true'

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
            # attitude update (2026.04+); on old firmware, I THINK (but am unsure) that it destabilizes the EKF
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
            parameters=[controller_params,
                        {'drone_name': name}, {'hover_speed': hover_speed}, {'real': real},
                        {'launch_height': launch_height}, {'delta_z': delta_z_by_name[name]},
                        {'auto_launch': auto_launch}],
        ))
        if name in ran_drones:
            # An empty list has no inferrable parameter type, and launch rejects
            # it ("got '()' of type tuple") -- so leave the pair out entirely and
            # let the RAN server's own declare_parameter defaults stand. Hit by
            # both use_static_targets:=false and :=true with nothing enabled.
            ran_node_params = [ran_params,
                               {'drone_name': name}, {'all_drones': ran_targets},
                               {'ran_enabled': ran_enabled}]
            if target_names and target_qualities:
                ran_node_params += [{'target_names': target_names},
                                    {'target_qualities': target_qualities}]
            actions.append(Node(
                package='spherical_ran',
                executable='spherical_RAN_server',
                name=f'spherical_RAN_server_{name}',
                output='screen',
                parameters=ran_node_params,
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
    args = _load_launch_args('real')

    real_arg = DeclareLaunchArgument('real', default_value=args['real'])
    launch_height_arg = DeclareLaunchArgument('launch_height', default_value=args['launch_height'])
    drone_names_arg = DeclareLaunchArgument('drone_names', default_value=args['drone_names'])
    ran_drones_arg = DeclareLaunchArgument('ran_drones', default_value=args['ran_drones'])
    target_names_arg = DeclareLaunchArgument('target_names', default_value=args['target_names'])
    target_qualities_arg = DeclareLaunchArgument('target_qualities', default_value=args['target_qualities'])
    record_arg = DeclareLaunchArgument('record', default_value=args['record'])
    rviz_arg = DeclareLaunchArgument('rviz', default_value=args['rviz'])

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

    # Open a teleop_twist_keyboard window, and NOTHING else.
    #
    # This argument used to mean three things at once: open the window, force
    # auto_launch off, and silence the RAN server. That made keyboard flight
    # and model flight mutually exclusive as a side effect of how the flag was
    # wired rather than as a decision anyone made. The other two now stand on
    # their own as `auto_launch` and `ran_enabled`, so any combination is
    # reachable -- including keyboard and model together, which is a useful
    # one: controller_server's teleop_timeout hands the drone back to the model
    # teleop_timeout seconds after the last keypress, so the keyboard behaves
    # as a manual override rather than a competitor.
    #
    # Wait for the climb to finish before you touch the keyboard. The first
    # /cmd_vel message clears `taking_off` and starts the setpoint stream, which
    # then competes with the onboard high-level commander that's still flying
    # the takeoff.
    teleop_arg = DeclareLaunchArgument('teleop', default_value=args['teleop'])

    # Whether the RAN server publishes <drone>/desired_heading at startup. Off
    # means the model still runs and still draws in RViz, it just doesn't
    # command the drone -- handy for watching a bump form while you fly the
    # thing yourself. The debug GUI's "RAN Publisher" button flips it live, so
    # this is only the starting state.
    ran_enabled_arg = DeclareLaunchArgument(
        'ran_enabled', default_value=args['ran_enabled'])

    # Whether the controller takes off by itself at startup or waits for a
    # takeoff from the debug GUI. No longer derived from `teleop`: that
    # coupling is what made the two inseparable. The yaml value now applies
    # whatever else is set.
    auto_launch_arg = DeclareLaunchArgument(
        'auto_launch', default_value=args['auto_launch'])
    teleop_speed_arg = DeclareLaunchArgument(
        'teleop_speed', default_value=args['teleop_speed'])
    teleop_turn_arg = DeclareLaunchArgument(
        'teleop_turn', default_value=args['teleop_turn'])
    ws_setup_arg = DeclareLaunchArgument(
        'ws_setup', default_value=os.path.expanduser(args['ws_setup']))

    # teleop_twist_keyboard reads from stdin, and a node launched by ros2 launch
    # doesn't get a terminal, so keypresses would go nowhere. That's why it runs
    # in its own gnome-terminal window. If that window doesn't appear, just run
    # it by hand in any sourced terminal; nothing else here depends on it:
    #
    #   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    #       --ros-args -p speed:=0.2 -p turn:=0.5
    teleop_keyboard = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('teleop')),
        cmd=['bash', '-c', TELEOP_WRAPPER, 'cf_teleop',
             LaunchConfiguration('ws_setup'),
             LaunchConfiguration('teleop_speed'),
             LaunchConfiguration('teleop_turn')],
        output='screen',
    )

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
            # Don't start crazyswarm2's gamepad teleop (joy_node + teleop node).
            # Unrelated to our own `teleop` argument: we drive via
            # teleop_twist_keyboard -> /cmd_vel -> our controller instead, and an
            # extra commander on the drone is a safety/conflict hazard.
            'teleop': 'False',
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
        controller_params_arg,
        ran_params_arg,
        teleop_arg,
        ran_enabled_arg,
        auto_launch_arg,
        teleop_speed_arg,
        teleop_turn_arg,
        ws_setup_arg,
        rviz_config_arg,
        # crazyswarm2's launch_arguments are NOT scoped to the include: each
        # one stays set in this file's configuration afterwards. `teleop`
        # collides (theirs is the gamepad stack, ours is the keyboard), so
        # without this push/pop the 'teleop': 'False' above would overwrite the
        # `teleop` argument for everything visited later -- the keyboard window
        # and launch_controllers included.
        PushLaunchConfigurations(),
        crazyswarm2,
        PopLaunchConfigurations(),
        teleop_keyboard,
        TimerAction(period=3.0, actions=[crazyflie_controllers]),
    ])
