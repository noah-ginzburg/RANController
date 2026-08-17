import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


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


# Manual keyboard flight on top of the normal hardware stack.
#
# This is real_drone.launch.py with teleop_enabled:=true, plus a
# teleop_twist_keyboard running in its own terminal. Two things change in the
# nodes as a result:
#
#   1. controller_server takes off by itself at startup. Teleop has no takeoff
#      button, and /cmd_vel can only steer a drone that's already in the air.
#      The takeoff service call is also what sets launch_requested, the
#      interlock that authorises the 50 Hz setpoint stream.
#   2. The RAN server stops publishing <drone>/desired_heading. The model still
#      runs and still draws in RViz, but if it published, the controller would
#      scale that heading by max_speed and command it, which means competing
#      with the keyboard.
#
# Wait for the climb to finish before you touch the keyboard. The first
# /cmd_vel message clears `taking_off` and starts the setpoint stream, which
# then competes with the onboard high-level commander that's still flying the
# takeoff.
#
# teleop_twist_keyboard reads from stdin, and a node launched by ros2 launch
# doesn't get a terminal, so keypresses would go nowhere. That's why it runs in
# its own gnome-terminal window. If that window doesn't appear, just run it by
# hand in any sourced terminal; nothing else here depends on it:
#
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
#       --ros-args -p speed:=0.2 -p turn:=0.5


def generate_launch_description():
    pkg_bringup = get_package_share_directory('crazyswarm_bringup')

    # Defaults come from config/launch_args.yaml -- the `real` section plus the
    # `teleop` one on top. The first five get forwarded straight through to
    # real_drone.launch.py; anything not listed keeps that file's own default.
    args = _load_launch_args('real')
    args.update(_load_launch_args('teleop'))

    drone_names_arg = DeclareLaunchArgument('drone_names', default_value=args['drone_names'])
    ran_drones_arg = DeclareLaunchArgument('ran_drones', default_value=args['ran_drones'])
    launch_height_arg = DeclareLaunchArgument('launch_height', default_value=args['launch_height'])
    record_arg = DeclareLaunchArgument('record', default_value=args['record'])
    rviz_arg = DeclareLaunchArgument('rviz', default_value=args['rviz'])

    teleop_speed_arg = DeclareLaunchArgument('teleop_speed', default_value=args['teleop_speed'])
    teleop_turn_arg = DeclareLaunchArgument('teleop_turn', default_value=args['teleop_turn'])
    ws_setup_arg = DeclareLaunchArgument(
        'ws_setup', default_value=os.path.expanduser(args['ws_setup']))

    real_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'real_drone.launch.py')),
        launch_arguments={
            'real': 'true',
            'teleop_enabled': 'true',
            'drone_names': LaunchConfiguration('drone_names'),
            'ran_drones': LaunchConfiguration('ran_drones'),
            'launch_height': LaunchConfiguration('launch_height'),
            'record': LaunchConfiguration('record'),
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )

    # --wait keeps this process alive for as long as the window is open, so
    # launch can shut it down along with everything else on Ctrl+C. Without it,
    # gnome-terminal forks and returns straight away.
    teleop = ExecuteProcess(
        cmd=['gnome-terminal', '--wait', '--title=cmd_vel teleop', '--',
             'bash', '-c',
             'source /opt/ros/humble/setup.bash && '
             'if [ -f "$0" ]; then source "$0"; fi && '
             'exec ros2 run teleop_twist_keyboard teleop_twist_keyboard '
             '--ros-args -p speed:=$1 -p turn:=$2',
             LaunchConfiguration('ws_setup'),
             LaunchConfiguration('teleop_speed'),
             LaunchConfiguration('teleop_turn')],
        output='screen',
    )

    return LaunchDescription([
        drone_names_arg,
        ran_drones_arg,
        launch_height_arg,
        record_arg,
        rviz_arg,
        teleop_speed_arg,
        teleop_turn_arg,
        ws_setup_arg,
        real_stack,
        teleop,
    ])
