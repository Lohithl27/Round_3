import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             TimerAction, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    lumi   = get_package_share_directory('lumi_r3')
    r1desc = get_package_share_directory('mini_r1_v1_description')
    r1gz   = get_package_share_directory('mini_r1_v1_gz')
    gz_pkg = get_package_share_directory('ros_gz_sim')

    tex_path  = os.path.join(lumi, 'materials', 'textures')
    world_src = os.path.join(lumi, 'worlds', 'grid_world_r3.sdf')

    with open(world_src) as f:
        content = f.read()
    content = content.replace('__TEXTURES__', tex_path)

    tmp = tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False)
    tmp.write(content); tmp.flush(); tmp.close()
    world_sdf = tmp.name

    ekf_cfg    = os.path.join(lumi,   'config', 'ekf_r3.yaml')
    bridge_cfg = os.path.join(r1gz,   'config', 'ros_gz_bridge.yaml')
    rsp_launch = os.path.join(r1desc, 'launch', 'rsp.launch.py')
    gz_launch  = os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch),
        launch_arguments={'use_sim_time':'true','use_control':'false'}.items()
    )

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={
            'gz_args': f'-r {world_sdf}',
            'on_exit_shutdown': 'true'
        }.items()
    )

    # FIXED: Removed -Y yaw flag — it was conflicting with -y position,
    # causing robot to spawn at wrong tile (+1.35 instead of -1.35).
    # Navigator auto-steers to first waypoint regardless of initial heading.
    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'mini_r1',
            '-x', '-1.35',
            '-y', '-1.80',
            '-z',  '0.0375',
        ],
        output='screen'
    )

    stamper = Node(
        package='twist_stamper', executable='twist_stamper',
        remappings=[('cmd_vel_in','cmd_vel'),('cmd_vel_out','cmd_vel_stamped')]
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['--ros-args','-p',f'config_file:={bridge_cfg}'],
        output='screen'
    )

    ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_cfg, {'use_sim_time': True}],
        output='screen'
    )

    apriltag = Node(
        package='lumi_r3', executable='apriltag_detector.py',
        name='apriltag_detector', output='screen',
        parameters=[{'use_sim_time': True}]
    )

    tile_det = Node(
        package='lumi_r3', executable='tile_detector.py',
        name='tile_detector', output='screen',
        parameters=[{'use_sim_time': True}]
    )

    navigator = Node(
        package='lumi_r3', executable='grid_navigator.py',
        name='grid_navigator', output='screen'
    )

    rviz_cfg = os.path.join(lumi, 'rviz', 'r3_challenge.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        prefix='xterm -e',
        remappings=[('cmd_vel','/cmd_vel')],
        condition=IfCondition(LaunchConfiguration('use_teleop')),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_teleop', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),

        LogInfo(msg='[LUMI R3] START: GREEN (-1.35,-1.80) | STOP: RED (1.35,-1.80)'),

        rsp, gz, stamper,
        TimerAction(period=5.0,  actions=[spawn, bridge]),
        TimerAction(period=9.0,  actions=[ekf]),
        TimerAction(period=14.0, actions=[apriltag, tile_det]),
        TimerAction(period=17.0, actions=[navigator]),
        TimerAction(period=18.0, actions=[rviz]),
        TimerAction(period=18.0, actions=[teleop]),
    ])
