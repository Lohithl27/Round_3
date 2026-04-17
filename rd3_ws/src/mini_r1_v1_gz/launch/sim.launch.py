import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():
    description_package = "mini_r1_v1_description"
    simulation_package = "mini_r1_v1_gz"

    description_package_share = get_package_share_directory(description_package)
    simulation_package_share = get_package_share_directory(simulation_package)

    default_world = os.path.join(description_package_share, 'worlds', 'empty.sdf')
    world_path = LaunchConfiguration('world')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        os.path.join(description_package_share, "launch", "rsp.launch.py")
                    ]
                ),
                launch_arguments={"use_sim_time": "true", 'use_control': "false"}.items()
            )
    gz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    os.path.join(get_package_share_directory("ros_gz_sim"),'launch', 'gz_sim.launch.py')
                ]
            ),
            launch_arguments={"gz_args": ['-r ', world_path], 'on_exit_shutdown': 'true'}.items(),
    )
    spawn_entity = Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-topic", 'robot_description',
                           '-name', 'mini_r1',
                           '-x', '0.0',
                           '-y', '0.0',
                           '-z', '0.07'],
                output="screen"
                    
                )
    
    stamper = Node(
                package="twist_stamper",
                executable="twist_stamper",
                remappings=[
                    ('cmd_vel_in', 'cmd_vel'),
                    ('cmd_vel_out', 'cmd_vel_stamped'),
                ],
    )
    bridge_params = os.path.join(get_package_share_directory(simulation_package), 'config', 'ros_gz_bridge.yaml')
    ros_gz_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                '--ros-args',
                '-p',
                f'config_file:={bridge_params}'
            ]
        )
    launch_args = [
        DeclareLaunchArgument(
            name="world",
            default_value=default_world,
            description="Enter the absolute path to the world in which the robot is to be spawned"
        )
    ]

    return LaunchDescription([ 
        *launch_args,
        rsp,
        stamper,
        gz,
        ros_gz_bridge,
        spawn_entity
       

    ])
