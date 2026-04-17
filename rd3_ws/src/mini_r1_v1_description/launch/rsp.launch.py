import os
from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration,Command
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
        use_sim_time = LaunchConfiguration('use_sim_time')
        use_control = LaunchConfiguration('use_control')
        pkg_name = 'mini_r1_v1_description'
        pkg_path = os.path.join(get_package_share_directory(pkg_name))
        use_control = LaunchConfiguration('use_control')


        xacro_file = os.path.join(pkg_path, 'urdf', 'mini_r1.urdf.xacro')
        
        robot_description_config = ParameterValue(Command(['xacro ', xacro_file,' use_control:=',use_control]), value_type=str)
        params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output='screen',
            parameters=[params]
        )

        odom_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        )
        launch_args  = [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use simulation (Gazebo) clock if true'
            ),
            DeclareLaunchArgument(
                'use_control',
                default_value='false',
                description='Toggle to either use ros2_control (true) or will use gz'
            ),




        ]

        return LaunchDescription([
            *launch_args,
            robot_state_publisher,
            #odom_tf,
            
        ])