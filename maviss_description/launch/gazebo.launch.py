from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_fastlivo = get_package_share_directory('fast_livo')
    pkg_description = get_package_share_directory('maviss_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths
    avia_yaml = os.path.join(pkg_fastlivo, 'config', 'avia.yaml')
    camera_yaml = os.path.join(pkg_fastlivo, 'config', 'camera_pinhole.yaml')
    xacro_file = os.path.join(pkg_description, 'urdf', 'drone.urdf.xacro')
    world_file = os.path.join(pkg_description, 'world', 'default.world')

    return LaunchDescription([
        # Ensure all nodes use /clock
        SetParameter(name='use_sim_time', value=True),
        
        # FAST-LIVO2 parameter args
        DeclareLaunchArgument('avia_params_file',
            default_value=avia_yaml),
        DeclareLaunchArgument('camera_params_file',
            default_value=camera_yaml),

        # 1) Start Gazebo with ROS 2 integration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
        ),

        # 2) Publish URDF to /robot_description
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(
                    ['xacro ', xacro_file])
            }],
            output='screen'
        ),

        # 3) Spawn robot entity
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'maviss_drone', '-topic', 'robot_description'],
            output='screen'
        ),

        # 4) Load camera intrinsics into vikit
        Node(
            package='demo_nodes_cpp', executable='parameter_blackboard',
            name='parameter_blackboard',
            parameters=[LaunchConfiguration('camera_params_file')],
            output='screen'
        ),

        # 5) Launch FAST-LIVO2 mapping node
        Node(
            package='fast_livo', executable='fastlivo_mapping',
            name='laserMapping',
            parameters=[LaunchConfiguration('avia_params_file')],
            output='screen'
        ),
    ])

