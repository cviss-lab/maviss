import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
 
    # Find paths
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")  
    xacro_file = get_package_share_directory('maviss_description') + '/urdf/drone.urdf.xacro'
    
    # Load parameters
    avia_config_cmd = os.path.join(config_file_dir, "avia.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_pinhole.yaml")  
    
    # FAST-LIVO2 configuration
    avia_config_arg = DeclareLaunchArgument(
        'avia_params_file',
        default_value=avia_config_cmd,
        description='Full path to the ROS2 parameters file to use for fast_livo2 nodes',
    )
 
    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file',
        default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file to use for vikit_ros nodes',
    )
   
    avia_params_file = LaunchConfiguration('avia_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    
    # Robot State Publisher 
    robot_state_publisher = Node(package    ='robot_state_publisher',
                                 executable ='robot_state_publisher',
                                 name       ='robot_state_publisher',
                                 output     ='both',
                                 parameters =[{'robot_description': Command(['xacro', ' ', xacro_file])           
                                }])

    # Spawn the robot in Gazebo
    spawn_entity_robot = Node(package     ='gazebo_ros', 
                              executable  ='spawn_entity.py', 
                              arguments   = ['-entity', 'maviss_description', '-topic', 'robot_description'],
                              output      ='screen')
                              
    # Gazebo environment
    world_file_name = 'default.world'
    world = os.path.join(get_package_share_directory('maviss_description'), 'world', world_file_name)
    gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world,'-s', 'libgazebo_ros_factory.so'], output='screen')
        
    # Magnetometer node
    magnetometer_node = Node(
        package='maviss_description', 
        executable='magnetometer_node.py', 
        name='magnetometer_node',
        output='screen'
    )

    # Use parameter_blackboard as global parameters server and load camera params
    camera_launch = Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='parameter_blackboard',
        # namespace='laserMapping',
        parameters=[camera_params_file,],
        output='screen'
    )
        
    # Fastlivo mapping node
    fast_livo2_launch = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='laserMapping',
        parameters=[avia_params_file],
        output='screen'   
    ) 
        
    return LaunchDescription([
        avia_config_arg,
        camera_config_arg,
        robot_state_publisher,
        spawn_entity_robot,
        gazebo_node,
        magnetometer_node,
        camera_launch,
        fast_livo2_launch        
        ])
        
