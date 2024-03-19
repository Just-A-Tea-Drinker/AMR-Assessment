
 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Set the path to different files and folders.  
    #tidy_launch = FindPackageShare(package='tidy_mapping').find('tidy_mapping')   
    
    # Launch SLAM Toolbox nodes
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name = 'slam',
        parameters=[
            # Adjust parameters as needed
            {'use_sim_time': True},  # Set to True if using simulated time
            
        ],
    
        output='screen',
    )
    robot_control = Node(
        package=  'tidy_mapping_mode',
        executable='Robot_start.py',
        name="tidy_mapping",
        output='screen',

    )
    
    # Create the launch description
    ld = LaunchDescription()

    # Add nodes to the launch description
    
  
    ld.add_action(robot_control)
    ld.add_action(slam_toolbox_node)
    return ld






