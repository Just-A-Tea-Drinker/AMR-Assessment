import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
#from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node,SetRemap
from launch.actions import ( GroupAction,IncludeLaunchDescription)

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('tidy_cleaning_mode'))
    map_file = '/workspaces/AMR-Assessment/World_map.yaml'
    param_file = os.path.join(config_dir,'limo_config.yaml')
    twist_file = os.path.join(config_dir,'twist_mux.yaml')

    return LaunchDescription([
       
        GroupAction(
            actions=[

                SetRemap(src='/cmd_vel',dst='/cmd_nav2_vel'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
                    launch_arguments = {
                        
                        'map' : map_file,
                        'autostart' : 'true',
                        'params_file' : param_file

                    }.items(),

                )
            ]
        ),
        #setting up twist mux
       
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[
            # Adjust parameters as needed
            twist_file
            
            ],
            remappings=[('/cmd_vel_out', '/cmd_vel')]
            # Add any necessary parameters here
        ),
        #seting up the robot nodes
        # Node(
        #     package=  'tidy_cleaning_mode',
        #     executable='Robot_start.py',
            
        #     output='screen', 
        # ),
        # Node(
        #     package=  'tidy_cleaning_mode',
        #     executable='Robot_pathplanning.py',
            
        #     output='screen', 
        # ),


    
    ])






    

   
    
    
    
    