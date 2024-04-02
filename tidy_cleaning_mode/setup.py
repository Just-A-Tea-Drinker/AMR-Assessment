from setuptools import setup, find_packages
package_name = 'tidy_cleaning_mode'
setup(
    
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    scripts=[
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/config/limo_config.yaml',
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/config/rviz_config.rviz',
             
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/cleaning_behaviour/Robot_start.py',
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/cleaning_behaviour/Beh_Controller.py',
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/cleaning_behaviour/Laser_node.py',
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/cleaning_behaviour/Position_node.py',
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/cleaning_behaviour/IMGpro.py',
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/cleaning_behaviour/BoxSort.py',
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/cleaning_behaviour/Robot_pathplanning.py',
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/cleaning_behaviour/Robot_move.py',
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/cleaning_behaviour/local_pathfinding.py',
             '/workspaces/AMR-Assessment/tidy_cleaning_mode/cleaning_behaviour/global_pathplanning.py',
             #'/workspaces/AMR-Assessment/tidy_cleaning_mode/cleaning_behaviour/.py',
                ],  # Specify executable scripts here
    data_files=[('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                ('share/' + package_name, ['launch/tidy_cleaning.launch.py']),
                ('share/' + package_name, ['config/limo_config.yaml']),
                ('share/' + package_name, ['config/twist_mux.yaml']),

                #('share/' + package_name, ['config/tidy_cleaning.launch.py']),
                

                
                
                ],
    package_data={'': ['launch/*.launch.py','config/*.yaml']},  # Include launch files her
)
