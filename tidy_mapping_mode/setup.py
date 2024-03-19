from setuptools import setup, find_packages
package_name = 'tidy_mapping_mode'
setup(
    
    version='0.0.1',
    packages=find_packages(),
    install_requires=[
        # List your dependencies here if any
    ],
    scripts=[
             '/workspaces/AMR-Assessment/tidy_mapping_mode/mapping_behaviour/Robot_start.py',
             '/workspaces/AMR-Assessment/tidy_mapping_mode/mapping_behaviour/Laser_node.py',
             '/workspaces/AMR-Assessment/tidy_mapping_mode/mapping_behaviour/Position_node.py',
             '/workspaces/AMR-Assessment/tidy_mapping_mode/mapping_behaviour/Robot_move.py'],  # Specify executable scripts here
    data_files=[('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                ('share/' + package_name, ['launch/mapping.launch.py']),
                

                
                
                ],
    package_data={'': ['launch/*.launch.py']},  # Include launch files her
)