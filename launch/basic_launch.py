from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from pathlib import Path

parameters_file_name = 'ncom_publisher_default_config.yaml'


def generate_launch_description():
    # get current path and go one level up
    share_dir = get_package_share_directory('ros-driver')
    param_path = Path(share_dir, 'config', parameters_file_name)
    with open(param_path, 'r') as f:
        params = yaml.safe_load(f)['ncom_publisher']['ros__parameters']

    ncom = LaunchConfiguration('ncom', default='')
    params['ncom'] = ncom

    oxts_driver_node = Node(package='ros-driver',
                            #namespace='unit1',
                            executable='ncom_publisher',
                            # name='ncom_publisher',
                            output='screen',
                            parameters=[params])

    return LaunchDescription([
        oxts_driver_node,
    ])

    # Remappings can also be done in this launch file