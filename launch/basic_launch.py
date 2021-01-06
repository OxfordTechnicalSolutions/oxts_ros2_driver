from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

parameters_file_name = 'ncom_publisher_default_config.yaml'


def generate_launch_description():
    # get current path and go one level up
    parameters_file_path = Path(get_package_share_directory('ros-driver'),'config', parameters_file_name )
    print(parameters_file_path)

    return LaunchDescription([
        Node(
            package='ros-driver',
            #namespace='unit1',
            executable='ncom_publisher',
            name='ncom_publisher',
            output='screen',
            parameters=[
                parameters_file_path
            ],
        ),
    ])

    # Remappings can also be done in this launch file