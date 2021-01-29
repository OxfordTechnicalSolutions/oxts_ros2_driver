from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml
from pathlib import Path

parameters_file_name = 'ncom_publisher_default_config.yaml'
urdf_file_name = 'medium.urdf.xml'


def generate_launch_description():
    # get current path and go one level up
    share_dir = get_package_share_directory('ros-driver')
    param_path = Path(share_dir, 'config', parameters_file_name)
    urdf_path = str(Path(share_dir, 'urdf', urdf_file_name))
    with open(param_path, 'r') as f:
        params = yaml.safe_load(f)['ncom_publisher']['ros__parameters']

    use_sim_time = LaunchConfiguration('use_tim_time', default='false')
    ncom = LaunchConfiguration('ncom', default='')
    params['ncom'] = ncom


    oxts_driver_node = Node(package='ros-driver',
                            #namespace='unit1',
                            executable='ncom_publisher',
                            # name='ncom_publisher',
                            output='screen',
                            parameters=[params])

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='screen',
                                 parameters=[{'use_tim_time': use_sim_time}],
                                 arguments=[urdf_path])

    launch_argument = DeclareLaunchArgument('use_tim_time',
                                            default_value='false')

    return LaunchDescription([
        launch_argument,
        oxts_driver_node,
        robot_state_publisher
    ])

    # Remappings can also be done in this launch file