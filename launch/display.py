import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

parameters_file_name = 'ncom_publisher_default_config.yaml'
urdf_file_name = 'medium.urdf.xml'


def generate_launch_description():
    # get current path and go one level up
    share_dir = get_package_share_directory('ros-driver')
    param_path = os.path.join(share_dir, 'config', parameters_file_name)
    urdf_path = os.path.join(share_dir, 'urdf', urdf_file_name)
    rviz_path = os.path.join(share_dir, 'rviz', 'display.rviz')
    with open(param_path, 'r') as f:
        params = yaml.safe_load(f)['ncom_publisher']['ros__parameters']

    use_sim_time = LaunchConfiguration('use_tim_time', default='false')
    # use_rviz = LaunchConfiguration('use_rviz', default='false')
    ncom = LaunchConfiguration('ncom', default='')
    params['ncom'] = ncom

    rviz_args = ['-f', 'base_link',
                 '-d', rviz_path]

    launch_argument = DeclareLaunchArgument(
        'use_tim_time',
        default_value='false')
    # declare_use_rviz = DeclareLaunchArgument(
    #     'use_rviz',
    #     default_value=False,
    #     help='Whether to start RVIZ')


    oxts_driver_node = Node(
        package='ros-driver',
        #namespace='unit1',
        executable='ncom_publisher',
        # name='ncom_publisher',
        output='screen',
        parameters=[params])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # parameters=[{'use_tim_time': use_sim_time}],
        arguments=[urdf_path])

    rviz_cmd = Node(
        # condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=rviz_args)


    # create launch descroption and populate
    ld = LaunchDescription()
    # launch options
    ld.add_action(launch_argument)
    # ld.add_action(declare_use_rviz)
    # launch nodes
    ld.add_action(oxts_driver_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz_cmd)

    return ld