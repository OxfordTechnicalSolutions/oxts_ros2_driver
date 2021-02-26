import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

parameters_file_name = 'default.yaml'
urdf_file_name = 'medium.urdf.xml'


def generate_launch_description():
    # get current path and go one level up
    driver_dir = get_package_share_directory('oxts_driver')
    ins_dir = get_package_share_directory('oxts_ins')

    driver_param_path = os.path.join(driver_dir, 'config', parameters_file_name)
    with open(driver_param_path, 'r') as f:
        driver_params = yaml.safe_load(f)['oxts_driver']['ros__parameters']

    ins_param_path = os.path.join(ins_dir, 'config', parameters_file_name)
    urdf_path = os.path.join(ins_dir, 'urdf', urdf_file_name)
    rviz_path = os.path.join(ins_dir, 'rviz', 'display.rviz')
    with open(ins_param_path, 'r') as f:
        ins_params = yaml.safe_load(f)['oxts_ins']['ros__parameters']

    use_sim_time = LaunchConfiguration('use_tim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz')
    wait_for_init = LaunchConfiguration('wait_for_init')
    ncom = LaunchConfiguration('ncom', default='')
    driver_params['ncom'] = ncom
    driver_params['wait_for_init'] = wait_for_init

    # declare launch arguments
    launch_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')
    declare_wait_for_init = DeclareLaunchArgument(
        'wait_for_init',
        default_value='True',
        description='Whether to publish before NCOM initialisation')
    declare_ncom = DeclareLaunchArgument(
        'ncom',
        default_value='',
        description='NCOM file to replay (optional)')


    oxts_driver_node = Node(
        package='oxts_driver',
        #namespace='unit1',
        executable='oxts_driver',
        name='oxts_driver',
        output='screen',
        parameters=[driver_params])

    oxts_ins_node = Node(
        package='oxts_ins',
        #namespace='unit1',
        executable='oxts_ins',
        name='oxts_ins',
        output='screen',
        parameters=[ins_params])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_path])

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_path])


    # create launch descroption and populate
    ld = LaunchDescription()
    # launch options
    ld.add_action(launch_argument)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_wait_for_init)
    # launch nodes
    ld.add_action(oxts_driver_node)
    ld.add_action(oxts_ins_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz_cmd)

    return ld