import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.conditions import IfCondition, LaunchConfigurationNotEquals

parameters_file_name = "default.yaml"


def generate_launch_description():
    # get current path and go one level up
    driver_dir = get_package_share_directory("oxts_driver")
    ins_dir = get_package_share_directory("oxts_ins")

    driver_param_path = os.path.join(driver_dir, "config", parameters_file_name)
    with open(driver_param_path, "r") as f:
        driver_params = yaml.safe_load(f)["oxts_driver"]["ros__parameters"]
    yaml_ncom = driver_params.pop("ncom", "")

    ins_param_path = os.path.join(ins_dir, "config", parameters_file_name)
    with open(ins_param_path, "r") as f:
        ins_params = yaml.safe_load(f)["oxts_ins"]["ros__parameters"]

    use_sim_time = LaunchConfiguration("use_tim_time", default="False")
    wait_for_init = LaunchConfiguration("wait_for_init", default="True")
    ncom = LaunchConfiguration("ncom", default=yaml_ncom)

    # declare launch arguments (this exposes the arcument
    # to IncludeLaunchDescriptionand to the command line)
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="False")
    declare_wait_for_init = DeclareLaunchArgument(
        "wait_for_init",
        default_value="True",
        description="Whether to publish before NCOM initialisation",
    )
    declare_ncom = DeclareLaunchArgument(
        "ncom", default_value=yaml_ncom, description="NCOM file to replay (optional)"
    )

    oxts_driver_node = Node(
        package="oxts_driver",
        executable="oxts_driver",
        name="oxts_driver",
        output="screen",
        parameters=[
            driver_params,
            {"use_sim_time": use_sim_time},
            {"wait_for_init": wait_for_init},
            {"ncom": ncom},
        ],
    )

    oxts_ins_node = Node(
        package="oxts_ins",
        executable="oxts_ins",
        name="oxts_ins",
        output="screen",
        parameters=[ins_params, {"use_sim_time": use_sim_time}],
    )

    # create launch descroption and populate
    ld = LaunchDescription()
    # launch options
    ld.add_action(declare_ncom)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_wait_for_init)
    # launch nodes
    ld.add_action(oxts_driver_node)
    ld.add_action(oxts_ins_node)

    return ld
