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
    yaml_ncom = driver_params.pop("ncom", '""')
    yaml_prefix = driver_params.pop("topic_prefix", "ins")
    yaml_ip = driver_params.pop("unit_ip","0.0.0.0")
    yaml_port = driver_params.pop("unit_port",3000)
    yaml_rate = driver_params.pop("ncom_rate",100)
    yaml_mode = driver_params.pop("timestamp_mode",0)
    
    ins_param_path = os.path.join(ins_dir, "config", parameters_file_name)
    with open(ins_param_path, "r") as f:
        ins_params = yaml.safe_load(f)["oxts_ins"]["ros__parameters"]
    yaml_frameid = ins_params.pop("frame_id","oxts_link")
    yaml_lrf = ins_params.pop("lrf_source",2)
    
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    wait_for_init = LaunchConfiguration("wait_for_init", default="True")
    ncom = LaunchConfiguration("ncom", default=yaml_ncom)
    topic_prefix = LaunchConfiguration("topic_prefix", default=yaml_prefix)
    unit_ip = LaunchConfiguration("unit_ip",default=yaml_ip)
    unit_port = LaunchConfiguration("unit_port",default=yaml_port)
    ncom_rate = LaunchConfiguration("ncom_rate",default=yaml_rate)
    timestamp_mode = LaunchConfiguration("timestamp_mode",default=yaml_timestamp)
    
    frame_id = LaunchConfiguration("frame_id",default=yaml_frameid)
    lrf_source = LaunchConfiguration("lrf_source",default=yaml_lrf)
    # declare launch arguments (this exposes the argument
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
    declare_prefix = DeclareLaunchArgument(
        "topic_prefix", default_value=yaml_prefix, description="Prefix to apply to all topics"
    )
    declare_ip = DeclareLaunchArgument(
        "unit_ip", default_value=yaml_ip, description="Unit IP to connect to"
    )
    declare_port = DeclareLaunchArgument(
        "unit_port", default_value=str(yaml_port), description="Unit Port to connect to"
    )
    declare_rate = DeclareLaunchArgument(
        "ncom_rate", default_value=str(yaml_rate), description="Set ncom rate /Hz"
    )
    declare_mode = DeclareLaunchArgument(
        "timestamp_mode", default_value=str(yaml_timestamp), description="Set timestamp mode"
    )
    declare_frameid = DeclareLaunchArgument(
        "frame_id", default_value=yaml_frameid, description="Set id of frame"
    )
    declare_lrf = DeclareLaunchArgument(
        "lrf_source", default_value=str(yaml_lrf), description="Set the lrf source"
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
            {"topic_prefix": topic_prefix},
            {"ncom": ncom},
            {"unit_ip": unit_ip},
            {"unit_port": unit_port},
            {"ncom_rate": ncom_rate},
            {"timestamp_mode": timestamp_mode},
        ],
    )

    oxts_ins_node = Node(
        package="oxts_ins",
        executable="oxts_ins",
        name="oxts_ins",
        output="screen",
        parameters=[
            ins_params,
            {"use_sim_time": use_sim_time},
            {"topic_prefix": topic_prefix},
            {"frame_id": frame_id},
            {"lrf_source": lrf_source},
        ],
    )

    # create launch descroption and populate
    ld = LaunchDescription()
    # launch options
    ld.add_action(declare_ncom)
    ld.add_action(declare_prefix)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_wait_for_init)
    ld.add_action(declare_ip)
    ld.add_action(declare_port)
    ld.add_action(declare_rate)
    ld.add_action(declare_mode)
    ld.add_action(declare_frameid)
    ld.add_action(declare_lrf)
    # launch nodes
    ld.add_action(oxts_driver_node)
    ld.add_action(oxts_ins_node)

    return ld
