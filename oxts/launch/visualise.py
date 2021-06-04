import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

urdf_file_name = "medium.urdf.xml"


def generate_launch_description():
    # get current path and go one level up
    oxts_dir = get_package_share_directory("oxts")
    ins_dir = get_package_share_directory("oxts_ins")

    rviz_path = os.path.join(ins_dir, "rviz", "display.rviz")

    urdf_path = os.path.join(ins_dir, "urdf", urdf_file_name)
    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    use_sim_time = LaunchConfiguration("use_tim_time", default="False")
    use_rviz = LaunchConfiguration("use_rviz", default="True")
    wait_for_init = LaunchConfiguration("wait_for_init", default="True")
    ncom = LaunchConfiguration("ncom", default="")
    topic_prefix = LaunchConfiguration("topic_prefix", default="ins")


    # declare launch arguments (this exposes the arcument
    # to IncludeLaunchDescriptionand to the command line)
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="False")
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )
    declare_wait_for_init = DeclareLaunchArgument(
        "wait_for_init",
        default_value="True",
        description="Whether to publish before NCOM initialisation",
    )
    declare_ncom = DeclareLaunchArgument(
        "ncom", default_value="", description="NCOM file to replay (optional)"
    )
    declare_prefix = DeclareLaunchArgument(
        "topic_prefix", default_value="ins", description="Prefix to apply to all topics"
    )

    # driver launch file
    launch_description = PythonLaunchDescriptionSource(f"{oxts_dir}/launch/minimal.py")
    oxts_launch = IncludeLaunchDescription(
        launch_description_source=launch_description,
        launch_arguments={
            "ncom": ncom,
            "topic_prefix": topic_prefix,
            "use_sim_time": use_sim_time,
            "wait_for_init": wait_for_init,
        }.items(),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_desc,
            }
        ],
        arguments=[urdf_path],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
    )

    # create launch descroption and populate
    ld = LaunchDescription()
    # launch options
    ld.add_action(declare_ncom)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_wait_for_init)
    # launch nodes
    ld.add_action(oxts_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld
