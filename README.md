# ROS2 Driver

A ROS2 driver which allows an OxTS INS to interact with a wider ROS network. Includes NCOM decoding and GAD encoding functionality. Where ROS is mentioned in this document this is in reference to ROS2. References to ROS1 will be explicit.

The ROS driver has been built with ROS release Foxy Fitzroy as a pre-built binary. The need has not arisen to build from source just yet. 

## Build from source

Dependencies:

- ROS2 (Foxy Fitzroy)

Build instructions will look something like this:

1. cd <colcon_ws>/src
2. git clone https://gitlab.com/oxts/navigation/generic-aiding/ros-driver.git
3. cd ros-driver
4. rosdep update
5. rosdep install --from-path .
6. cd ../..
7. colcon build
8. source install/setup.bash


## Driver Configuration

The driver is configured using .yaml files, as is the norm for ROS2 nodes. These should be kept in /config, and can be used to configure the node at run time like so:

`{
    ros2 run ros-driver ncom_publisher --ros-args --params-file  ~/code/ros2_ws/src/ros-driver/config/ncom_publisher_default_config.yaml
}'

The default files in this folder contain lists of all configurable parameters for the nodes. These can be deleted to make the file smaller / tidier. Values not in the config file will remain as defaults.

If you (quite rightly) don't want to write that into the command line each time, launch files can be used from /launch. Launch files are created in Python3 for ROS2, so be aware that Python3 will need to be installed on the machine.


## Output ROS messages

The publisher node included in this driver opens a socket to receive NCOM messages from an INS. Data from the NCOM messages are then converted into ROS messages and published to ROS topics for consumption in a wider ROS network.

## Input ROS messages

The subscriber node included in this driver listens for particular ROS topics to be sent from external aiding devices. These messages are converted from ROS to Generic Aiding messages, which are then sent to an INS. This allows straightforward integration of ROS devices as aiding sources to an OxTS INS.

## ROS1 compatibility

ROS1 and ROS2 are not directly compatible, as a result of breaking changes between the two. As such, ROS2 messages cannot be consumed directly by ROS1 nodes. To get around this, the ROS1 bridge can be used to convert messages and send them on.

LINK - https://github.com/ros2/ros1_bridge



# Developer notes (INTERNAL)

The driver has been developed on Ubuntu 20.04 using the Visual Studio Code IDE on a Windows 10 machine. This has been done using some useful VS Code packages:

- C/C++ : standard for developing C++
- Remote - SSH : Allows remote development via SSH. Developing in this way keeps the code local to the remote machine, not locally.
- colcon tasks (not actually used since I had some trouble getting it to source the ROS2 installation before building and documentation is sparse.)

In lieu of getting the colcon tasks package working, the following VS Code tasks.json file was used:

`{
    "version": "2.0.0",
    "command": "bash",
    "args": [
        "-c"
    ],
    "tasks": [
        {
            "type": "shell",
            "label": "source",
            "args":["cd ~/code/ros2_ws/ && . install/setup.bash"],
        },
        {
            "type": "shell",
            "label": "clean",
            "args":["cd ~/code/ros2_ws/ && rm -rf build/ install/ log/ && unset AMENT_PREFIX_PATH && unset CMAKE_PREFIX_PATH"],
        },
        {
            "type": "shell",
            "label": "colcon build",
            "args":["source /opt/ros/foxy/setup.bash  && cd ~/code/ros2_ws/ && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug && . install/setup.bash"],
        },
    ]
}`

This creates VS Code tasks for building a ROS2 / colcon workspace. Note that since this is for building a workspace, it should be at the workspace level, not at the package level. It's a bit hacky but it makes building easy. If anyone figures out the colcon tasks package that's probably the better approach.

## Intro to ROS2

ROS2 has some really nice intro documentation, and it should only improve over time. 

- Installation: https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/
- Create a workspace: https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/
- Create your first package: https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/
- Create a simple publisher/subscriber pair: https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/#cpppubsub

These tutorials don't take long and provide a pretty good introduction to get you set up and making very simple ROS packages. There are plenty more after these, depending how far you want to go.
