# ROS2 Driver

[![pipeline status](https://gitlab.com/oxts/navigation/generic-aiding/oxts/badges/master/pipeline.svg)](https://gitlab.com/oxts/navigation/generic-aiding/oxts/-/commits/master)

A ROS2 driver which allows an OxTS INS to interact with a wider ROS network. Includes NCOM decoding and GAD encoding functionality. Where ROS is mentioned in this document this is in reference to ROS2. References to ROS1 will be explicit.

The ROS driver has been built with ROS release Foxy Fitzroy as a pre-built binary. The need has not arisen to build from source just yet. 

## Build from source

Dependencies:

- ROS2 (Foxy Fitzroy). For an install guide, see the bottom of this README for a link to ROS documentation.

```bash
sudo apt install doxygen
pip3 install sphinx breathe sphinx_rtd_theme
```

Build instructions will look something like this:

1. . /opt/ros/foxy/setup.bash
2. cd <colcon_ws>/src
3. git clone https://gitlab.com/oxts/navigation/ros/oxts.git
4. cd oxts
5. rosdep update
6. rosdep install --from-path .
7. cd ../..
8. colcon build
9. source install/setup.bash


## Configuring and Launching the Driver

The driver is configured using .yaml files, as is the norm for ROS2 nodes. These should be kept in /config, and can be used to configure the node at run time like so:

```bash
    ros2 run oxts_driver ncom_publisher --ros-args --params-file  ~/code/ros2_ws/src/oxts_driver/config/ncom_publisher_default_config.yaml
```

The default files in this folder contain lists of all configurable parameters for the nodes. These can be deleted to make the file smaller / tidier. Values not in the config file will remain as defaults.

If you (quite rightly) don't want to write that into the command line each time, launch files can be used from /launch. Launch files are created in Python3 for ROS2, so be aware that Python3 will need to be installed on the machine.

To launch only the ncom publisher, use basic_launch.py like so:

```bash
    ros2 launch oxts_driver basic_launch.py
```

or, to replay from an ncom file:

```bash
    ros2 launch oxts_driver basic_launch.py ncom:=<absolute_path_to_ncom>
```


## Output ROS messages

The publisher node included in this driver opens a socket to receive NCOM messages from an INS. Data from the NCOM messages are then converted into ROS messages and published to ROS topics for consumption in a wider ROS network. Reference frames for each message can be found in headers. Where NCom is typically vehicle frame, ROS messages are output in INS/IMU frame.

* **ins/debug_string_pos** [std_msgs/msg/String](http://docs.ros.org/en/melodic/api/std_msgs/html/msg/String.html)
    This message is not useful for general use. It is currently included for debug purposes. It contains a timestamp from NCom and WGS84 coordinates in string form, which is output to the console.
* **ins/ecef_pos** [geometry_msgs/msg/PointStamped](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PointStamped.html)
    Contains a timestamped position of the INS in the ECEF reference frame.
* **ins/nav_sat_fix** [sensor_msgs/msg/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)
    Contains a WGS84 position of the INS. This differs from standard use of the NavSatFix message in that the position is not taken directly from a GNSS receiver. It is instead taken from the INS output and as a result, this message can be output at a higher rate than is typical with GNSS receivers.
* **imu/data** [sensor_msgs/msg/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)
    Contains IMU data from the INS, including orientation, angular rates, and linear accelerations. Orientation is typically taken from magnetometers in this message. Here it is taken from INS output.
* **ins/velocity** [geometry_msgs/msg/TwistStamped](http://docs.ros.org/en/hydro/api/geometry_msgs/html/msg/TwistStamped.html)
    Velocity of the INS, in the INS frame.
* **ins/time_reference** [sensor_msgs/msg/TimeReference](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/TimeReference.html)

Useful sources of information around frames used for these messages can be found in:

- [REP103](https://www.ros.org/reps/rep-0103.html)
- [REP105](https://www.ros.org/reps/rep-0105.html#id8)

## Input ROS messages

Subscriber node not yet created.

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
