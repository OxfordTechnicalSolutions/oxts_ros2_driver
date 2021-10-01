# ROS2 Driver

[![pipeline status](https://gitlab.com/oxts/navigation/ros/oxts/badges/master/pipeline.svg)](https://gitlab.com/oxts/navigation/ros/oxts/-/commits/master)

A ROS2 driver which allows an OxTS INS to interact with a wider ROS network. Includes NCOM decoding and GAD encoding functionality. Where ROS is mentioned in this document this is in reference to ROS2. References to ROS1 will be explicit.

The ROS driver has been built with ROS release Foxy Fitzroy as a pre-built binary. The need has not arisen to build from source just yet.

## Build from source

Dependencies:

- ROS2 (Foxy Fitzroy). (See [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/) for an installation guide.)

```bash
sudo apt install doxygen
pip3 install sphinx breathe sphinx_rtd_theme
```

To build the driver:

```bash
. /opt/ros/foxy/setup.bash # "." and "source" are interchangeable (unix)
mkdir -p ros_ws/src
cd ros_ws/src
git clone https://gitlab.com/oxts/navigation/ros/oxts.git
cd oxts
rosdep update
rosdep install --from-path .
cd ../..
colcon build
. install/setup.sh
```

**NOTE:** The `rosdep install` command may fail, because the modules in this repository are not yet available as packages in any official repositories. This is fine, and error messages mentioning missing packages (`oxts`, `oxts_driver` and `oxts_ins`) can be ignored.

## Configuring and launching the driver

The driver is configured using .yaml files, as is the norm for ROS2 nodes. The default files in this folder contain lists of all configurable parameters for the nodes. These can be deleted to make the file smaller / tidier. Values not in the config file will remain as defaults.

Launch files can be used from /launch. Launch files are created in Python3 for ROS2, so be aware that Python3 will need to be installed on the machine. They can be use like so:

```bash
ros2 launch oxts run.py
```

or, to replay from an ncom file:

```bash
ros2 launch oxts run.py ncom:=<path_to_ncom> # absolute or relative
```

To view the Odometry and Tf data from the INS, use the `visualise.py` launch file. This requires RViz to be installed. There is also the option to start publishing before the NCOM has initialised (not recommended) `wait_for_init:=false`.

The currently available launch files are as follows:

* `visualise.py` - Launches the driver, as well as `robot_state_publisher` and `rviz2` (the latter can be disabled with `use_rviz:=False`)
* `run.py` - Only launches the driver, without `robot_state_publisher` and no `use_rviz` option

## Output ROS messages

The publisher node included in this driver opens a socket to receive NCOM messages from an INS. Data from the NCOM messages are then converted into ROS messages and published to ROS topics for consumption in a wider ROS network. Reference frames for each message can be found in headers. Where NCom is typically vehicle frame, ROS messages are output in INS/IMU frame.

**NOTE:** There are `*_rate` parameters in the YAML configuration, which prevent these topics from being published when set to 0. If topics are missing, ensure that rates have been configured for them.

* **ins/debug_string_pos** [std_msgs/msg/String](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/String.html)
    This message is not useful for general use. It is currently included for debug purposes. It contains a timestamp from NCom (`TimeWeekSecond`) and WGS84 coordinates (`Lat`, `Lon`, `Alt`) in string form, which is output to the console.

* **ins/ecef_pos** [geometry_msgs/msg/PointStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PointStamped.html)
    Contains a timestamped position of the INS in the ECEF reference frame (derived from `Lat`, `Lon` and `Alt` from NCOM).

* **ins/nav_sat_fix** [sensor_msgs/msg/NavSatFix](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)
    Contains a WGS84 position of the INS. This differs from standard use of the NavSatFix message in that the position is not taken directly from a GNSS receiver. It is instead taken from the INS output (`Lat`, `Lon` and `Alt`) and as a result, this message can be output at a higher rate than is typical with GNSS receivers. Covariance is derived from `EastAcc`, `NorthAcc` and `AltAcc`.

* **ins/nav_sat_ref** [oxts_msgs/msg/NavSatRef](./oxts_msgs/msg/NavSatRef.msg)
    Contains the WGS84 reference position currently being used to calculate the local coordinates for **ins/odometry**. This can either be the:
    * LRF in NCOM (`RefLat`, `RefLon`, `RefAlt` and `RefHeading`)
    * Position (`Lat`, `Lon`, `Alt`) & heading (`Heading`) of the first NCOM packet received
    * Position of the first NCOM packet received, aligned to ENU.

    (Depending on the parameters set in the `.yaml`)

* **ins/imu** [sensor_msgs/msg/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)
    Contains IMU data from the INS. Orientation is typically taken from magnetometers in this message; here it is taken from INS output. This comprises orientation in ENU frame (`Roll`, `Pitch` and `Heading` from NCOM), angular rates in the IMU frame (`Wx`, `Wy` and `Wz`), and linear accelerations in the IMU frame (`Ax`, `Ay` and `Az`).

* **ins/velocity** [geometry_msgs/msg/TwistStamped](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)
    Velocity of the INS, in the INS frame. (Linear velocity corresponds to measurements `IsoVoX`, `IsoVoY` and `IsoVoZ`, while angular velocity is `Wx`, `Wy` and `Wz`.)

* **ins/odometry** [nav_msgs/msg/Odometry](https://github.com/ros2/common_interfaces/blob/foxy/nav_msgs/msg/Odometry.msg)
    Odometry data from the INS.
    - Position: In a local reference frame which, depending on your configuration, is defined either by:
        - The LRF in NCom (`RefLat`, `RefLon`, `RefAlt` and `RefHeading`).
        - The first NCom packet (`Lat`, `Lon`, `Alt` and `Heading`).
        - The first NCom packet, aligned to ENU.
    - Orientation: Rotation of the INS relative to the alignment of the LRF (computed from `Roll`, `Pitch` and `Heading`)
    - Linear Velocity: In the above reference frame, computed from `IsoVoX`, `IsoVoY` and `IsoVoZ`. (Does not yet have variances.)
    - Angular Velocity: In the above reference frame, computed from `Wx`, `Wy` and `Wz`. (Does not yet have variances.)

* **ins/path** [nav_msgs/msg/Path](https://github.com/ros2/common_interfaces/blob/foxy/nav_msgs/msg/Path.msg)
    Path taken by the INS, this path contains all historical positions and orientations from the **ins/odometry** topic.

* **ins/time_reference** [sensor_msgs/msg/TimeReference](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/TimeReference.html)
    From `TimeWeekSecond` in the NCOM.

* **ins/lever_arm** [oxts_msgs/msg/LeverArm](./oxts_msgs/msg/LeverArm.msg)
    Lever arm offsets, in (x, y, z) coordinates. (From the `GAPx`, `GAPy` and `GAPz` NCOM measurements.) The lever arm type is specified by `lever_arm_id` in the message. The reference frame will depend on the lever arm type. Currently, only `gap` (IMU to Primary GPS Antenna offset) is broadcast.

* **ins/imu_bias** [oxts_msgs/msg/ImuBias](./oxts_msgs/msg/ImuBias.msg)
    Biases for the accelerometer (`AxBias`, `AyBias`, `AzBias`) and gyroscope (`WxBias`, `WyBias`, `WzBias`).

* **ins/ncom** [oxts_msgs/msg/Ncom](./oxts_msgs/msg/Ncom.msg)
    Raw NCOM data, output by the `/oxts_driver` node, to be subsequently split by the `/oxts_ins` node into all the messages listed here (besides this one).

\* Links are for ROS1 messages, which are largely unchanged, but equivalent documentation for ROS2 doesn't exist yet.

Useful sources of information around frames used for these messages can be found in:

- [REP103](https://www.ros.org/reps/rep-0103.html)
- [REP105](https://www.ros.org/reps/rep-0105.html#id8)

## ROS1 compatibility

ROS1 and ROS2 are not directly compatible, as a result of breaking changes between the two. As such, ROS2 messages cannot be consumed directly by ROS1 nodes. To get around this, the [ROS1 bridge](https://github.com/ros2/ros1_bridge) can be used to convert messages and send them on.

LINK - https://github.com/ros2/ros1_bridge

## Licences

The code is this project is licenced under the Apache 2.0 licence, a copy of which can be found in the [LICENCE](./LICENCE) file.

This project uses multiple third-party libraries, the use of each of which being subject to its own terms and conditions. Licences for third-party software are present in the [licences directory](./licences), along with the software they apply to.

# Developer notes (INTERNAL)

The driver has been developed on Ubuntu 20.04 using the Visual Studio Code IDE on a Windows 10 machine. This has been done using some useful VS Code packages:

- C/C++ : standard for developing C++
- Remote - SSH : Allows remote development via SSH. Developing in this way keeps the code local to the remote machine, not locally.
- colcon tasks (not actually used since I had some trouble getting it to source the ROS2 installation before building and documentation is sparse.)

In lieu of getting the colcon tasks package working, the following VS Code tasks.json file was used:

```json
{
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
}
```

This creates VS Code tasks for building a ROS2 / colcon workspace. Note that since this is for building a workspace, it should be at the workspace level, not at the package level. It's a bit hacky but it makes building easy. If anyone figures out the colcon tasks package that's probably the better approach.

## Formatting the code

To format the code automatically according to the [ROS style guide](http://wiki.ros.org/CppStyleGuide), you can use `clang-format` with the `.clang-format` file in the root of this repo (from  [here](https://github.com/PickNikRobotics/roscpp_code_format)).

To format some file:

```bash
clang-format -i -style=file file/to/format.cpp
```

`-style=file` walks up to `/` looking for a `.clang-format` configuration file, reading style options from the first file it finds. (Hence the `.clang-format` in the root of this repo).

To format all the files recursively under `/path/to/project/`:

```bash
find /path/to/project/ -iname '*.h' -or -iname '*.hpp' -or -iname '*.cpp' | xargs clang-format -i -style=file $1
```

**BEWARE** when doing this that some parts of your code, which you've formatted in a particular way on purpose, will get mangled. (See [this commit](https://gitlab.com/oxts/navigation/ros/oxts/-/commit/48b62a91bab3b6dc5fba81f93a748a1377050d35) for a good example of this problem being undone.) *So be careful to only format files which you've changed*, and check that the changes are sane before you commit.

You may want to configure your editor to format changes when you save. The [Clang-Format extension to Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format) is a good example; see its about page for how to configure it to do this (in particular adding `"editor.formatOnSave": true` to your `settings.json`).

## Intro to ROS2

ROS2 has some really nice intro documentation, and it should only improve over time.

## Contributing

Contributions to this project are welcome. Information on making contributions can be found in [CONTRIBUTING.md](./CONTRIBUTING.md).
