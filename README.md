# ROS2 Driver

A ROS2 driver which allows an OxTS INS to interact with a wider ROS network. Includes NCOM decoding and GAD encoding functionality. Where ROS is mentioned in this document this is in reference to ROS2. References to ROS1 will be explicit.

## Build from source

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


## Output ROS messages

The publisher node included in this driver opens a socket to receive NCOM messages from an INS. Data from the NCOM messages are then converted into ROS messages and published to ROS topics for consumption in a wider ROS network.

## Input ROS messages

The subscriber node included in this driver listens for particular ROS topics to be sent from external aiding devices. These messages are converted from ROS to Generic Aiding messages, which are then sent to an INS. This allows straightforward integration of ROS devices as aiding sources to an OxTS INS.

## ROS1 compatibility

ROS1 and ROS2 are not directly compatible, as a result of breaking changes between the two. As such, ROS2 messages cannot be consumed directly by ROS1 nodes. To get around this, the ROS1 bridge can be used to convert messages and send them on.

LINK - https://github.com/ros2/ros1_bridge



