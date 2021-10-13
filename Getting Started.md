# What is the driver?
A collection of ROS 2 packages, providing nodes whose purpose is to:

1. Decode NCOM data from a live OxTS GNSS/INS device, or from a file.
2. Publish this data to ROS topics.

The effect is to provide compatibility with OxTS devices for your larger ROS system.

*Note: See the [prerequisites](#prerequisites) for more about ROS.*

# Prerequisites
Steps to complete/things to learn before following this tutorial.

## Install Ubuntu in VirtualBox
ROS 2 is officially supported on the latest stable version of Ubuntu, currently 20.04 (Focal Fossa), which will be used throughout this tutorial. (Selected for its popularity.) If you don't want to install Ubuntu on your computer, you can use a VirtualBox virtual machine inside a different host operating system.

- [VirtualBox manual](https://www.virtualbox.org/manual/).

  Reading the entire thing is not necessary; a basic understanding of what VirtualBox is will suffice. The [first chapter](https://www.virtualbox.org/manual/ch01.html) gives a good introduction.

- [Download Ubuntu Desktop 20.04](https://ubuntu.com/download/desktop).

- [Install it in VirtualBox](https://askubuntu.com/a/153098).

## Using the Bash shell
Much of this tutorial will involve using a command-line computer interface. In Linux, the Bash shell is by far the most commonly-used command interpreter, and comes installed by default in Ubuntu. Knowing some basic commands will make this tutorial easier to follow.

Opening the "terminal" application in your Ubuntu installation will give you a graphical window to enter commands into an instance of the Bash shell.

There are many beginners' introductions to Bash on the web; [here](https://programminghistorian.org/en/lessons/intro-to-bash) is a good one.

## Install ROS 2
Instructions for installing ROS, from official packages on Debian-based Linux distributions (e.g. Ubuntu) can be found [here](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
).

## Basics of ROS 2
There are quality, official, introductory tutorials for ROS 2 [here](https://docs.ros.org/en/galactic/Tutorials.html).

It is recommended to go through the "Beginner: CLI Tools" section for a basic understanding of ROS 2.

# Setup

## Setting up a workspace
To start using the ROS driver, we will need to create a workspace containing it. Essentially, a workspace is just a directory containing a collection of ROS packages. The point of workspaces is to configure your shell to have different ROS packages available, depending on which workspaces you've "activated" in the current session. For example, one might have different versions of ROS installed, each of which comprising a different "workspace", or collection of packages. These workspaces might have many packages in common, all of different versions. In each shell session, you can opt to "activate" the workspace corresponding to the version of ROS you'd like to use, allowing you to have multiple versions of ROS installed simultaneously, and easily decide which one to use at a given time. The first step to using the ROS driver is to configure a workspace which includes it, then "activate" that workspace.

What is meant by "activate"? Each ROS workspace comes with a setup script, which is a file containing a series of shell commands. When "sourcing" (or executing) this file, your shell will be configured to have certain ROS packages available. "Activating" a workspace is sourcing this file. For example, when running a ROS command such as `ros2 run some_package some_executable`, ROS needs to know where to look for a package named `some_package`. The setup script for the workspace containing `some_package` will configure your shell with this information when you source it (e.g. via environment variables). Then the `ros2 run` command will work, instead of producing an error.

You can source the setup script for any number of workspaces in a single shell session. When you do, the latest shell script overrides any setup done by previous ones. To explain, this means that if two workspaces contain the same package, whichever one was sourced later takes precedence. The words "overlay" and "underlay" are often used to refer to workspaces which override, or are overridden by, other workspaces, respectively. To use the ROS driver, we'll source the setup script for our ROS installation's workspace, which serves as the underlay. We'll also source a workspace we've created to contain the driver, acting as an overlay.

*[Here](https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html) is an official introduction to workspaces, if you want to learn more.*

To set up a workspace, we just create a directory for it, clone the driver's code into it, and build it. (For the time being, pre-built binaries are not available for this driver, hence the need to compile it ourselves.) Begin with making the workspace directory, and a directory inside it to put the driver's source code into, then change to that directory:

```bash
mkdir -p ros_ws/src # ros_ws for "ROS workspace"
cd ros_ws/src/
```

Now clone the driver's Git repository:

```bash
sudo apt install git # if you don't already have git
git clone https://github.com/OxfordTechnicalSolutions/oxts_ros2_driver
```

Source the underlay workspace, and ensure all needed dependencies are present in it:

```bash
source /opt/ros/galactic/setup.bash # path may vary on your system
cd oxts_ros2_driver
sudo apt install python3-rosdep2 # if you don't already have rosdep
rosdep update
rosdep install --from-path . # note; see below
cd ../..
sudo apt install python3-colcon-ros # if you don't already have colcon
colcon build
```

*NOTE: The `rosdep install` command may fail, because the modules in this repository are not yet available as packages in any official repositories. This is fine, and error messages mentioning missing packages (`oxts`, `oxts_driver` and `oxts_ins`) can be ignored.*

Now we just source the new setup script which has just been created for our workspace:

```bash
source install/setup.sh
```

In future, to start using the ROS driver, we just source the two workspaces. I.e.:

```bash
source /opt/ros/galactic/setup.bash
source /path/to/ros_ws/install/setup.sh
```

# Usage
The driver consists of two nodes: `/oxts_driver` and `/oxts_ins`.

- **`/oxts_driver`** - Publishes raw NCOM data.
- **`/oxts_ins`** - Decodes the raw NCOM data published by `/oxts_driver`, and re-publishes it in new topics with various message types.

The available topics are listed (and described) in [the README](./README.md).

## Starting
To start the driver:

```bash
ros2 launch oxts run.py
```

(To stop the driver, just press `^C` (`Ctrl-C`) in the terminal, which sends a signal to interrupt the running process (`SIGINT`). Wait for the driver to clean up and stop.)

## Options
The driver also accepts some options at the command-line:

- **`ncom`** - The path to an NCOM file to read, and publish data from. If this isn't given, NCOM data is read from a live device instead.
- **`topic_prefix`** - Prefix for every topic name; defaults to `ins`. E.g.  changing this to `my_ins` would produce topics such as `/my_ins/velocity`.
- **`wait_for_init`** - Whether to wait for NCOM initialisation before publishing messages, or start publishing them immediately. Expects a value of `True`/`False`; defaults to `True`.

To supply options:

```bash
ros2 launch oxts run.py ncom:=/path/to/ncom/file.ncom topic_prefix:=my_prefix wait_for_init:=False
```

You don't need to supply any of these; they all have default values.

## Configuration
Both the driver's nodes also read configuration values from e.g. `/path/to/ros_ws/install/oxts_ins/share/oxts_ins/config/default.yaml`, for the `/oxts_ins` node. (For the `/oxts_driver` node, substitute `oxts_ins` for `oxts_driver`.) More detailed information on the various configuration values is present within the configuration files themselves.

Some important examples:

- **`oxts_driver` → `unit_ip`/`unit_port`** - The IP address and port of the device to connect to when starting up (ignored when `ncom` is given in the command-line, or in this file).
- **`oxts_driver`/`oxts_ins` → `ncom_rate`** - The sample rate of the NCOM data being read. Must match the NCOM data being used.
- **`oxts_ins` → `/lrf_source`** - Source of the Local Reference Frame. Can use the value from the NCOM data, or set it automatically from the first NCOM packet (the latter of which can optionally be aligned to the ENU reference frame).

`oxts_ins` has configuration parameters for the message output rate of various topics, and their names.

**NOTE:** topics will not be published if they have a rate of zero, which many of them default to. If a topic is missing from the list, check that the rate has been set to something above zero.

The configuration is copied over from e.g. `/path/to/ros_ws/src/oxts_ros2_driver/oxts_ins/config/default.yaml` (in the case of `/oxts_ins`), as part of the build process (`colcon build`). Modifying this copy of the configuration won't do anything; ensure you're modifying e.g. `/path/to/ros_ws/install/oxts_ins/share/oxts_ins/config/default.yaml`.
