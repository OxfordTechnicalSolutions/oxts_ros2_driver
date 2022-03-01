# Contributing

## Compilation
Please see [the README](./README.md) for compilation instructions.

## Environment set-up
This driver is IDE-agnostic, so you can set up your development environment however you please. So far however, it has been developed on Ubuntu 20.04 using the Visual Studio Code IDE on a Windows 10 machine. This has been done using some useful VS Code packages:

- [**C/C++**](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools) Standard for developing C++.
- [**Remote - SSH**](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh) Allows remote development via SSH. Developing in this way keeps the code local to the remote machine, not locally.

The following VS Code `tasks.json` file was used:

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
            "args":["source /opt/ros/galactic/setup.bash  && cd ~/code/ros2_ws/ && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug && . install/setup.bash"],
        },
    ]
}
```

This creates VS Code tasks for building a ROS 2 / colcon workspace. Note that since this is for building a workspace, it should be at the workspace level, not at the package level. This makes building easier.

A suggested alternative is the ["Colcon Tasks" package](https://marketplace.visualstudio.com/items?itemName=deitry.colcon-helper).

## Licensing
This project is licensed under the Apache 2.0 licence; see the [LICENCE file](./LICENCE) for details.

This project also uses third-party code; relevant licences can be found under the [licences/ directory](./licences/).

Be aware that any modifications or contributions made to this driver are subject to the terms of the relevant licences. Please ensure you are meeting these conditions; for example, third-party code used when contributing may be subject to a licence stipulating that copies of it are distributed alongside derivative works. In which case, a new licensor must be listed in the appropriate file under the `licences/` directory, and a new licence file may need to be added.

## Project structure
This project is split into numerous modules, each in their own directory:

- **oxts/** A small ROS package wrapping `oxts_driver` and `oxts_ins`, providing a launch file `run.py` to load the configurations and start them both.
- **oxts_driver/** A ROS node which listens for data emitted from an OxTS device, and re-packages it into a ROS topic `/ins/ncom`, which just contains raw NCOM data.
- **oxts_ins/** A ROS node which subscribes to `oxts_driver`, decodes the raw NCOM data, and splits it into the ROS topics it publishes.
- **oxts_msgs/** A ROS package containing OxTS ROS message definitions, to describe data output from the two nodes.

A list of ROS topics published by this driver can be found in [the README](./README.md).

### Releases
* Release versions of this driver reside in branches corresponding to the distribution of ROS they are designed for use with, e.g. `galactic` or `eloquent`.
* Branches which are under development, and in the progress of updating the code for use with a new distribution of ROS, are suffixed with `-devel`, e.g. `galactic-devel`.
* The `master` branch contains a release for the latest distribution of ROS.
* The ROS driver uses [semantic versioning](https://semver.org/) for its version number.

## Code conventions
The code in this driver conforms to the [ROS C++ style guide](http://wiki.ros.org/CppStyleGuide). **Contributions must take care to follow this style guide also** (where possible) for consistency.

To format the code automatically according to the ROS style guide, you can use `clang-format` with the [.clang-format](/.clang-format) file in the root of this repository (from  [here](https://github.com/PickNikRobotics/roscpp_code_format)).

To format some file:

```bash
clang-format -i -style=file file/to/format.cpp
```

`-style=file` walks up to `/` looking for a `.clang-format` configuration file, reading style options from the first file it finds. (Hence the `.clang-format` in the root of this repo).

To format all the files recursively under `/path/to/project/`:

```bash
find /path/to/project/ -iname '*.h' -or -iname '*.hpp' -or -iname '*.cpp' | xargs clang-format -i -style=file $1
```

**BEWARE** when doing this that some parts of your code, which you've formatted in a particular way on purpose, will get mangled. (See [this commit](https://github.com/OxfordTechnicalSolutions/oxts_ros2_driver/commit/48b62a91bab3b6dc5fba81f93a748a1377050d35) for a good example of this problem being undone.) *So be careful to only format files which you've changed*, and check that the changes are sane before you commit.

You may want to configure your editor to format changes when you save. The [Clang-Format extension to Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format) is a good example; see its about page for how to configure it to do this (in particular adding `"editor.formatOnSave": true` to your `settings.json`).

The `clang-format` package on Ubuntu also provides a sub-command for Git, executed via `git clang-format`, which formats all the lines which have changed since the last commit. To execute it, run:

```bash
# In the root of the repository, i.e. next to your .git/ directory.
git add some/file.cpp         # Stage the changes you would like to format.
git stash -k                  # Stash anything unstaged before formatting.
git clang-format --style file # Format changed lines.
git stash pop                 # Pop the stash to restore your unstaged changes.
```

Again, check to ensure the changes are acceptable to you:

```bash
git diff
```

## Unit tests
This driver also contains a [tests module](./tests/), containing unit tests.

### Running
Building the test program requires something like:

```bash
# In the root directory of the compiled package, e.g. ~/ros_ws/
. /opt/ros/galactic/setup.bash
. install/setup.sh
colcon build
```

To execute it:

```bash
./install/tests/lib/tests/tests
```

This program accepts a `--show-progress` (or `-p`) flag, as well as a `--log-level=` flag to adjust the verbosity of the output, e.g. `--log-level=all`. This program also accepts a `--help` flag, which lists all the options.

### Maintaining
This driver uses the `Boost.Test` framework for its unit tests. Each file in the [tests/ directory](./tests/) tests public functionality in the driver's other modules. Each one contains its own test suite, and most of them contain a "`Fixture`" for holding common state between tests. See those files for examples.

Documentation on this unit testing framework, including an API reference and user's guide, can be found [here](https://www.boost.org/doc/libs/1_77_0/libs/test/doc/html/index.html).

## Learning ROS 2
See the following short tutorials for a quality introduction to package development for ROS 2 (Galactic Geochelone):

- [Installation](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).
- [Create a workspace](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/).
- [Create your first package](https://index.ros.org/doc/ros2/Tutorials/Creating-Your-First-ROS2-Package/).
- [Create a simple publisher/subscriber pair](https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/#cpppubsub).

There are many more advanced tutorials following these.
