# ROS2-Baxter
Written by Shaun Altmann

## Using with Baxter
1. Open up a new terminal in the Ubuntu 20.04 machine you are using to connect with Baxter.
2. Install `git` (if not already installed) by running `sudo apt install git`.
3. `git clone https://github.com/ShaunAlt/ROS2-Baxter.git`.
4. `cd ROS2-Baxter\`.
5. `bash setup.sh` (May take a couple of minutes).

## Aliases
The following aliases are defined in the `setup.sh` shell script:
* `baxter_bridge` opens up the `ros2_ws` and then runs the `bridge.sh` shell script using `bash`. This connects ROS1 Baxter topics to ROS2, however it requires that
    `baxter_init` has already been run.
* `baxter_connect` runs `baxter_init` followed by `baxter_bridge`, effectively initializing and then creating the baxter bridge.
* `baxter_init` opens up the `ros_ws` and then runs the `init.sh` shell script using `bash`. This connects ROS1 to Baxter.
* `baxter_ping` pings the Baxter robot using the `ping` command.
* `cd_moveit` uses the `cd` command to go into the `moveit_ws` workspace.
* `cd_ros1` uses the `cd` command to go into the `ros_ws` workspace.
* `cd_ros2` uses the `cd` command to go into the `ros2_ws` workspace.
* `src_galactic` sources the parent ROS2 Galactic `setup.bash` file.
* `src_noetic` sources the parent ROS1 Noetic `setup.bash` file.
* `src_ws1` sources the current ROS1 workspace. Requires that the user is currently in the root of an already built ROS1 workspace.
* `src_ws2` sources the current ROS2 workspace. Requires that the user is currently in the root of an already built ROS2 workspace.
* `moveit_ws` runs `cd_moveit` followed by `src_ws1`.
* `ros_ws` runs `cd_ros1` followed by `src_ws`.
* `ros2_ws` runs `cd_ros2` followed by `src_ws2`.

## Starting up the Bridge
Make sure that Baxter is plugged in to the machine and the network settings have been set so that only the connection to Baxter is being used.
Run `run_bridge` in a new terminal to connect to Baxter and run the bridge.

## Running the MoveIT Controller
Open a new terminal, and run the following command: `run_moveit`.

## Running the ROS2 Baxter Main Controller
Open a new terminal, and run the following command: `run_sweeper`.
Instead of running `run_bridge`, `run_moveit`, `run_image`, and `run_sweeper` all in different tabs, you can simply do `run_all` instead.

## Streaming Camera Data from Baxter
Open a new terminal, then run the following command: `run_image`.
