# ROS2-Baxter
Written by Shaun Altmann

## Using with Baxter
1. Open up a new terminal in the Ubuntu 20.04 machine you are using to connect with Baxter.
2. `git clone https://github.com/ShaunAlt/ROS2-Baxter.git`
3. `cd ROS2-Baxter\`
4. `bash setup.sh` (May take a couple of minutes).

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
Run `baxter_connect` in a new terminal to connect to Baxter and run the bridge.

## Running the MoveIT Controller
TODO: Create the Launch File for this.

## Running the ROS2 Baxter Main Controller
TODO: Finish creating this controller.
Currently there are many tests that can be completed with Baxter:
* Open up a new terminal and run the baxter bridge using `baxter_connect`.
* Whilst the bridge is running, open up a new terminal and go use `ros2_ws`.
* Then use `ros2 run baxter_dev ...` to run whichever tests you want. You can use `tab` to see the options.
* To see what each test does, go to `ros2_ws/src/baxter_dev/baxter_dev`.

## Streaming Camera Data from Baxter
