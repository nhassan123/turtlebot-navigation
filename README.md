# Turtlebot Autonomous Navigation
This repository contains code for a ROS2 Node to navigate on it's own in an unknown environment. It has been tested with a Turtlebot3 simulation using Gazebo.

## Getting started

After cloning the repository, you will need to build the package. This is best done from the main ROS2 workspace directory.

``` cd ~/ros_ws/ # mofidy to name of your workspace directory as needed ```
``` colcon build --packages-select turtlebot_navigation ```
``` source install/setup.sh ```

Next, set up the simulation environment in Gazebo. Open a new terminal and type the following commands.

``` export TURTLEBOT3_MODEL = waffle ```
``` ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py ```

This should start up Gazebo with a Turtlebot model in the environment. 

Return to the first terminal window and type the following command:

``` ros2 run turtlebot_navigation turtle_controller ```

You should now be able to see the Turtlebot moving in the Gazebo simulation. When an object is detected in front of it, it will stop moving