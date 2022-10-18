# MoveIt Packages with Python Scripts Created for ME 396P Lightning Talk, Fall 2022, Team 09

## Contents
1. **ur_moveit_config**
    * The ur_moveit_config package contains the simplest possible MoveIt configuration for a Universal Robots UR3 manipulator developed using the MoveIt Setup Assistant. It depends on UR-provided packages which can be found at the [Universal Robots ROS Drivers](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) page.
2. **ur_scripts**
    * The ur_scripts package contains the Python code used to make the robot compute and execute operations. Similar executable movement scripts can be easily created within this package.
---

## Requirements, Dependencies, and Building
These packages are built and tested on a system running ROS1 noetic on Ubuntu 20.04. Users are assumed to already have ROS noetic installed on a machine running Ubuntu 20.04 to execute this demonstration. Details of ROS installation can be found on the [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials) webpage.

Use of these packages in a non-simulated environment requires the use of the official [Universal Robots ROS Drivers](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).   
1. Create a Catkin workspace:
```console
mkdir -p catkin_ws/src && cd catkin_ws
```
2. Clone the contents of this repository:
```console
git clone https://github.com/steven-swanbeck/ur3_calculator.git src/lightning_talk
```
3. Clone the UR Robots ROS Driver:
```console
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver
```
4. And the associated description packages:
```console
git clone -b melodic-devel-staging https://github.com/ros-industrial/universal_robot.git src/universal_robot
```
5. Install all package dependencies:
```console
sudo apt update -qq
```
```console
rosdep update
```
```console
rosdep install --from-paths src --ignore-src -r -y
```
6. Make the workspace:
```console
catkin_make
```
7. Source the workspace:
```console
source devel/setup.bash
```
---

## Running Our Demo
To run our demo using RViz in a purely virtual environment, start by sourcing the workspace. Then run:
```console
roslaunch ur_moveit_config demo.launch
```
Open and source another terminal, then run:
```console
rosrun ur_scripts calculator.py
```
This will begin the calculator demo. Follow the in-terminal prompts, and enter a calculation string when requested. The string can contain any number of numbers and operations (valid inputs are '+', '-' , '*', '/' , '^' , '(', ')', and any integer or float), and will perform order of operations properly. The output will then be drawn by the robot in RViz. Non-integer outputs are rounded to one decimal place. The calculator will loop to allow several inputs to be processed in succession. Use the input 'q' to exit the calculator. The solutions are only communicated through the movement of the robot, so pay attention as it moves!

Here is an example of how this all looks in simulation:

[9.webm](https://user-images.githubusercontent.com/99771915/196276793-3d82f084-5775-41c3-ba76-3d7fdc4342c3.webm)


## Demo with Real UR3
Here is a demo of the robot computing and writing the answer to 1.2 * 4:

https://user-images.githubusercontent.com/99771915/196271793-8dd0f9d0-390c-4272-beca-e23260c3da04.mov

And writing the digits of Pi:

https://user-images.githubusercontent.com/99771915/196272522-7ab801b1-c791-4ff0-8863-72c1a9a1f0a7.mov



---

## Additional Information about MoveIt
The [MoveIt Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html) contain lots of helpful information to get started with MoveIt's functionality. To avoid redundance, users are directed there for most questions about using MoveIt for their projects.

For the purposes of this presentation, special attention is paid to the [Move Group Python Interface](https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html), which enables control of a robot using MoveIt and Python script. 

One other import functionality of MoveIt is the [MoveIt Setup Assistant](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html), which allows many basic configuration and launch files to be developed in a new configuration package for your robot given only a URDF file of the robot. The Setup Assistant makes configuring your robot to use MoveIt very easy. The ur_moveit_config package we made for this project was built using the Setup Assistant in less than 10 minutes! 

Our package can be easily modified using the Setup Assistant by running:
```console
roslaunch moveit_setup_assistant setup_assistant.launch
```
Mess around with all the different options and see how they change the behavior of the robot!




