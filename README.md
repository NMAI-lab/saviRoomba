# saviRoomba

This project connects an iRobot Create 2 to a BDI reasoner usign the savi_ros_bdi package, available at https://github.com/NMAI-lab/savi_ros_bdi, and the create_autonomy package, available at https://github.com/AutonomyLab/create_autonomy. Both of those packages are required dependencies for this project to work.

This project was based off of the savi_ros_demo example project, available at https://github.com/NMAI-lab/savi_ros_demo.

## Overview
TBC

## Configuration and Setup
These instructions assume that you already have a ros workspace with the savi_ros_bdi package set up, as per the instructions at that repository. It also assumes that the create_autonomy package has been setup. This means that you have a ros workspace at ~/Roomba/create_ws which contains the savi_ros_bdi project, as described in the savi_ros_bdi Readme.

First, clone this repository to the src directory of your workspace.

```
$ git clone https://github.com/NMAI-lab/saviRoomba.git
```
The scripts folder holds the Python scripts used for publishing and subscribing to ros topics. The asl folder is the location of the AgentSpeak programs. Lastly, the resources folder contains settings.cfg, which needs to be copied to the savi_ros_bdi package for it to correctly configure the agent. There is also a bash script called configProject, which can be used for correctly moving this settings file to the correct location in the savi_ros_bdi package. To use this you must first update line 6 of this script with the correct directory location for the savi_ros_bdi package. Also, the settings.cfg file should be checked to confirm that the parameters are correct, most notably the location of the ASL file, the agent type and the agent name. This script can be run at the command line without parameters.
```
./configProject
```
In order to use the scripts, return to the project home directory and run catkin_make and source the setup.bash file.
```
$ cd ~/SAVI_ROS/rosjavaWorkspace
$ catkin_make
$ source devel/setup.bash
```

## Running
Before running the the demo scripts, roscore and savi_ros_bdi.Main need to be running. Please see the savi_ros_bdi Readme for instructions. It is then recommended that you run the packages from this node. These will each need to be executed in thir own terminals.
```
$ rosrun saviRoomba packageName.py 
```
With these scripts running you will see details of the execution print to the terminals.
