# saviRoomba

This is a prototype demo of a mail delivery system using an iRobot create (Roomba), with line following and position detection with QR codes on the ground, visible by an external camera. This prototype has been published in AREA2020 (https://area2020.github.io/) and is planned to be published in JSAN (https://www.mdpi.com/journal/jsan/special_issues/REA).

This project connects an iRobot Create 2 to a BDI reasoner usign the savi_ros_bdi package, available at https://github.com/NMAI-lab/savi_ros_bdi, and the create_autonomy package, available at https://github.com/AutonomyLab/create_autonomy. Both of those packages are required dependencies for this project to work.

This project was based off of the savi_ros_demo example project, available at https://github.com/NMAI-lab/savi_ros_demo.

## Overview
A presentation video that explains this project is available at https://youtu.be/mHNSlqaH8Uc.

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
The following is the proceedure for running the applications. You will need to run roscore, create_autonomy, savi_ros_bdi, as well as the various scripts from this package. The following sections outline the proceedures (TODO: simplify this into a script or launch file).

### roscore
Open a terminal window and connect to the robot. Run roscore; this starts the main ROS process. Leave this terminal window open and set it aside: `roscore`

Optional: Once roscore is running, you can verify that it is running by examining the topics that are being published. In a new terminal window run the following to list the topics: `rostopic list`

### create_autonomy
Start the create_autonomy module. The documentation for this module is at: https://github.com/AutonomyLab/create_autonomy and http://wiki.ros.org/create_autonomy. The proceedure is under the "Running the driver" headding.

Optional: In a new terminal window, run `rostopic list` to verify that the create_autonomy publishers and subscribers have come online.

### savi_ros_bdi
Run savi_ros_bdi using the proceedure documented at https://github.com/NMAI-lab/savi_ros_bdi.

Optional: Run `rostopic list` to verify that the application is running. The publishers and subscribers include perceptions, actions, inbox and outbox.

### saviRoomba application node
With the other tools running, it is now possible to start the application node applications. To do this, open a new terminal window and run the following:
```
$ source devel_isolated/setup.bash
$ roslaunch saviRoomba roomba.launch
```

Lastly, it is necessary to start the user interface. This is necessary in order to tell the robot the location of the docking station as well as to command it to perform the mail delivery mission. In a new terminal window run the following:
```
$ source devel_isolated/setup.bash
$ rosrun saviRoomba userInterface.py
```
Follow the prompts in the terminal to start the robot. The interface will also prin all messages that are sent to and from the robot. These messages are very useful for troubleshooting.

## Notes about hardware
Please note that this is a prototype for experimental development. The current implementation uses line sensing and QR codes for navigation, however it was found that this is a highly sensitive and tricky means of navigating this robot. Future development is planned (and already under way) on an upgraded version that does not require line following or QR codes for navigation.

## Configuring BlueTooth Beacons
This project uses bluetooth beacons for navigation, such as these: https://blog.aprbrother.com/product/april-beacon-n04.
The beacons need to be configured. This is the proceedure.

The scripts folder contains a file called beacons, in CSV format, which needs to be edited with the information of the beacons you would like to add.
The format of the file is: MAC Address, Environmental Variable, Measured Value, Node Letter

The MAC Address is specific to the beacon hardware, consult the manufacturer for how to locate it.

The environmental variable, and measured value need to be calculated, follow the steps below todo so.

- Place Beacon exactly 1m away from the device that will be doing the measurements (1m from the measuring devices bluetooth antenna is preferred) 
- Install bluepy: sudo pip3 install bluepy
- Run RSSI_tools.py: python3 RSSI_tools.py
- Follow on screen instructions
- Create a new line entry in beacons which matches the information shown on screen.

Note: The script may take a while, this is normal! The script relies on averaging many samples, and taking a sample population takes a while. 

As a sanity check for the values you get from the script, environmental values typically range from 2 -> 4 inclusive, where 2 is a low signal strength in the space, and 4 is a strong signal strength. You may notice that in the beacons file the raspberry pi beacons are lower than 2, this is typical for the raspberry pi. 
