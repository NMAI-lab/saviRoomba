# saviRoomba

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
With the other tools running, it is now possible to start the application node applications. EAch of these applications will need to be run in it's own terminal window. (TODO: simplify this process with a launch script).

To run individual packages, use the following:
1. Source the setup.bash file: `source devel/setup.bash`
1. Run each package using `rosrun saviRoomba packageName.py` and substituting packageName for the file name of each package. The packages needed include the following:
   1. battery_manager: Provides the agent with perceptions related to the battery state of charge.
   1. actionDecoder: Decodes action commands from the agent and executes the actions.
   1. app_receiver: Receives messages from the user application (updates needed to this module)
   1. liner: Driver for the line sensor
   1. logger: Used for tracking the run time performance of the software execution. Generates a CSV file with time stamps of the perceptions and actions that are published to ROS.
   1. qr_scanner: Responsible for reading QR codes on the ground using a camera. Publishes these so that the agent can know the position of the robot.
