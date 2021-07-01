# This is a docker file for the saviRoomba project
FROM ros:kinetic-ros-core-xenial

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-kinetic-ros-base=1.3.2-0* \
    && rm -rf /var/lib/apt/lists/*

# install create autonomy
# install prerequisites
RUN sudo apt-get install python-rosdep python-catkin-tools
# create a catkin workspace
RUN mkdir -p ~/create_ws/src
RUN cd create_ws
RUN catkin init
# clone create autonomy repo
RUN cd ~/create_ws/src
RUN git clone https://github.com/AutonomyLab/create_autonomy.git  
# install dependencies
RUN cd ~/create_ws
RUN rosdep update 
RUN rosdep install --from-paths src -i  
RUN cd ~/create_ws
RUN catkin build
RUN sudo usermod -a -G dialout $USER
RUN source ~/create_ws/devel/setup.bash

# launch ros package
CMD ["roslaunch", "ca_driver", "create_2.launch"]