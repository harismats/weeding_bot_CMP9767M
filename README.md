# Summary
This is a ROS package for the CMP9767M course of Lincoln University. The focus area that we have chosen is **navigation**. The aim os this work is to develop a robot that can autonomously navigate between obstacles (both static and dynamic), identify different kinds of weeds and then spray at them in the Gazebo simulation environment. To achieve a precise localization we have used **amcl**, and with the help of **move_base** we can freely navigate in our map by using the action client-server interface. We identify the different types of weeds using contours and compute their final position in the "map" frame by correcting it, using the camera's intrinsic parameters. To spray the weeds we call a service that only considers the last 500 detected weeds to reduce computations.

### Prerequisites

To use this package you need to have: 
* Ubuntu 18.04 LTS
* OpenCV 4.5.1
* ROS Melodic

### Setup

To use our package you first have to install the L-CAS Ubuntu/ROS software distribution. To do so, do:
1. ```sudo ls``` (this is just to cache you admin password for the next steps)
2. ```sudo apt-get update && sudo apt-get install curl``` (curl is required for the next step)
3. ```curl https://raw.githubusercontent.com/LCAS/rosdistro/master/lcas-rosdistro-setup.sh | bash -``` (should install everything required)

Then clone the main CMP9767M package, and our package in your catkin workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/harismats/CMP9767M
git clone https://github.com/harismats/weeding_bot_CMP9767M
cd ..
catkin_make
```

After you are done with that with no erros, do:
```
pip install scipy==1.2.0
```

In case any other package is missing, you can install it with:
```
sudo apt-get install ros-melodic-<package_name>
```

### How to run

To run our system, do the followings in different terminals in this sequence:
1. ```roscore```
2. ```roslaunch weeding_bot our_sim.launch``` (wait for the whole simulation to load before running the next step's command)
3. ```roslaunch weeding_bot system_nodes.launch```
