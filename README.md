
# **Mobile Robots Robust Control** 
This repository includes simulation and some tools to design and simulate robust trajectory tracking control algorithms for mobile robots. Two cases are presented:
* Sliding Mode Control
* Adaptive Neural Networks based Control

This project is implemented in ROS and was tested with ROS Kinetic and Ubuntu Xenial 16.04



[image1]: ./catkin_ws/src/robust_control/measurements/simulation.png "Simulation"



![alt text][image1]

---
### 1. Installation
1.1 Required dependencies
  ROS Kinetic and Gazebo: http://wiki.ros.org/kinetic/Installation/Ubuntu
1.2 Build de Sources
Clone the Repo
```
$ git clone https://github.com/jdgalviss/mobile_robots_robust_control
```
Go to the repo's catkin workspace src folder
```
$ cd mobile_robots_robust_control/catkin_ws/src
```
Clone the following repos:
```
$ git clone https://github.com/jdgalviss/skid_steer_bot.git https://github.com/jdgalviss/hector_slam.git
```
Build your workspace
```
$ cd ..
$ catkin_make
$ source devel/setup.bash
```
Run the gazebo simulation
```
$ roslaunch skid_steer_bot udacity_world.launch
```
Test the SSMR's motion by using teleop_key
```
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py

```
Run the control system's simulation:
```
$ roslaunch robust_control ssmr_control.launch
```
### 2. Results
[Video](https://www.youtube.com/watch?v=fplOKW1xr6Q)
