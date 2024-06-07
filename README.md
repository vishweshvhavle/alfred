![C++](https://img.shields.io/badge/C++-17-gold?logo=c%2B%2B)
![Python](https://img.shields.io/badge/Python-3.8-gold?logo=python)
![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue?logo=ros)
![Gazebo](https://img.shields.io/badge/simulator-Gazebo-blue?logo=gazebo)
![PyTorch](https://img.shields.io/badge/PyTorch-1.10-EE4C2C?logo=pytorch)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu)
![License](https://img.shields.io/badge/license-MIT-green?logo=open-source-initiative)

# Alfred UGV

This is the official implementation of [Alfred UGV (Report)](https://drive.google.com/file/d/1xcbomTqWQFI5U3ZL3wshn6ydCLDhJ_Dn/view?usp=sharing).

![cover](imgs/cover.png)

# Installation

## 1. Install GPU Dependencies

Main dependencies: 

* [ROS Noetic](http://wiki.ros.org/noetic/Installation)
* [PyTorch](https://pytorch.org/get-started/locally/)

The network can be run with a standard 2D laser, but this implementation uses a simulated [3D Velodyne sensor](https://github.com/lmark1/velodyne_simulator). This package was developed on top [DRL-Robot-Navigation](https://github.com/reiniscimurs/DRL-robot-navigation).

Compile the workspace:
```shell
$ cd ~/alfred/gpu-packages/catkin_ws
$ catkin_make_isolated
```

Open a terminal and set up sources:
```shell
$ export ROS_HOSTNAME=localhost
$ export ROS_MASTER_URI=http://localhost:11311
$ export ROS_PORT_SIM=11311
$ export GAZEBO_RESOURCE_PATH=~/alfred/gpu-packages/catkin_ws/src/multi_robot_scenario/launch
$ source ~/.bashrc
$ cd ~/alfred/gpu-packages/catkin_ws
$ source devel_isolated/setup.bash
```

To kill the simulation:
```shell
$ killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3
```
## 2. Install CPU Dependencies

# Resources
## 3D Printable Files
## Links to Parts
# Issues
Please open Github issues for any installation/usage problems you run into.

# Citation

If you find this code useful for your research, please consider citing:

```
@inproceedings{alfred2024,
    author = {Vishwesh Vhavle and Jatin Kumar Sharma},
    title = {Alfred UGV},
    year = {2024},
}
```
