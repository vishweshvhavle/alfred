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
```shell
$ git clone git@github.com:vishweshvhavle/alfred.git
$ cd alfred/
$ conda create -n alfred python=3.8
$ conda activate alfred
$ pip install -r requirements.txt
```

## 1. Install GPU Dependencies

We have only tested our GPU packages on Nvidia AGX Orin with Ubuntu 20.04. We use the NVIDIA Orin for simulation as well as for creating occupancy grids, path-planning, and waypoint navigation.

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

## 2. Install CPU Dependencies

We have only tested our CPU packages on Intel NUC 12 Pro with Ubuntu 20.04. The SLAM node in this package was developed on top of [F-LOAM](https://github.com/wh200720041/floam).

Main dependencies: 

* [ROS Noetic](http://wiki.ros.org/noetic/Installation)
* [PyTorch](https://pytorch.org/get-started/locally/) (CPU Version)

# How to Run

## For GPU Packages

Open a terminal and set up sources using the shortcut shell script (Ensure appropriate permissions are granted to the shell script):
```shell
$ cd alfred/gpu-packages/sim/
$ sh ./exportros.sh
$ cd alfred/gpu-packages/waypoint/
$ python3 waypoints.py
$ python3 path_planning_node.py
```

To kill the simulation:
```shell
$ killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3
```

## For CPU Packages
Open a terminal and set up sources with the ros master at GPU node:
```shell
$ export ROS_HOSTNAME=gpu-node-ip
$ export ROS_MASTER_URI=http://gpu-node-ip:11311
```

To switch on the bot operations:
```shell
$ cd alfred/cpu-packages/controller/
$ python3 controller.py
```

For object detection inferences:
```shell
$ cd alfred/cpu-packages/detection/
$ python3 detection.py
```

For F-LOAM based localization and mapping:
```shell
$ cd alfred/cpu-packages/floam_mapping
$ catkin build
$ source devel/setup.bash
$ rosloaunch floam_mapping
```

# Resources
## Links to 3D Printable Files
(Coming Soon)

## Links to Parts
(Coming Soon)

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
