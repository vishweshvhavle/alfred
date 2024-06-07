![C++](https://img.shields.io/badge/C++-17-gold?logo=c%2B%2B)
![Python](https://img.shields.io/badge/Python-3.8-gold?logo=python)
![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue?logo=ros)
![Gazebo](https://img.shields.io/badge/simulator-Gazebo-blue?logo=gazebo)
![License](https://img.shields.io/badge/license-MIT-green?logo=open-source-initiative)

# Alfred UGV

This is the official implementation of [Alfred UGV (Report)](https://drive.google.com/file/d/1xcbomTqWQFI5U3ZL3wshn6ydCLDhJ_Dn/view?usp=sharing).

![teaser](imgs/in2n_teaser.png)

# Installation

## 1. Install GPU Dependencies

Main dependencies: 

* [ROS Noetic](http://wiki.ros.org/noetic/Installation)
* [PyTorch](https://pytorch.org/get-started/locally/)

The network can be run with a standard 2D laser, but this implementation uses a simulated [3D Velodyne sensor](https://github.com/lmark1/velodyne_simulator). This package was developed on top [DRL-Robot-Navigation](https://github.com/reiniscimurs/DRL-robot-navigation).

Compile the workspace:
```shell
$ cd ~/DRL-robot-navigation/catkin_ws
### Compile
$ catkin_make_isolated
```

Open a terminal and set up sources:
```shell
$ export ROS_HOSTNAME=localhost
$ export ROS_MASTER_URI=http://localhost:11311
$ export ROS_PORT_SIM=11311
$ export GAZEBO_RESOURCE_PATH=~/DRL-robot-navigation/catkin_ws/src/multi_robot_scenario/launch
$ source ~/.bashrc
$ cd ~/DRL-robot-navigation/catkin_ws
$ source devel_isolated/setup.bash
```

To kill the simulation:
```shell
$ killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3
```


## Training Notes

***Important***
Please note that training the NeRF on images with resolution larger than 512 will likely cause InstructPix2Pix to throw OOM errors. Moreover, it seems InstructPix2Pix performs significantly worse on images at higher resolution. We suggest training with a resolution that is around 512 (max dimension), so add the following tag to the end of both your `nerfacto` and `in2n` training command: `nerfstudio-data --downscale-factor {2,4,6,8}` to the end of your `ns-train` commands. Alternatively, you can downscale your dataset yourself and update your `transforms.json` file (scale down w, h, fl_x, fl_y, cx, cy), or you can use a smaller image scale provided by Nerfstudio.

We recommend capturing data using images from Polycam, as smaller datasets work better and faster with our method.

If you have multiple GPUs, training can be sped up by placing InstructPix2Pix on a separate GPU. To do so, add `--pipeline.ip2p-device cuda:{device-number}` to your training command.

Our method uses ~16K rays and LPIPS, but not all GPUs have enough memory to run this configuration. As a result, we have provided two alternative configurations which use less memory, but be aware that these configurations lead to decreased performance. The differences are the precision used for IntructPix2Pix and whether LPIPS is used (which requires 4x more rays). The details of each config is provided in the table below.

| Method | Description | Memory | Quality |
| ---------------------------------------------------------------------------------------------------- | -------------- | ----------------------------------------------------------------- | ----------------------- |
| `in2n` | Full model, used in paper | ~15GB | Best |
| `in2n-small` | Half precision model | ~12GB | Good |
| `in2n-tiny` | Half precision with no LPIPS | ~10GB | Ok |

Currently, we set the max number of iterations for `in2n` training to be 15k iteratios. Most often, the edit will look good after ~10k iterations. If you would like to train for longer, just reload your last `in2n` checkpoint and continue training, or change `--max-num-iterations 30000`.

# Issues
Please open Github issues for any installation/usage problems you run into.

# Citation

If you find this code useful for your research, please consider citing:

```
@inproceedings{alfred2024,
    author = {Vishwesh and Jatin},
    title = {Alfred UGV},
    year = {2024},
}
```
