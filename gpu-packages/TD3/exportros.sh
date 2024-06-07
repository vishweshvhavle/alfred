HOST_IP=10.10.10.108
export ROS_PORT_SIM=11311
export ROS_IP=$HOST_IP
export ROS_HOSTNAME=$HOST_IP
export ROS_MASTER_URI=http://$HOST_IP:$ROS_PORT_SIM
export GAZEBO_RESOURCE_PATH=~/DRL-robot-navigation/catkin_ws/src/multi_robot_scenario/launch
source ~/.bashrc
killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3
cd ~/DRL-robot-navigation/catkin_ws
catkin_make_isolated
source devel_isolated/setup.bash
cd ~/DRL-robot-navigation/TD3
clear
python3 sim.py
