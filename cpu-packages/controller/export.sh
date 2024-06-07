export ROS_PORT_SIM=11311
export ROS_IP=10.10.10.106
export ROS_MASTER_IP=10.10.10.116
export ROS_HOSTNAME=$HOST_IP
export ROS_MASTER_URI=http://$MASTER_IP:$ROS_PORT_SIM
source ~/.bashrc
python3 controller.py
