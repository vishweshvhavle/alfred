# evaluate the demo data
# compare the heading calculation from trajectory with that of floam

import math
import numpy as np
import matplotlib.pyplot as plt

poseDataPath  = '/home/deepak/IIITD/catkin_ws/src/floam/scripts/demo_10hz_synced.npy'
yawDataMavros = '/home/deepak/IIITD/catkin_ws/src/floam/scripts/mavrosYaw_demo.npy'

def getYawFromPoses(poses): 
    yaws = []
    yaws.append(0)
    
    for i in range(0,poses.shape[0]):
        if poses[i,1] != 0:
            yaw = math.atan2(poses[i,2]-poses[0,2], poses[i,1]-poses[0,1])*180.0/3.14
            yaws.append(yaw)
            print(yaw)
        

    return np.array(yaws)

def getYawFromFloam(poses): 
    print(f'Shape of floam waypoints is :{poses.shape}')   
    yaws = poses[:,6]*180.0/3.14
    return yaws


def compareYaw(calcYaw, floamYaw, mavrosYaw):
    # plot the trajectories
    print(f'Sizes are: {calcYaw.shape, floamYaw.shape, mavrosYaw.shape}')
    
    errYaw1 = np.linalg.norm(calcYaw-floamYaw)/calcYaw.shape[0]
    errYaw2 = np.linalg.norm(floamYaw[0:1132] - mavrosYaw[1:,2])/1132

    print(f"RMSE Yaw (calculated yaw and floam yaw) is: {errYaw1}")
    print(f'RMSE Yaw (floam yaw and mavros yaw) is: {errYaw2}')

    plt.plot(floamYaw[0:1132], '--b')
    plt.plot(mavrosYaw[:,2]*180/3.14 - mavrosYaw[0,2]*180/3.14, '--g')
    plt.xlabel('Frame Counts')
    plt.ylabel('Yaw in Degrees')
    plt.legend(["Floam_Yaw", "Mavros_Yaw"])
    plt.show()

    plt.plot(floamYaw[0:1132], '--b')
    plt.plot(calcYaw, '--r')
    plt.plot(mavrosYaw[:,2]*180/3.14 - mavrosYaw[0,2]*180/3.14, '--g')
    plt.xlabel('Frame Counts')
    plt.ylabel('Yaw in Degrees')
    plt.legend(["Floam_Yaw","Calculated_Yaw", "Mavros_Yaw"])
    plt.show()


if __name__=="__main__":
    poses     = np.load(poseDataPath)

    mavrosYaw = np.load(yawDataMavros)
    calcYaw   = getYawFromPoses(poses)
    floamYaw  = getYawFromFloam(poses)

    compareYaw(calcYaw, floamYaw, mavrosYaw)

