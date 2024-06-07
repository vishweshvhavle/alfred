# compare error between scan matcher and loam trajectories
# compute difference between values with same timestamps

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import tf

def plotTrajectory(floamTraj, scanMatcherTraj, loamTraj, floamLocalTraj_30, floamLocalTraj_25):
    lW = 2

    font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 22}

    plt.rc('font', **font)
    
  #  plt.plot(floamTraj[:,1], floamTraj[:,2], linewidth=lW)
  #  plt.plot(loamTraj[:,1], loamTraj[:,2], lineWidth=lW)
    plt.plot(scanMatcherTraj[:,1], scanMatcherTraj[:,2],lineWidth=lW)
    plt.plot(floamLocalTraj_30[:,1], floamLocalTraj_30[:,2],lineWidth=lW)
    plt.plot(floamLocalTraj_25[:,1], floamLocalTraj_25[:,2],lineWidth=lW)
    plt.legend(["Mapper", "FLOAM_30m_threshold","FLOAM_30m_threshold_35cm_Map"])
    plt.xlabel("X coordinate")
    plt.ylabel("Y Coordinate")
    plt.show()
    fig = plt.figure()

    ## Comparing the yaw values of the floam30, floam25 and mapper
    plt.plot(floamTraj[:,6]*180.0/3.14,linewidth=lW)
    plt.plot(floamLocalTraj_30[:,6]*180.0/3.14 ,linewidth=lW)
    plt.plot(scanMatcherTraj[:,6]*180.0/3.14,linewidth=lW)
    plt.plot(floamLocalTraj_25[:,6]*180.0/3.14, linewidth=lW)
    plt.legend(["Floam", "Floam_30", "Mapper", "Floam_35cm_Map"])
    plt.xlabel("Frame Count")
    plt.ylabel("Yaw")
    plt.show()

    ax = plt.axes(projection='3d')
    ax.plot3D(floamTraj[:,1], floamTraj[:,2], floamTraj[:,3], linewidth=lW)
    ax.plot3D(loamTraj[:,1], loamTraj[:,2], loamTraj[:,3], linewidth=lW)
    ax.plot3D(scanMatcherTraj[:,1], scanMatcherTraj[:,2], scanMatcherTraj[:,3], linewidth=lW)
    ax.plot3D(floamLocalTraj_30[:,1], floamLocalTraj_30[:,2], floamLocalTraj_30[:,3], linewidth=lW)
    ax.plot3D(floamLocalTraj_25[:,1], floamLocalTraj_25[:,2], floamLocalTraj_25[:,3], linewidth=lW)
    plt.legend(["FLOAM","LOAM","Mapper","FLOAM_30m_threshold","FLOAM_25m_threshold"])
    plt.show()

    
def computeMSE(floamTraj, scanMatcherTraj, loamTraj, gpsTraj, t265Traj, floamLocalTraj_30, floamLocalTraj_35):

    pitch = (15.0*3.14)/180.0

    Ry = np.array([
                    [np.cos(pitch), 0, np.sin(pitch)],
                    [0,             1,      0]       ,
                    [-np.sin(pitch),0, np.cos(pitch)]
                  ])

    print(floamTraj.shape)

    for i in range(scanMatcherTraj.shape[0]):
        r, p, y = tf.transformations.euler_from_quaternion([scanMatcherTraj[i,4], scanMatcherTraj[i,5], scanMatcherTraj[i,6], scanMatcherTraj[i,7]])
        scanMatcherTraj[i,4]    = r
        scanMatcherTraj[i,5]    = p
        scanMatcherTraj[i,6]    = y
        floamTraj[i,1:4]        = np.dot((Ry), floamTraj[i,1:4])
        scanMatcherTraj[i,1:4]  = np.dot((Ry), scanMatcherTraj[i,1:4])
        
    
    for i in range(loamTraj.shape[0]):
        loamTraj[i,1:4] = np.dot(Ry, loamTraj[i,1:4])
    
    for i in range(floamLocalTraj_30.shape[0]):
        floamLocalTraj_30[i,1:4] = np.dot((Ry), floamLocalTraj_30[i,1:4])
        floamLocalTraj_35[i,1:4] = np.dot((Ry), floamLocalTraj_35[i,1:4])
#        floamTraj[i,4:7]         += 15.0
#        scanMatcherTraj[i,4]   += 15.0
#        floamTraj[i,4:7]       = np.dot(np.linalg.inv(Ry), floamTraj[i,4:7])
#        scanMatcherTraj[i,4:7] = np.dot(np.linalg.inv(Ry), scanMatcherTraj[i,4:7])
    print(floamTraj.shape)

    print('\n Floam raw vs Mapper \n')
    err_ = abs(floamTraj[:,1:7] - scanMatcherTraj[:, 1:7])
    print(np.mean(err_[:,0]), np.mean(err_[:,1]), np.mean(err_[:,2]), np.mean(err_[:,3])*(180.0/3.14), np.mean(err_[:,4])*(180.0/3.14), np.mean(err_[:,5])*(180.0/3.14))

    err_new = abs(floamLocalTraj_30[:,1:7] - scanMatcherTraj[0:floamLocalTraj_30.shape[0],1:7])
    print("\n Floam 30 vs Mapper **** \n")
    print(np.mean(err_new[:,0]), np.mean(err_new[:,1]), np.mean(err_new[:,2]), np.mean(err_new[:,3])*(180.0/3.14), np.mean(err_new[:,4])*(180.0/3.14), np.mean(err_new[:,5])*(180.0/3.14))
    print("\n")

    err_new_2 = abs(floamLocalTraj_35[:,1:7] - scanMatcherTraj[0:floamLocalTraj_35.shape[0],1:7])
    print("\n Floam 25 vs Mapper *** \n")
    print(np.mean(err_new_2[:,0]), np.mean(err_new_2[:,1]), np.mean(err_new_2[:,2]), np.mean(err_new_2[:,3])*(180.0/3.14), np.mean(err_new_2[:,4])*(180.0/3.14), np.mean(err_new_2[:,5])*(180.0/3.14))

    angles = scanMatcherTraj[:,4:7]

    posErr    = 0
    orientErr = 0
    count     = 0

    mXFLOAM = np.mean(floamTraj[:,1])
    mXSM   = np.mean(scanMatcherTraj[:,1])

    mYFLOAM = np.mean(floamTraj[:,2])
    mYSM   = np.mean(scanMatcherTraj[:,2])

    mZFLOAM = np.mean(floamTraj[:,3])
    mZSM   = np.mean(scanMatcherTraj[:,3])

#   print(f'\n **** Mean values along each axis (x,y,z) [floam, scanMatch] are : {[mXFLOAM, mXSM], [mYFLOAM, mYSM], [mZFLOAM, mZSM]}')
    print([mXFLOAM, mXSM], [mYFLOAM, mYSM], [mZFLOAM, mZSM])
    tFloam = floamTraj[:,0]
    tScanMatcher = scanMatcherTraj[:,0]

    print("\n **************************")
    print(np.min(floamTraj[:,3]), np.min(scanMatcherTraj[:,3]), np.min(gpsTraj[:,2]), np.min(t265Traj[:,2]))
    print(np.max(floamTraj[:,3]), np.max(scanMatcherTraj[:,3]), np.max(gpsTraj[:,2]), np.max(t265Traj[:,2]))
    print("************************** \n")

    print("\n 1st and 2nd order moments of heights obtained from FLOAM, MAPPER and GPS \n")
    print(np.mean(floamTraj[:,3]), np.std(floamTraj[:,3]))
    print(np.mean(scanMatcherTraj[:,3]), np.std(scanMatcherTraj[:,3]))
    print(np.mean(gpsTraj[:,2]), np.std(gpsTraj[:,2]))
    print(np.mean(t265Traj[:,2]), np.std(t265Traj[:,2]))

    plt.plot(floamTraj[:,3], "-r", label="FLOAM")
    plt.plot(scanMatcherTraj[:,3], "-b", label="SCAN_MATCHER")
    plt.plot(gpsTraj[:,2], "-g", label="GPS")
    plt.show()

    plt.plot(floamTraj[:,3], scanMatcherTraj[0:862,3])
    plt.show()
    
    for t in tScanMatcher:
        if t in tFloam:
            tIndexSM = np.where(tScanMatcher == t)
            tIndexFloam = np.where(tFloam == t)
            posLoam = floamTraj[tIndexFloam, 1:7].reshape(6)
            posScanMatcher = scanMatcherTraj[tIndexSM, 1:7].reshape(6)
            posErr += abs(posLoam - posScanMatcher)
            count += 1

   # print(f'\n **** Mean Absolute Error is : {posErr/count, count} \n')
    print(posErr/count, count)

    plotTrajectory(floamTraj, scanMatcherTraj, loamTraj, floamLocalTraj_30, floamLocalTraj_35)

    
if __name__=="__main__":
    floamTraj        = np.load('/home/deepak/IIITD/catkin_ws/src/data/posesFloam025_TF.npy')#input("Enter loam trajectory file (npy): ")
    scanMatcherTraj  = np.load('/home/deepak/IIITD/loam_velodyne/data/posesMapper.npy')#input("Enter scan matcher trajectory file (npy): ")
    loamTraj         = np.load('/home/deepak/IIITD/loam_velodyne/data/posesLoam025.npy')
    gpsTraj          = np.load('/home/deepak/IIITD/catkin_ws/src/data/posesGps.npy')
    t265Traj         = np.load("/home/deepak/IIITD/catkin_ws/src/data/t265_data.npy")
    #floamLocalTraj_30 = np.load('/home/deepak/IIITD/catkin_ws/src/data/posesFloam_r100.npy')
    floamLocalTraj_30 = np.load('/home/deepak/IIITD/catkin_ws/src/data/posesFloam_noMap_30.npy')
    #floamLocalTraj_35 = np.load('/home/deepak/IIITD/catkin_ws/src/data/posesFloam_noMap_30_40cm_thresh.npy')
    floamLocalTraj_35 = np.load('/home/deepak/IIITD/catkin_ws/src/data/posesFloam_noMap_30_30cm_thresh_20kFeats.npy')
    #floamLocalTraj_35 =  np.load('/home/deepak/IIITD/catkin_ws/src/data/posesFloam_r100_35.npy')
    times             = np.load('/home/deepak/IIITD/catkin_ws/src/data/posesFloam_noMap_30_times_40cm.npy')
    times_20          = np.load('/home/deepak/IIITD/catkin_ws/src/data/posesFloam_noMap_30_times_30cm_20kFeats.npy')
    floam_demo        = np.load('/home/deepak/IIITD/catkin_ws/src/floam/scripts/demo_10hz.npy')
    
    plt.plot(floam_demo[:,6]*180/3.14,'r')
    plt.show()

    print(times.shape, floamLocalTraj_30.shape)
    plt.plot(times[2:])
    plt.plot(times_20)
    plt.legend(["FLOAM", "FLOAM_20K"])
    plt.show()
    computeMSE(floamTraj, scanMatcherTraj, loamTraj, gpsTraj, t265Traj, floamLocalTraj_30, floamLocalTraj_35)