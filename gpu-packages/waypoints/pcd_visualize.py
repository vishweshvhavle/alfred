import numpy as np
import open3d as o3d

import os
import glob


file_name = "/home/orinprayaas/DRL-robot-navigation/waypoints/gazebo_env.pcd"


pc = o3d.io.read_point_cloud(file_name)
print(pc)
o3d.visualization.draw_geometries([pc])

