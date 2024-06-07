import open3d as o3d
import numpy as np

orig_pcd = o3d.io.read_point_cloud("../maps/unscaled_test_track_lanelet.pcd")

points = np.asarray(orig_pcd.points)
pointsC = np.zeros(points.shape)
print(points.shape, pointsC.shape)
pointsC[:, 0] = points[:, 0] - 722481.5147677443
pointsC[:, 1] = points[:, 1] - 3160105.8432507385
pointsC[:, 2] = points[:, 2]

print(pointsC[1, 1],  points[1, 1])
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pointsC)
o3d.visualization.draw_geometries([pcd]) 
o3d.io.write_point_cloud("../maps/corrected_origin_lanelet_test_track.pcd", pcd)
corrected_pcd = o3d.io.read_point_cloud("corrected_origin_lanelet_test_track.pcd")
print(corrected_pcd)