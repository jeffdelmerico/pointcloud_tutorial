# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

import numpy as np
from open3d import *

if __name__ == "__main__":

    print("Load a point cloud, print it, and render it")
    pcd = read_point_cloud("../data/robot1.pcd")
    print(pcd)
    print(np.asarray(pcd.points))
    draw_geometries([pcd])

    print("Compute the normals of the downsampled point cloud")
    downpcd = voxel_down_sample(pcd, voxel_size = 0.01)
    estimate_normals(downpcd, search_param = KDTreeSearchParamHybrid(
            radius = 0.03, max_nn = 30))
    draw_geometries([downpcd])
    print("")
