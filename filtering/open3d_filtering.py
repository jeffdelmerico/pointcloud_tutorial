# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

import numpy as np
import math
from open3d import *

if __name__ == "__main__":

    print("Load a point cloud and render it")
    pcd = read_point_cloud("../data/table_scene_lms400.pcd")
    print(pcd)
    draw_geometries([pcd])

    print("Crop the original point cloud (pass through filter)")
    cropped = crop_point_cloud(pcd, min_bound=np.array([-0.75, -math.inf, -math.inf]), max_bound=np.array([0.5, math.inf, math.inf]))
    draw_geometries([cropped])
    print("")

    print("Downsample the point cloud with a voxel of 0.01")
    downpcd = voxel_down_sample(pcd, voxel_size = 0.01)
    draw_geometries([downpcd])
    print("")
