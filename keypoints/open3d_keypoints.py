import numpy as np
from open3d import *

if __name__ == "__main__":

    pcd = read_point_cloud("../data/robot1.pcd")
    print("Compute the normals of the downsampled point cloud")
    voxel_size = 0.01
    downpcd = voxel_down_sample(pcd, voxel_size = voxel_size)
    estimate_normals(downpcd, search_param = KDTreeSearchParamHybrid(
            radius = 0.02, max_nn = 30))
    print("Downsampled point cloud shape: ")
    print(np.asarray(downpcd.points).shape)

    radius_feature = voxel_size * 5
    print("Compute FPFH feature with search radius %.3f." % radius_feature)
    fpfh = compute_fpfh_feature(downpcd,
            KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))
    print("FPFH shape: ")
    print(fpfh.data.shape)
