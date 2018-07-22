# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

from open3d import *

if __name__ == "__main__":

    print("Testing IO for point cloud ...")
    pcd = read_point_cloud("../data/robot1.pcd")
    print(pcd)
    write_point_cloud("../data/copy_of_robot1.pcd", pcd)

    print("Testing IO for meshes ...")
    mesh = read_triangle_mesh("../data/bunny.ply")
    print(mesh)
    write_triangle_mesh("../data/copy_of_bunny.ply", mesh)

    print("Testing IO for images ...")
    img = read_image("../data/lena_color.jpg")
    print(img)
    write_image("../data/copy_of_lena_color.jpg", img)
