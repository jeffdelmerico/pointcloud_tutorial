cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pointcloud_tutorials)

find_package(PCL 1.2 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_write_pcd io/pcl_write_pcd.cpp)
target_link_libraries (pcl_write_pcd ${PCL_LIBRARIES})

add_executable (pcl_read_pcd io/pcl_read_pcd.cpp)
target_link_libraries (pcl_read_pcd ${PCL_LIBRARIES})

add_executable (pcl_openni_grabber openni_grabber/pcl_openni_grabber.cpp)
target_link_libraries (pcl_openni_grabber ${PCL_LIBRARIES})

add_executable (pcl_compute_normals 3D_features/pcl_compute_normals.cpp)
target_link_libraries (pcl_compute_normals ${PCL_LIBRARIES})

add_executable (pcl_filtering filtering/pcl_filtering.cpp)
target_link_libraries (pcl_filtering ${PCL_LIBRARIES})

add_executable (pcl_keypoints keypoints/pcl_keypoints.cpp)
target_link_libraries (pcl_keypoints ${PCL_LIBRARIES})

add_executable (pcl_sample_consensus sample_consensus/pcl_sample_consensus.cpp)
target_link_libraries (pcl_sample_consensus ${PCL_LIBRARIES})

add_executable (pcl_plane_segmentation segmentation/pcl_plane_segmentation.cpp)
target_link_libraries (pcl_plane_segmentation ${PCL_LIBRARIES})

add_executable (pcl_euclidean_cluster_extraction segmentation/pcl_euclidean_cluster_extraction.cpp)
target_link_libraries (pcl_euclidean_cluster_extraction ${PCL_LIBRARIES})

add_executable (pcl_kdtree trees/pcl_kdtree.cpp)
target_link_libraries (pcl_kdtree ${PCL_LIBRARIES})

add_executable (pcl_octree trees/pcl_octree.cpp)
target_link_libraries (pcl_octree ${PCL_LIBRARIES})

add_executable (pcl_icp registration/pcl_icp.cpp)
target_link_libraries (pcl_icp ${PCL_LIBRARIES})

add_executable (pcl_template_matching registration/pcl_template_matching.cpp)
target_link_libraries (pcl_template_matching ${PCL_LIBRARIES})

set(BINARIES pcl_write_pcd pcl_read_pcd pcl_openni_grabber pcl_compute_normals pcl_filtering pcl_keypoints pcl_sample_consensus pcl_plane_segmentation pcl_euclidean_cluster_extraction pcl_kdtree pcl_octree pcl_icp pcl_template_matching)
install(TARGETS ${BINARIES} DESTINATION bin)
