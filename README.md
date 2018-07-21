# Pointcloud Tutorial

The purpose of this tutorial is to provide examples of how to work with 3D or multidimensional data using two popular libraries: Point Cloud Library (PCL) and Open3D.
These examples will cover such topics as I/O, features, keypoints, registration, segmentation, and sample consensus.

## Preliminaries and Dependencies
Your system should have a modern compiler that supports C++11, as well as git and CMake. To use the python bindings for Open3D, you will also need either python 2.7 or 3.5+.

### PCL
If you have ROS installed already, then you should have a version of PCL on your computer already.  
If not, then you can install pre-built binaries (or build from source) for a variety of operating systems based on these instructions: [http://www.pointclouds.org/downloads/](http://www.pointclouds.org/downloads/).
The Ubuntu directions are copied below for convenience:
```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```
Even if you have a version of the library through ROS, you should also install some tools for interfacing with the library in a non-ROS way:
```
sudo apt-get install pcl-tools
```

### Open3D
This library can be installed from source, according to these directions [http://www.open3d.org/docs/getting_started.html#ubuntu](http://www.open3d.org/docs/getting_started.html#ubuntu).
First, get the code:
```
git clone https://github.com/IntelVCL/Open3D
```
and then follow the directions for your OS.

### Building the Examples
1. If you're reading this README, you've probably already cloned this repository, but if not, 
navigate to your desired install directory and run: 
```
git clone https://jdelmerico@bitbucket.org/jdelmerico/pcl_tutorials.git
```

2. Navigate into the pcl_tutorials directory, then execute the following commands to build the
examples:
```
mkdir build 
cd build 
cmake ..  
make
```

3. Assuming the examples all built correctly, you should have several executables in your build
directory, organized into   
