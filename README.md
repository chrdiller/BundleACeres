# BundleACeres - Structure from Motion using Bundle Adjustment with the Ceres Solver
This is an implementation of Bundle Adjustment with the Ceres Solver as optimizer. It matches a chain of incoming RGB images using ORB keypoints and BRISK descriptors in order to find 3D points. The Ceres Solver is then used to jointly solve for the locations of 3D points and camera poses of the frames. Ground-truth information about depth and camera poses is used for the first chain of frames in order to provide a usable initialization. The implementation utilizes the follwing dependencies and is written in modern C++14.

## Dependencies
* **Eigen3** for geometry operations
* **OpenCV 3** for data management and feature matching
* **Ceres** for solving the optimization problem
* **PCL 1.8** for visualization
* **OpenEXR** for loading RGB and depth information from exr files

## Usage
* Provide input data in the data/ folder, consisting of exr files with RGB-D information and a file containing all ground-truth poses
