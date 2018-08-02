cmake_minimum_required(VERSION 3.0)
project(BundleACeres)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

set(
        CMAKE_CXX_STANDARD
        14
)

add_definitions(-DPROJECT_DIR="${PROJECT_SOURCE_DIR}")

find_package(
        Eigen3
        REQUIRED
)
find_package(
        OpenCV 3
        REQUIRED
)
find_package(
        Ceres
        REQUIRED
)
find_package(
        PCL 1.8
        REQUIRED
)
find_package(
        OpenEXR
        REQUIRED
)

add_executable(BundleACeres
        src/main.cpp
        src/features.cpp
        src/optimization.cpp
        src/sensor.cpp
        src/point_cloud.cpp
        src/visualization.cpp
        )

target_include_directories(BundleACeres
        PRIVATE
        include
        ${PCL_INCLUDE_DIRS}
        ${OPENEXR_INCLUDE_DIR}
)

target_link_libraries(BundleACeres
        ceres
        Eigen3::Eigen
        opencv_core opencv_highgui opencv_features2d opencv_calib3d
        ${PCL_LIBRARIES}
        ${OPENEXR_LIBRARIES}
)
