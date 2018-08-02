#pragma once

#include <point_cloud.h>

void optimize(PointCloud& cloud, std::deque<CameraPose>& camera_poses, const Eigen::Matrix3d& intrinsics);