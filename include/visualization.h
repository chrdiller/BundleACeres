#pragma once

#include <point_cloud.h>

class Visualizer {
public:
    Visualizer();

    void set_pointclouds(const PointCloud& bundle_cloud, const PointCloud& final_cloud);
    void set_cameras(const std::vector<CameraPose>& camera_poses_history,
                     const std::deque<CameraPose>& camera_poses_active,
                     const std::vector<CameraPose>& camera_poses_GT);
    void visualize(const bool blocking = false);

private:

};
