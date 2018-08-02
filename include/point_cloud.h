#pragma once

#include <types.h>

struct Observation {
    cv::Point2d coordinates;
    size_t frame_id;
};

struct Point3D {
    Eigen::Vector3d coordinates;
    Eigen::Vector3d coordinates_GT;
    std::vector<Observation> observations;
};

auto generate_points3D_from_groundtruth(
        const std::vector<PointCorrespondence>& correspondences, const cv::Mat& depth_map,
        const Eigen::Matrix3d& intrinsics_inverse, const Sophus::SE3d& camera_pose) -> std::vector<Point3D>;

class PointCloud {
public:
    /**
     * Integrate given point correspondences into points in this pointcloud
     * @param correspondences A vector of point correspondences of which as many as possible should be integrated
     */
    void integrate_correspondences(std::vector<PointCorrespondence>& correspondences);

    /**
     * Remove observations coming from a specified frame
     * @param frame_id The id of the frame to remove observations from
     */
    void remove_observations_of_frame(const size_t frame_id);

    std::vector<Point3D> pop_observed_n_times(const int n_times);

    /**
     * Add 3D points with known 3D coordinates and groundtruth into the cloud
     * @param points A vector of 3D points to add, with coordinates and groundtruth
     */
    void add_points(const std::vector<Point3D>& new_points);

    std::vector<Point3D> points;
};
