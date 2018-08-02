#include <point_cloud.h>

auto generate_points3D_from_groundtruth(
        const std::vector<PointCorrespondence>& correspondences, const cv::Mat& depth_map,
        const Eigen::Matrix3d& intrinsics, const Sophus::SE3d& camera_pose) -> std::vector<Point3D>
{
    std::vector<Point3D> points3D;
    for(const auto& correspondence : correspondences) {
        Point3D current_point3D;
        const auto point2D = correspondence.from;
        const double z = depth_map.at<double>(static_cast<int>(std::round(point2D.y)),
                static_cast<int>(std::round(point2D.x)));

        // TODO take care of -INF?

        const double x = (z / intrinsics(0, 0)) * (point2D.x - intrinsics(0, 2));
        const double y = (z / intrinsics(1, 1)) * (point2D.y - intrinsics(1, 2));

        const Eigen::Vector4d point3D { x, y, z, 1.0 };
        //current_point3D.coordinates = {1.0, 1.0, 1.0, 1.0};
        current_point3D.coordinates = current_point3D.coordinates_GT = camera_pose.matrix3x4() * point3D;
        current_point3D.observations.push_back({correspondence.from, correspondence.from_frame});

        points3D.push_back(current_point3D);
    }

    return points3D;
}


void PointCloud::integrate_correspondences(std::vector<PointCorrespondence>& correspondences)
{
    correspondences.erase(std::remove_if(correspondences.begin(), correspondences.end(),
                                         [this](const auto& correspondence) {
        for(auto& point : this->points) {
            for (const auto& observation : point.observations) {
                if (observation.frame_id == correspondence.to_frame and observation.coordinates == correspondence.to) {
                    point.observations.push_back({ correspondence.from, correspondence.from_frame });
                    return true;
                }
            }
        }
        return false;
    }), correspondences.end());
}


void PointCloud::remove_observations_of_frame(const size_t frame_id)
{
    for(auto& point : points) {
        point.observations.erase(std::remove_if(point.observations.begin(), point.observations.end(), [frame_id](Observation& observation){
            return observation.frame_id == frame_id;
        }), point.observations.end());
    }
}


std::vector<Point3D> PointCloud::pop_observed_n_times(const int n_times)
{
    std::vector<Point3D> out;

    auto part = std::partition(points.begin(), points.end(), [n_times](Point3D& point){
        return point.observations.size() != n_times;
    });
    std::move(part, points.end(), std::back_inserter(out));
    points.erase(part, points.end());

    return out;
}


void PointCloud::add_points(const std::vector<Point3D>& new_points)
{
    points.insert(points.end(), new_points.cbegin(), new_points.cend());
}
