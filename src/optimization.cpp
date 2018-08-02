#include <optimization.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <sophus/local_parameterization_se3.hpp>

class ReprojectionError {
public:
    ReprojectionError(double _observed_x, double _observed_y, const Eigen::Matrix3d& _intrinsics) :
            observed_x{_observed_x}, observed_y{_observed_y}, intrinsics{_intrinsics} {}

    template <typename T>
    bool operator()(const T* const camera, const T* const point, T* residuals) const
    {
        Sophus::SE3d se3;
        for(int i = 0; i < 7; ++i)
            se3.data()[i] = camera[i];

        Eigen::Vector4d vec {point[0], point[1], point[2], 1.0};
        Eigen::Vector4d p = se3.matrix() * vec;

        T predicted_x = p[0] / p[2] * intrinsics(0, 0) + intrinsics(0, 2);
        T predicted_y = p[1] / p[2] * intrinsics(1, 1) + intrinsics(1, 2);

//        std::cout << predicted_x << "," << predicted_y << " vs. " << observed_x << "," << observed_y << std::endl;

        residuals[0] = predicted_x - T(observed_x);
        residuals[1] = predicted_y - T(observed_y);

        return true;
    }

private:
    double observed_x, observed_y;
    const Eigen::Matrix3d intrinsics;
};


void optimize(PointCloud& cloud, std::deque<CameraPose>& camera_poses, const Eigen::Matrix3d& intrinsics)
{
    ceres::Problem problem;
    ceres::LocalParameterization* se3_parameterization = new Sophus::LocalParameterizationSE3;

    std::map<size_t, Sophus::SE3d> ceres_poses;
    for(const auto& camera_pose : camera_poses) {
        ceres_poses[camera_pose.frame_index] = camera_pose.pose.inverse();
        problem.AddParameterBlock(ceres_poses[camera_pose.frame_index].data(), Sophus::SE3d::num_parameters, se3_parameterization);
        //problem.SetParameterBlockConstant(ceres_poses[camera_pose.frame_index].data());
    }

    Eigen::Vector3d coordinates[cloud.points.size()];

    int i = 0;
    for (auto& point : cloud.points) {
        coordinates[i] = point.coordinates;
        problem.AddParameterBlock(coordinates[i].data(), 3);
        //problem.SetParameterBlockConstant(coordinates[i].data());

        for (const auto& observation : point.observations) {
            ReprojectionError* constraint = new ReprojectionError(observation.coordinates.x, observation.coordinates.y, intrinsics);
            auto cost_func_numeric = new ceres::NumericDiffCostFunction<ReprojectionError, ceres::CENTRAL, 2, 7, 3>(constraint);

            problem.AddResidualBlock(cost_func_numeric,
                                     nullptr /* squared loss */,
                                     ceres_poses[observation.frame_id].data(),
                                     coordinates[i].data());

            if(observation.frame_id < 8)
                problem.SetParameterBlockConstant(ceres_poses[observation.frame_id].data());
        }
        i += 1;
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    i = 0;
    for (auto& point : cloud.points) {
        point.coordinates = coordinates[i];
        i += 1;
    }

    for(auto& camera_pose : camera_poses) {
        const auto ceres_pose = ceres_poses[camera_pose.frame_index];
        camera_pose.pose = ceres_pose.inverse();
    }
}
