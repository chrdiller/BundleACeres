#include <visualization.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Eigen>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ba_visualization_cloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
pcl::PointCloud<pcl::PointXYZRGB>::Ptr gt_visualization_cloud { new pcl::PointCloud<pcl::PointXYZRGB>() };
pcl::visualization::PCLVisualizer ba_viewer { "Bundle Adjustment Result" };
pcl::visualization::PCLVisualizer gt_viewer { "Ground Truth" };

auto Eigen2PointXYZRGB(const Eigen::Vector3d& vec, const bool active)
{
    const auto vecf = vec.cast<float>();
    pcl::PointXYZRGB point;
    point.r = (active ? 0 : 255); point.g = (active ? 255 : 0); point.b = 0;
    point.x = vecf[0]; point.y = vecf[1]; point.z = vecf[2];
    return point;
}

auto generate_camera_mesh(const Sophus::SE3d& camera_pose, const bool active)
{
    const Eigen::Matrix3d R = camera_pose.rotationMatrix();
    const Eigen::Vector3d t = camera_pose.translation();

    const double s = 0.1;
    const Eigen::Vector3d vright = R.col(0).normalized() * s;
    const Eigen::Vector3d vup = -R.col(1).normalized() * s;
    const Eigen::Vector3d vforward = R.col(2).normalized() * s;

    pcl::PointCloud<pcl::PointXYZRGB> mesh_cloud;
    mesh_cloud.push_back(Eigen2PointXYZRGB(t, active));
    mesh_cloud.push_back(Eigen2PointXYZRGB(t + vforward + vright/2.0 + vup/2.0, active));
    mesh_cloud.push_back(Eigen2PointXYZRGB(t + vforward + vright/2.0 - vup/2.0, active));
    mesh_cloud.push_back(Eigen2PointXYZRGB(t + vforward - vright/2.0 + vup/2.0, active));
    mesh_cloud.push_back(Eigen2PointXYZRGB(t + vforward - vright/2.0 - vup/2.0, active));

    pcl::PolygonMesh pm;
    pm.polygons.resize(4);
    pm.polygons[0].vertices = {0, 1, 2};
    pm.polygons[1].vertices = {0, 2, 4};
    pm.polygons[2].vertices = {0, 4, 3};
    pm.polygons[3].vertices = {0, 3, 1};

    pcl::toPCLPointCloud2(mesh_cloud, pm.cloud);

    return pm;
}


Visualizer::Visualizer()
{
    ba_viewer.setBackgroundColor(0, 0, 0);
    gt_viewer.setBackgroundColor(0, 0, 0);
    ba_viewer.initCameraParameters();
    gt_viewer.initCameraParameters();

    vtkObject::GlobalWarningDisplayOff();
}

void Visualizer::set_pointclouds(const PointCloud& bundle_cloud, const PointCloud& final_cloud)
{
    for (const auto& activePoint : bundle_cloud.points) {
        auto coords = activePoint.coordinates.cast<float>();
        auto coordsGT = activePoint.coordinates_GT.cast<float>();
        pcl::PointXYZRGB pcl_point;
        pcl_point.r = 0; pcl_point.g = 255; pcl_point.b = 0;
        pcl_point.x = coords[0]; pcl_point.y = coords[1]; pcl_point.z = coords[2];
        pcl::PointXYZRGB pcl_point_gt;
        pcl_point_gt.r = 0; pcl_point_gt.g = 255; pcl_point_gt.b = 0;
        pcl_point_gt.x = coordsGT[0]; pcl_point_gt.y = coordsGT[1]; pcl_point_gt.z = coordsGT[2];
        ba_visualization_cloud->push_back(pcl_point);
        gt_visualization_cloud->push_back(pcl_point_gt);
    }
    for (const auto& finalPoint : final_cloud.points) {
        auto coords = finalPoint.coordinates.cast<float>();
        auto coordsGT = finalPoint.coordinates_GT.cast<float>();
        pcl::PointXYZRGB pcl_point;
        pcl_point.r = 255; pcl_point.g = 0; pcl_point.b = 0;
        pcl_point.x = coords[0]; pcl_point.y = coords[1]; pcl_point.z = coords[2];
        pcl::PointXYZRGB pcl_point_gt;
        pcl_point_gt.r = 255; pcl_point_gt.g = 0; pcl_point_gt.b = 0;
        pcl_point_gt.x = coordsGT[0]; pcl_point_gt.y = coordsGT[1]; pcl_point_gt.z = coordsGT[2];
        ba_visualization_cloud->push_back(pcl_point);
        gt_visualization_cloud->push_back(pcl_point_gt);
    }
    ba_viewer.addPointCloud(ba_visualization_cloud, "ba_visualization_cloud");
    ba_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "ba_visualization_cloud");
    gt_viewer.addPointCloud(gt_visualization_cloud, "gt_visualization_cloud");
    gt_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "gt_visualization_cloud");
}

void Visualizer::set_cameras(const std::vector<CameraPose>& camera_poses_history,
                             const std::deque<CameraPose>& camera_poses_active,
                             const std::vector<CameraPose>& camera_poses_GT)
{
    auto id = 0;
    for(const auto& camera_pose : camera_poses_history)
        ba_viewer.addPolygonMesh(generate_camera_mesh(camera_pose.pose, false), "polygon_"+std::to_string(id++));
    for(const auto& camera_pose : camera_poses_active)
        ba_viewer.addPolygonMesh(generate_camera_mesh(camera_pose.pose, true), "polygon_"+std::to_string(id++));
    for(const auto& camera_pose : camera_poses_GT)
        gt_viewer.addPolygonMesh(generate_camera_mesh(camera_pose.pose, false), "polygon_"+std::to_string(id++));
}

void Visualizer::visualize(const bool blocking)
{
    do {
        gt_viewer.spinOnce(50);
        ba_viewer.spinOnce(50);
    } while (blocking and !gt_viewer.wasStopped() and !ba_viewer.wasStopped());
    gt_viewer.resetStoppedFlag();
    ba_viewer.resetStoppedFlag();
    ba_viewer.removeAllPointClouds();
    gt_viewer.removeAllPointClouds();
    ba_visualization_cloud->clear();
    gt_visualization_cloud->clear();
}