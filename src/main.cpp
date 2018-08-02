
#include <sensor.h>
#include <features.h>
#include <point_cloud.h>
#include <visualization.h>
#include <optimization.h>

const auto CHAIN_LENGTH = 8;
const auto MIN_OCCURENCES = 4;

int main()
{
    // Creating objects, processing first frame
    std::cout << "PHASE I: Initialization" << std::endl;
    SyntheticSensor sensor { "data/synthetic_dataset/" };
    FeatureComputation feature_computation;
    PointCloud waiting_cloud, bundle_cloud, final_cloud;
    Visualizer visualizer;
    std::vector<CameraPose> camera_poses_history, camera_poses_GT;
    std::deque<CameraPose> camera_poses_active;

    std::vector<FrameData> frames;

    frames.push_back(sensor.grab_frame());
    feature_computation.process(frames[0]);
    camera_poses_GT.push_back(frames[0].pose_GT);
    camera_poses_active.push_back(frames[0].pose_GT);

    // Use CHAIN_LENGTH amount of frames with their ground truth in order to initialize the optimization
    std::cout << "PHASE II: First Bundle" << std::endl;
    for(int init_frame = 1; init_frame < CHAIN_LENGTH; ++init_frame) {
        // Grab new frame from sensor
        auto current_frame = sensor.grab_frame();
        // Calculate keypoints and descriptors
        feature_computation.process(current_frame);
        frames.push_back(current_frame);
        // Get correspondences by feature matching between the current and the previous frame
        auto correspondences = feature_computation.match(current_frame, frames[current_frame.frame_index - 1]);
        // Try to integrate correspondences into existing clouds
        waiting_cloud.integrate_correspondences(correspondences);
        // Add the remaining ones with ground truth coordinates
        auto points = generate_points3D_from_groundtruth(correspondences, current_frame.depth,
                sensor.get_intrinsics(), current_frame.pose_GT.pose);
        waiting_cloud.add_points(points);

        camera_poses_GT.push_back(current_frame.pose_GT);
        camera_poses_active.push_back(current_frame.pose_GT);

        // Update the visualizer
        visualizer.set_pointclouds(waiting_cloud, final_cloud);
        visualizer.set_cameras(camera_poses_history, camera_poses_active, camera_poses_GT);

        visualizer.visualize(false);
    }

    std::cout << "PHASE III: Bundle adjustment" << std::endl;
    int interval = 0;
    while(!sensor.has_ended()) {
        // Grab new frame from sensor
        auto current_frame = sensor.grab_frame();
        // Calculate keypoints and descriptors
        feature_computation.process(current_frame);
        frames.push_back(current_frame);
        // Get correspondences by feature matching between the current and the previous frame
        auto correspondences = feature_computation.match(current_frame, frames[current_frame.frame_index - 1]);
        // Try to integrate correspondences into existing clouds and promote points with at least MIN_OCCURENCES observations
        waiting_cloud.integrate_correspondences(correspondences);
        auto promoted = waiting_cloud.pop_observed_n_times(MIN_OCCURENCES);
        bundle_cloud.add_points(promoted);
        bundle_cloud.integrate_correspondences(correspondences);

        // Add the remaining ones with ground truth coordinates
        auto points = generate_points3D_from_groundtruth(correspondences, current_frame.depth,
                                                         sensor.get_intrinsics(), current_frame.pose_GT.pose);
        waiting_cloud.add_points(points);

        camera_poses_GT.push_back(current_frame.pose_GT);
        camera_poses_history.push_back(camera_poses_active[0]);
        camera_poses_active.pop_front();
        camera_poses_active.push_back(CameraPose{current_frame.frame_index, camera_poses_active.back().pose});

        bundle_cloud.remove_observations_of_frame(current_frame.frame_index - CHAIN_LENGTH);
        waiting_cloud.remove_observations_of_frame(current_frame.frame_index - CHAIN_LENGTH);
        auto unobserved = bundle_cloud.pop_observed_n_times(0);
        final_cloud.add_points(unobserved);

        // Do optimization
        optimize(bundle_cloud, camera_poses_active, sensor.get_intrinsics());

        // Prepare for relative pose error evaluation
        std::vector<Sophus::SE3d> poses, posesGT;
        for(int i = 0; i < camera_poses_active.size(); ++i) {
            poses.push_back(camera_poses_active[i].pose);
            posesGT.push_back(camera_poses_GT[camera_poses_GT.size() - CHAIN_LENGTH + i].pose);
        }

        // Update the visualizer
        visualizer.set_pointclouds(bundle_cloud, final_cloud);
        visualizer.set_cameras(camera_poses_history, camera_poses_active, camera_poses_GT);

        if(interval++ == 5)
            visualizer.visualize(false), interval = 0;
        else
            visualizer.visualize(false);
    }

    std::cout << "Done. Goodbye." << std::endl;
    return EXIT_SUCCESS;
}
