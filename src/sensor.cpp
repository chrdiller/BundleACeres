#include <sensor.h>

#include <OpenEXR/ImfInputFile.h>
#include <OpenEXR/ImfArray.h>

#include <sophus/se3.hpp>

const auto SKIP_N_FRAMES = 10;

auto load_all_poses(const std::string& data_dir) -> std::vector<Sophus::SE3d> {
    std::vector<Sophus::SE3d> poses;

    for (int pose_idx = 0;; ++pose_idx) {
        // Construct filename and load file contents
        std::stringstream ss;
        ss << data_dir << std::setw(4) << std::setfill('0') << pose_idx + 1 << ".dat";
        std::ifstream file { ss.str() };
        if(!file.good())
            break;

        // Construct a matrix from file contents
        Eigen::Matrix4d extrinsics { Eigen::Matrix4d::Identity() };
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 3; j++)
                file >> extrinsics(j, i);
        extrinsics.col(0).normalize();
        extrinsics.col(1).normalize();
        extrinsics.col(2).normalize();

        // Put the new pose into the vector
        poses.push_back(Sophus::SE3d{Sophus::SE3d::fitToSE3(extrinsics)}.inverse());
    }

    return poses;
}

SyntheticSensor::SyntheticSensor(const std::string& _dataset_dir)
: current_frame_index{0}, dataset_dir{_dataset_dir}, image_width{640}, image_height{480} {
    const double fx = image_width * 4.1 / 4.54;
    const double fy = image_height * 4.1 / 3.42;
    const double cx = image_width / 2.0;
    const double cy = image_height / 2.0;

    intrinsics << fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0;

    all_poses = load_all_poses(_dataset_dir);
}

FrameData SyntheticSensor::grab_frame() const {
    std::stringstream ss;
    ss << dataset_dir << std::setw(4) << std::setfill('0') << current_frame_index*SKIP_N_FRAMES + 1 << ".exr";

    auto color = cv::imread(ss.str(), cv::IMREAD_UNCHANGED);
    if(color.empty())
        throw std::runtime_error { "Frame could not be grabbed" };
    color.convertTo(color, CV_8UC3, 255.0);

    Imf::InputFile exr_file { ss.str().c_str() };
    const auto data_window = exr_file.header().dataWindow();
    Imf::Array2D<float> z_pixels { image_height, image_width };

    Imf::FrameBuffer frameBuffer;
    frameBuffer.insert("Z", Imf::Slice(Imf::FLOAT, (char*)&z_pixels[0][0], sizeof(float)*1, sizeof(float)*1*image_width));
    exr_file.setFrameBuffer(frameBuffer);
    exr_file.readPixels(data_window.min.y, data_window.max.y);

    cv::Mat depth;
    cv::Mat{image_height, image_width, CV_32FC1, &z_pixels[0][0]}.convertTo(depth, CV_64FC1);

    return FrameData { current_frame_index, color, depth, all_poses[current_frame_index++*SKIP_N_FRAMES] };
}

bool SyntheticSensor::has_ended() const {
    return current_frame_index == all_poses.size();
}

Eigen::Matrix3d SyntheticSensor::get_intrinsics() {
    return intrinsics;
}
