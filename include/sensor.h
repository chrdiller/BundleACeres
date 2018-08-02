#pragma once

#include <types.h>

class SyntheticSensor {
public:
    explicit SyntheticSensor(const std::string& _dataset_dir);

    FrameData grab_frame() const;
    bool has_ended() const;

    Eigen::Matrix3d get_intrinsics();

private:
    const std::string dataset_dir;
    const int image_width, image_height;
    Eigen::Matrix3d intrinsics;

    mutable size_t current_frame_index;

    std::vector<Sophus::SE3d> all_poses;
};
