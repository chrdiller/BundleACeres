#pragma once

#define EIGEN_NO_CUDA
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <sophus/se3.hpp>

struct CameraPose {
    size_t frame_index;
    Sophus::SE3d pose;
};

struct FrameData {
    FrameData(const size_t _frame_index,
              const cv::Mat& _color, const cv::Mat _depth,
              const Sophus::SE3d& _pose) :
            frame_index{_frame_index},
            color{_color}, depth{_depth},
            pose_GT{_frame_index, _pose} {}

    size_t frame_index;

    cv::Mat color;
    cv::Mat depth;

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    CameraPose pose_GT;
};

struct PointCorrespondence {
    cv::Point2d from;
    size_t from_frame;
    cv::Point2d to;
    size_t to_frame;
};
