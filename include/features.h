#pragma once

#include <types.h>

/**
 * Computes features in a frame
 * Stores OpenCV's ORB, BRISK and Matcher
 */
class FeatureComputation {
public:
    FeatureComputation() :
            orb { cv::ORB::create() },
            brisk { cv::BRISK::create() },
            matcher { cv::DescriptorMatcher::create("BruteForce-Hamming") } {}

    void process(FrameData& data) const;
    auto match(FrameData& from, FrameData& to) const -> std::vector<PointCorrespondence>;

private:
    const cv::Ptr<cv::ORB> orb;
    const cv::Ptr<cv::BRISK> brisk;
    const cv::Ptr<cv::DescriptorMatcher> matcher;
};
