#include <features.h>

void FeatureComputation::process(FrameData& data) const {
    orb->detect(data.color, data.keypoints);
    brisk->compute(data.color, data.keypoints, data.descriptors);
}

/**
 * Match descriptors from an image to another
 * @param from The query descriptors of the current image
 * @param to The train descriptors of the previous image
 */
auto FeatureComputation::match(FrameData& from, FrameData& to) const -> std::vector<PointCorrespondence> {
    // Use OpenCV to match the descriptors
    std::vector<cv::DMatch> matches;
    matcher->match(from.descriptors, to.descriptors, matches);

    // Transform matched keypoints into lists of points
    std::vector<cv::Point2d> from_points, to_points;
    for(const auto& match : matches) {
        from_points.push_back(from.keypoints[match.queryIdx].pt);
        to_points.push_back(to.keypoints[match.trainIdx].pt);
    }

    // Use those to run RANSAC
    cv::Mat mask;
    cv::findHomography(from_points, to_points, mask, cv::RANSAC, 3);

//    cv::Mat out;
//    cv::drawMatches(from.color, from.keypoints, to.color, to.keypoints, matches, out, cv::Scalar::all(-1),
//                    cv::Scalar::all(-1), mask);
//    cv::imshow("out", out);
//    cv::waitKey();

    // Finally, extract the inliers that RANSAC identified
    std::vector<PointCorrespondence> corresponding_points;
    for(int match_idx = 0; match_idx < mask.rows; ++match_idx) {
        if (mask.at<uchar>(match_idx, 0) == 1) {
            corresponding_points.push_back(
                    {from_points[match_idx], from.frame_index, to_points[match_idx], to.frame_index});
        }
    }

    std::sort(corresponding_points.begin(), corresponding_points.end(),
              [](PointCorrespondence& lhs, PointCorrespondence& rhs) {
                  return (lhs.to_frame < rhs.to_frame) or
                         (lhs.to_frame == rhs.to_frame and lhs.to.x < rhs.to.x) or
                         (lhs.to_frame == rhs.to_frame and lhs.to.x == rhs.to.x and lhs.to.y < rhs.to.y);
              });

    corresponding_points.erase(
            std::unique(corresponding_points.begin(), corresponding_points.end(),
                        [](PointCorrespondence& lhs, PointCorrespondence& rhs) {
                            return (lhs.to_frame == rhs.to_frame and lhs.to.x == rhs.to.x and lhs.to.y == rhs.to.y);
                        }),
            corresponding_points.end());

    return corresponding_points;
}
