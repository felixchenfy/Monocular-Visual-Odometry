
#ifndef MY_SLAM_FEATURE_MATCH_H
#define MY_SLAM_FEATURE_MATCH_H

#include "my_slam/common_include.h"

namespace my_slam
{
namespace geometry
{

void calcKeyPoints(const cv::Mat &image,
                   vector<cv::KeyPoint> &keypoints);

/* @brief Compute the descriptors of keypoints.
 *      Meanwhile, keypoints might be changed.
 */
void calcDescriptors(const cv::Mat &image,
                     vector<cv::KeyPoint> &keypoints,
                     cv::Mat &descriptors);

void matchFeatures(
    const cv::Mat1b &descriptors_1, const cv::Mat1b &descriptors_2,
    vector<cv::DMatch> &matches,
    bool is_print_res = false,
    // Below are optional arguments for feature_matching_method_index==3
    const vector<cv::KeyPoint> &keypoints_1 = vector<cv::KeyPoint>(),
    const vector<cv::KeyPoint> &keypoints_2 = vector<cv::KeyPoint>(),
    const double max_dist_between_two_matched_kpts = 0.0);

// Remove duplicate matches.
// After cv's match func, many kpts in I1 might matched to a same kpt in I2.
// Sorting the trainIdx(I2), and make the match unique.
void removeDuplicatedMatches(vector<cv::DMatch> &matches);

// Use a grid to remove the keypoints that are too close to each other.
void selectUniformKptsByGrid(vector<cv::KeyPoint> &keypoints,
                             int image_rows, int image_cols);

// --------------------- Other assistant functions ---------------------
double computeMeanDistBetweenKeypoints(
    const vector<cv::KeyPoint> &kpts1, const vector<cv::KeyPoint> &kpts2, const vector<cv::DMatch> &matches);

// --------------------- Datatype conversion ---------------------
vector<cv::DMatch> inliers2DMatches(const vector<int> inliers);
vector<cv::KeyPoint> pts2Keypts(const vector<cv::Point2f> pts);

} // namespace geometry
} // namespace my_slam

#endif
