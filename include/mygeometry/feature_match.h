
#ifndef FEATURE_MATCH_H
#define FEATURE_MATCH_H

#include <vector>
#include <string>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;

namespace mygeometry
{
void extractKeyPoints(cv::Mat &image, vector<cv::KeyPoint> &keypoints);
void computeDescriptors(cv::Mat &image, vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
void matchFeatures(
    const cv::Mat &descriptors_1, const cv::Mat &descriptors_2,
    vector<cv::DMatch> &matches);

// Remove duplicate matches. 
// After cv's match func, many kpts in I1 might matched to a same kpt in I2.
// Sorting the trainIdx(I2), and make the match unique.
void _remove_duplicate_matches(vector<cv::DMatch> &matches);

// Use a grid to remove the keypoints that are too close to each other.
void remove_tooclose_keypoints_by_grid(vector<cv::KeyPoint> &keypoints,
    const int image_rows, const int image_cols);

} // namespace mygeometry
#endif
