
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
void extractKeyPoints(cv::Mat &image, vector<cv::KeyPoint> &keypoints,
    bool SET_PARAM_BY_YAML=true);

void computeDescriptors(cv::Mat &image, vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors,
    bool SET_PARAM_BY_YAML=true);

void matchFeatures(
    const cv::Mat &descriptors_1, const cv::Mat &descriptors_2,
    vector<cv::DMatch> &matches,
    const bool PRINT_RES=false,
    const bool SET_PARAM_BY_YAML=true);

// Remove duplicate matches. 
// After cv's match func, many kpts in I1 might matched to a same kpt in I2.
// Sorting the trainIdx(I2), and make the match unique.
void _remove_duplicate_matches(vector<cv::DMatch> &matches);

// Use a grid to remove the keypoints that are too close to each other.
void _remove_tooclose_keypoints_by_grid(vector<cv::KeyPoint> &keypoints,
    const int image_rows, const int image_cols, 
    const bool SET_PARAM_BY_YAML=true);

} // namespace mygeometry

#endif
