
#ifndef FEATURE_MATCH_H
#define FEATURE_MATCH_H

#include <vector>
#include <string>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;

namespace myslam
{
    void matchFeatures(
        const cv::Mat& descriptors_1, const cv::Mat& descriptors_2, 
        vector<cv::DMatch>& matches );
}

#endif
