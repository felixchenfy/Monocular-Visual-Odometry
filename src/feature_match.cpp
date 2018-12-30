
#include "myslam/feature_match.h"
#include "myslam/config.h"

namespace myslam
{
void matchFeatures(
    const cv::Mat &descriptors_1, const cv::Mat &descriptors_2,
    vector<cv::DMatch> &matches)
{
    static cv::FlannBasedMatcher matcher_flann(new cv::flann::LshIndexParams ( 5,10,2 ));
    static double match_ratio = Config::get<float>("match_ratio");
    // static double match_ratio = 2.0;

    // Match keypoints with similar descriptors.
    // For kpt_i, if kpt_j's descriptor if most similar to kpt_i's, then they are matched.
    vector<cv::DMatch> all_matches;
    matcher_flann.match(descriptors_1, descriptors_2, all_matches);

    // Find a min-distance threshold for selecting good matches
    float min_dis = std::min_element(
                        all_matches.begin(), all_matches.end(),
                        [](const cv::DMatch &m1, const cv::DMatch &m2) {
                            return m1.distance < m2.distance;
                        })
                        ->distance;
    double distance_threshold = max<float>(min_dis * match_ratio, 30.0);

    // Select good matches and push to the result vector.
    matches.clear();
    for (cv::DMatch &m : all_matches)
        if (m.distance < distance_threshold)
            matches.push_back(m);
}

} // namespace myslam