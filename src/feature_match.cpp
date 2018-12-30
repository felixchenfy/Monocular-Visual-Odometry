
#include "mygeometry/feature_match.h"
#include "myslam/config.h"

namespace mygeometry
{

void extractKeyPoints(cv::Mat &image, vector<cv::KeyPoint> &keypoints)
{
    static int num_of_features_ = myslam::Config::get<int>("number_of_keypoints_to_extract");
    static double scale_factor_ = myslam::Config::get<double>("scale_factor");
    static int level_pyramid_ = myslam::Config::get<int>("level_pyramid");
    static cv::Ptr<cv::ORB> orb = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
    orb->detect(image, keypoints);
    bool USE_GRID_RESAMPLE=true;
    remove_tooclose_keypoints_by_grid(keypoints, image.rows, image.cols);
}

void computeDescriptors(cv::Mat &image, vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    static int num_of_features_ = myslam::Config::get<int>("number_of_keypoints_to_extract");
    static double scale_factor_ = myslam::Config::get<double>("scale_factor");
    static int level_pyramid_ = myslam::Config::get<int>("level_pyramid");
    static cv::Ptr<cv::ORB> orb = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
    orb->compute(image, keypoints, descriptors);
}

void remove_tooclose_keypoints_by_grid(vector<cv::KeyPoint>& keypoints,
    const int image_rows, const int image_cols)
{
    static const int GRID_SIZE=8, MAX_PTS_IN_GRID=2; // x=8, y=2, (640/x)*(480/x)*y=4800*2=9600
    static int MAX_NUM_KEYPOINTS = myslam::Config::get<int>("max_number_of_keypoints");
    static vector<vector<int>> grid(image_rows/GRID_SIZE,
        vector<int>(image_cols/GRID_SIZE,0));
    
    // clear grid
    for (auto row: grid) //clear grid
        std::fill(row.begin(),row.end(),0);
    
    // Insert keypoints to grid. If not full, insert this keypoint to result
    vector<cv::KeyPoint> tmp_keypoints;
    int cnt=0;
    for (auto &kpt: keypoints){
        int row=((int)kpt.pt.y)/GRID_SIZE, col=((int)kpt.pt.x)/GRID_SIZE;
        if (++grid[row][col]<=MAX_PTS_IN_GRID){
            tmp_keypoints.push_back(kpt);
            cnt++;
            if (cnt>MAX_NUM_KEYPOINTS)break;
        }
    }

    // return
    keypoints=tmp_keypoints;
}



void matchFeatures(
    const cv::Mat &descriptors_1, const cv::Mat &descriptors_2,
    vector<cv::DMatch> &matches)
{
    static cv::FlannBasedMatcher matcher_flann(new cv::flann::LshIndexParams(5, 10, 2));
    static double match_ratio = myslam::Config::get<float>("match_ratio");
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

    // Sort res by "trainIdx", and then
    // remove duplicated "trainIdx" to obtain unique matches.
    _remove_duplicate_matches(matches);
}

void _remove_duplicate_matches(vector<cv::DMatch> &matches)
{
    // Sort res by "trainIdx".
    sort(matches.begin(), matches.end(),
         [](const cv::DMatch &m1, const cv::DMatch &m2) {
             return m1.trainIdx < m2.trainIdx;
         });
    // Remove duplicated "trainIdx", so that the matches will be unique.
    vector<cv::DMatch> res;
    if (!matches.empty()) res.push_back(matches[0]);
    for (int i = 1; i < matches.size(); i++)
    {
        if (matches[i].trainIdx != matches[i - 1].trainIdx)
        {
            res.push_back(matches[i]);
        }
    }
    res.swap(matches);
}

} // namespace mygeometry