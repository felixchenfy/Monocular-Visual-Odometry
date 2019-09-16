
#include "my_slam/geometry/feature_match.h"
#include "my_slam/basics/opencv_funcs.h"
#include "my_slam/basics/config.h"

namespace my_slam
{
namespace geometry
{

void calcKeyPoints(
    const cv::Mat &image,
    vector<cv::KeyPoint> &keypoints)
{
    // -- Set arguments
    static const int num_keypoints = basics::Config::get<int>("number_of_keypoints_to_extract");
    static const double scale_factor = basics::Config::get<double>("scale_factor");
    static const int level_pyramid = basics::Config::get<int>("level_pyramid");
    static const int score_threshold = basics::Config::get<int>("score_threshold");

    // -- Create ORB
    static cv::Ptr<cv::ORB> orb = cv::ORB::create(num_keypoints, scale_factor, level_pyramid,
                                                  31, 0, 2, cv::ORB::HARRIS_SCORE, 31, score_threshold);
    // Default arguments of ORB:
    //          int 	nlevels = 8,
    //          int 	edgeThreshold = 31,
    //          int 	firstLevel = 0,
    //          int 	WTA_K = 2,
    //          ORB::ScoreType 	scoreType = ORB::HARRIS_SCORE,
    //          int 	patchSize = 31,
    //          int 	fastThreshold = 20

    // compute
    orb->detect(image, keypoints);
    selectUniformKptsByGrid(keypoints, image.rows, image.cols);
}

void calcDescriptors(
    const cv::Mat &image,
    vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{
    static const int num_keypoints = basics::Config::get<int>("number_of_keypoints_to_extract");
    static const double scale_factor = basics::Config::get<double>("scale_factor");
    static const int level_pyramid = basics::Config::get<int>("level_pyramid");
    static cv::Ptr<cv::ORB> orb = cv::ORB::create(num_keypoints, scale_factor, level_pyramid);

    // compute
    orb->compute(image, keypoints, descriptors);
}

void selectUniformKptsByGrid(
    vector<cv::KeyPoint> &keypoints,
    int image_rows, int image_cols)
{
    // -- Set arguments
    static const int max_num_keypoints = basics::Config::get<int>("max_number_of_keypoints");
    static const int kpts_uniform_selection_grid_size = basics::Config::get<int>("kpts_uniform_selection_grid_size");
    static const int kpts_uniform_selection_max_pts_per_grid = basics::Config::get<int>("kpts_uniform_selection_max_pts_per_grid");
    static const int rows = image_rows / kpts_uniform_selection_grid_size, cols = image_cols / kpts_uniform_selection_grid_size;

    // Create an empty grid
    static vector<vector<int>> grid(rows, vector<int>(cols, 0));
    for (auto &row : grid) //clear grid
        std::fill(row.begin(), row.end(), 0);

    // Insert keypoints to grid. If not full, insert this cv::KeyPoint to result
    vector<cv::KeyPoint> tmp_keypoints;
    int cnt = 0;
    for (auto &kpt : keypoints)
    {
        int row = ((int)kpt.pt.y) / kpts_uniform_selection_grid_size, col = ((int)kpt.pt.x) / kpts_uniform_selection_grid_size;
        if (grid[row][col] < kpts_uniform_selection_max_pts_per_grid)
        {
            tmp_keypoints.push_back(kpt);
            grid[row][col]++;
            cnt++;
            if (cnt > max_num_keypoints)
                break;
        }
    }

    // return
    keypoints = tmp_keypoints;
}

// void MatcherBfRadius()
// {

// }
void matchFeatures(
    const cv::Mat1b &descriptors_1, const cv::Mat1b &descriptors_2,
    vector<cv::DMatch> &matches,
    bool is_print_res,
    const vector<cv::KeyPoint> &keypoints_1,
    const vector<cv::KeyPoint> &keypoints_2,
    const double max_dist_between_two_matched_kpts)
{
    // -- Set arguments
    static const double match_ratio = basics::Config::get<int>("match_ratio");
    static const double dist_ratio = basics::Config::get<int>("lowe_dist_ratio");
    static const int feature_matching_method_index = basics::Config::get<int>("feature_match_method_index");

    static cv::FlannBasedMatcher matcher_flann(new cv::flann::LshIndexParams(5, 10, 2));
    static cv::Ptr<cv::DescriptorMatcher> matcher_bf = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // -- Debug: see descriptors_1's content:
    //    Result: It'S 8UC1, the value ranges from 0 to 255. It's not binary!
    // basics::print_MatProperty(descriptors_1);
    // for (int i = 0; i < 32; i++)
    //     std::cout << int(descriptors_1.at<unsigned char>(0, i)) << std::endl;

    // Start matching
    matches.clear();
    double min_dis = 9999999, max_dis = 0, distance_threshold = -1;
    if (feature_matching_method_index == 1) // the method in Dr. Xiang Gao's slambook
        // Match keypoints with similar descriptors.
        // For kpt_i, if kpt_j's descriptor if most similar to kpt_i's, then they are matched.
    {
        vector<cv::DMatch> all_matches;
        matcher_flann.match(descriptors_1, descriptors_2, all_matches);

        // Find a min-distance threshold for selecting good matches
        for (int i = 0; i < all_matches.size(); i++)
        {
            double dist = all_matches[i].distance;
            if (dist < min_dis)
                min_dis = dist;
            if (dist > max_dis)
                max_dis = dist;
        }
        distance_threshold = std::max<float>(min_dis * match_ratio, 30.0);
        // Another way of getting the minimum:
        // min_dis = std::min_element(all_matches.begin(), all_matches.end(),
        //     [](const cv::DMatch &m1, const cv::DMatch &m2) {return m1.distance < m2.distance;})->distance;

        // Select good matches and push to the result vector.
        for (cv::DMatch &m : all_matches)
            if (m.distance < distance_threshold)
                matches.push_back(m);
    }
    else if (feature_matching_method_index == 2)
    { // method in Lowe's 2004 paper
        // Calculate the features's distance of the two images.
        vector<vector<cv::DMatch>> knn_matches;
        vector<cv::Mat> train_desc(1, descriptors_2);
        matcher_bf->add(train_desc);
        matcher_bf->train();
        // For a point "PA_i" in image A,
        // only return its nearest 2 points "PB_i0" and "PB_i1" in image B.
        // The result is saved in knn_matches.
        matcher_bf->knnMatch(descriptors_1, descriptors_2, knn_matches, 2);

        // Remove bad matches using the method proposed by Lowe in his SIFT paper
        //		by checking the ratio of the nearest and the second nearest distance.
        for (int i = 0; i < knn_matches.size(); i++)
        {

            double dist = knn_matches[i][0].distance;
            if (dist < dist_ratio * knn_matches[i][1].distance)
                matches.push_back(knn_matches[i][0]);
            if (dist < min_dis)
                min_dis = dist;
            if (dist > max_dis)
                max_dis = dist;
        }
    }
    else
        throw std::runtime_error("feature_match.cpp::matchFeatures: wrong method index.");

    // Sort res by "trainIdx", and then
    // remove duplicated "trainIdx" to obtain unique matches.
    removeDuplicatedMatches(matches);

    if (is_print_res)
    {
        printf("Matching features:\n");
        printf("Using method %d, threshold = %f\n", feature_matching_method_index, distance_threshold);
        printf("Number of matches: %d\n", int(matches.size()));
        printf("-- Max dist : %f \n", max_dis);
        printf("-- Min dist : %f \n", min_dis);
    }
}

void removeDuplicatedMatches(vector<cv::DMatch> &matches)
{
    // Sort res by "trainIdx".
    sort(matches.begin(), matches.end(),
         [](const cv::DMatch &m1, const cv::DMatch &m2) {
             return m1.trainIdx < m2.trainIdx;
         });
    // Remove duplicated "trainIdx", so that the matches will be unique.
    vector<cv::DMatch> res;
    if (!matches.empty())
        res.push_back(matches[0]);
    for (int i = 1; i < matches.size(); i++)
    {
        if (matches[i].trainIdx != matches[i - 1].trainIdx)
        {
            res.push_back(matches[i]);
        }
    }
    res.swap(matches);
}

// --------------------- Other assistant functions ---------------------
double computeMeanDistBetweenKeypoints(
    const vector<cv::KeyPoint> &kpts1, const vector<cv::KeyPoint> &kpts2, const vector<cv::DMatch> &matches)
{

    vector<double> dists_between_kpts;
    for (const cv::DMatch &d : matches)
    {
        cv::Point2f p1 = kpts1[d.queryIdx].pt;
        cv::Point2f p2 = kpts2[d.trainIdx].pt;
        dists_between_kpts.push_back(basics::calcDist(p1, p2));
    }
    double mean_dist = 0;
    for (double d : dists_between_kpts)
        mean_dist += d;
    mean_dist /= dists_between_kpts.size();
    return mean_dist;
}

// --------------------- datatype transform ---------------------

vector<cv::DMatch> inliers2DMatches(const vector<int> inliers)
{
    vector<cv::DMatch> matches;
    for (auto idx : inliers)
    {
        // cv::DMatch (int _queryIdx, int _trainIdx, float _distance)
        matches.push_back(cv::DMatch(idx, idx, 0.0));
    }
    return matches;
}
vector<cv::KeyPoint> pts2Keypts(const vector<cv::Point2f> pts)
{
    // cv.cv::KeyPoint(	x, y, _size[, _angle[, _response[, _octave[, _class_id]]]]	)
    // cv.cv::KeyPoint(	pt, _size[, _angle[, _response[, _octave[, _class_id]]]]	)
    vector<cv::KeyPoint> keypts;
    for (cv::Point2f pt : pts)
    {
        keypts.push_back(cv::KeyPoint(pt, 10));
    }
    return keypts;
}

} // namespace geometry
} // namespace my_slam