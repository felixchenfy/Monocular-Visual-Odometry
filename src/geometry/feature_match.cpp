
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

vector<cv::DMatch> matchByRadiusAndBruteForce(
    const vector<cv::KeyPoint> &keypoints_1,
    const vector<cv::KeyPoint> &keypoints_2,
    const cv::Mat1b &descriptors_1,
    const cv::Mat1b &descriptors_2,
    float max_matching_pixel_dist)
{
    int N1 = keypoints_1.size(), N2 = keypoints_2.size();
    assert(N1 == descriptors_1.rows && N2 == descriptors_2.rows);
    vector<cv::DMatch> matches;
    float r2 = max_matching_pixel_dist * max_matching_pixel_dist;
    for (int i = 0; i < N1; i++)
    {
        const cv::KeyPoint &kpt1 = keypoints_1[i];
        bool is_matched = false;
        float x = kpt1.pt.x, y = kpt1.pt.y;
        double min_feature_dist = 99999999.0, target_idx = 0;
        for (int j = 0; j < N2; j++)
        {
            float x2 = keypoints_2[j].pt.x, y2 = keypoints_2[j].pt.y;
            if ((x - x2) * (x - x2) + (y - y2) * (y - y2) <= r2)
            {
                // double feature_dist = cv::norm(descriptors_1.row(i), descriptors_2.row(j));
                cv::Mat diff;
                cv::absdiff(descriptors_1.row(i), descriptors_2.row(j), diff);
                double feature_dist = cv::sum(diff)[0] / descriptors_1.cols;
                if (feature_dist < min_feature_dist)
                {
                    min_feature_dist = feature_dist;
                    target_idx = j;
                    is_matched = true;
                }
            }
        }
        if (is_matched)
            matches.push_back(cv::DMatch(i, target_idx, static_cast<float>(min_feature_dist)));
    }
    return matches;
}

void matchFeatures(
    const cv::Mat1b &descriptors_1, const cv::Mat1b &descriptors_2,
    vector<cv::DMatch> &matches,
    int method_index,
    bool is_print_res,
    // Below are optional arguments for feature_matching_method_index==3
    const vector<cv::KeyPoint> &keypoints_1,
    const vector<cv::KeyPoint> &keypoints_2,
    float max_matching_pixel_dist)
{
    // -- Set arguments
    static const double xiang_gao_method_match_ratio = basics::Config::get<int>("xiang_gao_method_match_ratio");
    static const double lowe_method_dist_ratio = basics::Config::get<int>("lowe_method_dist_ratio");
    static const double method_3_feature_dist_threshold = basics::Config::get<int>("method_3_feature_dist_threshold");
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
    if (method_index == 1 || method_index == 3) // the method in Dr. Xiang Gao's slambook
        // Match keypoints with similar descriptors.
        // For kpt_i, if kpt_j's descriptor if most similar to kpt_i's, then they are matched.
    {
        vector<cv::DMatch> all_matches;
        if (method_index == 3)
            all_matches = matchByRadiusAndBruteForce(
                keypoints_1, keypoints_2, descriptors_1, descriptors_2,
                max_matching_pixel_dist);
        else
            matcher_flann.match(descriptors_1, descriptors_2, all_matches);

        // if (method_index == 3)
        //     distance_threshold = method_3_feature_dist_threshold;
        // else
        // {
        //     // Find a min-distance threshold for selecting good matches
        //     for (int i = 0; i < all_matches.size(); i++)
        //     {
        //         double dist = all_matches[i].distance;
        //         if (dist < min_dis)
        //             min_dis = dist;
        //         if (dist > max_dis)
        //             max_dis = dist;
        //     }
        //     distance_threshold = std::max<float>(min_dis * xiang_gao_method_match_ratio, 30.0);
        // }
        for (int i = 0; i < all_matches.size(); i++)
        {
            double dist = all_matches[i].distance;
            if (dist < min_dis)
                min_dis = dist;
            if (dist > max_dis)
                max_dis = dist;
        }
        distance_threshold = std::max<float>(min_dis * xiang_gao_method_match_ratio, 30.0);

        // Another way of getting the minimum:
        // min_dis = std::min_element(all_matches.begin(), all_matches.end(),
        //     [](const cv::DMatch &m1, const cv::DMatch &m2) {return m1.distance < m2.distance;})->distance;

        // Select good matches and push to the result vector.
        for (cv::DMatch &m : all_matches)
            if (m.distance < distance_threshold)
                matches.push_back(m);
    }
    else if (method_index == 2)
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
            if (dist < lowe_method_dist_ratio * knn_matches[i][1].distance)
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
        printf("Using method %d, threshold = %f\n", method_index, distance_threshold);
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