
#include "my_geometry/feature_match.h"
#include "my_basics/config.h"

namespace my_geometry
{

// set default ORB params
#define _num_keypoints 5000
#define _scale_factor 1.2
#define _level_pyramid 4

// set default Grid params for non-maximum suppression
#define _max_num_keypoints 1000
#define _grid_size 8
#define _max_pts_in_grid 2 /*x=8, y=2, (640/x)*(480/x)*y=4800*2=9600*/

// set default feature matching params
#define _match_ratio 2.0
#define _lowe_dist_ratio 0.6 /*for method 2*/

using namespace cv;

void extractKeyPoints(cv::Mat &image, vector<cv::KeyPoint> &keypoints,
    const bool SET_PARAM_BY_YAML)
{
    static int num_keypoints=_num_keypoints, level_pyramid=_level_pyramid;
    static double scale_factor=_scale_factor;
    static int cnt_call_times_=0;
    if (cnt_call_times_++==0 && SET_PARAM_BY_YAML){
        num_keypoints = my_basics::Config::get<int>("number_of_keypoints_to_extract");
        scale_factor = my_basics::Config::get<double>("scale_factor");
        level_pyramid = my_basics::Config::get<int>("level_pyramid");
    }
    static cv::Ptr<cv::ORB> orb = cv::ORB::create(num_keypoints, scale_factor, level_pyramid);

    // compute
    orb->detect(image, keypoints);
    selectUniformByGrid(keypoints, image.rows, image.cols, SET_PARAM_BY_YAML);
}

void computeDescriptors(cv::Mat &image, vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors,
    const bool SET_PARAM_BY_YAML)
{
    static int num_keypoints=_num_keypoints, level_pyramid=_level_pyramid;
    static double scale_factor=_scale_factor;
    static int cnt_call_times_=0;
    if (cnt_call_times_++==0 && SET_PARAM_BY_YAML){
        num_keypoints = my_basics::Config::get<int>("number_of_keypoints_to_extract");
        scale_factor = my_basics::Config::get<double>("scale_factor");
        level_pyramid = my_basics::Config::get<int>("level_pyramid");
    }
    static cv::Ptr<cv::ORB> orb = cv::ORB::create(num_keypoints, scale_factor, level_pyramid);

    // compute
    orb->compute(image, keypoints, descriptors);
}


void selectUniformByGrid(vector<cv::KeyPoint>& keypoints,
    const int image_rows, const int image_cols,
    const bool SET_PARAM_BY_YAML)
{
    static int GRID_SIZE=_grid_size, MAX_PTS_IN_GRID=_max_pts_in_grid;
    static int max_num_keypoints = _max_num_keypoints;
    static int cnt_call_times_=0;
    if (cnt_call_times_++==0 && SET_PARAM_BY_YAML){
        max_num_keypoints = my_basics::Config::get<int>("max_number_of_keypoints");
        GRID_SIZE = my_basics::Config::get<int>("GRID_SIZE");
        MAX_PTS_IN_GRID = my_basics::Config::get<int>("MAX_PTS_IN_GRID");
    }
    static int rows=image_rows/GRID_SIZE, cols=image_cols/GRID_SIZE;
    static vector<vector<int>> grid(rows,vector<int>(cols,0));
    // clear grid
    // for (int i=0;i<rows;i++)for(int j=0;j<cols;j++)grid[i][j]=0;
    for (auto &row: grid) //clear grid
        std::fill(row.begin(),row.end(),0);
    
    // Insert keypoints to grid. If not full, insert this keypoint to result
    vector<cv::KeyPoint> tmp_keypoints;
    int cnt=0;
    for (auto &kpt: keypoints){
        int row=((int)kpt.pt.y)/GRID_SIZE, col=((int)kpt.pt.x)/GRID_SIZE;
        if (grid[row][col]<MAX_PTS_IN_GRID){
            tmp_keypoints.push_back(kpt);
            grid[row][col]++;
            cnt++;
            if (cnt>max_num_keypoints)break;
        }
    }

    // return
    keypoints=tmp_keypoints;
}



void matchFeatures(
    const cv::Mat &descriptors_1, const cv::Mat &descriptors_2,
    vector<cv::DMatch> &matches,
    const bool PRINT_RES, const bool SET_PARAM_BY_YAML)
{
    matches.clear();
    static cv::FlannBasedMatcher matcher_flann(new cv::flann::LshIndexParams(5, 10, 2));
    static double MATCH_RATIO = _match_ratio, DIST_RATIO = _lowe_dist_ratio;
    static int cnt_call_times_=0, METHOD_INDEX=2;
    if (cnt_call_times_++==0 && SET_PARAM_BY_YAML){
        MATCH_RATIO = my_basics::Config::get<int>("match_ratio");
        DIST_RATIO = my_basics::Config::get<int>("lowe_dist_ratio");
        METHOD_INDEX = my_basics::Config::get<int>("feature_match_method_index");
    }

    double min_dis = 9999999, max_dis = 0, distance_threshold=-1;
    if(METHOD_INDEX==1){ // by the method in Dr. Xiang Gao's slambook
        // Match keypoints with similar descriptors.
        // For kpt_i, if kpt_j's descriptor if most similar to kpt_i's, then they are matched.
        vector<cv::DMatch> all_matches;
        matcher_flann.match(descriptors_1, descriptors_2, all_matches);

        // Find a min-distance threshold for selecting good matches
        for (int i = 0; i < all_matches.size(); i++){
            double dist = all_matches[i].distance;
            if (dist < min_dis) min_dis = dist;
            if (dist > max_dis) max_dis = dist;
        }
        distance_threshold = max<float>(min_dis * MATCH_RATIO, 30.0);
        // Another way of getting the minimum:
        // min_dis = std::min_element(all_matches.begin(), all_matches.end(),
        //     [](const cv::DMatch &m1, const cv::DMatch &m2) {return m1.distance < m2.distance;})->distance;

        // Select good matches and push to the result vector.
        for (cv::DMatch &m : all_matches)
            if (m.distance < distance_threshold)
                matches.push_back(m);
    }else if(METHOD_INDEX==2){ // method in Lowe's 2004 paper
        static Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
        // Calculate the features's distance of the two images. 
        vector<vector<DMatch>> knn_matches;
        vector<Mat> train_desc(1, descriptors_2);
        matcher->add(train_desc);
        matcher->train();
        // For a point "PA_i" in image A, 
        // only return its nearest 2 points "PB_i0" and "PB_i1" in image B.
        // The result is saved in knn_matches.
        matcher->knnMatch(descriptors_1, descriptors_2, knn_matches, 2);

        // Remove bad matches using the method proposed by Lowe in his SIFT paper
        //		by checking the ratio of the nearest and the second nearest distance.
        for (int i = 0; i < knn_matches.size(); i++){

            double dist = knn_matches[i][0].distance ;
            if (dist < min_dis) min_dis = dist; // This is not required. Do this for debug
            if (dist > max_dis) max_dis = dist; // This is not required. Do this for debug
            if (dist < DIST_RATIO * knn_matches[i][1].distance){
                matches.push_back(knn_matches[i][0]);
            }
        }
    }else{
        assert(0);
    }
    // Sort res by "trainIdx", and then
    // remove duplicated "trainIdx" to obtain unique matches.
    removeDuplicatedMatches(matches);

    if (PRINT_RES){
        printf("Matching features:\n");
        printf("Using method %d, threshold = %f\n", METHOD_INDEX, distance_threshold);
        printf("Number of matches: %d\n", (int)matches.size());
        printf("-- Max dist : %f \n", max_dis);
        printf("-- Min dist : %f \n", min_dis);
    }
    printf("Number of matches: %d\n", (int)matches.size());
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



// --------------------- Other assistant functions ---------------------
double computeMeanDistBetweenKeypoints(
    const vector<KeyPoint> &kpts1, const vector<KeyPoint> &kpts2, const vector<DMatch> &matches)
{

    vector<double> dists_between_kpts;
    for (const DMatch &d : matches)
    {
        Point2f p1 = kpts1[d.queryIdx].pt;
        Point2f p2 = kpts2[d.trainIdx].pt;
        dists_between_kpts.push_back(calcDist(p1, p2));
    }
    double mean_dist = 0;
    for (double d : dists_between_kpts)
        mean_dist += d;
    mean_dist /= dists_between_kpts.size();
    return mean_dist;
}

// --------------------- datatype transform ---------------------

vector<DMatch> inliers2DMatches(const vector<int> inliers){
    vector<DMatch> matches;
    for(auto idx:inliers){
        // DMatch (int _queryIdx, int _trainIdx, float _distance)
        matches.push_back(DMatch(idx,idx,0.0));
    }
    return matches;
}
vector<KeyPoint> pts2keypts(const vector<Point2f> pts){
    // cv.KeyPoint(	x, y, _size[, _angle[, _response[, _octave[, _class_id]]]]	)
    // cv.KeyPoint(	pt, _size[, _angle[, _response[, _octave[, _class_id]]]]	)
    vector<KeyPoint> keypts;
    for(Point2f pt:pts){
        keypts.push_back(KeyPoint(pt, 10));
    }
    return keypts;
}

} // namespace my_geometry