
#include "my_slam/vo/vo.h"
#include "my_slam/optimization/g2o_ba.h"
#include <numeric>

namespace my_slam
{
namespace vo
{

VisualOdometry::VisualOdometry() : map_(new (Map))
{
    vo_state_ = BLANK;
}

void VisualOdometry::getMappointsInCurrentView(
    vector<MapPoint::Ptr> &candidate_mappoints_in_map,
    cv::Mat &corresponding_mappoints_descriptors)
{
    // vector<MapPoint::Ptr> candidate_mappoints_in_map;
    // cv::Mat corresponding_mappoints_descriptors;
    candidate_mappoints_in_map.clear();
    corresponding_mappoints_descriptors.release();
    for (auto &iter_map_point : map_->map_points_)
    {
        MapPoint::Ptr &p = iter_map_point.second;
        if (curr_->isInFrame(p->pos_)) // check if p in curr frame image
        {
            // -- add to candidate_mappoints_in_map
            candidate_mappoints_in_map.push_back(p);
            corresponding_mappoints_descriptors.push_back(p->descriptor_);
            p->visible_times_++;
        }
    }
}

// --------------------------------- Initialization ---------------------------------

void VisualOdometry::estimateMotionAnd3DPoints()
{
    // -- Rename output
    vector<cv::DMatch> &inlier_matches = curr_->inliers_matches_with_ref_;
    vector<cv::Point3f> &pts3d_in_curr = curr_->inliers_pts3d_;
    vector<cv::DMatch> &inliers_matches_for_3d = curr_->inliers_matches_for_3d_;
    cv::Mat &T = curr_->T_w_c_;

    // -- Start: call this big function to compute everything
    // (1) motion from Essential && Homography, (2) inliers indices, (3) triangulated points
    vector<cv::Mat> list_R, list_t, list_normal;
    vector<vector<cv::DMatch>> list_matches; // these are the inliers matches
    vector<vector<cv::Point3f>> sols_pts3d_in_cam1_by_triang;
    const bool print_res = false, is_frame_cam2_to_cam1 = true;
    const bool compute_homography = true;
    cv::Mat &K = curr_->camera_->K_;
    int best_sol = helperEstimatePossibleRelativePosesByEpipolarGeometry(
        /*Input*/
        ref_->keypoints_, curr_->keypoints_, curr_->matches_with_ref_, K,
        /*Output*/
        list_R, list_t, list_matches, list_normal, sols_pts3d_in_cam1_by_triang,
        /*settings*/
        print_res, compute_homography, is_frame_cam2_to_cam1);

    // -- Only retain the data of the best solution
    const cv::Mat &R_curr_to_prev = list_R[best_sol];
    const cv::Mat &t_curr_to_prev = list_t[best_sol];
    inlier_matches = list_matches[best_sol];
    const vector<cv::Point3f> &pts3d_in_cam1 = sols_pts3d_in_cam1_by_triang[best_sol];
    pts3d_in_curr.clear();
    for (const cv::Point3f &p1 : pts3d_in_cam1)
        pts3d_in_curr.push_back(transCoord(p1, R_curr_to_prev, t_curr_to_prev));

    // -- Output

    // compute camera pose
    T = ref_->T_w_c_ * convertRt2T(R_curr_to_prev, t_curr_to_prev).inv();

    // Get points that are used for triangulating new map points
    retainGoodTriangulationResult();

    int N = curr_->inliers_pts3d_.size();
    if (N < 20)
    {
        printf("After remove bad triag, only %d pts. It's too few ...\n", N);
        return;
    }

    //Normalize Points Depth to 1, and
    double mean_depth = calcMeanDepth(curr_->inliers_pts3d_);
    t_curr_to_prev /= mean_depth;
    for (cv::Point3f &p : curr_->inliers_pts3d_)
        scalePointPos(p, 1 / mean_depth);
    T = ref_->T_w_c_ * convertRt2T(R_curr_to_prev, t_curr_to_prev).inv(); // update pose
}

bool VisualOdometry::checkIfVoGoodToInit()
{

    // -- Rename input
    const vector<cv::KeyPoint> &init_kpts = ref_->keypoints_;
    const vector<cv::KeyPoint> &curr_kpts = curr_->keypoints_;
    // const vector<cv::DMatch> &matches = curr_->inliers_matches_with_ref_;
    const vector<cv::DMatch> &matches = curr_->inliers_matches_for_3d_;

    // Params
    static const double min_pixel_dist = basics::Config::get<double>("min_pixel_dist");
    static const double min_median_triangulation_angle = basics::Config::get<double>("min_median_triangulation_angle");

    // -- Check CRITERIA_0: num inliers should be large
    if (matches.size() < 20)
    {
        printf("Check VO init: Too few inlier points ...\n");
        return false;
    }

    // -- Check CRITERIA_1
    bool CRITERIA_1 = false; // init vo only when distance between matched keypoints are large
    {
        vector<double> dists_between_kpts;
        double mean_dist = computeMeanDistBetweenkeypoints(init_kpts, curr_kpts, matches);
        printf("\nPixel movement of matched keypoints: %.1f \n", mean_dist);

        CRITERIA_1 = mean_dist > min_pixel_dist;
    }

    // -- Check CRITERIA_2
    bool CRITERIA_2 = false; // Triangulation angle of each point should be larger than threshold.
    {
        vector<double> sort_a = curr_->triangulation_angles_of_inliers_; // a copy of angles
        int N = sort_a.size();                                           // num of 3d points triangulated from inlier points
        sort(sort_a.begin(), sort_a.end());
        double mean_angle = accumulate(sort_a.begin(), sort_a.end(), 0.0) / N;
        double median_angle = sort_a[N / 2];
        printf("Triangulation angle: mean=%f, median=%f, min=%f, max=%f\n",
               mean_angle,   // mean
               median_angle, // median
               sort_a[0],    // min
               sort_a[N - 1] // max
        );

        // Thresholding
        if (median_angle > min_median_triangulation_angle)
            CRITERIA_2 = true;
    }

    // -- Return
    return CRITERIA_1 && CRITERIA_2;
}

bool VisualOdometry::isInitialized()
{
    return vo_state_ == OK;
}

// ------------------------------- Triangulation -------------------------------

// Compute the triangulation angle of each point, and get the statistics.
// Remove those with a too large or too small angle.
void VisualOdometry::retainGoodTriangulationResult()
{
    static const double min_triang_angle = basics::Config::get<double>("min_triang_angle");
    static const double max_ratio_between_max_angle_and_median_angle =
        basics::Config::get<double>("max_ratio_between_max_angle_and_median_angle");

    // -- Input
    // 1. vector<cv::DMatch>  curr_ -> inliers_matches_with_ref_; // input
    // 2. vector<cv::Point3f> pts3d_in_curr:  curr_ -> inliers_pts3d_; // update this

    // -- Output
    // 1. generate this:
    vector<double> &angles = curr_->triangulation_angles_of_inliers_;
    // 2. update this:
    //vector<cv::Point3f> pts3d_in_curr:  curr_ -> inliers_pts3d_;
    // 3. generate this:
    //vector<cv::DMatch> inliers_matches: curr_ -> inliers_matches_for_3d_;

    // -- Compute angles
    int N = (int)curr_->inliers_pts3d_.size();
    if (N == 0)
        return;
    for (int i = 0; i < N; i++)
    {
        cv::Point3f &p_in_curr = curr_->inliers_pts3d_[i];
        cv::Mat p_in_world = point3f_to_mat(preTranslatePoint3f(p_in_curr, curr_->T_w_c_));
        cv::Mat vec_p_to_cam_curr = getPosFromT(curr_->T_w_c_) - p_in_world;
        cv::Mat vec_p_to_cam_prev = getPosFromT(ref_->T_w_c_) - p_in_world;
        double angle = calcAngleBetweenTwoVectors(vec_p_to_cam_curr, vec_p_to_cam_prev);
        angles.push_back(angle / 3.1415926 * 180.0);
    }

    // Get statistics
    vector<double> sort_a = angles;
    sort(sort_a.begin(), sort_a.end());
    double mean_angle = accumulate(sort_a.begin(), sort_a.end(), 0.0) / N;
    double median_angle = sort_a[N / 2];
    printf("Triangulation angle: mean=%f, median=%f, min=%f, max=%f\n",
           mean_angle,   // mean
           median_angle, // median
           sort_a[0],    // min
           sort_a[N - 1] // max
    );

    // Get good triangulation points

    vector<cv::Point3f> old_inlier_points = curr_->inliers_pts3d_;
    curr_->inliers_pts3d_.clear();

    vector<double> old_angles = angles;
    angles.clear();

    for (int i = 0; i < N; i++)
    {
        if (old_angles[i] < min_triang_angle ||
            old_angles[i] / median_angle > max_ratio_between_max_angle_and_median_angle)
            continue;
        cv::DMatch dmatch = curr_->inliers_matches_with_ref_[i];
        curr_->inliers_matches_for_3d_.push_back(dmatch);
        curr_->inliers_pts3d_.push_back(old_inlier_points[i]);
        angles.push_back(old_angles[i]);
    }
    return;
}

// ------------------- Tracking -------------------
bool VisualOdometry::checkLargeMoveForAddKeyFrame(Frame::Ptr curr, Frame::Ptr ref)
{
    cv::Mat T_key_to_curr = ref->T_w_c_.inv() * curr->T_w_c_;
    cv::Mat R, t, R_vec;
    getRtFromT(T_key_to_curr, R, t);
    cv::Rodrigues(R, R_vec);

    static const double MIN_DIST_BETWEEN_KEYFRAME = basics::Config::get<double>("MIN_DIST_BETWEEN_KEYFRAME");
    static const double MIN_ROTATED_ANGLE = basics::Config::get<double>("MIN_ROTATED_ANGLE");

    double moved_dist = calcMatNorm(t);
    double rotated_angle = calcMatNorm(R_vec);

    printf("Wrt prev keyframe, relative dist = %.5f, angle = %.5f\n", moved_dist, rotated_angle);

    // Satisfy each one will be a good keyframe
    bool res = moved_dist > MIN_DIST_BETWEEN_KEYFRAME || rotated_angle > MIN_ROTATED_ANGLE;
    return res;
}

void VisualOdometry::poseEstimationPnP()
{
    // -- From the local map, find the keypoints that fall into the current view
    vector<MapPoint::Ptr> candidate_mappoints_in_map;
    cv::Mat corresponding_mappoints_descriptors;
    getMappointsInCurrentView(candidate_mappoints_in_map, corresponding_mappoints_descriptors);

    // -- Compare descriptors to find matches, and extract 3d 2d correspondance
    geometry::matchFeatures(corresponding_mappoints_descriptors, curr_->descriptors_, curr_->matches_with_map_);
    cout << "Number of 3d-2d pairs: " << curr_->matches_with_map_.size() << endl;
    vector<cv::Point3f> pts_3d;
    vector<cv::Point2f> pts_2d; // a point's 2d pos in image2 pixel curr_
    for (int i = 0; i < curr_->matches_with_map_.size(); i++)
    {
        cv::DMatch &match = curr_->matches_with_map_[i];
        MapPoint::Ptr mappoint = candidate_mappoints_in_map[match.queryIdx];
        pts_3d.push_back(mappoint->pos_);
        pts_2d.push_back(curr_->keypoints_[match.trainIdx].pt);
    }

    // -- Solve PnP, get T_world_to_camera
    cv::Mat R_vec, R, t;
    bool useExtrinsicGuess = false;
    int iterationsCount = 100;
    float reprojectionError = 5.0;
    double confidence = 0.999;
    cv::Mat pnp_inliers_mask; // type = 32SC1, size = 999x1
    solvePnPRansac(pts_3d, pts_2d, curr_->camera_->K_, cv::Mat(), R_vec, t,
                   useExtrinsicGuess, iterationsCount, reprojectionError, confidence, pnp_inliers_mask);
    Rodrigues(R_vec, R);

    // -- Get inlier matches used in PnP
    vector<cv::Point2f> tmp_pts_2d;
    vector<cv::Point3f *> inlier_candidates_pos;
    vector<MapPoint::Ptr> inlier_candidates;
    vector<cv::DMatch> tmp_matches_with_map_;
    int num_inliers = pnp_inliers_mask.rows;
    for (int i = 0; i < num_inliers; i++)
    {
        int good_idx = pnp_inliers_mask.at<int>(i, 0);

        // good match
        cv::DMatch &match = curr_->matches_with_map_[good_idx];
        tmp_matches_with_map_.push_back(match);

        // good pts 2d
        tmp_pts_2d.push_back(pts_2d[good_idx]);

        // good pts 3d
        MapPoint::Ptr inlier_mappoint = candidate_mappoints_in_map[match.queryIdx];
        inlier_candidates_pos.push_back(&(inlier_mappoint->pos_));
        inlier_mappoint->matched_times_++;

        // Update graph info
        curr_->inliers_to_mappt_connections_[match.trainIdx] = PtConn{-1, inlier_mappoint->id_};
    }
    pts_2d.swap(tmp_pts_2d);
    curr_->matches_with_map_.swap(tmp_matches_with_map_);

    // -- Update current camera pos
    curr_->T_w_c_ = convertRt2T(R, t).inv();
}

// bundle adjustment
void VisualOdometry::callBundleAdjustment()
{
    // Read settings from config.yaml
    static const bool USE_BA = basics::Config::getBool("USE_BA");
    static const int MAX_NUM_FRAMES_FOR_BA = basics::Config::get<int>("MAX_NUM_FRAMES_FOR_BA");
    static const vector<double> im = basics::str2vecdouble(
        basics::Config::get<string>("information_matrix"));
    static const bool FIX_MAP_PTS = basics::Config::getBool("FIX_MAP_PTS");
    // static const bool UPDATE_MAP_PTS = basics::Config::getBool("UPDATE_MAP_PTS");
    static const bool UPDATE_MAP_PTS = !FIX_MAP_PTS;
    // cout << FIX_MAP_PTS << UPDATE_MAP_PTS << endl;

    // Set params
    const int TOTAL_FRAMES = frames_buff_.size();
    const int NUM_FRAMES_FOR_BA = min(MAX_NUM_FRAMES_FOR_BA, TOTAL_FRAMES - 1);
    const static cv::Mat information_matrix = (cv::Mat_<double>(2, 2) << im[0], im[1], im[2], im[3]);

    if (USE_BA != true)
    {
        printf("\nNot using bundle adjustment ... \n");
        return;
    }
    printf("\nCalling bundle adjustment on %d frames ... \n", NUM_FRAMES_FOR_BA);

    // Measurement (which is fixed; truth)
    vector<vector<cv::Point2f *>> v_pts_2d;
    vector<vector<int>> v_pts_2d_to_3d_idx;

    // Things to to optimize
    unordered_map<int, cv::Point3f *> um_pts_3d;
    vector<cv::Point3f *> v_pts_3d_only_in_curr; // This is an alternative of the above
    vector<cv::Mat *> v_camera_poses;

    // Set up input vars
    int ith_frame = 0;
    for (int ith_frame_in_buff = TOTAL_FRAMES - 1; ith_frame_in_buff >= TOTAL_FRAMES - NUM_FRAMES_FOR_BA;
         ith_frame_in_buff--, ith_frame++)
    {
        Frame::Ptr frame = frames_buff_[ith_frame_in_buff];
        int num_mappt_in_frame = frame->inliers_to_mappt_connections_.size();
        if (num_mappt_in_frame < 3)
        {
            continue; // Too few mappoints. Not optimizing this frame
        }
        printf("Frame id: %d, num map points = %d\n", frame->id_, num_mappt_in_frame);
        v_pts_2d.push_back(vector<cv::Point2f *>());
        v_pts_2d_to_3d_idx.push_back(vector<int>());

        // Get camera poses
        v_camera_poses.push_back(&frame->T_w_c_);

        // Iterate through this camera's mappoints
        for (unordered_map<int, PtConn>::iterator ite = frame->inliers_to_mappt_connections_.begin();
             ite != frame->inliers_to_mappt_connections_.end(); ite++)
        {
            int kpt_idx = ite->first;
            int mappt_idx = ite->second.pt_map_idx;
            if (map_->map_points_.find(mappt_idx) == map_->map_points_.end())
                continue; // point has been deleted

            // Get 2d pos
            v_pts_2d[ith_frame].push_back(&(frame->keypoints_[kpt_idx].pt));
            v_pts_2d_to_3d_idx[ith_frame].push_back(mappt_idx);

            // Get 3d pos
            cv::Point3f *p = &(map_->map_points_[mappt_idx]->pos_);
            um_pts_3d[mappt_idx] = p;
            if (ith_frame == 0)
                v_pts_3d_only_in_curr.push_back(p);
        }
    }
    // Bundle Adjustment
    cv::Mat pose_src = getPosFromT(curr_->T_w_c_);
    if (1)
    {
        optimization::bundleAdjustment(
            v_pts_2d, v_pts_2d_to_3d_idx, curr_->camera_->K_,
            um_pts_3d, v_camera_poses,
            information_matrix,
            FIX_MAP_PTS, UPDATE_MAP_PTS);
    }
    else
    {
        optimization::optimizeSingleFrame(
            v_pts_2d[0], curr_->camera_->K_,
            v_pts_3d_only_in_curr, curr_->T_w_c_,
            FIX_MAP_PTS, UPDATE_MAP_PTS); // Update pts_3d and curr_->T_w_c_
    }

    // Print result
    cv::Mat pose_new = getPosFromT(curr_->T_w_c_);
    printf("Cam pos: Before:{%.5f,%.5f,%.5f}, After:{%.5f,%.5f,%.5f}\n",
           pose_src.at<double>(0, 0), pose_src.at<double>(1, 0), pose_src.at<double>(2, 0),
           pose_new.at<double>(0, 0), pose_new.at<double>(1, 0), pose_new.at<double>(2, 0));
    printf("Bundle adjustment finishes... \n\n");
}
// ------------------- Mapping -------------------
void VisualOdometry::addKeyFrame(Frame::Ptr frame)
{
    map_->insertKeyFrame(frame);
    ref_ = frame;
}

void VisualOdometry::optimizeMap()
{
    static const double default_erase = 0.1;
    static double map_point_erase_ratio = default_erase;

    // remove the hardly seen and no visible points
    for (auto iter = map_->map_points_.begin(); iter != map_->map_points_.end();)
    {
        if (!curr_->isInFrame(iter->second->pos_))
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }

        float match_ratio = float(iter->second->matched_times_) / iter->second->visible_times_;
        if (match_ratio < map_point_erase_ratio)
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }

        double angle = getViewAngle(curr_, iter->second);
        if (angle > M_PI / 6.)
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }
        iter++;
    }

    if (map_->map_points_.size() > 1000)
    {
        // TODO map is too large, remove some one
        map_point_erase_ratio += 0.05;
    }
    else
        map_point_erase_ratio = default_erase;
    cout << "map points: " << map_->map_points_.size() << endl;
}

void VisualOdometry::pushCurrPointsToMap()
{
    // -- Input
    const vector<cv::Point3f> &inliers_pts3d_in_curr = curr_->inliers_pts3d_;
    const cv::Mat &T_w_curr = curr_->T_w_c_;
    const cv::Mat &descriptors = curr_->descriptors_;
    const vector<vector<unsigned char>> &kpts_colors = curr_->kpts_colors_;
    const vector<cv::DMatch> &inliers_matches_for_3d = curr_->inliers_matches_for_3d_;

    // -- Output
    unordered_map<int, PtConn> &inliers_to_mappt_connections = curr_->inliers_to_mappt_connections_;

    // -- Start
    for (int i = 0; i < inliers_matches_for_3d.size(); i++)
    {
        const cv::DMatch &dm = inliers_matches_for_3d[i];
        int pt_idx = dm.trainIdx;
        int map_point_id;

        // Points already triangulated in previous frames.
        //      Just find the mappoint, no need to create new.
        if (1 && ref_->isMappoint(dm.queryIdx))
        {
            map_point_id = ref_->inliers_to_mappt_connections_[dm.queryIdx].pt_map_idx;
        }
        else // Not triangulated before. Create and push to map.
        {

            // Change coordinate of 3d points to world frame
            cv::Point3f world_pos = preTranslatePoint3f(inliers_pts3d_in_curr[i], T_w_curr);

            // Create map point
            MapPoint::Ptr map_point(new MapPoint( // createMapPoint
                world_pos,
                descriptors.row(pt_idx).clone(),                                       // descriptor
                getNormalizedMat(point3f_to_mat(world_pos) - curr_->getCamCenter()),   // view direction of the point
                kpts_colors[pt_idx][0], kpts_colors[pt_idx][1], kpts_colors[pt_idx][2] // rgb color
                ));
            map_point_id = map_point->id_;
            // cout<<map_point->id_ <<", "<< map_point->factory_id_<<endl;

            // Push to map
            map_->insertMapPoint(map_point);
        }
        // Update graph connection of current frame
        inliers_to_mappt_connections.insert({pt_idx, PtConn{dm.queryIdx, map_point_id}});
    }
    return;
}

double VisualOdometry::getViewAngle(Frame::Ptr frame, MapPoint::Ptr point)
{
    cv::Mat n = point3f_to_mat(point->pos_) - frame->getCamCenter();
    n = getNormalizedMat(n);
    cv::Mat vector_dot_product = n.t() * point->norm_;
    return acos(vector_dot_product.at<double>(0, 0));
}

} // namespace vo
} // namespace my_slam