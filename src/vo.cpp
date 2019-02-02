
#include "my_slam/vo.h"
#include "my_optimization/g2o_ba.h"

namespace my_slam
{

VisualOdometry::VisualOdometry() : map_(new (Map))
{
    vo_state_ = BLANK;
}

void VisualOdometry::getMappointsInCurrentView(
    vector<MapPoint::Ptr> &candidate_mappoints_in_map,
    Mat &corresponding_mappoints_descriptors)
{
    // vector<MapPoint::Ptr> candidate_mappoints_in_map;
    // Mat corresponding_mappoints_descriptors;
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
    vector<DMatch> &inlier_matches = curr_->inliers_matches_with_ref_;
    vector<Point3f> &pts3d_in_curr = curr_->inliers_pts3d_;
    Mat &T = curr_->T_w_c_;

    // -- Start: call this big function to compute everything
    // (1) motion from Essential && Homography, (2) inliers indices, (3) triangulated points
    vector<Mat> list_R, list_t, list_normal;
    vector<vector<DMatch>> list_matches; // these are the inliers matches
    vector<vector<Point3f>> sols_pts3d_in_cam1_by_triang;
    const bool print_res = false, is_frame_cam2_to_cam1 = true;
    const bool compute_homography = true;
    Mat &K = curr_->camera_->K_;
    int best_sol = helperEstimatePossibleRelativePosesByEpipolarGeometry(
        /*Input*/
        ref_->keypoints_, curr_->keypoints_, curr_->matches_with_ref_, K,
        /*Output*/
        list_R, list_t, list_matches, list_normal, sols_pts3d_in_cam1_by_triang,
        /*settings*/
        print_res, compute_homography, is_frame_cam2_to_cam1);

    // -- Only retain the data of the best solution
    const Mat &R_curr_to_prev = list_R[best_sol];
    const Mat &t_curr_to_prev = list_t[best_sol];
    inlier_matches = list_matches[best_sol];
    const vector<Point3f> &pts3d_in_cam1 = sols_pts3d_in_cam1_by_triang[best_sol];
    pts3d_in_curr.clear();
    for (const Point3f &p1 : pts3d_in_cam1)
        pts3d_in_curr.push_back(transCoord(p1, R_curr_to_prev, t_curr_to_prev));

    // -- Normalize Points Depth to 1, and compute camera pose
    const int num_inlier_pts = curr_->inliers_pts3d_.size();
    double mean_depth = calcMeanDepth(curr_->inliers_pts3d_);
    t_curr_to_prev /= mean_depth;
    for (Point3f &p : curr_->inliers_pts3d_)
        scalePointPos(p, 1 / mean_depth);
    T = ref_->T_w_c_ * transRt2T(R_curr_to_prev, t_curr_to_prev).inv();
}


bool VisualOdometry::checkIfVoGoodToInit(int checkIfVoGoodToInit)
{

    // -- Rename input
    const vector<KeyPoint> &init_kpts = ref_->keypoints_;
    const vector<KeyPoint> &curr_kpts = curr_->keypoints_;
    const vector<DMatch> &matches = curr_->inliers_matches_with_ref_;

    // -- Start
    if(checkIfVoGoodToInit==1){
        // CRITERIA 1: init vo only when distance between matched keypoints are large
        vector<double> dists_between_kpts;
        double mean_dist = computeMeanDistBetweenKeypoints(init_kpts, curr_kpts, matches);
        printf("\nPixel movement of matched keypoints: %.1f \n", mean_dist);
        static const double MIN_PIXEL_DIST_FOR_INIT_VO = my_basics::Config::get<double>("MIN_PIXEL_DIST_FOR_INIT_VO");
        return mean_dist > MIN_PIXEL_DIST_FOR_INIT_VO;
    }else
    {
        assert(0);
    }
    
}

bool VisualOdometry::isInitialized(){
    return vo_state_==OK;
}
// ------------------- Tracking -------------------
bool VisualOdometry::checkLargeMoveForAddKeyFrame(Frame::Ptr curr, Frame::Ptr ref)
{
    Mat T_key_to_curr = ref->T_w_c_.inv() * curr->T_w_c_;
    Mat R, t, R_vec;
    getRtFromT(T_key_to_curr, R, t);
    cv::Rodrigues(R, R_vec);

    static const double MIN_DIST_BETWEEN_KEYFRAME = my_basics::Config::get<double>("MIN_DIST_BETWEEN_KEYFRAME");
    static const double MIN_ROTATED_ANGLE = my_basics::Config::get<double>("MIN_ROTATED_ANGLE");

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
    Mat corresponding_mappoints_descriptors;
    getMappointsInCurrentView(candidate_mappoints_in_map, corresponding_mappoints_descriptors);

    // -- Compare descriptors to find matches, and extract 3d 2d correspondance
    my_geometry::matchFeatures(corresponding_mappoints_descriptors, curr_->descriptors_, curr_->matches_with_map_);
    cout << "Number of 3d-2d pairs: " << curr_->matches_with_map_.size() << endl;
    vector<Point3f> pts_3d, tmp_pts_3d;
    vector<Point2f> pts_2d, tmp_pts_2d; // a point's 2d pos in image2 pixel curr_
    for (int i = 0; i < curr_->matches_with_map_.size(); i++)
    {
        DMatch &match = curr_->matches_with_map_[i];
        MapPoint::Ptr mappoint = candidate_mappoints_in_map[match.queryIdx];
        pts_3d.push_back(Mat_to_Point3f(mappoint->pos_));
        pts_2d.push_back(curr_->keypoints_[match.trainIdx].pt);
    }

    // -- Solve PnP, get T_cam1_to_cam2
    Mat R_vec, R, t;
    bool useExtrinsicGuess = false;
    int iterationsCount = 100;
    float reprojectionError = 5.0;
    double confidence = 0.999;
    Mat pnp_inliers_mask; // type = 32SC1, size = 999x1
    solvePnPRansac(pts_3d, pts_2d, curr_->camera_->K_, Mat(), R_vec, t,
                   useExtrinsicGuess, iterationsCount, reprojectionError, confidence, pnp_inliers_mask);
    Rodrigues(R_vec, R);

    // -- Get inlier matches used in PnP
    vector<MapPoint::Ptr> inlier_candidates;
    vector<DMatch> tmp_matches_with_map_;
    int num_inliers = pnp_inliers_mask.rows;
    for (int i = 0; i < num_inliers; i++)
    {
        int good_idx = pnp_inliers_mask.at<int>(i, 0);

        // good pts 3d && 2d
        tmp_pts_3d.push_back(pts_3d[good_idx]);
        tmp_pts_2d.push_back(pts_2d[good_idx]);

        // good match
        DMatch &match = curr_->matches_with_map_[good_idx];
        tmp_matches_with_map_.push_back(match);

        // map point
        MapPoint::Ptr inlier_mappoint = candidate_mappoints_in_map[match.queryIdx];
        inlier_candidates.push_back(inlier_mappoint);
        inlier_mappoint->matched_times_++;
    }
    pts_3d.swap(tmp_pts_3d);
    pts_2d.swap(tmp_pts_2d);
    curr_->matches_with_map_.swap(tmp_matches_with_map_);
    
    // -- Update current camera pos
    curr_->T_w_c_ = transRt2T(R, t).inv();

    // -- Bundle Adjustment
    static const int USE_BA = my_basics::Config::get<int>("USE_BA");
    if (USE_BA==1)
    {
        // Update curr_->T_w_c_
        my_optimization::bundleAdjustment(pts_3d, pts_2d, curr_->camera_->K_, curr_->T_w_c_);

        // Update points' 3d pos
        for (int i = 0; i < num_inliers; i++)
            inlier_candidates[i]->resetPos(pts_3d[i]);
    }
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
        // if ( iter->second->good_ == false )
        // {
        //     // TODO try triangulate this map point
        // }
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

vector<Mat> VisualOdometry::pushCurrPointsToMap()
{
    // -- Input
    const vector<Point3f> &inliers_pts3d_in_curr = curr_->inliers_pts3d_;
    const Mat &T_w_curr = curr_->T_w_c_;
    const Mat &descriptors = curr_->descriptors_;
    const vector<vector<unsigned char>> &kpts_colors = curr_->kpts_colors_;
    const vector<DMatch> &inlier_matches = curr_->inliers_matches_with_ref_;

    // -- Output
    vector<Mat> newly_inserted_pts3d;

    // -- Start
    for (int i = 0; i < inlier_matches.size(); i++)
    {
        newly_inserted_pts3d.push_back(
            Point3f_to_Mat(preTranslatePoint3f(inliers_pts3d_in_curr[i], T_w_curr)));
    }

    for (int i = 0; i < inlier_matches.size(); i++)
    {
        int pt_idx = inlier_matches[i].trainIdx;
        const vector<unsigned char> &rgb = kpts_colors[pt_idx];

        const Mat &p_world = newly_inserted_pts3d[i];
        Mat norm = p_world - curr_->getCamCenter();
        my_basics::normalize(norm);

        MapPoint::Ptr map_point(new MapPoint( // createMapPoint
            p_world,
            descriptors.row(pt_idx).clone(),
            norm,
            rgb[0], rgb[1], rgb[2]));
        map_->insertMapPoint(map_point);
    }
    return newly_inserted_pts3d;
}

double VisualOdometry::getViewAngle(Frame::Ptr frame, MapPoint::Ptr point)
{
    Mat n = point->pos_ - frame->getCamCenter();
    my_basics::normalize(n);
    Mat vector_dot_product = n.t() * point->norm_;
    return acos(vector_dot_product.at<double>(0, 0));
}

} // namespace my_slam