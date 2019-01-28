
#include "my_slam/vo.h"

namespace my_slam
{

VisualOdometry::VisualOdometry() : map_(new (Map))
{
    vo_state_ = BLANK;
    keyframes_.clear();

    // debug
    DEBUG_STOP_PROGRAM_ = false;
}

void VisualOdometry::getMappointsInCurrentView(
    vector<MapPoint::Ptr> &candidate_mappoints_in_map,
    Mat &candidate_descriptors_in_map)
{
    // vector<MapPoint::Ptr> candidate_mappoints_in_map;
    // Mat candidate_descriptors_in_map;
    candidate_mappoints_in_map.clear();
    candidate_descriptors_in_map.release();
    for (auto &iter_map_point : map_->map_points_)
    {
        MapPoint::Ptr &p = iter_map_point.second;
        // check if p in curr frame image
        if (1)
        // if ( curr_->isInFrame(p->pos_) )
        {
            // -- add to candidate_mappoints_in_map
            // p->visible_times_++;
            candidate_mappoints_in_map.push_back(p);
            candidate_descriptors_in_map.push_back(p->descriptor_);
        }
    }
}

bool VisualOdometry::checkIfVoGoodToInit(
    const vector<KeyPoint> &init_kpts, const vector<KeyPoint> &curr_kpts, const vector<DMatch> &matches)
{

    vector<double> dists_between_kpts;
    double mean_dist = computeMeanDistBetweenKeypoints(init_kpts, curr_kpts, matches);
    return mean_dist > 70;
}

vector<Mat> VisualOdometry::pushCurrPointsToMap()
{
    // -- Input
    const vector<Point3f> &inliers_pts3d_in_curr = curr_->inliers_pts3d_;
    const Mat &T_w_curr = curr_->T_w_c_;
    const Mat &descriptors = curr_->descriptors_;
    const vector<vector<unsigned char>> &kpts_colors= curr_->kpts_colors_;
    const vector<DMatch> &inlier_matches = curr_->inlier_matches_;

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
        Mat norm =  p_world - curr_->getCamCenter();
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

void VisualOdometry::estimateMotionAnd3DPoints(Mat &R, Mat &t, 
    vector<DMatch> &inlier_matches, vector<Point3f> &pts3d_in_curr)
{

    // Estimation motion by Essential && Homography matrix and get inlier points
    vector<Mat> list_R, list_t, list_normal;
    vector<vector<DMatch>> list_matches; // these are the inliers matches
    vector<vector<Point3f>> sols_pts3d_in_cam1_by_triang;
    const bool print_res = false, is_frame_cam2_to_cam1 = true;
    const bool compute_homography = true;
    Mat &K=curr_->camera_->K_;
    int best_sol = helperEstimatePossibleRelativePosesByEpipolarGeometry(
        /*Input*/
        ref_->keypoints_, curr_->keypoints_, curr_->matches_, K,
        /*Output*/
        list_R, list_t, list_matches, list_normal, sols_pts3d_in_cam1_by_triang,
        /*settings*/
        print_res, compute_homography, is_frame_cam2_to_cam1);

    // Get the data of the best solution
    R = list_R[best_sol];
    t = list_t[best_sol];
    inlier_matches = list_matches[best_sol];
    const vector<Point3f> &pts3d_in_cam1 = sols_pts3d_in_cam1_by_triang[best_sol];
    pts3d_in_curr.clear();
    for (const Point3f &p1 : pts3d_in_cam1)
        pts3d_in_curr.push_back(transCoord(p1, R, t));
}


// ------------------- Tracking -------------------
bool VisualOdometry::checkInsertingKeyframe(Frame::Ptr curr, Frame::Ptr ref){
    Mat T_key_to_curr = ref->T_w_c_.inv() * curr->T_w_c_;
    Mat R, t, R_vec;
    getRtFromT(T_key_to_curr, R, t);
    cv::Rodrigues(R, R_vec);

    static double MIN_DIST_BETWEEN_KEYFRAME = my_basics::Config::get<double>("MIN_DIST_BETWEEN_KEYFRAME");
    static double MIN_ROTATED_ANGLE = my_basics::Config::get<double>("MIN_ROTATED_ANGLE");

    double moved_dist = calcMatNorm(t);
    double rotated_angle =  calcMatNorm(R_vec);

    printf("Movint dist = %.5f; Rotated angle = %.5f\n", moved_dist, rotated_angle);

    // Satisfy each one will be a good keyframe
    bool res = moved_dist > MIN_DIST_BETWEEN_KEYFRAME || rotated_angle > MIN_ROTATED_ANGLE;

    return res;
}

// ------------------- Mapping -------------------
void VisualOdometry::optimizeMap()
{
    static const double default_erase = 0.1;
    static double map_point_erase_ratio = default_erase;

    // remove the hardly seen and no visible points 
    for ( auto iter = map_->map_points_.begin(); iter != map_->map_points_.end(); )
    {
        if ( !curr_->isInFrame(iter->second->pos_) )
        {
            iter = map_->map_points_.erase(iter);
            continue;
        }

        // float match_ratio = float(iter->second->matched_times_)/iter->second->visible_times_;
        // if ( match_ratio < map_point_erase_ratio )
        // {
        //     iter = map_->map_points_.erase(iter);
        //     continue;
        // }
        
        // double angle = getViewAngle( curr_, iter->second );
        // if ( angle > M_PI/6. )
        // {
        //     iter = map_->map_points_.erase(iter);
        //     continue;
        // }
        // if ( iter->second->good_ == false )
        // {
        //     // TODO try triangulate this map point 
        // }
        iter++;
    }
    
    if ( map_->map_points_.size() > 1000 )  
    {
        // TODO map is too large, remove some one 
        map_point_erase_ratio += 0.05;
    }
    else 
        map_point_erase_ratio = default_erase;
    cout<<"map points: "<<map_->map_points_.size()<<endl;
}



} // namespace my_slam