
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
    return mean_dist > 40;
}

vector<Mat> VisualOdometry::pushPointsToMap(
    const vector<Point3f> &inliers_pts3d_in_curr,
    const Mat &T_w_curr,
    const Mat &descriptors, const vector<vector<unsigned char>> &kpts_colors,
    const vector<DMatch> &inlier_matches)
{
    vector<Mat> newly_inserted_pts3d;

    for (int i = 0; i < inlier_matches.size(); i++)
    {
        newly_inserted_pts3d.push_back(
            Point3f_to_Mat(preTranslatePoint3f(inliers_pts3d_in_curr[i], T_w_curr)));
    }

    for (int i = 0; i < inlier_matches.size(); i++)
    {
        int pt_idx = inlier_matches[i].trainIdx;
        const vector<unsigned char> &rgb = kpts_colors[pt_idx];

        MapPoint::Ptr map_point(new MapPoint(
            newly_inserted_pts3d[i],
            descriptors.row(pt_idx).clone(),
            rgb[0], rgb[1], rgb[2]));
        map_->insertMapPoint(map_point);
    }
    return newly_inserted_pts3d;
}

void VisualOdometry::estimateMotionAnd3DPoints(Mat &R, Mat &t, 
    vector<DMatch> &inlier_matches, vector<Point3f> &pts3d_in_curr)
{

    // -- Estimation motion by Essential && Homography matrix and get inlier points
    vector<Mat> list_R, list_t, list_normal;
    vector<vector<DMatch>> list_matches; // these are the inliers matches
    vector<vector<Point3f>> sols_pts3d_in_cam1_by_triang;
    const bool print_res = false, is_frame_cam2_to_cam1 = true;
    const bool compute_homography = true;
    Mat &K=curr_->camera_->K_;
    helperEstimatePossibleRelativePosesByEpipolarGeometry(
        /*Input*/
        ref_->keypoints_, curr_->keypoints_, curr_->matches_, K,
        /*Output*/
        list_R, list_t, list_matches, list_normal, sols_pts3d_in_cam1_by_triang,
        /*settings*/
        print_res, compute_homography, is_frame_cam2_to_cam1);

    // -- Compute errors of results of E/H estimation and choose the best one
    // Three things to compute: [epipolar error] and [trigulation error on norm plane] and [score]
    //      for the 3 solutions of (E, H1, H2).
    int idx_best_solution = helperEvalErrorsAndChooseEH(
        ref_->keypoints_, curr_->keypoints_, list_matches,
        sols_pts3d_in_cam1_by_triang, list_R, list_t, list_normal, K,
        false); // print result

    // -- Choose 1 solution from the 3 solutions.
    //      Results: R, t, inlier_matches, pts_3d in cam1 and cam2
    R = list_R[idx_best_solution];
    t = list_t[idx_best_solution];
    inlier_matches = list_matches[idx_best_solution];
    const vector<Point3f> &pts3d_in_cam1 = sols_pts3d_in_cam1_by_triang[idx_best_solution];
    pts3d_in_curr.clear();
    for (const Point3f &p1 : pts3d_in_cam1)
        pts3d_in_curr.push_back(transCoord(p1, R, t));
}

} // namespace my_slam