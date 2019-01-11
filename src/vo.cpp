
#include "my_slam/vo.h"

namespace my_slam
{

VisualOdometry::VisualOdometry():map_(new(Map))
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
    return mean_dist > 50;
}

void VisualOdometry::pushPointsToMap(
    const vector<Point3f> &inliers_pts3d_in_curr,
    const Mat &T_w_curr,
    const Mat &descriptors,
    const vector<DMatch> &inlier_matches)
{

    for (int i = 0; i < inlier_matches.size(); i++)
    {
        int pt_idx = inlier_matches[i].trainIdx;
        MapPoint::Ptr map_point(new MapPoint(
            Point3f_to_Mat(preTranslatePoint3f(inliers_pts3d_in_curr[i], T_w_curr)),
            descriptors.row(pt_idx).clone()));
        map_->insertMapPoint(map_point);
    }
}

} // namespace my_slam