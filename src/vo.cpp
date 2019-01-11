
#include "my_slam/vo.h"

namespace my_slam
{

VisualOdometry::VisualOdometry()
{
    vo_state_ = BLANK;
    frames_.clear();

    // debug
    DEBUG_STOP_PROGRAM_ = false;
}

bool VisualOdometry::checkIfVoGoodToInit(
    const vector<KeyPoint> &init_kpts, const vector<KeyPoint> &curr_kpts, const vector<DMatch> &matches)
{

    vector<double> dists_between_kpts;
    double mean_dist = computeMeanDistBetweenKeypoints(init_kpts, curr_kpts, matches);
    return mean_dist > 50;
}

} // namespace my_slam