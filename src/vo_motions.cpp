

#include "my_slam/vo.h"

namespace my_slam{


void VisualOdometry::matchFeatures()
{

}

void VisualOdometry::getMappointsInCurrentView(
    vector<MapPoint::Ptr> &candidate_mappoints_in_map,
    Mat &candidate_descriptors_in_map
)
{
    // vector<MapPoint::Ptr> candidate_mappoints_in_map;
    // Mat candidate_descriptors_in_map;
    candidate_mappoints_in_map.clear();
    candidate_descriptors_in_map.release();
    for ( auto& iter_map_point: map_->map_points_ )
    {
        MapPoint::Ptr& p = iter_map_point.second;
        // check if p in curr frame image 
        if ( 1 )
        // if ( curr_->isInFrame(p->pos_) )
        {
            // -- add to candidate_mappoints_in_map 
            // p->visible_times_++;
            candidate_mappoints_in_map.push_back( p );
            candidate_descriptors_in_map.push_back( p->descriptor_ );
        }
    }
}

}