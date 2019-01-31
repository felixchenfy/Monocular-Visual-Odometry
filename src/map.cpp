
#include "my_slam/map.h"

namespace my_slam
{

void Map::insertKeyFrame(Frame::Ptr frame)
{
    if (keyframes_.find(frame->id_) == keyframes_.end())
    {
        keyframes_.insert(make_pair(frame->id_, frame));
    }
    else
    {
        keyframes_[frame->id_] = frame;
    }
    printf("Insert keyframe!!! frame_id = %ld, total keyframes = %d\n", frame->id_, (int)keyframes_.size());
}

void Map::insertMapPoint(MapPoint::Ptr map_point)
{
    if (map_points_.find(map_point->id_) == map_points_.end())
    {
        map_points_.insert(make_pair(map_point->id_, map_point));
    }
    else
    {
        map_points_[map_point->id_] = map_point;
    }
}

Frame::Ptr Map::findKeyFrame(int frame_id)
{
    if (keyframes_.find(frame_id) == keyframes_.end())
        return NULL;
    else
        return keyframes_[frame_id];
}

bool Map::checkKeyFrame(int frame_id)
{
    if (keyframes_.find(frame_id) == keyframes_.end())
        return false;
    else
        return true;
}

} // namespace my_slam
