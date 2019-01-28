

#include "my_slam/common_include.h"
#include "my_slam/mappoint.h"

namespace my_slam
{

unsigned long MapPoint::factory_id_ = 0;
MapPoint::MapPoint(const Mat &pos, const Mat &descriptor,
                   unsigned char r, unsigned char g, unsigned char b) : pos_(pos), descriptor_(descriptor),
                    color_({r, g, b})
{
    id_ = factory_id_++;
}

// MapPoint::MapPoint()
// : id_(-1), pos_(Vector3d(0,0,0)), norm_(Vector3d(0,0,0)), good_(true), visible_times_(0), matched_times_(0)
// {

// }

// MapPoint::MapPoint ( long unsigned int id, const Vector3d& position, const Vector3d& norm, Frame* frame, const Mat& descriptor )
// : id_(id), pos_(position), norm_(norm), good_(true), visible_times_(1), matched_times_(1), descriptor_(descriptor)
// {
//     observed_frames_.push_back(frame);
// }

// MapPoint::Ptr MapPoint::createMapPoint()
// {
//     return MapPoint::Ptr(
//         new MapPoint( factory_id_++, Vector3d(0,0,0), Vector3d(0,0,0) )
//     );
// }

// MapPoint::Ptr MapPoint::createMapPoint (
//     const Vector3d& pos_world,
//     const Vector3d& norm,
//     const Mat& descriptor,
//     Frame* frame )
// {
//     return MapPoint::Ptr(
//         new MapPoint( factory_id_++, pos_world, norm, frame, descriptor )
//     );
// }

// unsigned long MapPoint::factory_id_ = 0;

} // namespace my_slam
