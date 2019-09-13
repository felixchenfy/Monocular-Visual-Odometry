

#include "my_slam/vo/common_include.h"
#include "my_slam/vo/mappoint.h"

namespace vo
{

int MapPoint::factory_id_ = 0;
MapPoint::MapPoint(
    const Point3f &pos, const Mat &descriptor, const Mat &norm,
    unsigned char r, unsigned char g, unsigned char b) : pos_(pos), descriptor_(descriptor), norm_(norm), color_({r, g, b}),
                                                         good_(true), visible_times_(1), matched_times_(1)

{
    id_ = factory_id_++;
}

// void MapPoint::resetPos(Point3f pos)
// {
//     pos_ = (Mat_<double>(3,1) << pos.x, pos.y, pos.z);
// }

} // namespace vo
