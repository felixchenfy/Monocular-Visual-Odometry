

#include "my_slam/common_include.h"
#include "my_slam/mappoint.h"

namespace my_slam
{

unsigned long MapPoint::factory_id_ = 0;
MapPoint::MapPoint(const Mat &pos, const Mat &descriptor, const Mat &norm,
                   unsigned char r, unsigned char g, unsigned char b) : pos_(pos), descriptor_(descriptor), norm_(norm), color_({r, g, b}),
                                                                        good_(true), visible_times_(1), matched_times_(1)

{
    id_ = factory_id_++;
}

} // namespace my_slam
