

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

} // namespace my_slam
