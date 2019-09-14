

#include "my_slam/common_include.h"
#include "my_slam/vo/mappoint.h"

namespace my_slam
{
namespace vo
{

int MapPoint::factory_id_ = 0;
MapPoint::MapPoint(
    const cv::Point3f &pos, const cv::Mat &descriptor, const cv::Mat &norm,
    unsigned char r, unsigned char g, unsigned char b) : pos_(pos), descriptor_(descriptor), norm_(norm), color_({r, g, b}),
                                                         good_(true), visible_times_(1), matched_times_(1)

{
    id_ = factory_id_++;
}

void MapPoint::setPos(const cv::Point3f &pos)
{
    pos_ = pos;
}

} // namespace vo
} // namespace my_slam
