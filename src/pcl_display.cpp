
#include "my_display/pcl_display.h"



using namespace std;
using namespace cv;
using namespace Eigen;

namespace my_display{

void setPointColor(pcl::PointXYZRGB &point, uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    point.rgb = *reinterpret_cast<float *>(&rgb);
}

void setPointPos(pcl::PointXYZRGB &point, float x, float y, float z){
    point.x=x;
    point.y=y;
    point.z=z;
}
void setPointPos(pcl::PointXYZRGB &point, double x, double y, double z){
   setPointPos(point, (float)x, float(y), float(z));
}
void setPointPos(pcl::PointXYZRGB &point, cv::Mat p)
{
    setPointPos(point, p.at<double>(0,0), p.at<double>(1,0), p.at<double>(2,0));
}
}
