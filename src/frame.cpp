
#include "my_slam/frame.h"

namespace my_slam
{

unsigned long Frame::factory_id_=0;

Frame::Ptr Frame::createFrame(Mat rgb_img, my_geometry::Camera::Ptr camera, double time_stamp)
{
    Frame::Ptr frame( new Frame() );
    frame -> rgb_img_ = rgb_img;
    frame -> id_=factory_id_++;
    frame -> time_stamp_ = time_stamp;
    frame -> camera_ = camera;
    return frame;
}

}
