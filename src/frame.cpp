
#include "my_slam/vo/frame.h"

namespace vo
{

int Frame::factory_id_ = 0;

Frame::Ptr Frame::createFrame(Mat rgb_img, geometry::Camera::Ptr camera, double time_stamp)
{
    Frame::Ptr frame(new Frame());
    frame->rgb_img_ = rgb_img;
    frame->id_ = factory_id_++;
    frame->time_stamp_ = time_stamp;
    frame->camera_ = camera;
    return frame;
}


bool Frame::isInFrame(const Point3f &p_world)
{
    Point3f p_cam = preTranslatePoint3f(p_world, T_w_c_.inv()); // T_c_w * p_w = p_c
    if (p_cam.z < 0)
        return false;
    Point2f pixel = cam2pixel(p_cam, camera_->K_);
    return pixel.x > 0 && pixel.y > 0 && pixel.x < rgb_img_.cols && pixel.y < rgb_img_.rows;
}
bool Frame::isInFrame ( const Mat& p_world){
    return isInFrame(Mat_to_Point3f(p_world));   
}
Mat Frame::getCamCenter(){
    return getPosFromT(T_w_c_);
}

} // namespace vo
