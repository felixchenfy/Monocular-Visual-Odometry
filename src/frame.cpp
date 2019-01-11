
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

// bool Frame::isInFrame ( const Vector3d& pt_world )
// {
//     Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
//     // cout<<"P_cam = "<<p_cam.transpose()<<endl;
//     if ( p_cam(2,0)<0 ) return false;
//     Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
//     // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
//     return pixel(0,0)>0 && pixel(1,0)>0 
//         && pixel(0,0)<color_.cols 
//         && pixel(1,0)<color_.rows;
// }

}
