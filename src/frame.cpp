#include "myslam/frame.h"

namespace myslam
{

unsigned long Frame::factory_id_=0;

Frame::Ptr Frame::createFrame(Mat rgb_img, Camera::Ptr camera, double time_stamp)
{
    Frame::Ptr frame( new Frame() );
    frame -> rgb_img_ = rgb_img;
    frame -> id_=factory_id_++;
    frame -> time_stamp_ = time_stamp;
    frame -> camera_ = camera;
    return frame;
}

bool Frame::isInFrame ( const Vector3d& pt_world )
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    // cout<<"P_cam = "<<p_cam.transpose()<<endl;
    if ( p_cam(2,0)<0 ) return false;
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<rgb_img_.cols 
        && pixel(1,0)<rgb_img_.rows;
}

// double Frame::findDepth ( const cv::KeyPoint& kp )
// {
//     int x = cvRound(kp.pt.x);
//     int y = cvRound(kp.pt.y);
//     ushort d = depth_img_.ptr<ushort>(y)[x];
//     if ( d!=0 )
//     {
//         return double(d)/camera_->depth_img_scale_;
//     }
//     else 
//     {
//         // check the nearby points 
//         int dx[4] = {-1,0,1,0};
//         int dy[4] = {0,-1,0,1};
//         for ( int i=0; i<4; i++ )
//         {
//             d = depth_img_.ptr<ushort>( y+dy[i] )[x+dx[i]];
//             if ( d!=0 )
//             {
//                 return double(d)/camera_->depth_img_scale_;
//             }
//         }
//     }
//     return -1.0;
// }


}
