
#ifndef FRAME_H
#define FRAME_H

#include "my_slam/common_include.h"
#include "my_slam/camera.h"

namespace my_slam
{

class MapPoint; // forward declare
class Frame
{
  private:
  public:
    typedef std::shared_ptr<Frame> Ptr;
    static unsigned long factory_id_;

    unsigned long id_;  // id of this frame
    double time_stamp_; // when it is recorded

    Mat rgb_img_;
    // Mat depth_img_;

    Camera::Ptr camera_; // stores the camera model here for easier computing pixel pos
    SE3 T_c_w_;  // transform from camera to world

    // bool is_key_frame_; // whether a key-frame
    // std::vector<cv::KeyPoint>      keypoints_;  // key points in image
    // std::vector<MapPoint*>         map_points_; // associated map points

  public: // Constructor
    // Frame(Mat rgb_img, Camera::Ptr camera, double time_stamp = -1)
    // {
    //     rgb_img_ = rgb_img;
    //     id_=factory_id_++;
    //     time_stamp_ = time_stamp;
    //     camera_ = camera;
    // }
    Frame(){}
    ~Frame() {}
    static Frame::Ptr createFrame(Mat rgb_img, Camera::Ptr camera, double time_stamp = -1);

  public: // Member functions
    Vector3d getCamCenter() const { return T_c_w_.inverse().translation(); }
    void setPose(const SE3 &T_c_w) { T_c_w_ = T_c_w; }

    // check if the project of a point is inside this frame
    bool isInFrame(const Vector3d &pt_world);

    // (This is for rgb_img_-d camera)
    // Find the depth of a keypoint in depth map
    // double findDepth(const cv::KeyPoint &kp);
};

} // namespace my_slam

#endif // FRAME_H
