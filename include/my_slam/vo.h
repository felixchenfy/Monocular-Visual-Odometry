

#ifndef VO_H
#define VO_H

// cv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// my

#include "my_basics/io.h"
#include "my_basics/config.h"

#include "my_geometry/camera.h"
#include "my_geometry/feature_match.h"
#include "my_geometry/motion_estimation.h"

#include "my_slam/common_include.h"
#include "my_slam/frame.h"

using namespace std;
using namespace cv;
using namespace my_geometry;

namespace my_slam
{

class VisualOdometry
{

  public: // Member variables
    typedef shared_ptr<VisualOdometry> Ptr;

    enum VOState
    {
        BLANK,
        INITIALIZATION,
        OK,
        LOST
    };
    VOState vo_state_;
    deque<my_slam::Frame::Ptr> frames_; // store the previous frames
    
    bool DEBUG_STOP_PROGRAM_;

  public: // Constructor
    VisualOdometry();
    void addFrame(my_slam::Frame::Ptr frame);

  public: // Functions
};

} // namespace my_slam

#endif // FRAME_H
