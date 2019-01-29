
#include "my_slam/vo.h"

namespace my_slam
{

void VisualOdometry::addFrame(Frame::Ptr frame)
{
    // Settings
    const int FRAME_FOR_FIRST_ESSENTIAL = 14;

    // Renamed vars
    curr_ = frame;
    const int img_id = curr_->id_;
    const Mat &K = curr_->camera_->K_;

    // Start
    printf("\n\n=============================================\n");
    printf("Start processing the %dth image.\n", img_id);

    curr_->extractKeyPoints();
    curr_->computeDescriptors();
    cout << "Number of keypoints: " << curr_->keypoints_.size() << endl;

    // vo_state_: BLANK -> INITIALIZATION
    if (vo_state_ == BLANK)
    {
        curr_->T_w_c_ = Mat::eye(4, 4, CV_64F);
        vo_state_ = INITIALIZATION;
        addKeyFrame(curr_);
    }
    else if (vo_state_ == INITIALIZATION)
    {
        // Match features
        my_geometry::matchFeatures(ref_->descriptors_, curr_->descriptors_, curr_->matches_);

        // Estimae motion and triangulate points
        estimateMotionAnd3DPoints();

        // Check initialization condition:
        //      init vo only when distance between matched keypoints are large
        if (checkIfVoGoodToInit(ref_->keypoints_, curr_->keypoints_, curr_->inlier_matches_))
        {
            cout << "Large movement detected at frame " << img_id << ". Start initialization" << endl;
            pushCurrPointsToMap();
            addKeyFrame(curr_);
            vo_state_ = OK;
            cout << "Inilialiation success !!!" << endl;
        }
        else // skip this frame
        {
            curr_->T_w_c_ = ref_->T_w_c_;
            cout << "Small movement. Not initialize..." << endl;
        }        
    }
    else if (vo_state_ == OK)
    {
        curr_->T_w_c_ = ref_->T_w_c_.clone(); // Initial estimation of the current pose
        poseEstimationPnP();

        // -- Insert a keyframe is motion is large
        if (checkLargeMoveForAddKeyFrame(curr_, ref_))
        {

            // --------------------- Triangulate more points --------------------
            // - Triangulate new points
            my_geometry::matchFeatures(ref_->descriptors_, curr_->descriptors_, curr_->matches_);

            // -- Use Essential matrix to find the inliers
            vector<DMatch> inlier_matches; // matches, that are inliers
            Mat dummy_R, dummy_t;
            helperEstiMotionByEssential(
                ref_->keypoints_, curr_->keypoints_,
                curr_->matches_, K,
                dummy_R, dummy_t, inlier_matches);
            cout << "Number of inliers with prev prev keyframe: " << inlier_matches.size() << endl;
            curr_->inlier_matches_ = inlier_matches;

            // / --Triangulate points
            Mat R_curr_to_key, t_curr_to_key;
            calcMotionFromFrame1to2(curr_, ref_, R_curr_to_key, t_curr_to_key);

            vector<Point3f> pts_3d_in_curr;
            helperTriangulatePoints(
                ref_->keypoints_, curr_->keypoints_,
                curr_->inlier_matches_, R_curr_to_key, t_curr_to_key, K,
                pts_3d_in_curr);
            curr_->inliers_pts3d_ = pts_3d_in_curr;

            // -- Update state
            pushCurrPointsToMap();
            optimizeMap();
            addKeyFrame(curr_);
        }
    }

    // Print relative motion
    if (vo_state_ == OK)
    {
        static Mat T_w_to_prev = Mat::eye(4, 4, CV_64F);
        const Mat &T_w_to_curr = curr_->T_w_c_;
        Mat T_prev_to_curr = T_w_to_prev.inv() * T_w_to_curr;
        Mat R, t;
        getRtFromT(T_prev_to_curr, R, t);
        cout << "R_prev_to_curr: " << R << endl;
        cout << "t_prev_to_curr: " << t.t() << endl;
    }

    cout << "\nEnd of a frame" << endl;
}

} // namespace my_slam