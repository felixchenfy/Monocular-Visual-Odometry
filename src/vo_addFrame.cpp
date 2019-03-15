
#include "my_slam/vo.h"

namespace my_slam
{

void VisualOdometry::addFrame(Frame::Ptr frame)
{
    // Settings
    const int FRAME_FOR_FIRST_ESSENTIAL = 14;
    pushFrameToBuff(frame);

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
        my_geometry::matchFeatures(ref_->descriptors_, curr_->descriptors_, curr_->matches_with_ref_);
        printf("Number of matches with the 1st frame: %d\n", (int)curr_->matches_with_ref_.size());

        // Estimae motion and triangulate points
        estimateMotionAnd3DPoints();
        printf("Number of inlier matches: %d\n", (int)curr_->inliers_matches_for_3d_.size());

        // Check initialization condition:
        if (checkIfVoGoodToInit())
        {
            cout << "Large movement detected at frame " << img_id << ". Start initialization" << endl;
            pushCurrPointsToMap();
            addKeyFrame(curr_);
            vo_state_ = OK;
            cout << "Inilialiation success !!!" << endl;
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
        }
        else // skip this frame
        {
            curr_->T_w_c_ = ref_->T_w_c_;
            cout << "Not initialize VO ..." << endl;
        }
    }
    else if (vo_state_ == OK)
    {
        curr_->T_w_c_ = ref_->T_w_c_.clone(); // Initial estimation of the current pose
        poseEstimationPnP();
        callBundleAdjustment();

        // -- Insert a keyframe is motion is large. Then, triangulate more points
        if (checkLargeMoveForAddKeyFrame(curr_, ref_))
        {
            // Feature matching
            my_geometry::matchFeatures(ref_->descriptors_, curr_->descriptors_, curr_->matches_with_ref_);

            // Find inliers by epipolar constraint
            curr_->inliers_matches_with_ref_ = helperFindInlierMatchesByEpipolarCons(
                ref_->keypoints_, curr_->keypoints_,curr_->matches_with_ref_, K);
            
            // Print
            printf("For triangulation: Matches with prev keyframe: %d; Num inliers: %d \n", 
                (int) curr_->matches_with_ref_.size(),(int) curr_->inliers_matches_with_ref_.size());

            // Triangulate points
            curr_->inliers_pts3d_ = helperTriangulatePoints(
                ref_->keypoints_, curr_->keypoints_,
                curr_->inliers_matches_with_ref_, getMotionFromFrame1to2(curr_, ref_), K);
    
            retainGoodTriangulationResult();

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
        cout << "\nCamera motion:" << endl;
        cout << "R_prev_to_curr: " << R << endl;
        cout << "t_prev_to_curr: " << t.t() << endl;
    }
    cout << "\nEnd of a frame" << endl;
}

} // namespace my_slam