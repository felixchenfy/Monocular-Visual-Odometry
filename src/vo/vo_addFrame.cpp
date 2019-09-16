// The member function `addFrame` of class VisualOdometry is defined here.

#include "my_slam/vo/vo.h"

namespace my_slam
{
namespace vo
{

void VisualOdometry::addFrame(Frame::Ptr frame)
{
    // Settings
    pushFrameToBuff_(frame);

    // Renamed vars
    curr_ = frame;
    const int img_id = curr_->id_;
    const cv::Mat &K = curr_->camera_->K_;

    // Start
    printf("\n\n=============================================\n");
    printf("Start processing the %dth image.\n", img_id);

    curr_->calcKeyPoints();
    curr_->calcDescriptors();
    cout << "Number of keypoints: " << curr_->keypoints_.size() << endl;
    prev_ref_ = ref_;

    // vo_state_: BLANK -> DOING_INITIALIZATION
    if (vo_state_ == BLANK)
    {
        curr_->T_w_c_ = cv::Mat::eye(4, 4, CV_64F);
        vo_state_ = DOING_INITIALIZATION;
        addKeyFrame_(curr_); // curr_ becomes the ref_
    }
    else if (vo_state_ == DOING_INITIALIZATION)
    {
        // Match features
        static const float max_matching_pixel_dist_in_initialization =
            basics::Config::get<float>("max_matching_pixel_dist_in_initialization");
        static const int method_index = basics::Config::get<float>("feature_match_method_index_initialization");
        geometry::matchFeatures(
            ref_->descriptors_, curr_->descriptors_, curr_->matches_with_ref_, method_index,
            false,
            ref_->keypoints_, curr_->keypoints_,
            max_matching_pixel_dist_in_initialization);

        printf("Number of matches with the 1st frame: %d\n", (int)curr_->matches_with_ref_.size());

        // Estimae motion and triangulate points
        estimateMotionAnd3DPoints_();
        printf("Number of inlier matches: %d\n", (int)curr_->inliers_matches_for_3d_.size());

        // Check initialization condition:
        printf("\nCheck VO init conditions: \n");
        if (isVoGoodToInit_())
        {
            cout << "Large movement detected at frame " << img_id << ". Start initialization" << endl;
            pushCurrPointsToMap_();
            addKeyFrame_(curr_);
            vo_state_ = DOING_TRACKING;
            cout << "Inilialiation success !!!" << endl;
            cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
        }
        else // skip this frame
        {
            curr_->T_w_c_ = ref_->T_w_c_;
            cout << "Not initialize VO ..." << endl;
        }
    }
    else if (vo_state_ == DOING_TRACKING)
    {
        printf("\nDoing tracking\n");
        curr_->T_w_c_ = ref_->T_w_c_.clone(); // Initial estimation of the current pose
        bool is_pnp_good = poseEstimationPnP_();
        if (!is_pnp_good) // pnp failed. Print log.
        {
            int num_matches = curr_->matches_with_map_.size();
            constexpr int kMinPtsForPnP = 5;
            printf("PnP failed.\n");
            printf("    Num inlier matches: %d.\n", num_matches);
            if (num_matches >= kMinPtsForPnP)
            {
                printf("    Computed world to camera transformation:\n");
                std::cout << curr_->T_w_c_ << std::endl;
            }
            printf("PnP result has been reset as R=identity, t=zero.\n");
        }
        else // pnp good
        {
            callBundleAdjustment_();
            // -- Insert a keyframe is motion is large. Then, triangulate more points
            if (checkLargeMoveForAddKeyFrame_(curr_, ref_))
            {
                // Feature matching
                static const float max_matching_pixel_dist_in_triangulation =
                    basics::Config::get<float>("max_matching_pixel_dist_in_triangulation");
                static const int method_index = basics::Config::get<float>("feature_match_method_index_pnp");
                geometry::matchFeatures(
                    ref_->descriptors_, curr_->descriptors_, curr_->matches_with_ref_, method_index,
                    false,
                    ref_->keypoints_, curr_->keypoints_,
                    max_matching_pixel_dist_in_triangulation);

                // Find inliers by epipolar constraint
                curr_->inliers_matches_with_ref_ = geometry::helperFindInlierMatchesByEpipolarCons(
                    ref_->keypoints_, curr_->keypoints_, curr_->matches_with_ref_, K);

                // Print
                printf("For triangulation: Matches with prev keyframe: %d; Num inliers: %d \n",
                       (int)curr_->matches_with_ref_.size(), (int)curr_->inliers_matches_with_ref_.size());

                // Triangulate points
                curr_->inliers_pts3d_ = geometry::helperTriangulatePoints(
                    ref_->keypoints_, curr_->keypoints_,
                    curr_->inliers_matches_with_ref_, getMotionFromFrame1to2(curr_, ref_), K);

                retainGoodTriangulationResult_();

                // -- Update state
                pushCurrPointsToMap_();
                optimizeMap_();
                addKeyFrame_(curr_);
            }
        }
    }

    // Print relative motion
    if (vo_state_ == DOING_TRACKING)
    {
        static cv::Mat T_w_to_prev = cv::Mat::eye(4, 4, CV_64F);
        const cv::Mat &T_w_to_curr = curr_->T_w_c_;
        cv::Mat T_prev_to_curr = T_w_to_prev.inv() * T_w_to_curr;
        cv::Mat R, t;
        basics::getRtFromT(T_prev_to_curr, R, t);
        cout << "\nCamera motion:" << endl;
        cout << "R_prev_to_curr: " << R << endl;
        cout << "t_prev_to_curr: " << t.t() << endl;
    }
    prev_ = curr_;
    cout << "\nEnd of a frame" << endl;
}

} // namespace vo
} // namespace my_slam
