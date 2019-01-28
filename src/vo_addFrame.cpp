
#include "my_slam/vo.h"

namespace my_slam
{

void VisualOdometry::addFrame(Frame::Ptr frame)
{
    // Settings
    const int VO_UNITS_PER_METER = 20; // num units per meter. (10 for dm, 100 for cm, 1000 for mmm)
    const int FRAME_FOR_FIRST_ESSENTIAL = 14;

    // Renamed vars
    curr_ = frame;
    const int img_id = curr_->id_;
    const Mat &K = curr_->camera_->K_;


    // Start
    while (1)
    {
        printf("\n\n=============================================\n");
        printf("Start processing the %dth image.\n", img_id);

        // Init a curr_. Extract keypoints and descriptors.
        curr_->extractKeyPoints();
        curr_->computeDescriptors();
        cout << "Number of keypoints: " << curr_->keypoints_.size() << endl;

        // vo_state_: BLANK -> INITIALIZATION
        if (vo_state_ == BLANK)
        {
            curr_->T_w_c_ = Mat::eye(4, 4, CV_64F);
            vo_state_ = INITIALIZATION;

            // Update params of the first frame
            keyframes_.push_back(curr_);
            cout << "!!! INSERT KEYFRAME: " << img_id << " !!!" << endl;
            ref_ = curr_;
        }
        else if (vo_state_ == INITIALIZATION)
        {
            // Match features
            my_geometry::matchFeatures(ref_->descriptors_, curr_->descriptors_, curr_->matches_);
            
            // Estimae motion and triangulate points
            Mat R_curr_to_prev, t_curr_to_prev;
            estimateMotionAnd3DPoints(R_curr_to_prev, t_curr_to_prev, curr_->inlier_matches_, curr_->inliers_pts3d_);
            
            // -- Check initialization condition
            // Check the distance between inlier points, run vo until there is a large movement.
            if (checkIfVoGoodToInit(ref_->keypoints_, curr_->keypoints_, curr_->inlier_matches_))
            {
                cout << "Large movement detected at frame " << img_id << ". Start initialization" << endl;
            }
            else
            { // skip this frame
                curr_->T_w_c_ = ref_->T_w_c_;
                break;
            }

            // -- Normalize the mean depth of points to be 1m
            const int num_inlier_pts = curr_->inliers_pts3d_.size();
            double mean_depth = calcMeanDepth(curr_->inliers_pts3d_) / VO_UNITS_PER_METER;
            t_curr_to_prev /= mean_depth;
            for (Point3f &p : curr_->inliers_pts3d_)
                scalePointPos(p, 1 / mean_depth);

            // -- Push points to local map
            curr_->T_w_c_ = ref_->T_w_c_ * transRt2T(R_curr_to_prev, t_curr_to_prev).inv();
            pushPointsToMap(curr_->inliers_pts3d_, curr_->T_w_c_,
                curr_->descriptors_,curr_->kpts_colors_,curr_->inlier_matches_);

            // --Update vo state
            vo_state_ = OK;
            keyframes_.push_back(curr_);

            cout << "!!! INSERT KEYFRAME: " << img_id << " !!!" << endl;
            cout << "Inilialiation success !!!" << endl;
        }
        else if (vo_state_ == OK)
        {
            // curr_->T_w_c_ =prev_T_w_c_; // give an initial estimation of the current pos
            // 爲什麼我加了這句以後，在第5帧左右会出错？

            // Push previous curr_'s keypoints to a local map
            // Match features (Simulate the matching process)

            // -- From the local map, find the keypoints that fall into the current view
            vector<MapPoint::Ptr> candidate_mappoints_in_map;
            Mat candidate_descriptors_in_map;
            getMappointsInCurrentView(candidate_mappoints_in_map, candidate_descriptors_in_map);

            // -- Compare descriptors to find matches, and extract 3d 2d correspondance
            my_geometry::matchFeatures(candidate_descriptors_in_map, curr_->descriptors_, curr_->matches_);
            cout << "Number of 3d-2d pairs: " << curr_->matches_.size() << endl;
            vector<Point3f> pts_3d;
            vector<Point2f> pts_2d; // a point's 2d pos in image2 pixel curr_
            for (int i = 0; i < curr_->matches_.size(); i++)
            {
                DMatch &dm = curr_->matches_[i];
                MapPoint::Ptr mappoint = candidate_mappoints_in_map[dm.queryIdx];
                pts_3d.push_back(Mat_to_Point3f(mappoint->pos_));
                pts_2d.push_back(curr_->keypoints_[dm.trainIdx].pt);
            }

            // -- Solve PnP, get T_cam1_to_cam2
            Mat R_vec, R, t;
            bool useExtrinsicGuess = false;
            int iterationsCount = 100;
            float reprojectionError = 8.0;
            double confidence = 0.99;
            Mat pnp_inliers_mask;
            solvePnPRansac(pts_3d, pts_2d, K, Mat(), R_vec, t,
                           useExtrinsicGuess, iterationsCount, reprojectionError, confidence, pnp_inliers_mask);
            Rodrigues(R_vec, R);

            // -- Update current camera pos
            curr_->T_w_c_ = transRt2T(R, t).inv();

            // -- Insert a keyframe is motion is large
            Frame::Ptr frame_for_tri;
            frame_for_tri = keyframes_[keyframes_.size() - 2];
            Mat T_key_to_curr = frame_for_tri->T_w_c_.inv() * curr_->T_w_c_;
            if (calcMatNorm(T_key_to_curr.t()) > 0.05 * VO_UNITS_PER_METER)
            {
                keyframes_.push_back(curr_);
                cout << "!!! INSERT KEYFRAME: " << img_id << " !!!" << endl;

                // ---------------------下面的需要进行更改－－－－－－－－－－－－－－－－－
                // - Triangulate new points
                frame_for_tri = keyframes_[keyframes_.size() - 2]; // frame_for_triangulation
                my_geometry::matchFeatures(frame_for_tri->descriptors_, curr_->descriptors_, curr_->matches_);

                // -- Use Essential matrix to find the inliers
                vector<DMatch> inlier_matches; // matches, that are inliers
                Mat dummy_R, dummy_t;
                helperEstiMotionByEssential(
                    frame_for_tri->keypoints_, curr_->keypoints_,
                    curr_->matches_, K,
                    dummy_R, dummy_t, inlier_matches);
                cout << "Number of inliers with prev prev keyframe: " << inlier_matches.size() << endl;
                curr_->inlier_matches_ = inlier_matches;

                // / --Triangulate points
                Mat R_curr_to_key, t_curr_to_key;
                calcMotionFromFrame1to2(curr_, frame_for_tri, R_curr_to_key, t_curr_to_key);

                vector<Point3f> pts_3d_in_curr;
                helperTriangulatePoints(
                    frame_for_tri->keypoints_, curr_->keypoints_,
                    curr_->inlier_matches_, R_curr_to_key, t_curr_to_key, K,
                    pts_3d_in_curr);
                curr_->inliers_pts3d_ = pts_3d_in_curr;

                // -- Push points into local map
                pushPointsToMap(
                    curr_->inliers_pts3d_,
                    curr_->T_w_c_,
                    curr_->descriptors_,
                    curr_->kpts_colors_,
                    curr_->inlier_matches_);
                
            }

            // --Update vo state
            vo_state_ = vo_state_; // still OK
        }
        break;
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

    // Save to buff.
    // keyframes_.push_back(curr_);
    // if (keyframes_.size() > 10)
    // keyframes_.pop_front();
    prev_T_w_c_ = frame->T_w_c_;
    cout << "\nEnd of a frame" << endl;
}

} // namespace my_slam