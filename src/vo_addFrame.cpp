
#include "my_slam/vo.h"

namespace my_slam
{

void VisualOdometry::addFrame(Frame::Ptr frame)
{
    // Settings
    const int VO_LENGHTH_UNIT = 100; // num units per meter. (10 for dm, 100 for cm, 1000 for mmm)
    const int FRAME_FOR_FIRST_ESSENTIAL = 14;

    // Renamed vars
    curr_ = frame;
    const int img_id = curr_->id_;
    const Mat &K = curr_->camera_->K_;
    // Vars

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
            init_frame_ = curr_;
            init_frame_keypoints_ = curr_->keypoints_;
            init_frame_descriptors_ = curr_->descriptors_.clone();
        }
        else if (vo_state_ == INITIALIZATION)
        {
            // Match features
            my_geometry::matchFeatures(init_frame_descriptors_, curr_->descriptors_, curr_->matches_);

            // -- Estimation motion by Essential && Homography matrix and get inlier points
            vector<Mat> list_R, list_t, list_normal;
            vector<vector<DMatch>> list_matches; // these are the inliers matches
            vector<vector<Point3f>> sols_pts3d_in_cam1_by_triang;
            const bool print_res = false, is_frame_cam2_to_cam1 = true;
            const bool compute_homography = true;
            helperEstimatePossibleRelativePosesByEpipolarGeometry(
                /*Input*/
                init_frame_keypoints_, curr_->keypoints_, curr_->matches_, K,
                /*Output*/
                list_R, list_t, list_matches, list_normal, sols_pts3d_in_cam1_by_triang,
                /*settings*/
                print_res, compute_homography, is_frame_cam2_to_cam1);

            // -- Compute errors of results of E/H estimation:
            // [epipolar error] and [trigulation error on norm plane]
            // for the 3 solutions of (E, H1, H2).
            // Choosing a good solution might based on these criterias.
            vector<double> list_error_epipolar;
            vector<double> list_error_triangulation;
            helperEvaluateEstimationsError(
                init_frame_keypoints_, curr_->keypoints_, list_matches,
                sols_pts3d_in_cam1_by_triang, list_R, list_t, list_normal, K,
                /*output*/
                list_error_epipolar, list_error_triangulation,
                true); // print result

            // -- Check initialization condition
            // Method 1: manually set an image id
            const bool VO_INIT_FLAG_MANUAL = img_id >= FRAME_FOR_FIRST_ESSENTIAL;

            // Method 2: check the distance between inlier points, run vo until there is a large movement.
            const bool VO_INIT_FLAG = checkIfVoGoodToInit(
                init_frame_keypoints_, curr_->keypoints_, list_matches[0]);

            if (VO_INIT_FLAG)
            {
                cout << "Large movement detected at frame " << img_id << ". Start initialization" << endl;
            }
            else
            { // skip this frame
                curr_->T_w_c_ = init_frame_->T_w_c_;
                break;
            }

            // -- Choose 1 solution from the 3 solutions.
            //      Results: R, t, inlier_matches, pts_3d in cam1 and cam2
            // Currently, I'll just simply choose the result from essential matrix.
            const int SOL_IDX = 0; // 0 corresponds to Essential matrix
            // Need to read papers such as ORB-SLAM2.
            vector<DMatch> &inlier_matches = list_matches[SOL_IDX];
            Mat &R = list_R[SOL_IDX], &t = list_t[SOL_IDX];
            vector<Point3f> &pts3d_in_cam1 = sols_pts3d_in_cam1_by_triang[SOL_IDX];
            vector<Point3f> pts3d_in_cam2;
            for (const Point3f &p1 : pts3d_in_cam1)
                pts3d_in_cam2.push_back(transCoord(p1, R, t));
            const int num_inlier_pts = pts3d_in_cam2.size();

            // -- Normalize the mean depth of points to be 1m
            double mean_depth=calcMeanDepth(pts3d_in_cam2);
            mean_depth = mean_depth / VO_LENGHTH_UNIT;
            t /= mean_depth;
            for (Point3f &p : pts3d_in_cam2)
                scalePointPos(p, 1/mean_depth);

            // -- Update current camera pos
            Mat T_curr_to_prev = transRt2T(R, t);
            curr_->T_w_c_ = init_frame_->T_w_c_ * T_curr_to_prev.inv();
            curr_->inlier_matches_ = inlier_matches;
            curr_->inliers_pts3d_ = pts3d_in_cam2;

            // --Update vo state
            vo_state_ = OK;

            ref_ = curr_;
        }
        else if (vo_state_ == OK)
        {

            // Push previous curr_'s keypoints to a local map
            // Match features (Simulate the matching process)


            // -- Insert prev frame inlier_matches' keypoints into the local map
            map_->map_points_.clear();
            const vector<DMatch> &prev_inlier_matches = ref_->inlier_matches_;
            for (int i = 0; i < prev_inlier_matches.size(); i++)
            {
                int pt_idx = prev_inlier_matches[i].trainIdx;
                MapPoint::Ptr map_point(new MapPoint(
                    Point3f_to_Mat(ref_->inliers_pts3d_[i]),
                    ref_->descriptors_.row(pt_idx).clone()));
                map_->insertMapPoint(map_point);
            }

            // -- From the local map, find the keypoints that fall into the current view
            vector<MapPoint::Ptr> candidate_mappoints_in_map;
            Mat candidate_descriptors_in_map;
            getMappointsInCurrentView(candidate_mappoints_in_map, candidate_descriptors_in_map);
    
            // -- Compare descriptors to find matches
            my_geometry::matchFeatures(candidate_descriptors_in_map, curr_->descriptors_, curr_->matches_);
            cout << "Number of 3d-2d pairs: " << curr_->matches_.size() << endl;
            
            // Triangulate
            vector<Point3f> pts_3d; 
            vector<Point2f> pts_2d; // a point's 2d pos in image2 pixel curr_
            for(int i=0;i<curr_->matches_.size();i++){
                DMatch &dm=curr_->matches_[i];
                MapPoint::Ptr mappoint=candidate_mappoints_in_map[dm.queryIdx];
                pts_3d.push_back(Mat_to_Point3f(mappoint->pos_));
                pts_2d.push_back(curr_->keypoints_[dm.trainIdx].pt);
            }

            // -- Solve PnP, get T_cam1_to_cam2
            Mat R_vec, R, t;
            bool 	useExtrinsicGuess = false;
            int 	iterationsCount = 100;
            float 	reprojectionError = 8.0;
            double 	confidence = 0.99;
            Mat pnp_inliers_mask ;
            solvePnPRansac(pts_3d, pts_2d, K, Mat(), R_vec, t,
                useExtrinsicGuess, iterationsCount, reprojectionError, confidence, pnp_inliers_mask);
            
            print_MatProperty(pnp_inliers_mask);            
            Rodrigues(R_vec, R);

            // ---------------------下面的需要进行更改－－－－－－－－－－－－－－－－－
            
            // --Triangulate points
            // my_geometry::matchFeatures(ref_->descriptors_, curr_->descriptors_, curr_->matches_);
            vector<Point3f> pts_3d_in_curr;
            helperTriangulatePoints(
                ref_->keypoints_, curr_->keypoints_,
                curr_->inlier_matches_, R, t, K,
                pts_3d_in_curr);
            curr_->inliers_pts3d_ = pts_3d_in_curr;

            // -- Update current camera pos
            Mat T_curr_to_prev = transRt2T(R, t);
            curr_->T_w_c_ = ref_->T_w_c_ * T_curr_to_prev.inv();

            //DEBUG
            invRt(R, t);
            cout << "\nprev_to_curr displaycement: " << t.t() << endl;

            // --Update vo state
            vo_state_ = vo_state_; // still OK
            ref_ = curr_;
        }
        break;
    }

    // Print
    if (vo_state_ == OK)
    {
        Frame::Ptr tmp_ref=frames_.back();
        cout<<tmp_ref->T_w_c_<<endl;
        Mat R, t, T = computeT_FromFrame1to2(tmp_ref, curr_);
        getRtFromT(T, R, t);
        cout << "R_prev_to_curr: " << R << endl;
        cout << "t_prev_to_curr: " << t.t() << endl;
    }

    // Save to buff.
    frames_.push_back(curr_);
    if (frames_.size() > 10)
        frames_.pop_front();

    cout<<"\nEnd of a frame"<<endl;
}

} // namespace my_slam