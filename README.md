
# Monocular Visual Odometry
A simple Monocular Visual Odometry (VO) with initialization, tracking, local mapping, and bundle adjustment.
![link to gif]

![](result_frame.png)


# Algorithm
This VO is achieved by the following procedures/algorithms:

1. Initialization

    When to initialize: Given a video, set the 1st frame(image) as reference, and do feature matching with the 2nd frame. If the average displacement between inlier matched keypoints exceeds a threshold, the initialization will be started. Otherwise, skip to the 3rd, 4th, etc frames until the criteria satisfies at the K_th frame.

    How to compute initial movement: Then, estimating the relative pose between 1st and K_th frame: Compute the Essential Matrix (E) and Homography Matrix (H) between the two frames. Compute their Symmetric Transfer Error by method in [ORB-SLAM paper](https://arxiv.org/abs/1502.00956) and choose the better one (i.e., choose H if H/(E+H)>0.45). Decompose E/H into the relative pose of rotation (R) and translation (t). By using OpenCV, E gives 1 result, and H gives 2 results, satisfying the criteria that points are in front of camera. For E, only one result to choose; For H, choose the one that makes the image plane and world-points plane more parallel.

    Recover scale: Scale the translation t to be either: (1) Features points have mean depth of 1m. Or (2) make it same scale as the corresponding groundth data so that I can draw and compare.

    Keyframe and local map: Insert both 1st and K_th frame as **keyframe**. Triangulate their inlier points to obtain the points' world positions. These points are called **map points** and are pushed to **local map**.

2. Tracking

    Estimate new camera pose: For the following ith frame, find map points that are in the camera view. Do feature matching to find 2d-3d correspondance between 3d map points and 2d image keypoints. Estimate camera pose by RANSAC and PnP.

3. Optimization

    Apply bundle adjustment to this single frame: Using the inlier 3d-2d corresponding from PnP, we can compute the sum of reprojection error of each point pair to form the cost function. By computing the deriviate wrt (1) points 3d pos and (2) camera pose, we can solve the optimization problem using Gauss-Newton Method and its variants. These are done by **g2o** and its built-in datatypes of "VertexSBAPointXYZ", "VertexSE3Expmap", and "EdgeProjectXYZ2UV". See [Slambook](https://github.com/gaoxiang12/slambook) Chapter 4 and Chapter 7.8.2 for more details.

    Then the camera pose and inlier points' 3d pos are updated, at a level of 0.0001 meter. (Though the final result show that this optimization doesn't make much difference.) 

4. Local Map

    Insert keyframe: If the relative pose between current frame and previous keyframe is large enough, with a translation or rotation larger than the threshold, insert current frame as a keyframe. Triangulate 3d points and push to local map.

    Clean up local map: Remove map points that are: (1) not in current view, (2) view angle larger than threshold, (3) ratio of match/visible times smaller than threshold. (This reference Slambook Chapter 9.4.)

5. Other details

* Image features: ORB. Then, a simple grid sampling on keypoint's pixel pos is applied to avoid the keypoints being too dense. 
* Feature matching: Two methods are implemented, where good match is: (1) Feature's distance is smaller than threshold, described in Slambook. (2) Ratio of smallest and second smallest distance is smaller than threshold, proposed in Prof. Lowe's 2004 SIFT paper. The first one is adopted, which generates fewer error matches.

# Software Architecture

