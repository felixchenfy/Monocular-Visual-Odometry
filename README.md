
Monocular Visual Odometry
=======================================

**Content:** A simple **Monocular Visual Odometry** (VO) with initialization, tracking, local map, and optimization on single frame.

**Video demo**: http://feiyuchen.com/wp-content/uploads/vo_with_opti.mp4  
On the left, **white line** is the estimated camera trajectory, **green line** is ground truth. On the right, **green** are keypoints, and **red** are inlier matches with map points.

![](https://github.com/felixchenfy/Monocular-Visual-Odometry-Data/raw/master/result/vo_with_opti.gif)

**Project purpose:** This is a practice after I read the [Slambook](https://github.com/gaoxiang12/slambook). This will also be my final project of the course EESC-432 Advanced Computer Vision, so this repo will be kept on updating until and completed by **March 15th**. 

**Data files:** Please download from here: https://github.com/felixchenfy/Monocular-Visual-Odometry-Data

**Directory:**
<!-- TOC -->

- [Monocular Visual Odometry](#monocular-visual-odometry)
- [1. Algorithm](#1-algorithm)
  - [1.1. Initialization](#11-initialization)
  - [1.2. Tracking](#12-tracking)
  - [1.3. Optimization](#13-optimization)
  - [1.4. Local Map](#14-local-map)
  - [1.5. Other details](#15-other-details)
- [2. Software Architecture](#2-software-architecture)
- [3. How to Run](#3-how-to-run)
- [4. Result](#4-result)
- [5. Reference](#5-reference)
- [6. To Do](#6-to-do)

<!-- /TOC -->




# 1. Algorithm
This VO is achieved by the following procedures/algorithms:

## 1.1. Initialization

**When to initialize**:  
Given a video, set the 1st frame(image) as reference, and do feature matching with the 2nd frame. If the average displacement in pixel between inlier matched keypoints exceeds a threshold, the initialization will be started. Otherwise, skip to the 3rd, 4th, etc frames until the criteria satisfies at the K_th frame. Then we estimate the relative camera pose between the 1st and K_th frame.

**Estimate relative camera pose**:  
Compute the **Essential Matrix** (E) and **Homography Matrix** (H) between the two frames. Compute their **Symmetric Transfer Error** by method in [ORB-SLAM paper](https://arxiv.org/abs/1502.00956) and choose the better one (i.e., choose H if H/(E+H)>0.45). **Decompose E or H** into the relative pose of rotation (R) and translation (t). By using OpenCV, E gives 1 result, and H gives 2 results, satisfying the criteria that points are in front of camera. For E, only one result to choose; For H, choose the one that makes the image plane and world-points plane more parallel.

**Recover scale**:  
Scale the translation t to be either: (1) Features points have average depth of 1m. Or (2) make it same scale as the corresponding groundth data so that I can draw and compare.

**Keyframe and local map**:  
Insert both 1st and K_th frame as **keyframe**. **Triangulate** their inlier points to obtain the points' world positions. These points are called **map points** and are pushed to **local map**.

## 1.2. Tracking

Keep on estimating the next camera pose. First, find map points that are in the camera view. Do feature matching to find 2d-3d correspondance between 3d map points and 2d image keypoints. Estimate camera pose by RANSAC and PnP.

## 1.3. Optimization

Apply optimization to this single frame : Using the inlier 3d-2d corresponding from PnP, we can compute the sum of reprojection error of each point pair to form the cost function. By computing the deriviate wrt (1) points 3d pos and (2) camera pose, we can solve the optimization problem using Gauss-Newton Method and its variants. These are done by **g2o** and its built-in datatypes of "VertexSBAPointXYZ", "VertexSE3Expmap", and "EdgeProjectXYZ2UV". See Slambook Chapter 4 and Chapter 7.8.2 for more details.

Then the camera pose and inlier points' 3d pos are updated, at a level of about 0.0001 meter. (Though my final result shows that this optimization doesn't make much difference. Maybe I need to optimize more frames and keypoints) 

(TODO: Apply optimization to multiple frames, and then I call make it a real bundle adjustment.)

## 1.4. Local Map

**Insert keyframe:** If the relative pose between current frame and previous keyframe is large enough, with a translation or rotation larger than the threshold, insert current frame as a keyframe. Triangulate 3d points and push to local map.

**Clean up local map:** Remove map points that are: (1) not in current view, (2) whose view_angle is larger than threshold, (3) rarely be matched as inlier point. (See Slambook Chapter 9.4.)

## 1.5. Other details

**Image features**:  
Extract ORB keypoints and features. Then, a simple grid sampling on keypoint's pixel pos is applied to retain uniform keypoints.

**Feature matching**:  
Two methods are implemented, where good match is:  
(1) Feature's distance is smaller than threshold, described in Slambook.  
(2) Ratio of smallest and second smallest distance is smaller than threshold, proposed in Prof. Lowe's 2004 SIFT paper.  
The first one is adopted, which generates fewer error matches.

# 2. Software Architecture
TODO

# 3. How to Run  
$ mkdir build && mkdir lib && mkdir bin   
$ cd build && cmake .. && make && cd ..  
$ bin/run_vo config/config.yaml  

# 4. Result
TODO

# 5. Reference

**(1) Slambook**  
I read this Dr. Xiang Gao's [slambook](https://github.com/gaoxiang12/slambook) before writing code. The book provides both vSLAM theory as well as easy-to-read code examples in every chapter. 

The framework of my program is based on Chapter 9 of Slambook, which is a RGB-D visual odometry project. Classes declared in [include/my_slam/](include/my_slam/) are using its structure.

These files are mainly copied from Slambook and then modified:
* CMakeLists.txt
* [include/my_basics/config.h](include/my_basics/config.h) and its .cpp.
*  [include/my_optimization/g2o_ba.h](include/my_optimization/g2o_ba.h) and its .cpp.

I also borrowed other codes from the slambook. But since they are small pieces and lines, I didn't list them here.

In short, the Slambook provides huge help for me and my this project.

**(2) ORB-SLAM paper**

Slambook doesn't write a lot about monocular visual odometry, so I resorted to this paper for help. Besides learning import ideas for vSLAM, I borrowed its code of **the criteria for choosing Essential or Homography** for VO initialization. 

For my next stage, I will read more of this paper's code in order to keep on improving my project.


# 6. To Do

* Bugs:
In release mode, the program throws an error:
> *** stack smashing detected ***: <unknown> terminated  
Please run in debug mode.

