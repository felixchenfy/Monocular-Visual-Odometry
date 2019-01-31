
Monocular Visual Odometry
=======================================

**Content:** A simple **Monocular Visual Odometry** (VO) with initialization, tracking, local map, and optimization on single frame.

**Video demo**: http://feiyuchen.com/wp-content/uploads/vo_with_opti.mp4   
  On the left, **white line** is the estimated camera trajectory, **green line** is ground truth.   
  On the right, **green** are keypoints, **red** are inlier matches with map points.  

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
- [2. File Structure](#2-file-structure)
  - [2.1. Folders](#21-folders)
  - [2.2. Functions](#22-functions)
- [3. Dependencies](#3-dependencies)
- [4. How to Run](#4-how-to-run)
- [5. Results](#5-results)
- [6. Reference](#6-reference)
- [7. To Do](#7-to-do)

<!-- /TOC -->


# 1. Algorithm
This VO is achieved by the following procedures/algorithms:

## 1.1. Initialization

**When to initialize**:  
Given a video, set the 1st frame(image) as reference, and do feature matching with the 2nd frame. If the average displacement in pixel between inlier matched keypoints exceeds a threshold, the initialization will be started. Otherwise, skip to the 3rd, 4th, etc frames until the criteria satisfies at the K_th frame. Then estimate the relative camera pose between the 1st and K_th frame.

**Estimate relative camera pose**:  
Compute the **Essential Matrix** (E) and **Homography Matrix** (H) between the two frames. Compute their **Symmetric Transfer Error** by method in [ORB-SLAM paper](https://arxiv.org/abs/1502.00956) and choose the better one (i.e., choose H if H/(E+H)>0.45). **Decompose E or H** into the relative pose of rotation (R) and translation (t). By using OpenCV, E gives 1 result, and H gives 2 results, satisfying the criteria that points are in front of camera. For E, only single result to choose; For H, choose the one that makes the image plane and world-points plane more parallel.

**Recover scale**:  
Scale the translation t to be either: (1) Features points have average depth of 1m. Or (2) make it same scale as the corresponding groundth data so that I can draw and compare.

**Keyframe and local map**:  
Insert both 1st and K_th frame as **keyframe**. **Triangulate** their inlier matched keypoints to obtain the points' world positions. These points are called **map points** and are pushed to **local map**.

## 1.2. Tracking

Keep on estimating the next camera pose. First, find map points that are in the camera view. Do feature matching to find 2d-3d correspondance between 3d map points and 2d image keypoints. Estimate camera pose by RANSAC and PnP.

## 1.3. Optimization

Apply optimization to this single frame : Using the inlier 3d-2d corresponding from PnP, we can compute the sum of reprojection error of each point pair to form the cost function. By computing the deriviate wrt (1) points 3d pos and (2) camera pose, we can solve the optimization problem using Gauss-Newton Method and its variants. These are done by **g2o** and its built-in datatypes of `VertexSBAPointXYZ`, `VertexSE3Expmap`, and `EdgeProjectXYZ2UV`. See Slambook Chapter 4 and Chapter 7.8.2 for more details.

Then the camera pose and inlier points' 3d pos are updated, at a level of about 0.0001 meter. (I found that that this optimization doesn't make much difference compared to the one without it. I need to make improvement by optimizing multiple frames at the same time.) 

(TODO: Apply optimization to multiple frames, so that I can call this process bundle adjustment.)

## 1.4. Local Map

**Insert keyframe:** If the relative pose between current frame and previous keyframe is large enough with a translation or rotation larger than the threshold, insert current frame as a keyframe. Triangulate 3d points and push to local map.

**Clean up local map:** Remove map points that are: (1) not in current view, (2) whose view_angle is larger than threshold, (3) rarely be matched as inlier point. (See Slambook Chapter 9.4.)

## 1.5. Other details

**Image features**:  
Extract ORB keypoints and features. Then, a simple grid sampling on keypoint's pixel pos is applied to retain uniform keypoints.  
(Notes: The ORB-SLAM paper says that they do grid sampling in all pyramids, and extract more keypoints if somewhere has few points.)


**Feature matching**:  
Two methods are implemented, where good match is:  
(1) Feature's distance is smaller than threshold, described in Slambook.  
(2) Ratio of smallest and second smallest distance is smaller than threshold, proposed in Prof. Lowe's 2004 SIFT paper.  
The first one is adopted, which is easier to tune the parameters to generate fewer error matches.  
(Notes: ORB-SLAM paper is doing guided search for finding matches)

# 2. File Structure
## 2.1. Folders
* [include/](include/): c++ header files.
* [src/](src/): c++ definitions.
* [src_main/](src_main/): Main script to run VO.
* [test/](test/): Test scripts for c++ functions.

Main scripts and classes for VO are in [include/my_slam/](include/my_slam/). I referenced the [Slambook Chapter 9](https://github.com/gaoxiang12/slambook/tree/master/project/0.4) for setting this up.

## 2.2. Functions
Functions are declared in [include/](include/). Some of its folders contain a README. See the tree structure for overview:

```
include
├── my_basics
│   ├── basics.h
│   ├── config.h
│   ├── eigen_funcs.h
│   ├── io.h
│   ├── opencv_funcs.h
│   └── README.md
├── my_display
│   ├── pcl_display.h
│   └── pcl_display_lib.h
├── my_geometry
│   ├── camera.h
│   ├── common_include.h
│   ├── epipolar_geometry.h
│   ├── feature_match.h
│   └── motion_estimation.h
├── my_optimization
│   └── g2o_ba.h
└── my_slam
    ├── common_include.h
    ├── commons.h
    ├── frame.h
    ├── map.h
    ├── mappoint.h
    ├── README.md
    └── vo.h

```
# 3. Dependencies
Require: OpenCV, Eigen, Sophus, g2o.  
See details below:

**(1) OpenCV 4.0**   
Tutorial for install OpenCV 4.0: [link](https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/).

You may need a version newer than 3.4.5, because I used this function:  
`filterHomographyDecompByVisibleRefpoints`, which appears in OpenCV 3.4.5. 

**(2) Eigen 3**  
It's about matrix arithmetic. See its [official page]( http://eigen.tuxfamily.org/index.php?title=Main_Page). Install by:  
> $ sudo apt-get install libeigen3-dev 

(Note: Eigen only has header files. No ".so" or ".a".) 


**(3) Sophus**  
It's based on Eigen, and contains data type of SE3/SO3/se3/so3.

Download here: https://github.com/strasdat/Sophus. Do cmake and make. Since I failed to make install it, I manually moved “/Sophus/sophus” to “/usr/include/sophus”, and moved “libSophus.so” to “usr/lib”. Then, in my CMakeLists.txt, I do `set (THIRD_PARTY_LIBS libSophus.so )`.

**(4) g2o**  
Download here: https://github.com/RainerKuemmerle/g2o. Checkout to the last version in year 2017. Do cmake, make, make install.

If the csparse library is not found during cmake, please install the following package:
> $ sudo apt-get install libsuitesparse

# 4. How to Run  
> $ mkdir build && mkdir lib && mkdir bin   
> $ cd build && cmake .. && make && cd ..  

Then, set up things in [config/config.yaml](config/config.yaml), and run:  
> $ bin/run_vo config/config.yaml  

# 5. Results

I tested the current implementation on [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/download) fr1_desk and fr1_xyz dataset, but both performances are **bad**. I guess one of the **cause** is that high quality keypoints are too few, so the feature matching returns few matches. The **solution** I guess is to use the ORB-SLAM's method for extracting enough uniformly destributed keypoints, and doing guided matching based on the estimated camera motion. 

However, my program does work on this [New Tsukuba Stereo Database](http://cvlab.cs.tsukuba.ac.jp/), whose images and scenes are synthetic and have abundant high quality keypoints. Though large error still exists, the VO could roughly estimated the camera motion.  
See the gif **at the beginning of this README**.

I also put two video links here which I recorded on my computer of running this VO program:  
[1. VO video, with optimization on single frame](
https://github.com/felixchenfy/Monocular-Visual-Odometry-Data/blob/master/result/vo_with_opti.mp4)  
[2. VO video, no optimization](https://github.com/felixchenfy/Monocular-Visual-Odometry-Data/blob/master/result/vo_no_opti.mp4)  
The sad thing is, with or without this optimization on single frame, the result is about the same.


# 6. Reference

**(1) Slambook**:    
I read this Dr. Xiang Gao's [Slambook](https://github.com/gaoxiang12/slambook) before writing code. The book provides both vSLAM theory as well as easy-to-read code examples in every chapter. 

The framework of my program is based on Chapter 9 of Slambook, which is a RGB-D visual odometry project. Classes declared in [include/my_slam/](include/my_slam/) are based on this Chapter.

These files are mainly copied from Slambook and then modified:
* CMakeLists.txt
* [include/my_basics/config.h](include/my_basics/config.h).
*  [include/my_optimization/g2o_ba.h](include/my_optimization/g2o_ba.h).

I also borrowed other codes from the slambook. But since they are small pieces and lines, I didn't list them here.

In short, the Slambook provides huge help for me and my this project.


**(2) Matlab VO tutorial**:  
[This](https://www.mathworks.com/help/vision/examples/monocular-visual-odometry.html?searchHighlight=visual%20odometry&s_tid=doc_srchtitle) is a matlab tutorial of monocular visual odometry. Since Slambook doesn't write a lot about monocular VO, I resorted to this Matlab tutorial for solution. It helped me a lot for getting clear the whole workflow.

The dataset I used is also the same as this Matlab tutorial, which is the [New Tsukuba Stereo Database](http://cvlab.cs.tsukuba.ac.jp/).

**(3) ORB-SLAM paper**

I borrowed its code of **the criteria for choosing Essential or Homography** for VO initialization. See the functions of `checkEssentialScore` and `checkHomographyScore` in [motion_estimation.h](include/my_geometry/motion_estimation.h).



# 7. To Do

**Bugs**    
* In release mode, the program throws an error:
  > *** stack smashing detected ***: <unknown> terminated  
  Please run in debug mode.

**Improvements**  
* Build up the connections keypoints and frames. Then, add bundle adjustment.

