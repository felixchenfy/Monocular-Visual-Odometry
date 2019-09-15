
Monocular Visual Odometry
=======================================

A monocular visual odometry (VO) with 4 components: initialization, tracking, local map, and bundle adjustment.

This is a practice after I read the [Slambook](htcdtps://github.com/gaoxiang12/slambook). Its also my final project for the course EESC-432 Advanced Computer Vision.

A demo:  
<p align = "center">
  <img src = "https://github.com/felixchenfy/Data-Storage/raw/master/Monocular-Visual-Odometry/VO_Opti_5frames.gif" height = "240px">
</p>

In the above figure:  
**Left** is a video and the detected key points.    
**Right** is the camera trajectory corresponding to the left video: **White** line is from VO; **Green** line is ground truth. Red markers on white line are the keyframes. Points are the map points, where points with red color are newly triangulated.  
You can download [video](https://github.com/felixchenfy/Data-Storage/raw/master/Monocular-Visual-Odometry/VO_Opti_5frames.avi) here.  


# Report
My pdf-version course report is [here](
https://github.com/felixchenfy/Data-Storage/blob/master/Monocular-Visual-Odometry/FeiyuChen_Report_EECS432.pdf). It has a more clear decription about the algorithms than this README, so I suggest to read it.


# Directory
<!-- TOC -->

- [Monocular Visual Odometry](#Monocular-Visual-Odometry)
- [Report](#Report)
- [Directory](#Directory)
- [1. Algorithm](#1-Algorithm)
  - [1.1. Initialization](#11-Initialization)
  - [1.2. Tracking](#12-Tracking)
  - [1.3. Local Map](#13-Local-Map)
  - [1.4. Bundle Adjustment](#14-Bundle-Adjustment)
  - [1.5. Other details](#15-Other-details)
- [2. File Structure](#2-File-Structure)
  - [2.1. Folders](#21-Folders)
  - [2.2. Functions](#22-Functions)
- [3. Dependencies](#3-Dependencies)
- [4. How to Run](#4-How-to-Run)
- [5. Results](#5-Results)
- [6. Reference](#6-Reference)
- [7. To Do](#7-To-Do)

<!-- /TOC -->


# 1. Algorithm
This VO is achieved by the following procedures/algorithms:

## 1.1. Initialization

**Estimate relative camera pose**:  
Given a video, set the 1st frame(image) as reference, and do feature matching with the 2nd frame. Compute the **Essential Matrix** (E) and **Homography Matrix** (H) between the two frames. Compute their **Symmetric Transfer Error** by method in [ORB-SLAM paper](https://arxiv.org/abs/1502.00956) and choose the better one (i.e., choose H if H/(E+H)>0.45). **Decompose E or H** into the relative pose between two frames, which is the rotation (R) and translation (t). By using OpenCV, E gives 1 result, and H gives 2 results, satisfying the criteria that points are in front of camera. For E, only single result to choose; For H, choose the one that makes the image plane and world-points plane more parallel.

**Keyframe and local map**:  
Insert both 1st and K_th frame as **keyframe**. **Triangulate** their inlier matched keypoints to obtain the points' world positions. These points are called **map points** and are pushed to **local map**.


**Check Triangulation Result**  
If the median triangulation angle is smaller than threshold, I will abandon this 2nd frame, and repeat the above process on frame 3, 4, etc. If at frame K, the triangulation angle is large than threshold, the initialization is completed.


**Change scale**:  
Scale the translation t to be the same length as the ground truth, so that I can make comparison with ground truth. Then, scale the map points correspondingly. 


## 1.2. Tracking

Keep on estimating the next camera pose. First, find map points that are in the camera view. Do feature matching to find 2d-3d correspondance between 3d map points and 2d image keypoints. Estimate camera pose by RANSAC and PnP.

## 1.3. Local Map

**Insert keyframe:** If the relative pose between current frame and previous keyframe is large enough with a translation or rotation larger than the threshold, insert current frame as a keyframe.   
Do feature matching between current and previous keyframe. Get inliers by epipoloar constraint. If a inlier cv::KeyPoint hasn't been triangulated before, then triangulate it and push it to local map.

**Clean up local map:** Remove map points that are: (1) not in current view, (2) whose view_angle is larger than threshold, (3) rarely be matched as inlier point. (See Slambook Chapter 9.4.)

**Graph/Connections between map points and frames:**  
Graphs are built at two stages of the algorithm:
1) After PnP, based on the 3d-2d correspondances, I update the connectionts between map points and current keypoints.
2) During triangulation, I also update the 2d-3d correspondance between current keypoints and triangulated mappoints, by either a direct link or going through previous keypoints that have been triangulated.


## 1.4. Bundle Adjustment

Since I've built the graph in previous step, I know what the 3d-2d point correspondances are in all frames.

Apply optimization to the previous N frames, where the cost function is the sum of reprojection error of each 3d-2d point pair. By computing the deriviate wrt (1) points 3d pos and (2) camera poses, we can solve the optimization problem using Gauss-Newton Method and its variants. These are done by **g2o** and its built-in datatypes of `VertexSBAPointXYZ`, `VertexSE3Expmap`, and `EdgeProjectXYZ2UV`. See Slambook Chapter 4 and Chapter 7.8.2 for more details.


## 1.5. Other details

**Image features**:  
Extract ORB keypoints and features. Then, a simple grid sampling is applied to obtain keypoints uniformly distributed across image.


**Feature matching**:  
Two methods are implemented, where good match is:  
(1) Feature's distance is smaller than threshold, described in Slambook.  
(2) Ratio of smallest and second smallest distance is smaller than threshold, proposed in Prof. Lowe's 2004 SIFT paper.  
The first one is adopted, which is easier to tune the parameters to generate fewer error matches.  

# 2. File Structure
## 2.1. Folders
* [include/](include/): c++ header files.
* [src/](src/): c++ definitions.
* [src_main/](src_main/): Main script to run VO.
* [test/](test/): Testing scripts for c++ functions.
* [data/](data/): Store images.

Main scripts and classes for VO are in [include/vo/](include/vo/). I referenced this structure from the [Slambook Chapter 9](https://github.com/gaoxiang12/slambook/tree/master/project/0.4).

## 2.2. Functions
Functions are declared in [include/](include/). Some of its folders contain a README. See the tree structure for overview:

```
include/
└── my_slam
    ├── basics
    │   ├── basics.h
    │   ├── config.h
    │   ├── eigen_funcs.h
    │   ├── io.h
    │   ├── opencv_funcs.h
    │   └── README.md
    ├── common_include.h
    ├── display
    │   ├── pcl_display.h
    │   └── pcl_display_lib.h
    ├── geometry
    │   ├── camera.h
    │   ├── epipolar_geometry.h
    │   ├── feature_match.h
    │   └── motion_estimation.h
    ├── optimization
    │   └── g2o_ba.h
    └── vo
        ├── frame.h
        ├── map.h
        ├── mappoint.h
        ├── README.md
        ├── vo_commons.h
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

(Note: Eigen only has header files. No ".so" or ".a" files.) 


**(3) Sophus**  

It's based on Eigen, and contains datatypes for Lie Group and Lie Algebra (SE3/SO3/se3/so3).

Download this lib here: https://github.com/strasdat/Sophus. Do cmake and make. Since I failed to make install it, I manually moved “/Sophus/sophus” to “/usr/include/sophus”, and moved “libSophus.so” to “usr/lib”. Then, in my CMakeLists.txt, I add this: `set (THIRD_PARTY_LIBS libSophus.so )`.

If there is an error of "unit_complex_.real() = 1.;"
replace it and its following line with "unit_complex_ = std::complex<double>(1,0);"


**(4) g2o**  

First install either of the following two packages:
> $ sudo apt-get install libsuitesparse
> $ sudo apt-get install libsuitesparse-dev

Download here: https://github.com/RainerKuemmerle/g2o.  
Checkout to the last version in year 2017. Do cmake, make, make install.

# 4. How to Run  
> $ mkdir build && mkdir lib && mkdir bin   
> $ cd build && cmake .. && make && cd ..  

Then, set up things in [config/config.yaml](config/config.yaml), and run:  
> $ bin/run_vo config/config.yaml  

# 5. Results

I tested the current implementation on [TUM](https://vision.in.tum.de/data/datasets/rgbd-dataset/download) fr1_desk and fr1_xyz dataset, but both performances are **bad**. I guess its due to too few detected keypoints, which causes too few keypoints matches. The **solution** I guess is to use the ORB-SLAM's method for extracting enough uniformly destributed keypoints across different scales, and doing guided matching based on the estimated camera motion. 

Despite bad performance on fr1 dataset, my program does work well on this [New Tsukuba Stereo Database](http://cvlab.cs.tsukuba.ac.jp/), whose images and scenes are synthetic and have abundant high quality keypoints. The results are shown below.

I tested my VO with 3 different settings: (1) No optimization. (2) Optimize on map points and current camera pose. (3) Optimize on previous 5 camera poses. See videos below: 

(1) No optimization:
<p align = "center">
  <img src = "https://github.com/felixchenfy/Data-Storage/raw/master/Monocular-Visual-Odometry/VO_No_Opti.gif" height = "240px">
</p>


(2) Optimize on points + current pose:
<p align = "center">
  <img src = "https://github.com/felixchenfy/Data-Storage/raw/master/Monocular-Visual-Odometry/VO_Opti_1frame_and_points.gif" height = "240px">
</p>

(2) Optimize on prev 5 poses: 
<p align = "center">
  <img src = "https://github.com/felixchenfy/Data-Storage/raw/master/Monocular-Visual-Odometry/VO_Opti_5frames.gif" height = "240px">
</p>

The result shows: (1) Optimization improves accuracy. (2) The estiamted trajectory is close to the ground truth. 

# 6. Reference

**(1) Slambook**:    
I read this Dr. Xiang Gao's [Slambook](https://github.com/gaoxiang12/slambook) before writing code. The book provides both vSLAM theory as well as easy-to-read code examples in every chapter. 

The framework of my program is based on Chapter 9 of Slambook, which is a RGB-D visual odometry project. Classes declared in [include/vo/](include/vo/) are based on this Chapter.

These files are mainly copied or built on top of the Slambook's code:
* CMakeLists.txt
* [include/basics/config.h](include/basics/config.h).
*  [include/optimization/g2o_ba.h](include/optimization/g2o_ba.h).

I also borrowed other codes from the slambook. But since they are small pieces and lines, I didn't list them here.

In short, the Slambook provides huge help for me and my this project.


**(2) Matlab VO tutorial**:  
[This](https://www.mathworks.com/help/vision/examples/monocular-visual-odometry.html?searchHighlight=visual%20odometry&s_tid=doc_srchtitle) is a matlab tutorial of monocular visual odometry. Since Slambook doesn't write a lot about monocular VO, I resorted to this Matlab tutorial for solution. It helped me a lot for getting clear the whole workflow.

The dataset I used is also the same as this Matlab tutorial, which is the [New Tsukuba Stereo Database](http://cvlab.cs.tsukuba.ac.jp/).

**(3) ORB-SLAM/ORB-SLAM2 papers**

I borrowed its code of the criteria for choosing Essential or Homography (for decomposition to obtain relative camera pose.). The copied functions are `checkEssentialScore` and `checkHomographyScore` in [motion_estimation.h](include/geometry/motion_estimation.h).



# 7. To Do

**Bugs**    
* In release mode, the program throws an error:
  > *** stack smashing detected ***: <unknown> terminated  
  Please run in debug mode.

**Improvements**  
* In bundle adjustment, I cannot optimize  (1) multiple frames and (b) map points **at the same time**. It returns huge error. I haven't figure out why.

