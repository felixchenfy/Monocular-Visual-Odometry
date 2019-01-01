
This is a very simple vSLAM project (or **practice**) of using monocular camera for localization and building sparse map. 

Video demo: link.

# 0. Reference

This project is largely based on Dr. Xiang Gao's [slambook](https://github.com/gaoxiang12/slambook). This book provides both vSLAM theory as well as easy-to-read code examples for every chapter. 

I've borrowed lots of code from this slambook:
* I started from a empty repo, then incrementally borrowed codes from [Chapter 9](https://github.com/gaoxiang12/slambook/tree/master/project/0.4) (a demo project for **RGB-D Visual Odometry**) to build up my project. So the framework of my software is almost the same as the one taught on Chapter 9.
* Since I'm using RGB camera, I then borrowed pieces of codes from [Chapter 7](https://github.com/gaoxiang12/slambook/tree/master/ch7) for Essential/Homography matrix and Triangulation.
* Borrowed code of "Bag-or-Words" from [Chapter 12](https://github.com/gaoxiang12/slambook/tree/master/ch12) for loop closure.

Despite these lots of borrowing, I still learned a lot. I've read large percent of the book's code and then managed to assemble, build up, and put this MonoSLAM project into practice.

# Change on algorithm

* add grid sampling for ORB features.

# 1. Algorithm

# 测试记录
` 定位方法
pnp用两张图中共同区域的点估计相机2的姿态。估计完后，第一次不需要补充，但第二次就需要通过三角化补充特征点。
然而，问题是如何补充特征点？哪些是真正匹配上的特征点？这还是需要用E/H matrix来估计。所以还需要PnP吗。

` 如何剔除H矩阵分解结果中的错误结果？
用Triangulation并不能剔除错误结果。我看了一下，H的4个解，在归一化平面上的投影坐标全是一样的。
深度一正一负，一大一小。负的很容易就能剔除掉。正的需要根据场景来分析。
方法1：通常情况下，法向量在相机的Z轴上有一个较大的分量。
方法2：如果已知一些点的3d坐标（在上一时刻得到的），那么就可以用这一信息来选择一个homo matrix.
方法3：如果已知上一时刻特征点的norm，那么这一时刻的norm和上一时刻不能变化太大。

` E在特征点共面的时候误差较大

