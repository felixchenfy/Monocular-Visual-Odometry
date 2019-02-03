
I record some bugs and problems during this project. Just for myself's usage.


# Problems met ==============================================
` 定位方法
pnp用两张图中共同区域的点估计相机2的姿态。估计完后，第一次不需要补充，但第二次就需要通过三角化补充特征点。
然而，问题是如何补充特征点？哪些是真正匹配上的特征点？这还是需要用E/H matrix来估计。所以还需要PnP吗。

` 如何剔除H矩阵分解结果中的错误结果？
用Triangulation并不能剔除错误结果。我看了一下，H的4个解，在归一化平面上的投影坐标全是一样的。
深度一正一负，一大一小。负的很容易就能剔除掉。正的需要根据场景来分析。

方法1：通常情况下，特征点所在的面的法向量(norm, 一般指向相机内侧方向）在相机的Z轴上有一个较大的分量。
方法2：如果已知一些点的3d坐标（在上一时刻得到的），那么就可以用这一信息来选择一个homo matrix.
方法3：如果已知上一时刻特征点的norm，那么这一时刻的norm和上一时刻不能变化太大。

` 01/22: （图１－图２的误差）和（图１－图１５）的误差几乎没有差异。然而，理论上，前者位移小，应该有更大的误差。



# Algorithms ==============================================

## 特征点匹配

* 用find essntial mat 得到的inlier里，有好多是错的！！！？？？
现象：用2004 lowe的论文的方法，能得到很多内点，但也有很多是错误的。
如果只有平面运动，可以通过H来做剔除。

## 对极几何

* 从两张图的匹配点找内点时，如果两张图的相机的位姿不准确，会导致对极几何算出来的误差直接爆炸。
所以还是用OpenCV的findEssentialMat重新估算下K、找下内点吧。

## VO

* 单目初始化，初始位移一定要大，不然误差大。

* 在相机的角度变化时，相机的位移变成了很小的数，同时特征点的位置很接近相机。
我把E/H的 epilolar error 和 reprojection error打印了出来，发现都很小，没有啥问题。 
所以可能出错的原因仍然是 scale drift.
考虑使用local map解决这个问题。

## g2o
函数1：对单帧的地图点和相机做优化，没问题。
函数2：bundle adjustment。支持多帧，函数里是同时对地图点和相机做优化。

把帧数设为1：如果我只更新相机，那是对的；更新地图点，就会出现很大的偏差。

把帧数设为3，报错：int g2o::csparse_extension::cs_cholsolsymb(const cs_di*, double*, const cs_dis*, double*, int*): cholesky failed!
Cholesky failure, writing debug.txt (Hessian loadable by Octave)，
函数返回错误的相机位置。

下一步是读一下网上的教程，看我是不是设置错了。


# Others ==============================================

## Eigen -------------------------
### AngleAxisf: how to change to matrix
Correct:
	Matrix3f m;
	m = AngleAxisf(1.0, Vector3f::UnitY());
Correct:
	Matrix3f m=AngleAxisf(1.0, Vector3f::UnitY()).toRotationMatrix();
Wrong:
	Matrix3f m=AngleAxisf(1.0, Vector3f::UnitY());

### Sometimes, include Eigen first, then include opencv
For this one: #include <opencv2/core/eigen.hpp>,
I must include some eigen libraries beforehand.

### data type: use double all the time
When I change data from opencv's double to eigen's float,
the result goes wrong.
Whatever the reason, it's better to keep them consistent.

## YAML -------------------------
###, string cannot start with "."
The setence below in .yaml gives an error: 
> suffix: .png

Error: "Bad format of floating-point constant in function icvProcessSpecialDouble".
Solution:
> suffix: ".png"

## Smart pointer
`` I met a bug of "Segmentation fault" due to Smart pointer.
(Not explicitly tested, but it seems its due to this matter.
 Althought it looks strange and unreasonable.)
	pointer p1=xxx, p2=xxx
	pointer p3=p1
	p1=p2 // After this, p1's originally pointed data will be destroyed
	cout << p3 // This gives an error
`` Feb/02, I met a "segmentation fault" again

pointer p1;
bool is_in=false;
if(1){
	p1=mappoints[mappoint_id];
	or 
    p1.reset( mappoints[mappoint_id].get() );
	is_in=true;
}else{
	None
}
if (is_in) cout << p1->content; or delete this sentence

Then, an segmentation error occurs at some random location.
It seems that p1=mappoints[mappoint_id] has ruin the original data.
