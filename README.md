# Iterative Global Similarity Points (IGSP) 
A robust coarse-to-fine pairwise point cloud registration method 

### [Video](https://www.youtube.com/watch?v=DZr-8AceSqA)

### [Paper](https://arxiv.org/abs/1808.03899) 
Iterative Global Similarity Points: A robust coarse-to-fine integration solution for pairwise 3D point cloud registration, Yue Pan, Bisheng Yang, Fuxun Liang, Zhen Dong, Accepted by 3DV-2018 

### [Poster](https://github.com/YuePanEdward/YuePanEdward.github.io/blob/master/assets/3DVposter.pdf)


Now available on Windows


### How to use
1.Input Target cloud

2.Input Source cloud

3.Input parameter list

### Parameter list
Take the ALS registration parameter setting as an example

> 0.5
> 100000
> 3.0
> 0.6
> 50
> 1
> 2.0
> 1.5
> 1.5
> 6
> 0.05
> 0.01
> 0.01
> 1
> 1

Notification
1  resolution of voxel downsampling (unit:m)
2  num_point_bb (the maximum number of keypoints remaining in the bounding box of the target point cloud)
3  feature_r (BSC[1] feature calulation radius) 
4  keypoint_max_ratio (the max curvature for keypoint detection, the larger this value is, the more keypoints would be)
5  keypoint_min_num (the minimum surrounding points for keypoint detection)
6  scale (a scale parameter to balance Euclidean and Feature distance. For example, if actual Euclidean distance between two points is d1, then the distance used to calculate energy function would be d1*scale)
7  p_pre
8  p_ED
9  p_FD
10  m
11 km eps
12 converge_t
13 converge_r
14 output or not output
15 use feature or not
