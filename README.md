# GH-ICP：Iterative Closest Point algorithm with global optimal matching and hybrid metric 
GH-ICP is a robust coarse-to-fine pairwise point cloud registration method. 

Two key innovative points over ICP are: 

1. Global optimal matching (Using Bipartite Graph and KM algorithm)

2. Hybrid metrics (Using Euclidean distance and feature distance at the same time)

The earlier conference version of GH-ICP is called Iterative Global Similarity Point (IGSP).

After the conference, we have improved the original algorithm on its efficiency and robustness. Besides, we've done more experiments on more datasets. 

To highlight two key innovative points of the algorithm, we renamed IGSP as GH-ICP.

### [Video](https://www.youtube.com/watch?v=DZr-8AceSqA)

### [Paper](https://arxiv.org/abs/1808.03899) 
Iterative Global Similarity Points: A robust coarse-to-fine integration solution for pairwise 3D point cloud registration, Yue Pan, Bisheng Yang, Fuxun Liang, Zhen Dong, Accepted by 3DV-2018 

### Citation
If you find our work useful in your research, please consider citing:
        
        @INPROCEEDINGS{yue2018igsp,    
          author={Yue, Pan and Bisheng, Yang and Fuxun, Liang and Zhen, Dong},
          booktitle = {2018 International Conference on 3D Vision (3DV)},
          title={Iterative Global Similarity Points: A robust coarse-to-fine integration solution for pairwise 3D point cloud registration},
          year={2018}
        }

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

> 1  resolution of voxel downsampling (unit:m)

> 2  num_point_bb (the maximum number of keypoints remaining in the bounding box of the target point cloud)

> 3  feature_r (BSC[1] feature calulation radius, unit:m) 

> 4  keypoint_max_ratio (the max curvature for keypoint detection, the larger this value is, the more keypoints would be)

> 5  keypoint_min_num (the minimum surrounding points for keypoint detection)

> 6  scale (a scale parameter to balance Euclidean and Feature distance. For example, if actual Euclidean distance between two points is d1, then the distance used to calculate energy function would be d1*scale)

> 7  p_pre (the first weight paramter p1 in the original paper, the larger this value is, the less points would be matched at the very beginning)

> 8  p_ED (the second weight paramter p2 in the original paper, the smaller this value is, the less points would be matched)

> 9  p_FD (the third weight paramter p3 in the original paper, the smaller this value is, the less points would be matched)

> 10  m (the iteration rate parameter which control the changing rate of the weight of Euclidean and Feature Distance)

> 11 km eps (the terminal threshold of KM algorithm, the larger this value is, the quicker the algorithm would be. However, if this value is too large, the accuracy of this algorithm would be sacrificed.

> 12 converge_t  (the iterative termination condition for translation, unit: m)

> 13 converge_r  (the iterative termination condition for rotation, unit: degree. Only when these two conditions are met at the same time， the algorithm would be seemed as converged)

> 14 output or not output (If this value is 1, then the point cloud of every single iteration would be output. Or there would be no output except for the last one)

> 15 use feature or not (If this value is 1, BSC feature would be extracted and used for the feature distance calculation. Or this algorithm would not consider the hybrid metrics and directly use Euclidean metrics instead.)


[1]BSC feature: Please refer to our previous article：Dong, Z., Yang, B., Liu, Y., Liang, F., Li, B., & Zang, Y., 2017. A novel binary shape context for 3d local surface description. Isprs Journal of Photogrammetry & Remote Sensing, 130, 431-452.

(You can also use FPFH, SHOT... as the feature)
