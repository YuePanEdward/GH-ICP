//
// This file is used for the Voxel Downsampling of Point Cloud.
// Dependent 3rd Libs: PCL (>1.7)  
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//

#ifndef CLOUD_F
#define CLOUD_F

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <vector>
#include <limits>
#include <iostream>

#include "utility.h"

using namespace utility;

template<typename PointT>
class CFilter : public CloudUtility<PointT>
{
public:
    struct IDPair
	{
		IDPair() :idx(0), voxel_idx(0) {}

		unsigned long long voxel_idx;
		unsigned int idx;

		bool operator<(const IDPair& pair) { return voxel_idx < pair.voxel_idx; }
	};
    
    bool voxelfilter(const typename pcl::PointCloud<PointT>::Ptr& cloud_in, typename pcl::PointCloud<PointT>::Ptr& cloud_out, float voxel_size);
    
	bool SORFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, int MeanK, double std);
    
	bool DisFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, double xy_dis_max, double z_min, double z_max);  

    bool ActiveObjectFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, std::vector<Bounds> & active_bbxs);
    
	//Reference: Two-step adaptive extraction method for ground points and breaklines from lidar point clouds, Bisheng Yang, Ronggang Huang, et al. ISPRS Journal of Photogrammetry and Remote Sensing
	bool ExtractGroundPoint(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_ground, typename pcl::PointCloud<PointT>::Ptr & cloud_unground, float grid_resolution, float max_height_difference); // 0.5 , 0.2 
	

private:
    void preprocessing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in,
			double max_x, double max_y, double min_x, double min_y,
			int row, int col, int num_voxel, Voxel* grid, float grid_resolution);

	void processing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in,
			typename pcl::PointCloud<PointT>::Ptr & cloud_ground,
			typename pcl::PointCloud<PointT>::Ptr & cloud_unground,
			Voxel* grid, int num_voxel, float grid_resolution, float max_height_difference);

	void postprocessing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in,
			typename pcl::PointCloud<PointT>::Ptr & cloud_ground,
			typename pcl::PointCloud<PointT>::Ptr & cloud_unground);

};


#endif //CLOUD_F