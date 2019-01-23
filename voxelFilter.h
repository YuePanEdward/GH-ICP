#ifndef VOXEL_F
#define VOXEL_F

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include <vector>
#include <limits>
#include <iostream>

#include "utility.h"
using namespace utility;

template<typename PointT>
class VoxelFilter
{
public:
	typedef typename pcl::PointCloud<PointT>		PointCloud;
	typedef typename pcl::PointCloud<PointT>::Ptr	PointCloudPtr;
	float V_boundingbox;
	float _voxel_size;		//格网大小

	VoxelFilter(float voxel_size):_voxel_size(voxel_size) {}

	struct IDPair
	{
		IDPair() :idx(0), voxel_idx(0) {}

		unsigned long long voxel_idx;
		unsigned int idx;

		//重载比较函数
		bool operator<(const IDPair& pair) { return voxel_idx < pair.voxel_idx; }
	};

	//进行抽稀
	PointCloudPtr filter(const PointCloudPtr& cloud_ptr)
	{
		//voxel大小的倒数
		float inverse_voxel_size = 1.0f / _voxel_size;

		//获取最大最小
		Eigen::Vector4f min_p, max_p;
		pcl::getMinMax3D(*cloud_ptr, min_p, max_p);

		//计算总共的格子数量
		Eigen::Vector4f gap_p;  //boundingbox gap
		gap_p = max_p - min_p;
		
		unsigned long long max_vx = ceil(gap_p.coeff(0)*inverse_voxel_size)+1;
		unsigned long long max_vy = ceil(gap_p.coeff(1)*inverse_voxel_size)+1;
		unsigned long long max_vz = ceil(gap_p.coeff(2)*inverse_voxel_size)+1;
		
		//判定格子数量是否超过给定值
		if (max_vx*max_vy*max_vz >= std::numeric_limits<unsigned long long>::max())
		{
			std::cout << "抽稀失败，最大格子数量过多";
		}

		//计算乘子
		unsigned long long mul_vx = max_vy*max_vz;
		unsigned long long mul_vy = max_vz;
		unsigned long long mul_vz = 1;

		//计算所有点的位置
		std::vector<IDPair> id_pairs(cloud_ptr->size());
		unsigned int idx = 0;
		for (PointCloud::iterator it = cloud_ptr->begin(); it != cloud_ptr->end(); it++)
		{
			//计算编号
			unsigned long long vx = floor((it->x - min_p.coeff(0))*inverse_voxel_size);
			unsigned long long vy = floor((it->y - min_p.coeff(1))*inverse_voxel_size);
			unsigned long long vz = floor((it->z - min_p.coeff(2))*inverse_voxel_size);

			//计算格子编号
			unsigned long long voxel_idx = vx*mul_vx + vy*mul_vy + vz*mul_vz;

			IDPair pair;
			pair.idx = idx;
			pair.voxel_idx = voxel_idx;
			id_pairs.push_back(pair);

			idx++;
		}

		//进行排序
		std::sort(id_pairs.begin(), id_pairs.end());

		//保留每个格子中的一个点
		unsigned int begin_id = 0;
		PointCloudPtr result_ptr(new PointCloud);
		while (begin_id < id_pairs.size())
		{
			//保留第一个点
			result_ptr->push_back(cloud_ptr->points[id_pairs[begin_id].idx]);

			//往后相同格子的点都不保留
			unsigned int compare_id = begin_id + 1;
			while (compare_id < id_pairs.size() && id_pairs[begin_id].voxel_idx == id_pairs[compare_id].voxel_idx)
				compare_id++;
			begin_id = compare_id;
		}
		//计算boundingbox 体积
		//cout << "bounding box: "<<"   x:  "<< gap_p[0] <<"   y:  "<< gap_p[1] <<"  z:  "<< gap_p[2]<<endl;
		V_boundingbox = gap_p[0] * gap_p[1] * gap_p[2];

		return result_ptr;
	}


	bool outputSimplifiedPCD(const std::string &fileName, const pcXYZIPtr &pointCloud)
	{
		if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't write file\n");
			return false;
		}
		return true;
	}
};

#endif //VOXEL_F