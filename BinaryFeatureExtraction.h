#ifndef BINARY_FEATRUE_EXTRACTION
#define BINARY_FEATRUE_EXTRACTION


#include "StereoBinaryFeature.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>
#include <xmmintrin.h>
#include <limits>

#include <boost/tuple/tuple.hpp>

#include<pcl\kdtree\kdtree_flann.h>
#include<pcl\point_cloud.h>
#include<pcl\point_types.h>
#include<pcl\common\distances.h>

#include <Eigen\Eigenvalues>
#include "utility.h"


//宏控制
#define SHOW_PROCESS	//是否显示当前提取进度
//#define DISTANCE_WEIGHT	//是否进行距离加权

using namespace std;


class StereoBinaryFeatureExtractor:public StereoBinaryFeature
{
public:
	/************************************************************************/
	/*               目前所用的使用立体格网，二值化为0,1的特征                 */
	/************************************************************************/
	float extract_radius_;				//计算特征的半径大小
	unsigned int voxel_side_num_;		//计算特征的格网个数 voxel_side_num_=N 则格网共有N*N*N
	float unit_side_length_;			//每个格子的格网边长
	vector<float> side_length_thresh_;	//存储边的临界值


	int gridFeatureDimension_;
	int compairFeatureDimensionInEachPlane_;//每一个投影面上比较特征的维数;
	int compairFeatureDimension_;

	///////////////////////////随机格网对的存储//////////////////////////
	vector<pair<int, int>> grid_index_pairs_2d_;
	
	/*存储相关信息的格网*/
	struct GridVoxel
	{
		double point_num;
		float point_weight;
		float intensity;
		float density;
		float normalized_point_weight;
		float average_depth;
		GridVoxel() :point_num(0.0), point_weight(0.0f), intensity(0.0f), density(0.0f), 
			         normalized_point_weight(0.0f), average_depth(0.0f)
		{

		}
	};

	/*构造函数*/
	StereoBinaryFeatureExtractor(float extract_radius,			//特征提取半径
								 unsigned int voxel_side_num,   //划分格子数目 划分数目最好为奇数，避免边缘效应
								 bool build_sample_pattern = false
								 ):	extract_radius_(extract_radius),voxel_side_num_(voxel_side_num)
	{

		gridFeatureDimension_ = 3 * voxel_side_num_*voxel_side_num_;
		compairFeatureDimensionInEachPlane_ = voxel_side_num_*voxel_side_num_;
		compairFeatureDimension_ = 6 * compairFeatureDimensionInEachPlane_;
		

		//计算单位边长
		unit_side_length_=2*extract_radius_/voxel_side_num_;

		
		//计算临界值
		side_length_thresh_.resize(voxel_side_num_);
		for(int i=0;i<voxel_side_num_;i++)
		{
			side_length_thresh_[i]=-extract_radius_+i*unit_side_length_;
		}

		if (build_sample_pattern)
		{
			//比较点对;
			for (int i = 0; i < compairFeatureDimensionInEachPlane_; i++)
			{
				int pair1, pair2;
				do 
				{
					pair1 = rand() % (voxel_side_num_*voxel_side_num_);
					pair2 = rand() % (voxel_side_num_*voxel_side_num_);
				} while (pair1 == pair2 || contain2DPair(pair1, pair2));
				
				grid_index_pairs_2d_.push_back(pair<int, int>(pair1, pair2));
			}

			ofstream fout("sample_pattern.txt");
			cout << "随机采样点对---------------------------------" << endl;
			for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
			{
				fout << grid_index_pairs_2d_[i].first << " " << grid_index_pairs_2d_[i].second << endl;
				//cout << grid_index_pairs_2d_[i].first << " " << grid_index_pairs_2d_[i].second << endl;

			}
			fout.flush();
			fout.close();
		}

		else
		{
			ifstream fin("sample_pattern.txt");
			grid_index_pairs_2d_.resize(compairFeatureDimensionInEachPlane_);
			for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
			{
				fin >> grid_index_pairs_2d_[i].first >> grid_index_pairs_2d_[i].second;
			}
			fin.close();
		}
	}

	

	/*建立每个关键点的局部坐标系（主方向为Z轴正向,第二主方向为X轴正向）,并将变换到该局部坐标系下的点压入result_cloud中;*/
	bool computeLocalCoordinateSystem(const pcXYZIPtr & input_cloud,//输入点云
								      int test_index,    //关键点的索引号
									  const vector<int> & search_indices, //关键点的邻域点索引号
								      CoordinateSystem &localCoordinateSystem);	//局部坐标系的方向;

	void transformPointCloudToLocalSystem(const pcXYZIPtr & input_cloud,//输入点云
								      int test_index,    //关键点的索引号
									  const vector<int> & search_indices, //关键点的邻域点索引号  
									  const CoordinateSystem &localCoordinateSystem,
									  pcXYZIPtr & result_cloud);//输出的已经获得旋转不变性的点云;
	
	/*按照点密度计算的点大小来生成立体格网*/
	void constructCubicGrid(const pcXYZIPtr & rotated_cloud,		//由旋转不变形计算得到的局部点云
						    vector<GridVoxel> & grid);									//结果格网


	//根据立体格网计算三个投影面的投影特征;
	StereoBinaryFeature  computeFeatureProjectedGrid(const vector<GridVoxel> & grid);

	//根据立体格网计算三个投影面的投影特征和两两比较的特征;
	StereoBinaryFeature  computeFeatureProjectedGridAndCompareFeature(const vector<GridVoxel> & grid);

	//根据立体格网计算三个投影面的投影特征和两两比较的特征(每个投影面分别比较);
	StereoBinaryFeature  computeFeatureProjectedGridAndCompareFeature2D(const vector<GridVoxel> & grid);

	/*将生成的格网二值化作为特征返回*/
	StereoBinaryFeature computeFeatureBinarizeGrid(const vector<GridVoxel> & grid);	//输入格网,输出二进制特征;

	//二维格网的随机采样;
	void randomSamplePointPairs();

	/*提取特征函数,提取失败则返回的vector为空*/
	void extractBinaryFeatures(const pcXYZIPtr & input_cloud,	//输入的点云
		                       const pcl::PointIndicesPtr &indices,
		                       doubleVectorSBF & features);

	bool extractBinaryFeatureOfKeypoint(const pcXYZIPtr & input_cloud,	//输入的点云
		                                size_t ptIndex, const std::vector<int> &searchIndexes,
										vectorSBF & feature);
  
private:
	
	inline int getVoxelNum(float coor)
	{
		for(int i=0;i<voxel_side_num_;++i)
		{
			if(coor<side_length_thresh_[i])
				return i-1;
		}
		return voxel_side_num_-1;
	}

	/*
	得到第x,y,z个格子的索引值 从0开始
	*/
	inline int getVoxelIndex(int x, int y, int z)
	{
		return x + y*voxel_side_num_ + z*voxel_side_num_*voxel_side_num_;
	}


	bool contain2DPair(int pair1, int pair2)
	{
		if (grid_index_pairs_2d_.empty() == true)
		{
			return false;
		}
		else
		{
			for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
			{
				if (((pair1 == grid_index_pairs_2d_[i].first) && (pair2 == grid_index_pairs_2d_[i].second))
					|| ((pair1 == grid_index_pairs_2d_[i].second) && (pair2 == grid_index_pairs_2d_[i].first)))
				{
					return true;
				}	
			}
			return false;
		}
		
	}

	//利用2D的pca计算局部邻域分布的特征向量;
	void computeEigenVectorsBy2Dpca(const pcXYZIPtr & input_cloud,//输入点云
		                            const vector<int> & search_indices, //关键点的邻域点索引号
									int test_index,
								    Eigen::Vector3f  &principalDirection);

	bool computeEigenVectorsByWeightPCA(const pcXYZIPtr & input_cloud,//输入点云
		                                const vector<int> & search_indices, //关键点的邻域点索引号
								        int test_index,
								        Eigen::Vector3f  &principalDirection,
								        Eigen::Vector3f  &middleDirection,
								        Eigen::Vector3f  &normalDirection);

	bool computeEigenVectorsByPCA(const pcXYZIPtr & input_cloud,//输入点云
								const vector<int> & search_indices, //关键点的邻域点索引号
								int test_index,
								Eigen::Vector3f  &principalDirection,
								Eigen::Vector3f  &middleDirection,
								Eigen::Vector3f  &normalDirection);

	//计算局部坐标系和场景坐标系的旋转矩阵;
	void computeTranformationMatrixBetweenCoordinateSystems(const CoordinateSystem & coordinate_src,
		                                                    const CoordinateSystem & coordinate_traget,
															Eigen::Matrix4f  & tragetToSource);

	int getNumof1InFeature(const StereoBinaryFeature &feature)
	{
		int count(0);
		for (int i = 0; i < feature.size_; i++)
		{
			int bit_num = i % 8;
			int byte_num = i / 8;
			char test_num = 1 << bit_num;
			if (feature.feature_[byte_num] & test_num)
			{
				count++;
			}

		}
		return count;
	}


	//计算两个点之间的3维空间距离;
	float Comput3DDistanceBetweenPoints(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2)
	{
		float dertax, dertay, dertaz, dis;
		dertax = pt1.x - pt2.x;
		dertay = pt1.y - pt2.y;
		dertaz = pt1.z - pt2.z;
		dis = (dertax)*(dertax)+(dertay)*(dertay)+(dertaz)*(dertaz);
		dis = sqrt(dis);
		return dis;
	}

};

#endif