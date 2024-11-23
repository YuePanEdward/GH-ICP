#ifndef _INCLUDE_BINARY_FEATRUE_EXTRACTION_HPP
#define _INCLUDE_BINARY_FEATRUE_EXTRACTION_HPP

#include "stereo_binary_feature.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>
#include <sstream>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/registration/transformation_estimation_svd.h>

// #include <concurrent_vector.h>
// #include <ppl.h>

#include <Eigen/Eigenvalues>
#include "utility.h"

#define SHOW_PROCESS //Show current schedule or not
//#define DISTANCE_WEIGHT	//Do distance weighting or not

using namespace std;

namespace ghicp
{

template <typename PointT>
class BSCEncoder : public StereoBinaryFeature
{
  public:
	// BSC grids
	float extract_radius_;			   //radius of feature calculation
	unsigned int voxel_side_num_;	  //number of voxel grid  (voxel_side_num_=N Then there are N*N*N grids in total)
	float unit_side_length_;		   //size of each grid (length)
	vector<float> side_length_thresh_; //store threshold

	int gridFeatureDimension_;
	int compareFeatureDimensionInEachPlane_; //dimension of feature comparation on each projected plane;
	int compareFeatureDimension_;

	// Store the random grid pair
	vector<pair<int, int>> grid_index_pairs_2d_;

	struct GridVoxel
	{
		double point_num;
		float point_weight;
		float density;
		float normalized_point_weight;
		float average_depth;
		float average_intensity;
		GridVoxel() : point_num(0.0), point_weight(0.0f), density(0.0f),
					  normalized_point_weight(0.0f), average_depth(0.0f), average_intensity(0.0f) {}
	};

	/*Constructor*/
	BSCEncoder(float extract_radius,
			   unsigned int voxel_side_num, //num of grids on one side (better to be odd number for avoiding boundary effect)
			   bool build_sample_pattern = false) : extract_radius_(extract_radius), voxel_side_num_(voxel_side_num)
	{

		gridFeatureDimension_ = 3 * voxel_side_num_ * voxel_side_num_;
		compareFeatureDimensionInEachPlane_ = voxel_side_num_ * voxel_side_num_;
		compareFeatureDimension_ = 6 * compareFeatureDimensionInEachPlane_;

		unit_side_length_ = 2 * extract_radius_ / voxel_side_num_;

		//Threshold value
		side_length_thresh_.resize(voxel_side_num_);
		for (int i = 0; i < voxel_side_num_; i++)
		{
			side_length_thresh_[i] = -extract_radius_ + i * unit_side_length_;
		}

		if (build_sample_pattern)
		{
			//Compare grid pair
			for (int i = 0; i < compareFeatureDimensionInEachPlane_; i++)
			{
				int pair1, pair2;
				do
				{
					pair1 = rand() % (voxel_side_num_ * voxel_side_num_);
					pair2 = rand() % (voxel_side_num_ * voxel_side_num_);
				} while (pair1 == pair2 || contain2DPair(pair1, pair2));

				grid_index_pairs_2d_.push_back(pair<int, int>(pair1, pair2));
			}

			ofstream fout("sample_pattern.txt");

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
			grid_index_pairs_2d_.resize(compareFeatureDimensionInEachPlane_);
			for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
			{
				fin >> grid_index_pairs_2d_[i].first >> grid_index_pairs_2d_[i].second;
			}
			fin.close();
		}
	}

	/* Construct local coordinate system (LCS) for each key point (take the primary direction as the Z axis and the second primary direction
	 as the X axis), push the points that has been transformed to the LCS into the result_cloud.*/
	bool computeLocalCoordinateSystem(const typename pcl::PointCloud<PointT>::Ptr &input_cloud, //input point cloud
									  int test_index,											//index of the keypoint
									  const vector<int> &search_indices,						//indices of the keypoint's neighbor points
									  CoordinateSystem &localCoordinateSystem)					//LCS's origin and direction;
	{
		Eigen::Vector3f principalDirectionPca, middleDirectionPca, normalDirectionPca;

		//computeEigenVectorsBy2Dpca(input_cloud, search_indices, test_index, principalDirectionPca); //For 4DOF case

		computeEigenVectorsByWeightPCA(input_cloud, search_indices, test_index, principalDirectionPca, middleDirectionPca, normalDirectionPca);

		//Get LCS (4DOF)
		//localCoordinateSystem.zAxis.x() = 0.0f;
		//localCoordinateSystem.zAxis.y() = 0.0f;
		//localCoordinateSystem.zAxis.z() = 1.0f;

		//Get LCS (6DOF)
		localCoordinateSystem.zAxis.x() = 0.0f;
		localCoordinateSystem.zAxis.y() = 0.0f;
		localCoordinateSystem.zAxis.z() = 0.0f;

		localCoordinateSystem.xAxis = principalDirectionPca;
		localCoordinateSystem.yAxis = middleDirectionPca;
		localCoordinateSystem.zAxis = localCoordinateSystem.xAxis.cross(localCoordinateSystem.yAxis);

		localCoordinateSystem.origin.x() = input_cloud->points[test_index].x;
		localCoordinateSystem.origin.y() = input_cloud->points[test_index].y;
		localCoordinateSystem.origin.z() = input_cloud->points[test_index].z;

		//Normalize axis;
		localCoordinateSystem.xAxis.normalize();
		localCoordinateSystem.yAxis.normalize();

		return true;
	}

	void transformPointCloudToLocalSystem(const typename pcl::PointCloud<PointT>::Ptr &input_cloud, //input point cloud
										  int test_index,											//index of the keypoint
										  const vector<int> &search_indices,						//indices of the keypoint's neighbor points
										  const CoordinateSystem &localCoordinateSystem,			//LCS's origin and direction
										  typename pcl::PointCloud<PointT>::Ptr &result_cloud)		//Output the rotation-variant point cloud under the LCS
	{
		CoordinateSystem coordinate_scene;
		coordinate_scene.xAxis.x() = 1.0f;
		coordinate_scene.xAxis.y() = 0.0f;
		coordinate_scene.xAxis.z() = 0.0f;
		coordinate_scene.yAxis.x() = 0.0f;
		coordinate_scene.yAxis.y() = 1.0f;
		coordinate_scene.yAxis.z() = 0.0f;
		coordinate_scene.zAxis.x() = 0.0f;
		coordinate_scene.zAxis.y() = 0.0f;
		coordinate_scene.zAxis.z() = 1.0f;

		Eigen::Matrix4f tragetToSource;
		computeTranformationMatrixBetweenCoordinateSystems(coordinate_scene, localCoordinateSystem, tragetToSource);

		typename pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>());
		pcl::PointCloud<PointT>().swap(*result_cloud);
		result_cloud->resize(search_indices.size());
		for (int i = 0; i < search_indices.size(); i++)
		{
			PointT pt;
			pt.x = input_cloud->points[search_indices[i]].x - input_cloud->points[test_index].x;
			pt.y = input_cloud->points[search_indices[i]].y - input_cloud->points[test_index].y;
			pt.z = input_cloud->points[search_indices[i]].z - input_cloud->points[test_index].z;
			//pt.intensity = input_cloud->points[search_indices[i]].intensity;

			temp_cloud->points.push_back(pt);
		}

		pcl::transformPointCloud(*temp_cloud, *result_cloud, tragetToSource);
		pcl::PointCloud<PointT>().swap(*temp_cloud);
	}

	/*Generate voxel according to point density*/
	void constructCubicGrid(const typename pcl::PointCloud<PointT>::Ptr &rotated_cloud, //the rotation-variant point cloud under the LCS
							vector<GridVoxel> &grid)									//grid result
	{
		//Used for neighborhood search
		pcl::KdTreeFLANN<pcl::PointXY> tree;
		vector<int> search_indices;
		vector<float> distances;

		float delta = unit_side_length_ * 0.5;

		/*project each point to XOY plane, and then use Gaussian distance weighting to calculate the number of point in each grid.*/
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXY>);
		for (size_t i = 0; i < rotated_cloud->size(); i++)
		{
			pcl::PointXY pt;
			pt.x = rotated_cloud->points[i].x;
			pt.y = rotated_cloud->points[i].y;
			cloud_xy->points.push_back(pt);
		}
		tree.setInputCloud(cloud_xy);

		// For each grid, take its center point to calculate the weighted point number
		for (size_t i = 0; i < voxel_side_num_; i++)
		{
			for (size_t j = 0; j < voxel_side_num_; j++)
			{
				pcl::PointXY pt;

				//grid centroid coordinate
				pt.x = (i + 0.5) * unit_side_length_ - extract_radius_;
				pt.y = (j + 0.5) * unit_side_length_ - extract_radius_;

				//Search neighboring points (r=1.5*unit_side_length) = 3 * bandwidth (delta)
				vector<int>().swap(search_indices);
				vector<float>().swap(distances);
				tree.radiusSearch(pt, 1.5 * unit_side_length_, search_indices, distances);

				if (distances.empty() != true)
				{
					// take exp(-(x-u)*(x-u)* / (2 * delta * delta)) as the weights
					for (size_t n = 0; n < search_indices.size(); n++)
					{
						grid[i + j * voxel_side_num_].point_num += exp(-distances[n] / (2 * delta * delta));
						float depth;
						depth = rotated_cloud->points[search_indices[n]].z + extract_radius_;
						grid[i + j * voxel_side_num_].average_depth += depth * exp(-distances[n] / (2 * delta * delta));
					}
				}
			}
		}

		/*project each point to XOZ plane*/
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_xz(new pcl::PointCloud<pcl::PointXY>);
		for (size_t i = 0; i < rotated_cloud->size(); i++)
		{
			pcl::PointXY pt;
			pt.x = rotated_cloud->points[i].x;
			pt.y = rotated_cloud->points[i].z;
			cloud_xz->points.push_back(pt);
		}
		tree.setInputCloud(cloud_xz);

		for (size_t i = 0; i < voxel_side_num_; i++)
		{
			for (size_t j = 0; j < voxel_side_num_; j++)
			{
				pcl::PointXY pt;
				pt.x = (i + 0.5) * unit_side_length_ - extract_radius_;
				pt.y = (j + 0.5) * unit_side_length_ - extract_radius_;
				tree.radiusSearch(pt, 1.5 * unit_side_length_, search_indices, distances);
				if (distances.empty() != true)
				{
					for (size_t n = 0; n < search_indices.size(); n++)
					{
						grid[i + j * voxel_side_num_ + voxel_side_num_ * voxel_side_num_].point_num += exp(-distances[n] / (2 * delta * delta));
						float depth;
						depth = rotated_cloud->points[search_indices[n]].y + extract_radius_;
						grid[i + j * voxel_side_num_ + voxel_side_num_ * voxel_side_num_].average_depth += depth * exp(-distances[n] / (2 * delta * delta));
					}
				}
			}
		}

		/*project each point to YOZ plane*/
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_yz(new pcl::PointCloud<pcl::PointXY>);
		for (size_t i = 0; i < rotated_cloud->size(); i++)
		{
			pcl::PointXY pt;
			pt.x = rotated_cloud->points[i].y;
			pt.y = rotated_cloud->points[i].z;
			cloud_yz->points.push_back(pt);
		}
		tree.setInputCloud(cloud_yz);

		for (size_t i = 0; i < voxel_side_num_; i++)
		{
			for (size_t j = 0; j < voxel_side_num_; j++)
			{
				pcl::PointXY pt;
				float delta;
				delta = unit_side_length_ / 2;
				pt.x = (i + 0.5) * unit_side_length_ - extract_radius_;
				pt.y = (j + 0.5) * unit_side_length_ - extract_radius_;
				tree.radiusSearch(pt, 1.5 * unit_side_length_, search_indices, distances);
				if (distances.empty() != true)
				{
					for (size_t n = 0; n < search_indices.size(); n++)
					{
						grid[i + j * voxel_side_num_ + 2 * voxel_side_num_ * voxel_side_num_].point_num += exp(-distances[n] / (2 * delta * delta));
						float depth;
						depth = rotated_cloud->points[search_indices[n]].x + extract_radius_;
						grid[i + j * voxel_side_num_ + 2 * voxel_side_num_ * voxel_side_num_].average_depth += depth * exp(-distances[n] / (2 * delta * delta));
					}
				}
			}
		}

		/*Do not consider the boundary effect;*/
		/*for (size_t i = 0; i < rotated_cloud->size(); i++)
	{
		x = getVoxelNum(rotated_cloud->points[i].x);
		y = getVoxelNum(rotated_cloud->points[i].y);
		index = x + y*voxel_side_num_ ;
		grid[index].point_num++;
	}

	for (size_t i = 0; i < rotated_cloud->size(); i++)
	{
		x = getVoxelNum(rotated_cloud->points[i].x);
		z = getVoxelNum(rotated_cloud->points[i].z);
		index = x + z*voxel_side_num_ + voxel_side_num_*voxel_side_num_;
		grid[index].point_num++;
	}

	    for (size_t i = 0; i < rotated_cloud->size(); i++)
	{
		y = getVoxelNum(rotated_cloud->points[i].y);
		z = getVoxelNum(rotated_cloud->points[i].z);
		index = y + z*voxel_side_num_ + 2 * voxel_side_num_*voxel_side_num_;
		grid[index].point_num++;
	}*/

		//Calculate the normalized weight of each grid (Ng/Sg)/(Nn/Sn)
		//Ng, Nn: point number in the grid and its local neighborhood.
		//Sg, Sn: area of the grid and its local neighborhood.
		float grid_density, grid_area, neighbourhood_density, neighbourhood_area;

		//Calculate local density;
		neighbourhood_area = M_PI * extract_radius_ * extract_radius_;
		neighbourhood_density = rotated_cloud->size() / neighbourhood_area;

		for (size_t i = 0; i < grid.size(); i++)
		{
			//Calculate the average depth of each grid
			if (grid[i].point_num == 0.0)
			{
				grid[i].average_depth = 0.0f;
			}
			else
			{
				grid[i].average_depth /= grid[i].point_num;
			}

			//Calculate the point density of each grid
			grid_area = unit_side_length_ * unit_side_length_;
			grid_density = grid[i].point_num / grid_area;

			//Calculate the normalized weight of each grid
			if (neighbourhood_density != 0.0f)
			{
				grid[i].normalized_point_weight = grid_density / neighbourhood_density;
			}
			else
			{
				grid[i].normalized_point_weight = 0.0f;
			}
		}
	}

	//Calculate the feature of 3 projection planes according to the voxel
	StereoBinaryFeature computeFeatureProjectedGrid(const vector<GridVoxel> &grid)
	{
		float normalized_point_weightT = 0.1;
		StereoBinaryFeature result_feature(gridFeatureDimension_);

		for (int i = 0; i < grid.size(); i++)
		{
			if (grid[i].normalized_point_weight > normalized_point_weightT)
			{
				int bit_num = i % 8;
				int byte_num = i / 8;
				char test_num = 1 << bit_num;
				result_feature.feature_[byte_num] |= test_num;
			}
		}
		return result_feature;
	}

	//Calculate the feature of 3 projection planes according to the voxel and the pairwise comparison feature
	StereoBinaryFeature computeFeatureProjectedGridAndCompareFeature(const vector<GridVoxel> &grid)
	{
		float normalized_point_weightT = 0.1;
		StereoBinaryFeature result_feature(gridFeatureDimension_ + compareFeatureDimension_);

		for (int i = 0; i < grid.size(); i++)
		{
			if (grid[i].normalized_point_weight > normalized_point_weightT)
			{
				int bit_num = i % 8;
				int byte_num = i / 8;
				char test_num = 1 << bit_num;
				result_feature.feature_[byte_num] |= test_num;
			}
		}

		/*Grid point density comparison feature*/

		//Calculate the standard deviation and mean of the density variation of all the compared grid pair
		double average(0.0), variance(0.0), standardDeviation(0.0), x(0.0);
		for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
		{
			x = grid[grid_index_pairs_2d_[i].first].normalized_point_weight - grid[grid_index_pairs_2d_[i].second].normalized_point_weight;
			average += x;
		}
		average /= grid_index_pairs_2d_.size();

		for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
		{
			x = grid[grid_index_pairs_2d_[i].first].normalized_point_weight - grid[grid_index_pairs_2d_[i].second].normalized_point_weight;
			variance += (x - average) * (x - average);
		}
		variance /= grid_index_pairs_2d_.size();
		standardDeviation = sqrt(variance);

		for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
		{
			//If the two grids are both vacant, then the value should be 0; if one of them is vacant, then the value should be 1.
			if ((grid[grid_index_pairs_2d_[i].first].normalized_point_weight > normalized_point_weightT && grid[grid_index_pairs_2d_[i].second].normalized_point_weight < normalized_point_weightT) || (grid[grid_index_pairs_2d_[i].first].normalized_point_weight < normalized_point_weightT && grid[grid_index_pairs_2d_[i].second].normalized_point_weight > normalized_point_weightT))
			{
				x = grid[grid_index_pairs_2d_[i].first].normalized_point_weight - grid[grid_index_pairs_2d_[i].second].normalized_point_weight;
				int k = i + (int)grid.size();
				int bit_num = k % 8;
				int byte_num = k / 8;
				char test_num = 1 << bit_num;
				result_feature.feature_[byte_num] |= test_num;
			}

			// If neither of the grids are vacant, then compare them
			if (grid[grid_index_pairs_2d_[i].first].normalized_point_weight > normalized_point_weightT && grid[grid_index_pairs_2d_[i].second].normalized_point_weight > normalized_point_weightT)
			{
				x = grid[grid_index_pairs_2d_[i].first].normalized_point_weight - grid[grid_index_pairs_2d_[i].second].normalized_point_weight;

				// if the difference of density is larger than its mean + 1 sigma -> then its feature value would be 1, or it would be 0
				if (abs(x - average) > standardDeviation)
				{
					int k = i + (int)grid.size();
					int bit_num = k % 8;
					int byte_num = k / 8;
					char test_num = 1 << bit_num;
					result_feature.feature_[byte_num] |= test_num;
				}
			}
		}

		return result_feature;
	}

	//Calculate the feature of 3 projection planes according to the voxel and the pairwise comparison feature (compare with each projection plane)
	StereoBinaryFeature computeFeatureProjectedGridAndCompareFeature2D(const vector<GridVoxel> &grid)
	{
		float normalized_point_weightT = 0.1;
		StereoBinaryFeature result_feature(gridFeatureDimension_ + compareFeatureDimension_);
		int featureDimenshions = 0;

		for (int i = 0; i < grid.size(); i++)
		{
			if (grid[i].normalized_point_weight > normalized_point_weightT)
			{
				int bit_num = i % 8;
				int byte_num = i / 8;
				char test_num = 1 << bit_num;
				result_feature.feature_[byte_num] |= test_num;
			}
			featureDimenshions++;
		}

		//depth and density
		double average_depth(0.0), variance_depth(0.0);
		double standardDeviation_depth(0.0), depth(0.0);
		double average_density(0.0), variance_density(0.0);
		double standardDeviation_density(0.0), density(0.0);
		int offset = 0;

		//Calculate difference of density and depth and calculate their mean and standard deviation
		for (int nn = 0; nn < 3; nn++)
		{
			average_depth = 0.0;
			variance_depth = 0.0;
			standardDeviation_depth = 0.0;
			depth = 0.0;
			average_density = 0.0;
			variance_density = 0.0;
			standardDeviation_density = 0.0;
			density = 0.0;

			for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
			{
				depth = grid[grid_index_pairs_2d_[i].first + offset].average_depth - grid[grid_index_pairs_2d_[i].second + offset].average_depth;
				average_depth += depth;

				density = grid[grid_index_pairs_2d_[i].first + offset].normalized_point_weight - grid[grid_index_pairs_2d_[i].second + offset].normalized_point_weight;
				average_density += density;
			}
			average_depth /= grid_index_pairs_2d_.size();
			average_density /= grid_index_pairs_2d_.size();

			for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
			{
				depth = grid[grid_index_pairs_2d_[i].first + offset].average_depth - grid[grid_index_pairs_2d_[i].second + offset].average_depth;
				variance_depth += (depth - average_depth) * (depth - average_depth);

				density = grid[grid_index_pairs_2d_[i].first + offset].normalized_point_weight - grid[grid_index_pairs_2d_[i].second + offset].normalized_point_weight;
				variance_density += (density - average_density) * (density - average_density);
			}
			variance_depth /= grid_index_pairs_2d_.size();
			standardDeviation_depth = sqrt(variance_depth);

			variance_density /= grid_index_pairs_2d_.size();
			standardDeviation_density = sqrt(variance_density);

			for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
			{

				// Deduce the feature value from the difference of depth
				depth = grid[grid_index_pairs_2d_[i].first + offset].average_depth - grid[grid_index_pairs_2d_[i].second + offset].average_depth;
				if (abs(depth - average_depth) > standardDeviation_depth)
				{
					int k = featureDimenshions;
					int bit_num = k % 8;
					int byte_num = k / 8;
					char test_num = 1 << bit_num;
					result_feature.feature_[byte_num] |= test_num;
					//cout << 1 << endl;
				}
				featureDimenshions++;

				// Deduce the feature value from the difference of density
				if (grid[grid_index_pairs_2d_[i].first].normalized_point_weight < normalized_point_weightT && grid[grid_index_pairs_2d_[i].second].normalized_point_weight < normalized_point_weightT)
				{
					// Then the value would be 0
				}
				else
				{
					density = grid[grid_index_pairs_2d_[i].first + offset].normalized_point_weight - grid[grid_index_pairs_2d_[i].second + offset].normalized_point_weight;
					if (abs(density - average_density) > standardDeviation_density)
					{
						int k = featureDimenshions;
						int bit_num = k % 8;
						int byte_num = k / 8;
						char test_num = 1 << bit_num;
						result_feature.feature_[byte_num] |= test_num;
					}
				}
				featureDimenshions++;
			}

			offset += (voxel_side_num_ * voxel_side_num_);
		}
		return result_feature;
	}

	//Binarize the feature
	StereoBinaryFeature computeFeatureBinarizeGrid(const vector<GridVoxel> &grid)
	{
		StereoBinaryFeature result_feature(grid.size());

		for (int i = 0; i < grid.size(); i++)
		{
			if (grid[i].normalized_point_weight > 0.2)
			{
				int bit_num = i % 8;
				int byte_num = i / 8;
				char test_num = 1 << bit_num;
				result_feature.feature_[byte_num] |= test_num;
			}
		}
		return result_feature;
	}

	//Random sampling of grid
	void randomSamplePointPairs()
	{
		//128 pairs
		for (int i = 0; i < voxel_side_num_ * voxel_side_num_ * 2; i++)
		{
			int pair1 = rand() % voxel_side_num_ * voxel_side_num_;
			int pair2 = rand() % voxel_side_num_ * voxel_side_num_;
			while (pair1 == pair2 || contain2DPair(pair1, pair2))
			{
				pair1 = rand() % voxel_side_num_ * voxel_side_num_;
				pair2 = rand() % voxel_side_num_ * voxel_side_num_;
			}
			grid_index_pairs_2d_.push_back(pair<int, int>(pair1, pair2));
		}
	}

	//main entrance
	void extractBinaryFeatures(const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
							   const pcl::PointIndicesPtr &indices, int dof_type,
							   doubleVectorSBF &bscFeatures)
	{
		size_t feature_num;
		if (indices->indices.size() > 0)
		{
			feature_num = indices->indices.size();
		}
		else
		{
			cout << "The input indice is NaN\n";
			return;
		}

		//Initialization
		vector<StereoBinaryFeature> features_coor0(feature_num), features_coor1(feature_num),
			features_coor2(feature_num), features_coor3(feature_num);

		//KD Tree indexing
		pcl::KdTreeFLANN<PointT> tree;
		tree.setInputCloud(input_cloud);

		//concurrency
		//concurrency::parallel_for(size_t(0), feature_num, [&](size_t i) //for each keypoint
		for (size_t i = 0; i < feature_num; i++)
		{
			vector<StereoBinaryFeature> features;
			vector<StereoBinaryFeature>().swap(features);

			vector<int> searchIndexes;
			vector<float> distances;

			vector<float>().swap(distances);
			vector<int>().swap(searchIndexes);

			size_t ptIndex;
			ptIndex = indices->indices[i];
			tree.radiusSearch(ptIndex, sqrt(3.0) * extract_radius_, searchIndexes, distances);

			extractBinaryFeatureOfKeypoint(input_cloud, ptIndex, searchIndexes, dof_type, features); // For each keypoint , extract the features

			features_coor0[i] = features[0];
			features_coor0[i].keypointIndex_ = i;

			if (dof_type > 0)
			{
				features_coor1[i] = features[1];
				features_coor1[i].keypointIndex_ = i;

				if (dof_type > 4)
				{
					features_coor2[i] = features[2];
					features_coor2[i].keypointIndex_ = i;

					features_coor3[i] = features[3];
					features_coor3[i].keypointIndex_ = i;
				}
			}
		}
		//);

		bscFeatures.push_back(features_coor0);
		bscFeatures.push_back(features_coor1);
		bscFeatures.push_back(features_coor2);
		bscFeatures.push_back(features_coor3);

		vector<StereoBinaryFeature>().swap(features_coor0);
		vector<StereoBinaryFeature>().swap(features_coor1);
        vector<StereoBinaryFeature>().swap(features_coor2);
		vector<StereoBinaryFeature>().swap(features_coor3);

		cout << "Extract BSC feature done." << endl;
	}

	bool ReArrangeGrid(vector<GridVoxel> &grid_a, vector<GridVoxel> &grid_b, int tran_xy, int tran_xz, int tran_yz)
	{
		int size_vector = grid_a.size();
		int size_one_plane = voxel_side_num_ * voxel_side_num_;

		vector<GridVoxel> grid_a_xy(grid_a.begin(), grid_a.begin() + size_one_plane);
		vector<GridVoxel> grid_a_xz(grid_a.begin() + size_one_plane, grid_a.begin() + 2 * size_one_plane);
		vector<GridVoxel> grid_a_yz(grid_a.begin() + 2 * size_one_plane, grid_a.begin() + 3 * size_one_plane);

		vector<GridVoxel> grid_b_xy, grid_b_xz, grid_b_yz;

		ReArrange_2D(grid_a_xy, grid_b_xy, tran_xy);
		ReArrange_2D(grid_a_xz, grid_b_xz, tran_xz);
		ReArrange_2D(grid_a_yz, grid_b_yz, tran_yz);

		grid_b.insert(grid_b.end(), grid_b_xy.begin(), grid_b_xy.end());
		grid_b.insert(grid_b.end(), grid_b_xz.begin(), grid_b_xz.end());
		grid_b.insert(grid_b.end(), grid_b_yz.begin(), grid_b_yz.end());

		return 1;
	}

	//1
	bool ReArrange_reverse_all(vector<GridVoxel> &grid_2d_a, vector<GridVoxel> &grid_2d_b)
	{
		grid_2d_b.resize(grid_2d_a.size());
		for (int k = 0; k < grid_2d_a.size(); k++)
		{
			grid_2d_b[k] = grid_2d_a[compareFeatureDimensionInEachPlane_ - 1 - k];
		}
		return 1;
	}

	//2
	bool ReArrange_reverse_sym_2(vector<GridVoxel> &grid_2d_a, vector<GridVoxel> &grid_2d_b)
	{
		grid_2d_b.resize(grid_2d_a.size());
		int i, j, original_k;
		for (int k = 0; k < grid_2d_a.size(); k++)
		{
			i = k / voxel_side_num_;
			j = k % voxel_side_num_;
			original_k = (voxel_side_num_ - 1 - i) * voxel_side_num_ + j;
			grid_2d_b[k] = grid_2d_a[original_k];
		}
		return 1;
	}

	//3
	bool ReArrange_reverse_sym_1(vector<GridVoxel> &grid_2d_a, vector<GridVoxel> &grid_2d_b)
	{
		grid_2d_b.resize(grid_2d_a.size());
		int i, j, original_k;
		for (int k = 0; k < grid_2d_a.size(); k++)
		{
			i = k / voxel_side_num_;
			j = k % voxel_side_num_;
			original_k = i * voxel_side_num_ + voxel_side_num_ - 1 - j;
			grid_2d_b[k] = grid_2d_a[original_k];
		}
		return 1;
	}

	bool ReArrange_2D(vector<GridVoxel> &grid_2d_a, vector<GridVoxel> &grid_2d_b, int tran_type)
	{
		switch (tran_type)
		{
		case 1:
			ReArrange_reverse_all(grid_2d_a, grid_2d_b);
			break;
		case 2:
			ReArrange_reverse_sym_2(grid_2d_a, grid_2d_b);
			break;
		case 3:
			ReArrange_reverse_sym_1(grid_2d_a, grid_2d_b);
			break;
		default:
			return 0;
		}
		return 1;
	}

	//entrance for each keypoint
	//4DOF -> 6DOF
	bool extractBinaryFeatureOfKeypoint(const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
										size_t ptIndex, const std::vector<int> &searchIndexes, int dof_type,
										vectorSBF &features)
	{
		StereoBinaryFeature feature;

		if (searchIndexes.empty())
		{
			cout << "Fail to find point in neighborhood" << endl;
			features.push_back(feature);
			features.push_back(feature);
			return false;
		}

		//Fixed coordinate system
		CoordinateSystem localSystem1;
		typename pcl::PointCloud<PointT>::Ptr result_cloud(new pcl::PointCloud<PointT>);
		computeLocalCoordinateSystem(input_cloud, ptIndex, searchIndexes, localSystem1);
		transformPointCloudToLocalSystem(input_cloud, ptIndex, searchIndexes, localSystem1, result_cloud);

		vector<GridVoxel> grid_1(gridFeatureDimension_);
		constructCubicGrid(result_cloud, grid_1);						  //3* 2D Grid (on XOY, XOZ, YOZ plane)
		feature = computeFeatureProjectedGridAndCompareFeature2D(grid_1); // Calculate the binary feature based on weighted depth and density difference on 3 project plane
		feature.localSystem_.xAxis = localSystem1.xAxis;
		feature.localSystem_.yAxis = localSystem1.yAxis;
		feature.localSystem_.zAxis = localSystem1.zAxis;
		feature.localSystem_.origin = localSystem1.origin;
		features.push_back(feature);

		if (dof_type > 0)
		{
			//reverse X and Y axis and keep Z axis (new coordiante system used for 4DOF matching), 2 features/CS in total
			vector<GridVoxel> grid_2(gridFeatureDimension_);
			ReArrangeGrid(grid_1, grid_2, 1, 2, 2);
			feature = computeFeatureProjectedGridAndCompareFeature2D(grid_2);
			feature.localSystem_.xAxis = -localSystem1.xAxis;
			feature.localSystem_.yAxis = -localSystem1.yAxis;
			feature.localSystem_.zAxis = localSystem1.zAxis;
			feature.localSystem_.origin = localSystem1.origin;
			features.push_back(feature);

			vector<GridVoxel>(gridFeatureDimension_).swap(grid_2);

			if (dof_type > 4) //(new coordiante system used for 6DOF matching), 4 features/CS in total
			{
				vector<GridVoxel> grid_3(gridFeatureDimension_);
				ReArrangeGrid(grid_1, grid_3, 3, 2, 1);
				feature = computeFeatureProjectedGridAndCompareFeature2D(grid_3);
				feature.localSystem_.xAxis = localSystem1.xAxis;
				feature.localSystem_.yAxis = -localSystem1.yAxis;
				feature.localSystem_.zAxis = -localSystem1.zAxis;
				feature.localSystem_.origin = localSystem1.origin;
				features.push_back(feature);

				vector<GridVoxel> grid_4(gridFeatureDimension_);
				ReArrangeGrid(grid_1, grid_4, 2, 1, 3);
				feature = computeFeatureProjectedGridAndCompareFeature2D(grid_4);
				feature.localSystem_.xAxis = -localSystem1.xAxis;
				feature.localSystem_.yAxis = localSystem1.yAxis;
				feature.localSystem_.zAxis = -localSystem1.zAxis;
				feature.localSystem_.origin = localSystem1.origin;
				features.push_back(feature);

				vector<GridVoxel>(gridFeatureDimension_).swap(grid_3);
				vector<GridVoxel>(gridFeatureDimension_).swap(grid_4);
			}
		}

		//Clear grid and result cloud
		vector<GridVoxel>(gridFeatureDimension_).swap(grid_1);
		result_cloud->points.clear();

		pcl::PointCloud<PointT>().swap(*result_cloud);

		return true;
	}

  private:
	inline int getVoxelNum(float coor)
	{
		for (int i = 0; i < voxel_side_num_; ++i)
		{
			if (coor < side_length_thresh_[i])
				return i - 1;
		}
		return voxel_side_num_ - 1;
	}

	inline int getVoxelIndex(int x, int y, int z)
	{
		return x + y * voxel_side_num_ + z * voxel_side_num_ * voxel_side_num_;
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
				if (((pair1 == grid_index_pairs_2d_[i].first) && (pair2 == grid_index_pairs_2d_[i].second)) || ((pair1 == grid_index_pairs_2d_[i].second) && (pair2 == grid_index_pairs_2d_[i].first)))
				{
					return true;
				}
			}
			return false;
		}
	}

	//Do 2D PCA
	void computeEigenVectorsBy2Dpca(const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
									const vector<int> &search_indices,
									int test_index,
									Eigen::Vector3f &principalDirection)
	{
		if (search_indices.size() < 3)
			return;

		double radius;
		radius = sqrt(2.0) * extract_radius_;

		double center_x(0.0), center_y(0.0), center_z(0.0), dis_all(0.0);
		for (size_t it = 0; it < search_indices.size(); ++it)
		{
			center_x += input_cloud->points[search_indices[it]].x;
			center_y += input_cloud->points[search_indices[it]].y;
			center_z += input_cloud->points[search_indices[it]].z;

			dis_all += (radius - Comput3DDistanceBetweenPoints(input_cloud->points[search_indices[it]], input_cloud->points[test_index]));
		}
		center_x /= search_indices.size();
		center_y /= search_indices.size();
		center_z /= search_indices.size();

		Eigen::Matrix<float, 2, 2> covariance;
		covariance = Eigen::Matrix<float, 2, 2>::Zero(2, 2);
		covariance(0, 0) = 0.0f;
		covariance(0, 1) = 0.0f;
		covariance(1, 0) = 0.0f;
		covariance(1, 1) = 0.0f;

		for (size_t it = 0; it < search_indices.size(); ++it)
		{
			float dis, weight;
			dis = Comput3DDistanceBetweenPoints(input_cloud->points[search_indices[it]], input_cloud->points[test_index]);
			weight = radius - dis;
			covariance(0, 0) += weight * (input_cloud->points[search_indices[it]].x - center_x) * (input_cloud->points[search_indices[it]].x - center_x);
			covariance(0, 1) += weight * (input_cloud->points[search_indices[it]].x - center_x) * (input_cloud->points[search_indices[it]].y - center_y);
			covariance(1, 0) += weight * (input_cloud->points[search_indices[it]].x - center_x) * (input_cloud->points[search_indices[it]].y - center_y);
			covariance(1, 1) += weight * (input_cloud->points[search_indices[it]].y - center_y) * (input_cloud->points[search_indices[it]].y - center_y);
		}
		covariance(0, 0) /= dis_all;
		covariance(0, 1) /= dis_all;
		covariance(1, 0) /= dis_all;
		covariance(1, 1) /= dis_all;

		Eigen::EigenSolver<Eigen::Matrix2f> es(covariance, true);
		Eigen::EigenSolver<Eigen::Matrix2f>::EigenvalueType ev = es.eigenvalues();
		Eigen::EigenSolver<Eigen::Matrix2f>::EigenvectorsType evc = es.eigenvectors();

		if (ev(0).real() > ev(1).real())
		{
			principalDirection.x() = evc.col(0).x().real();
			principalDirection.y() = evc.col(0).y().real();
			principalDirection.z() = 0.0f;
		}
		else
		{
			principalDirection.x() = evc.col(1).x().real();
			principalDirection.y() = evc.col(1).y().real();
			principalDirection.z() = 0.0f;
		}
	}

	//Do weighted 3D PCA
	bool computeEigenVectorsByWeightPCA(const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
										const vector<int> &search_indices,
										int test_index,
										Eigen::Vector3f &principalDirection,
										Eigen::Vector3f &middleDirection,
										Eigen::Vector3f &normalDirection)
	{
		if (search_indices.size() < 3)
			return false;

		double radius;
		radius = sqrt(2.0) * extract_radius_;

		//Calculate the coordinate of center point.
		//Take the radius- [distance to the search point] as the weight
		double center_x(0.0), center_y(0.0), center_z(0.0), dis_all(0.0);
		for (size_t it = 0; it < search_indices.size(); ++it)
		{
			center_x += input_cloud->points[search_indices[it]].x;
			center_y += input_cloud->points[search_indices[it]].y;
			center_z += input_cloud->points[search_indices[it]].z;

			dis_all += (radius - Comput3DDistanceBetweenPoints(input_cloud->points[search_indices[it]], input_cloud->points[test_index]));
		}
		center_x /= search_indices.size();
		center_y /= search_indices.size();
		center_z /= search_indices.size();

		//Calculate covariance matrix
		Eigen::Matrix<float, 3, 3> covariance;
		covariance = Eigen::Matrix<float, 3, 3>::Zero(3, 3);

		for (size_t it = 0; it < search_indices.size(); ++it)
		{
			float dis, weight;
			dis = Comput3DDistanceBetweenPoints(input_cloud->points[search_indices[it]], input_cloud->points[test_index]);
			weight = radius - dis;
			covariance(0, 0) += weight * (input_cloud->points[search_indices[it]].x - center_x) * (input_cloud->points[search_indices[it]].x - center_x);
			covariance(0, 1) += weight * (input_cloud->points[search_indices[it]].x - center_x) * (input_cloud->points[search_indices[it]].y - center_y);
			covariance(0, 2) += weight * (input_cloud->points[search_indices[it]].x - center_x) * (input_cloud->points[search_indices[it]].z - center_z);

			covariance(1, 0) += weight * (input_cloud->points[search_indices[it]].y - center_y) * (input_cloud->points[search_indices[it]].x - center_x);
			covariance(1, 1) += weight * (input_cloud->points[search_indices[it]].y - center_y) * (input_cloud->points[search_indices[it]].y - center_y);
			covariance(1, 2) += weight * (input_cloud->points[search_indices[it]].y - center_y) * (input_cloud->points[search_indices[it]].z - center_z);

			covariance(2, 0) += weight * (input_cloud->points[search_indices[it]].z - center_z) * (input_cloud->points[search_indices[it]].x - center_x);
			covariance(2, 1) += weight * (input_cloud->points[search_indices[it]].z - center_z) * (input_cloud->points[search_indices[it]].y - center_y);
			covariance(2, 2) += weight * (input_cloud->points[search_indices[it]].z - center_z) * (input_cloud->points[search_indices[it]].z - center_z);
		}
		covariance /= dis_all;

		//Calculate eigen value
		Eigen::EigenSolver<Eigen::Matrix3f> es(covariance, true);
		//Get eigen value and vector
		Eigen::EigenSolver<Eigen::Matrix3f>::EigenvalueType ev = es.eigenvalues();
		Eigen::EigenSolver<Eigen::Matrix3f>::EigenvectorsType evc = es.eigenvectors();

		if (es.info() == Eigen::Success)
		{
			float MaxEigenValue = ev(0).real();
			float MinEigenValue = ev(0).real();
			unsigned int MaxEigenValueId(0), MinEigenValueId(0);

			for (int i = 0; i < 3; i++)
			{
				if (ev(i).real() > MaxEigenValue)
				{
					MaxEigenValueId = i;
					MaxEigenValue = ev(i).real();
				}

				if (ev(i).real() < MinEigenValue)
				{
					MinEigenValueId = i;
					MinEigenValue = ev(i).real();
				}
			}

			principalDirection.x() = evc.col(MaxEigenValueId).x().real();
			principalDirection.y() = evc.col(MaxEigenValueId).y().real();
			principalDirection.z() = evc.col(MaxEigenValueId).z().real();

			normalDirection.x() = evc.col(MinEigenValueId).x().real();
			normalDirection.y() = evc.col(MinEigenValueId).y().real();
			normalDirection.z() = evc.col(MinEigenValueId).z().real();

			middleDirection = principalDirection.cross(normalDirection);
		}
		else
		{
			cout << "Fail to solve" << endl;
			return false;
		}

		return true;
	}

#if 0
	//Do 3D PCA (with the assist of OpenCV)
	bool computeEigenVectorsByPCA(const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
								  const vector<int> &search_indices,
								  int test_index,
								  Eigen::Vector3f &principalDirection,
								  Eigen::Vector3f &middleDirection,
								  Eigen::Vector3f &normalDirection)
	{
		if (search_indices.size() < 3)
			return false;

		CvMat *pData = cvCreateMat((int)search_indices.size(), 3, CV_32FC1);
		CvMat *pMean = cvCreateMat(1, 3, CV_32FC1);
		CvMat *pEigVals = cvCreateMat(1, 3, CV_32FC1);
		CvMat *pEigVecs = cvCreateMat(3, 3, CV_32FC1);

		for (size_t i = 0; i < search_indices.size(); ++i)
		{
			cvmSet(pData, (int)i, 0, input_cloud->points[search_indices[i]].x);
			cvmSet(pData, (int)i, 1, input_cloud->points[search_indices[i]].y);
			cvmSet(pData, (int)i, 2, input_cloud->points[search_indices[i]].z);
		}
		cvCalcPCA(pData, pMean, pEigVals, pEigVecs, CV_PCA_DATA_AS_ROW);

		principalDirection.x() = cvmGet(pEigVecs, 0, 0);
		principalDirection.y() = cvmGet(pEigVecs, 0, 1);
		principalDirection.z() = cvmGet(pEigVecs, 0, 2);

		//middleDirection.x() = cvmGet(pEigVecs, 1, 0);
		//middleDirection.y() = cvmGet(pEigVecs, 1, 1);
		//middleDirection.z() = cvmGet(pEigVecs, 1, 2);

		normalDirection.x() = cvmGet(pEigVecs, 2, 0);
		normalDirection.y() = cvmGet(pEigVecs, 2, 1);
		normalDirection.z() = cvmGet(pEigVecs, 2, 2);

		middleDirection = principalDirection.cross(normalDirection);

		cvReleaseMat(&pEigVecs);
		cvReleaseMat(&pEigVals);
		cvReleaseMat(&pMean);
		cvReleaseMat(&pData);

		return true;
	}
#endif

	void computeTranformationMatrixBetweenCoordinateSystems(const CoordinateSystem &coordinate_src,
															const CoordinateSystem &coordinate_traget,
															Eigen::Matrix4f &tragetToSource)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud_src, cloud_target;
		pcl::PointXYZ pt;

		pt.x = coordinate_src.xAxis.x();
		pt.y = coordinate_src.xAxis.y();
		pt.z = coordinate_src.xAxis.z();
		cloud_src.points.push_back(pt);

		pt.x = coordinate_src.yAxis.x();
		pt.y = coordinate_src.yAxis.y();
		pt.z = coordinate_src.yAxis.z();
		cloud_src.points.push_back(pt);

		pt.x = coordinate_src.zAxis.x();
		pt.y = coordinate_src.zAxis.y();
		pt.z = coordinate_src.zAxis.z();
		cloud_src.points.push_back(pt);

		pt.x = coordinate_traget.xAxis.x();
		pt.y = coordinate_traget.xAxis.y();
		pt.z = coordinate_traget.xAxis.z();
		cloud_target.points.push_back(pt);

		pt.x = coordinate_traget.yAxis.x();
		pt.y = coordinate_traget.yAxis.y();
		pt.z = coordinate_traget.yAxis.z();
		cloud_target.points.push_back(pt);

		pt.x = coordinate_traget.zAxis.x();
		pt.y = coordinate_traget.zAxis.y();
		pt.z = coordinate_traget.zAxis.z();
		cloud_target.points.push_back(pt);

		/*Construct correspondences;*/
		pcl::Correspondences correspondences;
		pcl::Correspondence correspondence;
		for (size_t i = 0; i < 3; i++)
		{
			correspondence.index_match = (int)i;
			correspondence.index_query = (int)i;
			correspondences.push_back(correspondence);
		}

		//Use SVD to calculate transformation matrix;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
		Eigen::Matrix4f trans;
		trans_est.estimateRigidTransformation(cloud_src, cloud_target, correspondences, trans);
		tragetToSource = trans.inverse();
	}

	//count feature value of 1
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

	float Comput3DDistanceBetweenPoints(const PointT &pt1, const PointT &pt2)
	{
		float dertax, dertay, dertaz, dis;
		dertax = pt1.x - pt2.x;
		dertay = pt1.y - pt2.y;
		dertaz = pt1.z - pt2.z;
		dis = (dertax) * (dertax) + (dertay) * (dertay) + (dertaz) * (dertaz);
		dis = sqrt(dis);
		return dis;
	}
};

} // namespace ghicp
#endif //_INCLUDE_BINARY_FEATRUE_EXTRACTION_HPP