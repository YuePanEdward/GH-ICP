#ifndef _INCLUDE_PCA_H_
#define _INCLUDE_PCA_H_

//pcl
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>

#include <vector>

#include "utility.h"

namespace ghicp
{
struct eigenValue // Eigen Value ,lamada1 > lamada2 > lamada3;
{
	double lamada1;
	double lamada2;
	double lamada3;
};

struct eigenVector //the eigen vector corresponding to the eigen value
{
	Eigen::Vector3f principalDirection;
	Eigen::Vector3f middleDirection;
	Eigen::Vector3f normalDirection;
};

struct pcaFeature //PCA
{
	eigenValue values;
	eigenVector vectors;
	double curvature;
	double linear;
	double planar;
	double spherical;
	double linear_2;
	double planar_2;
	double spherical_2;
	pcl::PointNormal pt;
	int ptId;
	int ptNum = 0;
	std::vector<int> neighbor_indices;
};

template <typename PointT>
class PrincipleComponentAnalysis
{
  public:
	/**
		* \brief Estimate the normals of the input Point Cloud by PCL speeding up with OpenMP
		* \param[in] inputPointCloud is the input Point Cloud Pointer
		* \param[in] radius is the neighborhood search radius (m) for KD Tree
		* \param[out] normals is the normal of all the points from the Point Cloud
		*/
	bool CalculateNormalVector_Radius(typename pcl::PointCloud<PointT>::Ptr inputPointCloud,
									  float radius,
									  pcl::PointCloud<pcl::Normal>::Ptr &normals)
	{
		// Create the normal estimation class, and pass the input dataset to it;
		pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
		ne.setNumberOfThreads(1); //More threads sometimes would not speed up the procedure
		ne.setInputCloud(inputPointCloud);
		// Create an empty kd-tree representation, and pass it to the normal estimation object;
		typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
		ne.setSearchMethod(tree);
		// Use all neighbors in a sphere of radius;
		ne.setRadiusSearch(radius);
		// Compute the normal
		ne.compute(*normals);
		CheckNormals(normals);
		return true;
	}

	bool CalculatePointCloudWithNormal_Radius(typename pcl::PointCloud<PointT>::Ptr inputPointCloud,
											  float radius,
											  pcl::PointCloud<pcl::PointNormal>::Ptr &pointnormals)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		bool normal_ready = CalculateNormalVector_Radius(inputPointCloud, radius, normals);
		if (normal_ready)
		{
			// Concatenate the XYZ and normal fields*
			pcl::concatenateFields(*inputPointCloud, *normals, *pointnormals);
			return true;
		}
		else
			return false;
	}

	bool CalculateNormalVector_KNN(typename pcl::PointCloud<PointT>::Ptr inputPointCloud,
								   int K,
								   pcl::PointCloud<pcl::Normal>::Ptr &normals)
	{
		// Create the normal estimation class, and pass the input dataset to it;
		pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
		ne.setNumberOfThreads(1);
		ne.setInputCloud(inputPointCloud);
		// Create an empty kd-tree representation, and pass it to the normal estimation object;
		typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
		ne.setSearchMethod(tree);
		// Use all neighbors in a sphere of radius;
		ne.setKSearch(K);
		// Compute the normal
		ne.compute(*normals);
		CheckNormals(normals);
		return true;
	}

	bool CalculatePointCloudWithNormal_KNN(typename pcl::PointCloud<PointT>::Ptr inputPointCloud,
										   int K,
										   pcl::PointCloud<pcl::PointNormal>::Ptr &pointnormals)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
		bool normal_ready = CalculateNormalVector_KNN(inputPointCloud, K, normals);
		if (normal_ready)
		{
			// Concatenate the XYZ and normal fields*
			pcl::concatenateFields(*inputPointCloud, *normals, *pointnormals);
			return true;
		}
		else
			return false;
	}

	/**
		* \brief Principle Component Analysis (PCA) of the Point Cloud with fixed search radius
		* \param[in] inputPointCloud is the input Point Cloud (XYZI) Pointer
		* \param[in]     radius is the neighborhood search radius (m) for KD Tree
		* \param[out]features is the pcaFeature vector of all the points from the Point Cloud
		*/
	bool CalculatePcaFeaturesOfPointCloud(typename pcl::PointCloud<PointT>::Ptr inputPointCloud,
										  std::vector<pcaFeature> &features,
										  const float radius)
	{
		//LOG(INFO) << "input cloud size is " << inputPointCloud->points.size();
		pcl::KdTreeFLANN<PointT> tree;
		tree.setInputCloud(inputPointCloud);
		features.resize(inputPointCloud->size());

		//concurrency::parallel_for(size_t(0), inputPointCloud->points.size(), [&](size_t i)   //Multi-thread
		//{
		for (int i = 0; i < inputPointCloud->points.size(); i++)
		{
			std::vector<int> search_indices; //point index Vector
			std::vector<float> distances;	//distance Vector
			std::vector<int>().swap(search_indices);
			std::vector<float>().swap(distances);

			tree.radiusSearch(i, radius, search_indices, distances); //KD tree
			features[i].pt.x = inputPointCloud->points[i].x;
			features[i].pt.y = inputPointCloud->points[i].y;
			features[i].pt.z = inputPointCloud->points[i].z;
			features[i].ptId = i;
			features[i].ptNum = search_indices.size();
			CalculatePcaFeature(inputPointCloud, search_indices, features[i]);
			// inputPointCloud->points[i].normal_x =  features[i].vectors.normalDirection.x();
			// inputPointCloud->points[i].normal_y =  features[i].vectors.normalDirection.y();
			// inputPointCloud->points[i].normal_z =  features[i].vectors.normalDirection.z();
		}
		//});

		return true;
	}

	bool CalculatePcaFeaturesOfPointCloud(typename pcl::PointCloud<PointT>::Ptr inputPointCloud,
										  std::vector<pcaFeature> &features,
										  const int K)
	{
		pcl::KdTreeFLANN<PointT> tree;
		tree.setInputCloud(inputPointCloud);
		features.resize(inputPointCloud->size());

		//concurrency::parallel_for(size_t(0), inputPointCloud->points.size(), [&](size_t i)   //Multi-thread
		//{
		for (int i = 0; i < inputPointCloud->points.size(); i++)
		{
			std::vector<int> search_indices; //point index Vector
			std::vector<float> distances;	//distance Vector
			std::vector<int>().swap(search_indices);
			std::vector<float>().swap(distances);

			tree.nearestKSearch(i, K, search_indices, distances); //KD tree
			features[i].pt = inputPointCloud->points[i];
			features[i].ptId = i;
			features[i].ptNum = search_indices.size();
			CalculatePcaFeature(inputPointCloud, search_indices, features[i]);
		}
		//});

		return true;
	}

	/**
		* \brief Use PCL to accomplish the Principle Component Analysis (PCA)
		* of one point and its neighborhood
		* \param[in] inputPointCloud is the input Point Cloud Pointer
		* \param[in] search_indices is the neighborhood points' indices of the search point.
		* \param[out]feature is the pcaFeature of the search point.
		*/
	bool CalculatePcaFeature(typename pcl::PointCloud<PointT>::Ptr inputPointCloud,
							 std::vector<int> &search_indices,
							 pcaFeature &feature)
	{
		size_t ptNum;
		ptNum = search_indices.size();

		if (ptNum < 3)
			return false;

		typename pcl::PointCloud<PointT>::Ptr selected_cloud(new pcl::PointCloud<PointT>());
		for (size_t i = 0; i < ptNum; ++i)
		{
			selected_cloud->points.push_back(inputPointCloud->points[search_indices[i]]);
		}

		pcl::PCA<PointT> pca_operator;
		pca_operator.setInputCloud(selected_cloud);

		// Compute eigen values and eigen vectors
		Eigen::Matrix3f eigen_vectors = pca_operator.getEigenVectors();
		Eigen::Vector3f eigen_values = pca_operator.getEigenValues();

		feature.vectors.principalDirection = eigen_vectors.col(0);
		feature.vectors.normalDirection = eigen_vectors.col(2);

		feature.values.lamada1 = eigen_values(0);
		feature.values.lamada2 = eigen_values(1);
		feature.values.lamada3 = eigen_values(2);

		if ((feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3) == 0)
		{
			feature.curvature = 0;
		}
		else
		{
			feature.curvature = feature.values.lamada3 / (feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3);
		}

		// feature.linear = (sqrt(feature.values.lamada1) - sqrt(feature.values.lamada2)) / sqrt(feature.values.lamada1);
		// feature.planar = (sqrt(feature.values.lamada2) - sqrt(feature.values.lamada3)) / sqrt(feature.values.lamada1);
		// feature.spherical = sqrt(feature.values.lamada3) / sqrt(feature.values.lamada1);
		feature.linear_2 = ((feature.values.lamada1) - (feature.values.lamada2)) / (feature.values.lamada1);
		feature.planar_2 = ((feature.values.lamada2) - (feature.values.lamada3)) / (feature.values.lamada1);
		feature.spherical_2 = (feature.values.lamada3) / (feature.values.lamada1);

		search_indices.swap(feature.neighbor_indices);
		return true;
	}

  protected:
  private:
	/**
		* \brief Check the Normals (if they are finite)
		* \param  normals is the input Point Cloud (XYZI)'s Normal Pointer
		*/
	void CheckNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals)
	{
		//It is advisable to check the normals before the call to compute()
		for (int i = 0; i < normals->points.size(); i++)
		{
			if (!pcl::isFinite<pcl::Normal>(normals->points[i]))
			{
				normals->points[i].normal_x = 0.577; // 1/ sqrt(3)
				normals->points[i].normal_y = 0.577;
				normals->points[i].normal_z = 0.577;
				normals->points[i].curvature = 0.0;
			}
		}
	}
};

} // namespace ghicp
#endif //_INCLUDE_PCA_H_
