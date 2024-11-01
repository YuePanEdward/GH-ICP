#ifndef _INCLUDE_FPFH_HPP
#define _INCLUDE_FPFH_HPP

#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/PointIndices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <vector>

#include "utility.h"

using namespace std;

namespace ghicp
{

template <typename PointT>
class FPFHfeature
{
  public:
	FPFHfeature(double neighbor_radius) : radius(neighbor_radius) {}

	bool compute_fpfh_feature(const typename pcl::PointCloud<PointT>::Ptr &input_cloud, fpfhFeaturePtr &fpfh)
	{
		NormalsPtr point_normal(new Normals);
		typename pcl::NormalEstimation<PointT, pcl::Normal> est_normal;
		typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>());
		est_normal.setInputCloud(input_cloud);
		est_normal.setSearchMethod(tree);
		est_normal.setKSearch(20);
		//est_normal.setRadiusSearch(0.02);
		est_normal.compute(*point_normal);

		typename pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
		est_fpfh.setNumberOfThreads(4);
		est_fpfh.setInputCloud(input_cloud);
		est_fpfh.setInputNormals(point_normal);
		est_fpfh.setSearchMethod(tree);
		est_fpfh.setKSearch(20);
		//est_fpfh.setRadiusSearch(0.03);
		est_fpfh.compute(*fpfh);
        
		cout<<"Extract FPFH feature done."<<endl;
		return 1;
	}

	bool compute_fpfh_keypoint(const typename pcl::PointCloud<PointT>::Ptr &input_cloud,
							   const pcl::PointIndicesPtr &indices, fpfhFeaturePtr &fpfh)

	{
		typename pcl::PointCloud<PointT>::Ptr inputkp(new typename pcl::PointCloud<PointT>);
		for (size_t i = 0; i < indices->indices.size(); i++)
		{
			inputkp->points[i] = input_cloud->points[indices->indices[i]];
		}

		NormalsPtr point_normal(new Normals);
		typename pcl::NormalEstimation<PointT, pcl::Normal> est_normal;
		typename pcl::search::KdTree<PointT>::Ptr tree(new typename pcl::search::KdTree<PointT>());
		est_normal.setInputCloud(inputkp);
		est_normal.setSearchMethod(tree);
		//est_normal.setKSearch(100);
		est_normal.setRadiusSearch(radius);
		est_normal.compute(*point_normal);

		pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
		est_fpfh.setNumberOfThreads(4);
		est_fpfh.setInputCloud(inputkp);
		est_fpfh.setInputNormals(point_normal);
		est_fpfh.setSearchMethod(tree);
		//est_fpfh.setKSearch(50);
		est_fpfh.setRadiusSearch(radius);
		est_fpfh.compute(*fpfh);

		cout<<"Extract FPFH feature done."<<endl;

		return 1;
	}

	bool keyfpfh(const fpfhFeaturePtr &source_fpfh, const fpfhFeaturePtr &target_fpfh,
				 const pcl::PointIndicesPtr &sindices, const pcl::PointIndicesPtr &tindices,
				 fpfhFeaturePtr &source_kfpfh, fpfhFeaturePtr &target_kfpfh)
	{
		source_kfpfh->width = sindices->indices.size();
		source_kfpfh->height = 1;
		target_kfpfh->width = tindices->indices.size();
		target_kfpfh->height = 1;
		source_kfpfh->points.resize(source_kfpfh->width * source_kfpfh->height);
		target_kfpfh->points.resize(target_kfpfh->width * target_kfpfh->height);

		for (size_t i = 0; i < sindices->indices.size(); i++)
		{
			source_kfpfh->points[i] = source_fpfh->points[sindices->indices[i]];
		}

		for (size_t i = 0; i < tindices->indices.size(); i++)
		{
			target_kfpfh->points[i] = target_fpfh->points[tindices->indices[i]];
		}

		return 1;
	}

	bool fpfhalign(const typename pcl::PointCloud<PointT>::Ptr &sourcecloud,
				   const typename pcl::PointCloud<PointT>::Ptr &targetcloud,
				   const fpfhFeaturePtr source_fpfh, const fpfhFeaturePtr target_fpfh,
				   typename pcl::PointCloud<PointT>::Ptr &reg_sourcecloud)
	{
		pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
		sac_ia.setInputSource(sourcecloud);
		sac_ia.setSourceFeatures(source_fpfh);
		sac_ia.setInputTarget(targetcloud);
		sac_ia.setTargetFeatures(target_fpfh);

		//sac_ia.setNumberOfSamples(20);  
		sac_ia.setCorrespondenceRandomness(6); 
		sac_ia.align(*reg_sourcecloud);
		cout << "Has converged:" << sac_ia.hasConverged() << "score" << sac_ia.getFitnessScore() << endl;
		cout << "FPFH-SAC Registration completed" << endl;
	}

	float compute_fpfh_distance(float his1[33], float his2[33])
	{
		//how to calculate the distance/similarity of histogram
		// First method : Correlation
		// Correlation= COV(x,y)/sqrt(D(x)*D(y))

		float d_correlation = 0;
		float d_correlation_up = 0;
		float d_correlation_down1 = 0;
		float d_correlation_down2 = 0;
		float mean_his1 = 0;
		float mean_his2 = 0;

		for (int i = 0; i < 33; i++)
		{
			mean_his1 += his1[i];
			mean_his2 += his2[i];
		}
		mean_his1 /= 33;
		mean_his2 /= 33;

		for (int i = 0; i < 33; i++)
		{
			d_correlation_up += (his1[i] - mean_his1) * (his2[i] - mean_his2);
			d_correlation_down1 += (his1[i] - mean_his1) * (his1[i] - mean_his1);
			d_correlation_down2 += (his2[i] - mean_his2) * (his2[i] - mean_his2);
		}
		d_correlation = d_correlation_up / sqrt(d_correlation_down1 * d_correlation_down2);

		return abs(d_correlation);
	}

	//void displayhistogram(const fpfhFeaturePtr fpfh,int index);
	//void displaycorrespondence(const pcXYZIPtr sourcecloud, const pcXYZIPtr targetcloud, const pcXYZIPtr aligncloud, const fpfhFeaturePtr source_fpfh,const fpfhFeaturePtr target_fpfh);

  protected:
  private:
	double radius;
};
} // namespace ghicp

#endif //_INCLUDE_FPFH_HPP