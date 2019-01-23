#include "principleComponentAnalysis.h"

#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include <opencv2/core/types_c.h>  //这里要用到OpenCV的，注意啊
#include <opencv2/core/core_c.h>
#include <concurrent_vector.h>
#include <ppl.h>

using namespace  std;

bool PrincipleComponentAnalysis::CalculateNormalVectorOpenMP(const pcXYZIPtr &inputPointCloud, float radius,NormalsPtr &normals)
{
	// Create the normal estimation class, and pass the input dataset to it;
	pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;
	ne.setInputCloud(inputPointCloud);
	// Create an empty kd-tree representation, and pass it to the normal estimation object;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	ne.setSearchMethod(tree);
	// Use all neighbors in a sphere of radius;
	ne.setRadiusSearch(radius);
	// Compute the features
	ne.compute(*normals);

	CheckNormals(normals);

	return true;
}


void PrincipleComponentAnalysis::CheckNormals(NormalsPtr &normals)
{
	//It is advisable to check the normals before the call to compute()
	for (int i = 0; i < normals->points.size(); i++)
	{
		if (!pcl::isFinite<pcl::Normal>(normals->points[i]))
		{
			normals->points[i].normal_x = 0.577;
			normals->points[i].normal_y = 0.577;
			normals->points[i].normal_z = 0.577;
			normals->points[i].curvature = 0.0;
		}

		if (_isnan(normals->points[i].curvature))
		{
			normals->points[i].normal_x = 0.577;
			normals->points[i].normal_y = 0.577;
			normals->points[i].normal_z = 0.577;
			normals->points[i].curvature = 0.0;
		}
	}
}



bool PrincipleComponentAnalysis::CalculatePcaFeaturesOfPointCloud(const pcXYZIPtr &inputPointCloud,float radius, vector<pcaFeature> &features)
{
	pcl::KdTreeFLANN<pcl::PointXYZI> tree;
	tree.setInputCloud(inputPointCloud);

	concurrency::parallel_for(size_t(0), inputPointCloud->points.size(), [&](size_t i)
	{
		//邻域搜索所用变量
		vector<int> search_indices;
		vector<float> distances;
		vector<int>().swap(search_indices);
		vector<float>().swap(distances);

		tree.radiusSearch(i, radius, search_indices, distances);
		features[i].pt = inputPointCloud->points[i];
		features[i].ptId = i;
		features[i].ptNum = search_indices.size();
		CalculatePcaFeature(inputPointCloud, search_indices,features[i]);
	});

	return true;
}

bool PrincipleComponentAnalysis::CalculatePcaFeature(const pcXYZIPtr &inputPointCloud,const vector<int> &search_indices, pcaFeature &feature)
{
	size_t ptNum;
	ptNum = search_indices.size();

	if (ptNum < 3)
	 return false;

	CvMat* pData = cvCreateMat(ptNum, 3, CV_32FC1);
	CvMat* pMean = cvCreateMat(1, 3, CV_32FC1);
	CvMat* pEigVals = cvCreateMat(1, 3, CV_32FC1);
	CvMat* pEigVecs = cvCreateMat(3, 3, CV_32FC1);

	for (size_t i = 0; i < ptNum; ++i)
	{
		cvmSet(pData, i, 0, inputPointCloud->points[search_indices[i]].x);
		cvmSet(pData, i, 1, inputPointCloud->points[search_indices[i]].y);
		cvmSet(pData, i, 2, inputPointCloud->points[search_indices[i]].z);
	}

	cvCalcPCA(pData, pMean, pEigVals, pEigVecs, CV_PCA_DATA_AS_ROW);

	feature.vectors.principalDirection.x() = cvmGet(pEigVecs, 0, 0);
	feature.vectors.principalDirection.y() = cvmGet(pEigVecs, 0, 1);
	feature.vectors.principalDirection.z() = cvmGet(pEigVecs, 0, 2);

	feature.vectors.middleDirection.x() = cvmGet(pEigVecs, 1, 0);
	feature.vectors.middleDirection.y() = cvmGet(pEigVecs, 1, 1);
	feature.vectors.middleDirection.z() = cvmGet(pEigVecs, 1, 2); 

	feature.vectors.normalDirection.x() = cvmGet(pEigVecs, 2, 0);
	feature.vectors.normalDirection.y() = cvmGet(pEigVecs, 2, 1);
	feature.vectors.normalDirection.z() = cvmGet(pEigVecs, 2, 2);

	feature.values.lamada1 = cvmGet(pEigVals, 0, 0);
	feature.values.lamada2 = cvmGet(pEigVals, 0, 1);
	feature.values.lamada3 = cvmGet(pEigVals, 0, 2);

	if ((feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3) == 0)
	{
		feature.curvature = 0;
	}
	else
	{
		feature.curvature = feature.values.lamada3 / (feature.values.lamada1 + feature.values.lamada2 + feature.values.lamada3);

	}

	cvReleaseMat(&pEigVecs);
	cvReleaseMat(&pEigVals);
	cvReleaseMat(&pMean);
	cvReleaseMat(&pData);
	return true;
}
