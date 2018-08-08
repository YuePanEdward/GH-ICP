#ifndef PCA_H
#define PCA_H


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include <vector>

#include "utility.h"

class PrincipleComponentAnalysis
{

public:
	struct eigenValue
	{
		double lamada1;
		double lamada2;
		double lamada3;
	};

	struct eigenVector
	{
		Eigen::Vector3f principalDirection;
		Eigen::Vector3f middleDirection;
		Eigen::Vector3f normalDirection;
	};

	struct pcaFeature
	{
		eigenValue values;
		eigenVector vectors;
		double curvature;
		pcl::PointXYZI pt;
		size_t ptId;
		size_t ptNum;
	};

	/*计算法向量;*/
	bool  CalculateNormalVectorOpenMP(const pcXYZIPtr &inputPointCloud, float radius,NormalsPtr &normals);

	/*主成分分析;*/
	bool CalculatePcaFeaturesOfPointCloud(const pcXYZIPtr &inputPointCloud, float radius,std::vector<pcaFeature> &features);

	bool CalculatePcaFeature(const pcXYZIPtr &inputPointCloud, const std::vector<int> &search_indices, pcaFeature &feature);

protected:


private:

	void  CheckNormals(NormalsPtr &normals);//检查是否存在不合法的法向量值;
};

#endif
