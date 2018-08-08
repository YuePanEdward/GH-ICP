#ifndef KEYPOINTDETECTION
#define KEYPOINTDETECTION


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#include "principleComponentAnalysis.h"
#include "utility.h"

#include <string.h>

namespace keypoint
{
	struct keypointOption
	{
		float radiusFeatureCalculation;
		float ratioMax;
		size_t minPtNum;
		float radiusNonMax;

		keypointOption()
		{
			radiusFeatureCalculation = 0.4;
			ratioMax = 0.92;
			minPtNum = 20;
			radiusNonMax = 0.4;
		}
	};

	class CkeypointDetection :public PrincipleComponentAnalysis
	{

	public:
		CkeypointDetection(keypointOption option)
		{
			m_option = option;
		}

		bool keypointDetectionBasedOnCurvature(const pcXYZIPtr &inputPointCloud,pcl::PointIndicesPtr &keypointIndices);
		bool keypointDetectionBasedOnCurvature2(const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputPointCloud, pcl::PointIndicesPtr &keypointIndices);
		void outputKeypoints(const std::string &filenames, const pcl::PointIndicesPtr & indices, const pcXYZIPtr &cloud);
		void savecoordinates(const pcXYZIPtr &Source_FPC, const pcXYZIPtr &Target_FPC, pcl::PointIndicesPtr &Source_KPI, pcl::PointIndicesPtr &Target_KPI
			,Eigen::MatrixX3d &SXYZ, Eigen::MatrixX3d &TXYZ);

	protected:
	private:
		bool pruneUnstablePoints(const std::vector<pcaFeature> &features, float ratioMax, pcl::PointIndicesPtr &indices);

		bool nonMaximaSuppression(std::vector<pcaFeature> &features, pcl::PointIndicesPtr &indices);

		keypointOption m_option;
	};
}


#endif