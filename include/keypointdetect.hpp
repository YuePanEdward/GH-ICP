#ifndef _INCLUDE_KEY_POINT_DETECT_HPP
#define _INCLUDE_KEY_POINT_DETECT_HPP

#include "utility.h"
#include "pca.h"

#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <fstream>
#include <string.h>

namespace ghicp
{

template <typename PointT>
class CKeypointDetect
{
  public:
	CKeypointDetect(float neighborhood_radius, float ratio_unstable_thre, int min_point_num_neighborhood,
					float curvature_non_max_radius) : _neighborhood_radius(neighborhood_radius),
													  _ratio_unstable_thre(ratio_unstable_thre),
													  _min_point_num_neighborhood(min_point_num_neighborhood),
													  _curvature_non_max_radius(curvature_non_max_radius) {}


	bool keypointDetectionBasedOnCurvature(const typename pcl::PointCloud<PointT>::Ptr &inputPointCloud,
										   pcl::PointIndicesPtr &keypointIndices)
	{
		std::vector<pcaFeature> features(inputPointCloud->points.size());
		PrincipleComponentAnalysis<PointT> pca_operator;
		pca_operator.CalculatePcaFeaturesOfPointCloud(inputPointCloud,features,_neighborhood_radius);

		int keypointNum = 0;

		pcl::PointIndicesPtr candidateIndices(new pcl::PointIndices());
		pruneUnstablePoints(features, _ratio_unstable_thre, candidateIndices);

		std::vector<pcaFeature> stableFeatures;
		for (int i = 0; i < candidateIndices->indices.size(); ++i)
		{
			stableFeatures.push_back(features[candidateIndices->indices[i]]);
		}

		pcl::PointIndicesPtr nonMaximaIndices(new pcl::PointIndices());
		nonMaximaSuppression(stableFeatures, nonMaximaIndices);
		keypointIndices = nonMaximaIndices;

		std::cout << "Keypoint detection done (" << keypointIndices->indices.size() << " keypoints)" << std::endl;
		return true;
	}

	bool keypointDetectionBasedOnCurvature_adaptive(const typename pcl::PointCloud<PointT>::Ptr &inputPointCloud,
													pcl::PointIndicesPtr &keypointIndices)
	{
		std::vector<pcaFeature> features(inputPointCloud->points.size());
		PrincipleComponentAnalysis<PointT> pca_operator;
		pca_operator.CalculatePcaFeaturesOfPointCloud(inputPointCloud,features,_neighborhood_radius);

		int keypointNum = 0;

		pcl::PointIndicesPtr candidateIndices(new pcl::PointIndices());
		pruneUnstablePoints(features, _ratio_unstable_thre, candidateIndices);

		std::vector<pcaFeature> stableFeatures;
		for (size_t i = 0; i < candidateIndices->indices.size(); ++i)
		{
			stableFeatures.push_back(features[candidateIndices->indices[i]]);
		}

		pcl::PointIndicesPtr nonMaximaIndices(new pcl::PointIndices());
		nonMaximaSuppression(stableFeatures, nonMaximaIndices);
		keypointIndices = nonMaximaIndices;
		keypointNum = keypointIndices->indices.size();

		bool finishIteration = false;
		float ratioMax = _ratio_unstable_thre;

		if (keypointNum > 50000)
		{
			do
			{
				if (keypointNum < 5000)
				{
					ratioMax += 0.025;
					finishIteration = true;
				}
				else
				{
					ratioMax -= 0.05;
				}

				candidateIndices->indices.clear();
				pruneUnstablePoints(features, ratioMax, candidateIndices);
				stableFeatures.clear();
				for (size_t i = 0; i < candidateIndices->indices.size(); ++i)
				{
					stableFeatures.push_back(features[candidateIndices->indices[i]]);
				}

				nonMaximaIndices->indices.clear();
				nonMaximaSuppression(stableFeatures, nonMaximaIndices);
				keypointIndices = nonMaximaIndices;
				keypointNum = keypointIndices->indices.size();

			} while ((keypointNum < 5000 || keypointNum > 50000) && (finishIteration == false) && ratioMax >= 0.65);
		}

		//std::cout << setiosflags(std::ios::fixed) << std::setprecision(3) << ratioMax << std::endl;
		return true;
	}

  protected:
  private:
	float _neighborhood_radius;
	float _ratio_unstable_thre;
	int _min_point_num_neighborhood;
	float _curvature_non_max_radius;
    
    static bool cmpBasedOnCurvature(const pcaFeature &a, const pcaFeature &b)
	{
		if (a.curvature > b.curvature)
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	bool pruneUnstablePoints(const std::vector<pcaFeature> &features, float ratioMax, pcl::PointIndicesPtr &indices)
	{
		for (int i = 0; i < features.size(); ++i)
		{
			float ratio1, ratio2;
			ratio1 = features[i].values.lamada2 / features[i].values.lamada1;
			ratio2 = features[i].values.lamada3 / features[i].values.lamada2;

			if (ratio1 < ratioMax && ratio2 < ratioMax && features[i].ptNum > _min_point_num_neighborhood)
			{
				indices->indices.push_back(i);
			}
		}

		return true;
	}

	bool nonMaximaSuppression(std::vector<pcaFeature> &features, pcl::PointIndicesPtr &indices)
	{
		std::sort(features.begin(), features.end(), cmpBasedOnCurvature);
		pcl::PointCloud<pcl::PointNormal> pointCloud;

		std::set<int, std::less<int>> unVisitedPtId;
		std::set<int, std::less<int>>::iterator iterUnseg;
		for (int i = 0; i < features.size(); ++i)
		{
			unVisitedPtId.insert(i);
			pointCloud.points.push_back(features[i].pt);
		}

		pcl::KdTreeFLANN<pcl::PointNormal> tree;
		tree.setInputCloud(pointCloud.makeShared());

		std::vector<int> search_indices;
		std::vector<float> distances;

		int keypointNum = 0;
		do
		{
			keypointNum++;
			std::vector<int>().swap(search_indices);
			std::vector<float>().swap(distances);

			int id;
			iterUnseg = unVisitedPtId.begin();
			id = *iterUnseg;
			indices->indices.push_back(features[id].ptId);
			unVisitedPtId.erase(id);

			tree.radiusSearch(features[id].pt, _curvature_non_max_radius, search_indices, distances);

			for (int i = 0; i < search_indices.size(); ++i)
			{
				unVisitedPtId.erase(search_indices[i]);
			}

		} while (!unVisitedPtId.empty());

		return true;
	}
};
} // namespace ghicp

#endif //_INCLUDE_KEY_POINT_DETECT_HPP