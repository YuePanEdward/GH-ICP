#include "keypointDetection.h"

#include <pcl\filters\extract_indices.h>

#include "utility.h"

#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <fstream>
#include <string.h>

using namespace boost::filesystem;
using namespace std;
using namespace utility;
using namespace keypoint;
using namespace pcl;

bool cmpBasedOnCurvature(CkeypointDetection::pcaFeature &a, CkeypointDetection::pcaFeature &b)
{
	if (a.curvature>b.curvature)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool CkeypointDetection::keypointDetectionBasedOnCurvature2(const PointCloud<PointXYZI>::Ptr &inputPointCloud, PointIndicesPtr &keypointIndices)
{

	vector<pcaFeature> features(inputPointCloud->points.size());
	CalculatePcaFeaturesOfPointCloud(inputPointCloud, m_option.radiusFeatureCalculation, features);

	int keypointNum = 0;

	pcl::PointIndicesPtr candidateIndices(new pcl::PointIndices());
	pruneUnstablePoints(features, m_option.ratioMax, candidateIndices);

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
	float ratioMax = m_option.ratioMax;

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

	cout << setiosflags(ios::fixed) << setprecision(3) << ratioMax << endl;
	return true;
}

bool CkeypointDetection::keypointDetectionBasedOnCurvature(const pcXYZIPtr &inputPointCloud, PointIndicesPtr &keypointIndices)
{
	
	vector<pcaFeature> features(inputPointCloud->points.size());
	CalculatePcaFeaturesOfPointCloud(inputPointCloud, m_option.radiusFeatureCalculation, features);

	int keypointNum = 0;

	pcl::PointIndicesPtr candidateIndices(new pcl::PointIndices());
	pruneUnstablePoints(features, m_option.ratioMax, candidateIndices);

	std::vector<pcaFeature> stableFeatures;
	for (size_t i = 0; i < candidateIndices->indices.size(); ++i)
	{
		stableFeatures.push_back(features[candidateIndices->indices[i]]);
	}

	pcl::PointIndicesPtr nonMaximaIndices(new pcl::PointIndices());
	nonMaximaSuppression(stableFeatures, nonMaximaIndices);
	keypointIndices = nonMaximaIndices;
	return true;
}

bool CkeypointDetection::pruneUnstablePoints(const vector<pcaFeature> &features, float ratioMax, pcl::PointIndicesPtr &indices)
{
	for (size_t i = 0; i < features.size(); ++i)
	{
		float ratio1, ratio2;
		ratio1 = features[i].values.lamada2 / features[i].values.lamada1;
		ratio2 = features[i].values.lamada3 / features[i].values.lamada2;

		if (ratio1 < ratioMax && ratio2 < ratioMax && features[i].ptNum > m_option.minPtNum)
		{
			indices->indices.push_back(i);
		}
	}

	return true;
}


bool CkeypointDetection::nonMaximaSuppression(std::vector<pcaFeature> &features, pcl::PointIndicesPtr &indices)
{
	sort(features.begin(), features.end(), cmpBasedOnCurvature);
	pcl::PointCloud<pcl::PointXYZI> pointCloud;

	/*建立UnSegment以及UnSegment的迭代器,存储未分割的点号;*/
	set<size_t, less<size_t>> unVisitedPtId;
	set<size_t, less<size_t>>::iterator iterUnseg;
	for (size_t i = 0; i < features.size(); ++i)
	{
		unVisitedPtId.insert(i);
		pointCloud.points.push_back(features[i].pt);
	}

	pcl::KdTreeFLANN<pcl::PointXYZI> tree;
	tree.setInputCloud(pointCloud.makeShared());

	//邻域搜索所用变量
	vector<int> search_indices;
	vector<float> distances;

	size_t keypointNum = 0;
	do 
	{
		keypointNum++;
		vector<int>().swap(search_indices);
		vector<float>().swap(distances);

		size_t id;
		iterUnseg = unVisitedPtId.begin();
		id = *iterUnseg;
		indices->indices.push_back(features[id].ptId);
		unVisitedPtId.erase(id);

		tree.radiusSearch(features[id].pt, m_option.radiusNonMax, search_indices, distances);

		for (size_t i = 0; i < search_indices.size(); ++i)
		{
			unVisitedPtId.erase(search_indices[i]);
		}

	} while (!unVisitedPtId.empty());

	return true;
}

void CkeypointDetection::outputKeypoints(const string &filename, const PointIndicesPtr & indices, const pcXYZIPtr &cloud)
{
	ofstream ofs;
	ofs.open(filename);

	if (ofs.is_open())
	{
		for (size_t i = 0; i < indices->indices.size(); ++i)
		{
			ofs << setiosflags(ios::fixed) << setprecision(3) << cloud->points[indices->indices[i]].x << "  "
				<< setiosflags(ios::fixed) << setprecision(3) << cloud->points[indices->indices[i]].y << "  "
				<< setiosflags(ios::fixed) << setprecision(3) << cloud->points[indices->indices[i]].z << endl;
		}
		ofs.close();
	}
}

void CkeypointDetection::savecoordinates(const pcXYZIPtr &Source_FPC, const pcXYZIPtr &Target_FPC, 
	pcl::PointIndicesPtr &Source_KPI, pcl::PointIndicesPtr &Target_KPI, Eigen::MatrixX3d &SXYZ, Eigen::MatrixX3d &TXYZ)
{
	
	SXYZ.resize(Source_KPI->indices.size(), 3);
	TXYZ.resize(Target_KPI->indices.size(), 3);
	for (size_t i = 0; i < Source_KPI->indices.size(); ++i)
	{
		SXYZ.row(i) << Source_FPC->points[Source_KPI->indices[i]].x
			, Source_FPC->points[Source_KPI->indices[i]].y
			, Source_FPC->points[Source_KPI->indices[i]].z;
	}
	for (size_t i = 0; i < Target_KPI->indices.size(); ++i)
	{
		TXYZ.row(i) << Target_FPC->points[Target_KPI->indices[i]].x
			, Target_FPC->points[Target_KPI->indices[i]].y
			, Target_FPC->points[Target_KPI->indices[i]].z;
	}
	cout << "key points saved" << endl;
}

