
#ifndef _INCLUDE_UTILITY_H
#define _INCLUDE_UTILITY_H

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/point_representation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/fpfh.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/impl/pcl_base.hpp>

#include <vector>
#include <list>

//Max and Min
#define max_(a, b) (((a) > (b)) ? (a) : (b))
#define min_(a, b) (((a) < (b)) ? (a) : (b))

//TypeDef
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr pcXYZIPtr;
typedef pcl::PointCloud<pcl::PointXYZI> pcXYZI;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZ> pcXYZ;

typedef pcl::PointCloud<pcl::PointXY>::Ptr pcXYPtr;
typedef pcl::PointCloud<pcl::PointXY> pcXY;

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> pcXYZRGB;

typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcXYZRGBAPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBA> pcXYZRGBA;

typedef pcl::PointCloud<pcl::Normal>::Ptr NormalsPtr;
typedef pcl::PointCloud<pcl::Normal> Normals;

typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcXYZINPtr;
typedef pcl::PointCloud<pcl::PointXYZINormal> pcXYZIN;

typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhFeaturePtr;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

namespace ghicp
{

enum FeatureType
{
	BSC,
	RoPS,
	FPFH,
	None
};

enum CorrespondenceType
{
	NN,
	NNR,
	KM
};

struct CenterPoint
{
	double x;
	double y;
	double z;
	CenterPoint(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z)
	{
		z = 0.0;
		x = y = 0.0;
	}
};

struct Bounds
{
	double min_x;
	double min_y;
	double min_z;
	double max_x;
	double max_y;
	double max_z;
	Bounds()
	{
		min_x = min_y = min_z = max_x = max_y = max_z = 0.0;
	}
};

struct Grid
{
	bool is_empty;
	Grid()
	{
		is_empty = true;
	}
};

struct Voxel
{
	std::vector<int> point_id;
	float min_z;
	float max_z;
	float dertaz;
	float min_z_x; //X of Lowest Point in the Voxel;
	float min_z_y; //Y of Lowest Point in the Voxel;
	float NeighborMin_z;
	int PointsNumber;
	float mean_z;
	Voxel()
	{
		min_z = min_z_x = min_z_y = NeighborMin_z = mean_z = 0.f;
		PointsNumber = 1;
		dertaz = 0.0;
	}
};

struct SimplifiedVoxel
{
	std::vector<int> point_id;
	float max_curvature;
	int max_curvature_point_id;
	bool has_keypoint;
	SimplifiedVoxel()
	{
		has_keypoint = false;
	}
};

template <typename PointT>
class CloudUtility
{
  public:
	//Get Center of a Point Cloud
	void getCloudCenterPoint(const typename pcl::PointCloud<PointT> &cloud, CenterPoint &centerPoint)
	{
		double cx = 0, cy = 0, cz = 0;

		for (int i = 0; i < cloud.size(); i++)
		{
			cx += cloud.points[i].x / cloud.size();
			cy += cloud.points[i].y / cloud.size();
			cz += cloud.points[i].z / cloud.size();
		}
		centerPoint.x = cx;
		centerPoint.y = cy;
		centerPoint.z = cz;
	}

	//Get Bound of a Point Cloud
	void getCloudBound(const typename pcl::PointCloud<PointT> &cloud, Bounds &bound)
	{
		double min_x = cloud[0].x;
		double min_y = cloud[0].y;
		double min_z = cloud[0].z;
		double max_x = cloud[0].x;
		double max_y = cloud[0].y;
		double max_z = cloud[0].z;

		for (int i = 0; i < cloud.size(); i++)
		{
			if (min_x > cloud.points[i].x)
				min_x = cloud.points[i].x;
			if (min_y > cloud.points[i].y)
				min_y = cloud.points[i].y;
			if (min_z > cloud.points[i].z)
				min_z = cloud.points[i].z;
			if (max_x < cloud.points[i].x)
				max_x = cloud.points[i].x;
			if (max_y < cloud.points[i].y)
				max_y = cloud.points[i].y;
			if (max_z < cloud.points[i].z)
				max_z = cloud.points[i].z;
		}
		bound.min_x = min_x;
		bound.max_x = max_x;
		bound.min_y = min_y;
		bound.max_y = max_y;
		bound.min_z = min_z;
		bound.max_z = max_z;
	}

	//Get Bound and Center of a Point Cloud
	void getBoundAndCenter(const typename pcl::PointCloud<PointT> &cloud, Bounds &bound, CenterPoint &centerPoint)
	{
		getCloudCenterPoint(cloud, centerPoint);
		getCloudBound(cloud, bound);
	}

	//Get Bound of Subsets of a Point Cloud
	void GetSubsetBoundary(typename pcl::PointCloud<PointT>::Ptr &cloud, std::vector<int> &index, Bounds &bound)
	{
		typename pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
		for (int i = 0; i < index.size(); i++)
		{
			temp_cloud->push_back(cloud->points[index[i]]);
		}
		getCloudBound(*temp_cloud, bound);
	}

	/**
* \brief Transform a Point Cloud using a given transformation matrix
* \param[in]  Cloud : A pointer of the Point Cloud before transformation
* \param[out] TransformedCloud : A pointer of the Point Cloud after transformation
* \param[in]  transformation : A 4*4 transformation matrix
*/

	void transformcloud(typename pcl::PointCloud<PointT>::Ptr &Cloud,
						typename pcl::PointCloud<PointT>::Ptr &TransformedCloud,
						Eigen::Matrix4f &transformation)
	{
		Eigen::Matrix4Xf PC;
		Eigen::Matrix4Xf TPC;
		PC.resize(4, Cloud->size());
		TPC.resize(4, Cloud->size());
		for (int i = 0; i < Cloud->size(); i++)
		{
			PC(0, i) = Cloud->points[i].x;
			PC(1, i) = Cloud->points[i].y;
			PC(2, i) = Cloud->points[i].z;
			PC(3, i) = 1;
		}
		TPC = transformation * PC;
		for (int i = 0; i < Cloud->size(); i++)
		{
			PointT pt;
			pt.x = TPC(0, i);
			pt.y = TPC(1, i);
			pt.z = TPC(2, i);
			TransformedCloud->points.push_back(pt);
		}
		std::cout << "Transform done ..." << std::endl;
	}

  protected:
  private:
};
} // namespace ghicp

#endif //_INCLUDE_UTILITY_H