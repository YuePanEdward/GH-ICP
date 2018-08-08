#ifndef UTILITY
#define UTILITY

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

typedef  pcl::PointCloud<pcl::PointXYZI>::Ptr      pcXYZIPtr;
typedef  pcl::PointCloud<pcl::PointXYZI>            pcXYZI;

typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr      pcXYZPtr;
typedef  pcl::PointCloud<pcl::PointXYZ>            pcXYZ;

typedef  pcl::PointCloud<pcl::PointXY>::Ptr       pcXYPtr;
typedef  pcl::PointCloud<pcl::PointXY>            pcXY;

typedef  pcl::PointCloud<pcl::PointXYZRGB>::Ptr      pcXYZRGBPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGB>           pcXYZRGB;

typedef  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr       pcXYZRGBAPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGBA>            pcXYZRGBA;

typedef pcl::PointCloud<pcl::Normal>::Ptr NormalsPtr;
typedef pcl::PointCloud<pcl::Normal> Normals;


typedef std::vector<pcXYZI>  vectorPCXYZI;

namespace utility
{
	struct pointCloudBound
	{
		double minx;
		double maxx;
		double miny;
		double maxy;
		double minz;
		double maxz;
		pointCloudBound()
		{
			minx = maxx = miny = maxy = minz = maxz = 0.0;
		}
	};

	struct pointCloudBound2d
	{
		double minx;
		double maxx;
		double miny;
		double maxy;
		pointCloudBound2d()
		{
			minx = maxx = miny = maxy= 0.0;
		}
	};


	template<typename PointCloudType>
	void getCloudBound(const PointCloudType & cloud, pointCloudBound & bound)
	{
		double min_x = DBL_MAX;
		double min_y = DBL_MAX;
		double min_z = DBL_MAX;
		double max_x = -DBL_MAX;
		double max_y = -DBL_MAX;
		double max_z = -DBL_MAX;

		for (size_t i = 0; i<cloud.size(); ++i)
		{
			//»ñÈ¡±ß½ç
			if (min_x>cloud.points[i].x)
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
		bound.minx = min_x;
		bound.maxx = max_x;
		bound.miny = min_y;
		bound.maxy = max_y;
		bound.minz = min_z;
		bound.maxz = max_z;
	}
}

#endif