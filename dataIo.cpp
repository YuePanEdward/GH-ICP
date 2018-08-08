#include "dataIo.h"
#include "utility.h"

#include <string>
#include <fstream>
#include <vector>
#include <pcl/io/pcd_io.h>


using namespace  std;
using namespace  utility;

bool DataIo::readPcdFile(const std::string &fileName, const pcXYZIPtr &pointCloud)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZI>(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file\n");
		return false;
	}

	return true;
}

bool DataIo::readPcdFileXYZ(const std::string &fileName, const pcXYZPtr &pointCloud)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file\n");
		return false;
	}

	return true;
}


bool DataIo::writePcdFile(const string &fileName, const pcXYZIPtr &pointCloud)
{
	if (pcl::io::savePCDFileBinary(fileName,*pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't write file\n");
		return false;
	}
	return true;
}

bool DataIo::writePcdFileXYZ(const string &fileName, const pcXYZPtr &pointCloud)
{
	if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't write file\n");
		return false;
	}
	return true;
}

void DataIo::outputwhat()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	
	cloud->width = 5000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	cloud2->width = 10000;
	cloud2->height = 1;
	cloud2->points.resize(cloud2->width * cloud2->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 4096.0f * rand() / (RAND_MAX + 1.0f);
	}

	for (size_t i = 0; i < (cloud2->points.size())/2; ++i)
	{
		cloud->points[i].x = 4000.0f+1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 4000.0f+1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 4096.0f * rand() / (RAND_MAX + 1.0f);
	}
	for (size_t i = (cloud2->points.size()) / 2; i <cloud2->points.size(); ++i)
	{
		cloud->points[i].x = 5000.0f + 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 5000.0f + 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 4096.0f * rand() / (RAND_MAX + 1.0f);
	}


	pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("newtest.pcd", *cloud, false);
	writer.write<pcl::PointXYZ>("newtest2.pcd", *cloud2, false);

}

void DataIo::readParalist(string paralistfile)
{
	ifstream infile;   // ‰»Î¡˜
	infile.open(paralistfile, ios::in);
	if (!infile.is_open()) cout << "Open file failure" << endl;
    
	infile >> paralist.num_point_bb;
	infile >> paralist.feature_r;
	infile >> paralist.keypoint_max_ratio;
	infile >> paralist.keypoint_min_num;
	infile >> paralist.scale;
	infile >> paralist.p_pre;
	infile >> paralist.p_ED;
	infile >> paralist.p_FD;
	infile >> paralist.m;
	infile >> paralist.converge_t;
	infile >> paralist.converge_r;
	infile >> paralist.output;
}