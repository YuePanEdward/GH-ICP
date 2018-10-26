#include "dataIo.h"
#include "utility.h"

#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
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
	ifstream infile;   //输入流
	infile.open(paralistfile, ios::in);
	if (!infile.is_open()) cout << "Open file failure" << endl;
	infile >> paralist.downsample_resolution;
	infile >> paralist.num_point_bb;
	infile >> paralist.feature_r;
	infile >> paralist.keypoint_max_ratio;
	infile >> paralist.keypoint_min_num;
	infile >> paralist.scale;
	infile >> paralist.p_pre;
	infile >> paralist.p_ED;
	infile >> paralist.p_FD;
	infile >> paralist.m;
	infile >> paralist.kmeps;
	infile >> paralist.converge_t;
	infile >> paralist.converge_r;
	infile >> paralist.output;  
	infile >> paralist.feature; //是否使用FD，使用为1，只用ED为0
}

void DataIo::display(const pcXYZIPtr &cloudS, const pcXYZIPtr &cloudT)
{


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;

	for (size_t i = 0; i < cloudS->points.size(); ++i)
	{
		pcl::PointXYZ pt;
		pt.x = cloudS->points[i].x;
		pt.y = cloudS->points[i].y;
		pt.z = cloudS->points[i].z;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(pt, 0.001, 0.0, 0.0, 1.0, s); //small scale
		//viewer->addSphere(pt, 0.8, 0.0, 0.0, 1.0, s); //large scale
		n++;

	}


	for (size_t i = 0; i < cloudT->points.size(); ++i)
	{
		pcl::PointXYZ pt;
		pt.x = cloudT->points[i].x;
		pt.y = cloudT->points[i].y;
		pt.z = cloudT->points[i].z;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(pt, 0.001, 0.0, 1.0, 0.0, s); //small scale
		//viewer->addSphere(pt, 0.8, 0.0, 0.0, 1.0, s); //large scale
		n++;

	}

	cout << "Click X(close) to continue..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void DataIo::displaymulti(const pcXYZIPtr &cloudS, const pcXYZIPtr &cloudICP, const pcXYZPtr &cloudIGSP)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;

	for (size_t i = 0; i < cloudS->points.size(); ++i)
	{
		pcl::PointXYZ pt;
		pt.x = cloudS->points[i].x;
		pt.y = cloudS->points[i].y;
		pt.z = cloudS->points[i].z;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(pt, 0.001, 0.0, 0.0, 1.0, s); //small scale
		//viewer->addSphere(pt, 0.8, 0.0, 0.0, 1.0, s); //large scale
		n++;

	}


	for (size_t i = 0; i < cloudICP->points.size(); ++i)
	{
		pcl::PointXYZ pt;
		pt.x = cloudICP->points[i].x;
		pt.y = cloudICP->points[i].y;
		pt.z = cloudICP->points[i].z;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(pt, 0.001, 0.0, 1.0, 0.0, s); //small scale
		//viewer->addSphere(pt, 0.8, 0.0, 0.0, 1.0, s); //large scale
		n++;

	}


	for (size_t i = 0; i < cloudIGSP->points.size(); ++i)
	{
		pcl::PointXYZ pt;
		pt.x = cloudIGSP->points[i].x;
		pt.y = cloudIGSP->points[i].y;
		pt.z = cloudIGSP->points[i].z;
		sprintf(t, "%d", n);
		s = t;
		viewer->addSphere(pt, 0.001, 1.0, 0.0, 0.0, s); //small scale
		//viewer->addSphere(pt, 0.8, 0.0, 0.0, 1.0, s); //large scale
		n++;

	}

	cout << "Click X(close) to continue..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void  DataIo::displayparameter(){
	cout << "!--------------------------Parameter Specification---------------------------!" << endl;
	if (paralist.feature == 0) cout << "G-ICP: Do not use feature to do the registration."<<endl;
	if (paralist.feature == 1) cout << "GH-ICP: Use BSC feature to do the registration." << endl;
	if (paralist.feature == 2) cout << "GH-ICP: Use FPFH feature to do the registration." << endl;
	if (paralist.output == 0) cout << "Do not output each iteration's result." << endl;
	if (paralist.output == 1) cout << "Output each iteration's result." << endl;
	cout << "Voxel Filtering Resolution:\t" << paralist.downsample_resolution << endl;
	cout << "Feature Calculation radius (m):\t" << paralist.feature_r << endl;
	cout << "Max Number of the KeyPoints in Point Cloud's Bounding Box:\t" << paralist.num_point_bb << endl;
	cout << "Keypoint Detection Max Curvature:\t" << paralist.keypoint_max_ratio << endl;
	cout << "Keypoint Detection Min Basing Points:\t" << paralist.keypoint_min_num << endl;
	cout << "Euclidean Scale:\t" << paralist.scale << endl;
	cout << "Penalty Parameter 1 (Initial):\t" << paralist.p_pre<< endl;
	cout << "Penalty Parameter 2 (Euclidean Distance):\t" << paralist.p_ED << endl;
	cout << "Penalty Parameter 3 (Feature Distance):\t" << paralist.p_FD << endl;
	cout << "Weight Changing Rate:\t" << paralist.m << endl;
	cout << "Terminal Threshold for K-M Algorithm:\t" << paralist.kmeps << endl;
	cout << "Convergence Condition for Translation (m)\t" << paralist.converge_t << endl;
	cout << "Convergence Condition for Rotation (deg)\t" << paralist.converge_r << endl;
	cout << "!----------------------------------------------------------------------------!" << endl;



}
/*点云分组*/
/* 奇一组，偶一组
string filename;
cout << "input file" << endl;
cin >> filename;
pcXYZPtr pointCloud0(new pcXYZ()), pointCloud1(new pcXYZ()), pointCloud2(new pcXYZ());
DataIo io;
io.readPcdFileXYZ(filename, pointCloud0);
cout << "Data loaded" << endl;
cout << "Point number:" << pointCloud0->size() << endl;
//io.writePcdFileXYZ("cloud0.pcd", pointCloud0);
//cout << "Output complete 0" << endl;

pointCloud1->width = pointCloud0->size() / 2;
pointCloud1->height = 1;
//pointCloud1->is_dense = false;

pointCloud1->points.resize(pointCloud1->width * pointCloud1->height);

pointCloud2->width = pointCloud0->size() / 2;
pointCloud2->height = 1;
//pointCloud2->is_dense = false;
pointCloud2->points.resize(pointCloud2->width * pointCloud2->height);

for (size_t i = 0; i < pointCloud0->size(); i++){


	if (i % 2 == 0){
		pointCloud1->points[(i + 2) / 2 - 1] = pointCloud0->points[i];

		//cout << pointCloud1->points[(i + 2) / 2 - 1].x << pointCloud1->points[(i + 2) / 2 - 1].y<<pointCloud1->points[(i + 2) / 2 - 1].z<<endl;
	}

	else{
		pointCloud2->points[(i + 1) / 2 - 1] = pointCloud0->points[i];

		//cout << pointCloud2->points[(i + 1) / 2 - 1 ].x << pointCloud2->points[(i + 1) / 2 - 1 ].y << pointCloud2->points[(i + 1) / 2 - 1 ].z << endl;
	}

}
io.writePcdFileXYZ("cloud1.pcd", pointCloud1);
io.writePcdFileXYZ("cloud2.pcd", pointCloud2);
cout << "Output complete" << endl;
*/