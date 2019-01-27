#include "dataio.h"
#include "utility.h"

#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <fstream>
#include <vector>
#include <pcl/io/pcd_io.h>


using namespace  std;
using namespace  utility;

bool DataIo::readPcdFile(const std::string &fileName, const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZI>(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file\n");
		return false;
	}

	return true;
}

bool DataIo::readPcdFileXYZ(const std::string &fileName, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file\n");
		return false;
	}

	return true;
}


bool DataIo::writePcdFile(const string &fileName, const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud)
{
	if (pcl::io::savePCDFileBinary(fileName,*pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't write file\n");
		return false;
	}
	return true;
}

bool DataIo::writePcdFileXYZ(const string &fileName, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud)
{
	if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't write file\n");
		return false;
	}
	return true;
}

bool DataIo::readLasFileHeader(const std::string &fileName, liblas::Header& header)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}
	else
	{
		std::ifstream ifs;
		ifs.open(fileName, std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			return 0;
		}

		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);

		header = reader.GetHeader();
	}

	return 1;
}


bool DataIo::readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "未发现匹配项" << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();

	//header里可以直接提bounding box 出来
	double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
	Xmin = header.GetMinX();
	Ymin = header.GetMinY();
	Zmin = header.GetMinZ();
	Xmax = header.GetMaxX();
	Ymax = header.GetMaxY();
	Zmax = header.GetMaxZ();

	while (reader.ReadNextPoint())
	{
		const liblas::Point& p = reader.GetPoint();
		pcl::PointXYZI  pt;
		pt.x = p.GetX();
		pt.y = p.GetY();
		pt.z = p.GetZ();
		pt.intensity = p.GetIntensity();
		pointCloud->points.push_back(pt);
	}

	return 1;
}

bool DataIo::readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud, double & X_min, double  & Y_min)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "未发现匹配项" << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);
	liblas::Header const& header = reader.GetHeader();

	//header里可以直接提bounding box 出来
	double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
	Xmin = header.GetMinX();
	Ymin = header.GetMinY();
	Zmin = header.GetMinZ();
	Xmax = header.GetMaxX();
	Ymax = header.GetMaxY();
	Zmax = header.GetMaxZ();

	//int i = 0;
	/*while循环中遍历所有的点;*/
	while (reader.ReadNextPoint())
	{
		const liblas::Point& p = reader.GetPoint();
		//i++;
		pcl::PointXYZI  pt;
		/*将重心化后的坐标和强度值赋值给PCL中的点;*/
		/*做一个平移，否则在WGS84 下的点坐标太大了，会造成精度损失的 因为las的读取点数据是double的，而pcd是int的;*/
		pt.x = p.GetX() - Xmin;
		pt.y = p.GetY() - Ymin;
		pt.z = p.GetZ();
		pt.intensity = p.GetIntensity();
		pointCloud->points.push_back(pt);
	}

	// 平移后坐标原点
	X_min = Xmin;
	Y_min = Ymin;

	return 1;
}

bool DataIo::writeLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud, double X_min, double Y_min)
{
	Bounds bound;
	getCloudBound(*pointCloud, bound);

	ofstream ofs;
	ofs.open(fileName, std::ios::out | std::ios::binary);
	if (ofs.is_open())
	{
		liblas::Header header;
		header.SetDataFormatId(liblas::ePointFormat2);
		header.SetVersionMajor(1);
		header.SetVersionMinor(2);
		header.SetMin(bound.min_x + X_min, bound.min_y + Y_min, bound.min_z);
		header.SetMax(bound.max_x + X_min, bound.max_y + Y_min, bound.max_z);
		header.SetOffset((bound.min_x + bound.max_x) / 2.0 + X_min, (bound.min_y + bound.max_y) / 2.0 + Y_min, (bound.min_z + bound.max_z) / 2.0);
		header.SetScale(0.01, 0.01, 0.01);
		header.SetPointRecordsCount(pointCloud->points.size());


		liblas::Writer writer(ofs, header);
		liblas::Point pt(&header);

		for (int i = 0; i < pointCloud->points.size(); i++)
		{
			pt.SetCoordinates(double(pointCloud->points[i].x) + X_min, double(pointCloud->points[i].y) + Y_min, double(pointCloud->points[i].z));
			pt.SetIntensity(pointCloud->points[i].intensity);
			writer.WritePoint(pt);
		}
		ofs.flush();
		ofs.close();
	}

	return 1;
}

bool DataIo::readPlyFile(const string &fileName, const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud)
{
	if (pcl::io::loadPLYFile<pcl::PointXYZI>(fileName, *pointCloud) == -1) //* load the file 
	{
		PCL_ERROR("Couldn't read file \n");
		return (-1);
	}
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
	if (!infile.is_open()) cout << "Open file failure, Use default value" << endl;
	infile >> paralist.downsample_resolution;
	infile >> paralist.keypoint_non_max;
	infile >> paralist.feature_r;
	infile >> paralist.keypoint_max_ratio;
	infile >> paralist.keypoint_min_num;
	infile >> paralist.scale;
	infile >> paralist.p_pre;
	infile >> paralist.p_ED;
	infile >> paralist.p_FD;
	infile >> paralist.iter_speed;
	infile >> paralist.kmeps;
	infile >> paralist.weight_adjustment_ratio;
	infile >> paralist.weight_adjustment_step;
	infile >> paralist.converge_t;
	infile >> paralist.converge_r;
	infile >> paralist.feature;
	infile >> paralist.correspondence_type;
	infile >> paralist.output;

}

void DataIo::display(const pcXYZIPtr &cloudS, const pcXYZIPtr &cloudT, string displayname)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;

	pcXYZRGBPtr pointcloudS(new pcXYZRGB());
	pcXYZRGBPtr pointcloudT(new pcXYZRGB());

	for (size_t i = 0; i < cloudT->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = cloudT->points[i].x;
		pt.y = cloudT->points[i].y;
		pt.z = cloudT->points[i].z;
		pt.r = 255;
		pt.g = 215;
		pt.b = 0;
		pointcloudT->points.push_back(pt);
	} // Golden

	viewer->addPointCloud(pointcloudT, "pointcloudT");

	for (size_t i = 0; i < cloudS->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = cloudS->points[i].x;
		pt.y = cloudS->points[i].y;
		pt.z = cloudS->points[i].z;
		pt.r = 233;
		pt.g = 233;
		pt.b = 216;
		pointcloudS->points.push_back(pt);
	} // Silver

	viewer->addPointCloud(pointcloudS, "pointcloudS");
	
	cout << "Click X(close) to continue..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void DataIo::displaymulti(const pcXYZIPtr &cloud0, const pcXYZIPtr &cloud1, const pcXYZIPtr &cloud2, string displayname)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
	viewer->setBackgroundColor(255, 255, 255);
	char t[256];
	string s;
	int n = 0;

	pcXYZRGBPtr pointcloud0(new pcXYZRGB());
	pcXYZRGBPtr pointcloud1(new pcXYZRGB());
	pcXYZRGBPtr pointcloud2(new pcXYZRGB());

	for (size_t i = 0; i < cloud0->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = cloud0->points[i].x;
		pt.y = cloud0->points[i].y;
		pt.z = cloud0->points[i].z;
		pt.r = 255;
		pt.g = 0;
		pt.b = 0;
		pointcloud0->points.push_back(pt);
	} // Red

	viewer->addPointCloud(pointcloud0, "pointcloud_reference");

	for (size_t i = 0; i < cloud1->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = cloud1->points[i].x;
		pt.y = cloud1->points[i].y;
		pt.z = cloud1->points[i].z;
		pt.r = 0;
		pt.g = 0;
		pt.b = 255;
		pointcloud1->points.push_back(pt);
	} // Blue

	viewer->addPointCloud(pointcloud1, "pointcloud_reg_method1");

	for (size_t i = 0; i < cloud2->points.size(); ++i)
	{
		pcl::PointXYZRGB pt;
		pt.x = cloud2->points[i].x;
		pt.y = cloud2->points[i].y;
		pt.z = cloud2->points[i].z;
		pt.r = 0;
		pt.g = 255;
		pt.b = 0;
		pointcloud2->points.push_back(pt);
	} // Green

	viewer->addPointCloud(pointcloud2, "pointcloud_reg_method2");

	cout << "Click X(close) to continue..." << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void  DataIo::displayparameter(){
	cout << "!--------------------------Parameter Specification---------------------------!" << endl;
	if (paralist.feature == 0 && paralist.correspondence_type == 1) cout << "G-ICP: Do not use feature to do the registration." << endl;
	if (paralist.feature == 1 && paralist.correspondence_type == 1) cout << "GH-ICP: Use BSC feature to do the registration." << endl;
	if (paralist.feature == 2 && paralist.correspondence_type == 1) cout << "GH-ICP: Use FPFH feature to do the registration." << endl;
	if (paralist.feature == 0 && paralist.correspondence_type == 0) cout << "Classic ICP: Do not use feature to do the registration." << endl;
	if (paralist.feature == 1 && paralist.correspondence_type == 0) cout << "H-ICP: Use BSC feature to do the registration." << endl;
	if (paralist.feature == 2 && paralist.correspondence_type == 0) cout << "H-ICP: Use FPFH feature to do the registration." << endl;
	
	if (paralist.output == 0) cout << "Do not output each iteration's result." << endl;
	if (paralist.output == 1) cout << "Output each iteration's result." << endl;
	
	cout << "Voxel Filtering Resolution:\t" << paralist.downsample_resolution << endl;
	cout << "Feature Calculation Radius (m):\t" << paralist.feature_r << endl;
	//cout << "Max Number of the KeyPoints in Point Cloud's Bounding Box:\t" << paralist.num_point_bb << endl;
	cout << "Keypoint Detection Non-max Supression Radius (m):\t" << paralist.keypoint_non_max << endl;
	cout << "Keypoint Detection Max Curvature:\t" << paralist.keypoint_max_ratio << endl;
	cout << "Keypoint Detection Min Basing Points:\t" << paralist.keypoint_min_num << endl;
	cout << "Euclidean Distance Scale:\t" << paralist.scale << endl;
	cout << "Penalty Parameter 1 (Initial):\t" << paralist.p_pre<< endl;
	cout << "Penalty Parameter 2 (Euclidean Distance):\t" << paralist.p_ED << endl;
	cout << "Penalty Parameter 3 (Feature Distance):\t" << paralist.p_FD << endl;
	cout << "Weight Changing Rate:\t" << paralist.iter_speed << endl;
	cout << "Terminal Threshold for K-M Algorithm:\t" << paralist.kmeps << endl;
	cout << "Weight Adjustment Judgement Ratio:\t" << paralist.weight_adjustment_ratio << endl;
	cout << "Weight Adjustment Step:\t" << paralist.weight_adjustment_step<< endl;
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