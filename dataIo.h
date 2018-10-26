#ifndef DATAIO 
#define DATAIO

#include<pcl\point_cloud.h>
#include<pcl\point_types.h>

#include <vector>
#include "utility.h"

using namespace utility;
using namespace std;

struct Paralist{

	float downsample_resolution;

	float num_point_bb;

	float feature_r;
	float keypoint_max_ratio;
	int keypoint_min_num;

	float scale;
	float p_pre;
	float p_ED;
	float p_FD;
	float m;
	float kmeps; //km算法的阈值 eps

	float converge_t;
	float converge_r;

	bool output;
	int feature;
};


class DataIo
{
public:
	// pcd 文件读写;
	bool readPcdFile(const string &fileName,  const pcXYZIPtr &pointCloud);
	bool readPcdFileXYZ(const string &fileName, const pcXYZPtr &pointCloud);
	bool writePcdFile(const string &fileName, const pcXYZIPtr &pointCloud);
	bool writePcdFileXYZ(const string &fileName, const pcXYZPtr &pointCloud);
	void outputwhat();
	void readParalist(string paralistfile);
	void display(const pcXYZIPtr &cloudS, const pcXYZIPtr &cloudT);
	void displaymulti(const pcXYZIPtr &cloudS, const pcXYZIPtr &cloudICP, const pcXYZPtr &cloudIGSP);
	void displayparameter();
	Paralist paralist;
protected:

private:
	
};


#endif