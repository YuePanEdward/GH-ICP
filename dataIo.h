#ifndef DATAIO 
#define DATAIO

#include<pcl\point_cloud.h>
#include<pcl\point_types.h>

#include <vector>
#include "utility.h"

using namespace utility;
using namespace std;

struct Paralist{

	float num_point_bb;

	float feature_r;
	float keypoint_max_ratio;
	int keypoint_min_num;

	float scale;
	float p_pre;
	float p_ED;
	float p_FD;
	float m;

	float converge_t;
	float converge_r;

	bool output;
};


class DataIo
{
public:
	// pcd ÎÄ¼þ¶ÁÐ´;
	bool readPcdFile(const string &fileName,  const pcXYZIPtr &pointCloud);
	bool readPcdFileXYZ(const string &fileName, const pcXYZPtr &pointCloud);
	bool writePcdFile(const string &fileName, const pcXYZIPtr &pointCloud);
	bool writePcdFileXYZ(const string &fileName, const pcXYZPtr &pointCloud);
	void outputwhat();
	void readParalist(string paralistfile);
	
	Paralist paralist;
protected:

private:
	
};


#endif