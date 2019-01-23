#ifndef DATAIO 
#define DATAIO

#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>  
#include <pcl/io/ply_io.h>

#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

#include <vector>
#include "utility.h"

using namespace utility;
using namespace std;

struct Paralist{

	/*--------------------------------------------/
		Notification
		1  resolution of voxel downsampling
		2  keypoint_non_max_r
		3  feature_r
		4  keypoint_max_ratio
		5  keypoint_min_num
		6  scale
		7  p_pre
		8  p_ED
		9  p_FD
		10 iteration_speed
		11 km_eps
		12 weight_adjustment_ratio
		13 weight_adjustment_step
		14 converge_t
		15 converge_r
		16 Which Feature(0:No, 1 : BSC, 2 : FPFH)
		17 Which Method(0:NN, 1 : KM)
		18 Output or not(0:No, 1 : Yes)
		/--------------------------------------------*/

	// GH-ICP Point Cloud Registration Parameter List (Default Value) 
	// Down sampling Parameter
	float downsample_resolution =0.04;    //1  resolution of down sampling

	// Key point Detection Parameter
	//float num_point_bb;
	float keypoint_non_max = 1.0;         //2  keypoint_non_max_r (m)  [Key]
	float feature_r = 0.6;                //3  feature_r (m)           
	float keypoint_max_ratio = 0.75;      //4  keypoint_max_ratio      [Key]
	int keypoint_min_num = 30;            //5  keypoint_min_num

	// Registration Iterative Parameter
	float scale = 1;                      //6  Euclidean distance scale
	float p_pre = 2.5;                    //7  paremeter_pre
	float p_ED = 1;                       //8  parameter_ED_iteration
	float p_FD = 1;                       //9  parameter_FD_iteration
	float iter_speed = 6;                 //10 iteration speed parameter   
	float kmeps=0.01;                     //11 KM's eps parameter       
	float weight_adjustment_ratio = 1.1;  //12 Weight would be adjusted if the IoU between expected value and calculated value is beyond this value  
	float weight_adjustment_step = 0.1;   //13 Weight adjustment for one iteration 
	float converge_t=0.001;               //14 translation_iteration_terminal_condition (m)
	float converge_r=0.001;               //15 rotation_iteration_terminal_condition (degree)
	
	// Options (GH-ICP default)
	int feature=1;                        //16 Which Feature to use (0: No, 1: BSC, 2: FPFH)   [Key]
	int correspondence_type=1;            //17 Which Method to use (0: NN, 1: KM)              [Key]
	bool output=1;                        //18 Output or not (0: No, 1: Yes)

};


class DataIo :CloudUtility
{
public:
	// pcd 文件读写;
	bool readPcdFile(const string &fileName, const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud);
	bool readPcdFileXYZ(const string &fileName, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);
	bool writePcdFile(const string &fileName, const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud);
	bool writePcdFileXYZ(const string &fileName, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud);
	
	// las 文件读写;
	bool readLasFileHeader(const std::string &fileName, liblas::Header& header);
	bool readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud); 
	bool readLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud, double & X_min, double & Y_min);
	bool writeLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud, double X_min, double Y_min);
	
	// ply 文件读写;
	bool readPlyFile(const string &fileName, const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloud);

	// txt 文件读写;

	// Others
	void outputwhat();

	void display(const pcXYZIPtr &cloudS, const pcXYZIPtr &cloudT);
	void displaymulti(const pcXYZIPtr &cloudS, const pcXYZIPtr &cloudICP, const pcXYZPtr &cloudIGSP);

	// Parameter_list 文件读写;
	void readParalist(string paralistfile);
	void displayparameter();

	Paralist paralist;
protected:

private:
	
};


#endif