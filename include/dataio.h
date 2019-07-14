//
// This file is used for the Reading, Writing and Displaying of Point Cloud of various formats.
// Dependent 3rd Libs: PCL (>1.7)  liblas  VTK
// Author: Zhen Dong , Yue Pan et al. @ WHU LIESMARS
//

#ifndef DATAIO_H 
#define DATAIO_H

//pcl
#include <pcl/io/pcd_io.h>  
#include <pcl/io/ply_io.h>

//liblas
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
	float downsample_resolution =0.05;    //1  resolution of down sampling

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
	int feature=1;                        //16 Which Feature to use (0: No, 1: BSC, 2: FPFH)     [Key]
	int correspondence_type=1;            //17 Which Method to use (0: NN, 1: KM)                [Key]
	bool output=1;                        //18 Output or not (0: No, 1: Yes)

};


template <typename PointT>
class DataIo : public CloudUtility<PointT>
{
public:

	/* //Constructor
	DataIo(){
		
	}*/
	
	// Please add some randomly sub-sampling output methods
    
	bool HDmap_data_import (const string &pointcloud_folder, const string &pointcloud_fileList,
       const string &pose_fileName, const string &imu_fileName,int begin_frame_id, int end_frame_id, Transaction & transaction_data);

	// Point Cloud IO for all kinds of formats;
	bool readCloudFile     (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writeCloudFile    (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);

	// pcd IO;
	bool readPcdFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writePcdFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	
	// las IO;
	bool readLasFileHeader (const string &fileName, liblas::Header& header);
	bool readLasFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud); //Without translation
	bool writeLasFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud); //Without translation
	bool readLasFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not);     //With translation @Overload
	bool writeLasFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not);     //With translation @Overload
	bool readLasFileLast   (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud); //With translation of the last global shift

	// ply IO;
	bool readPlyFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writePlyFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	
	// txt IO;
	bool readTxtFile       (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writeTxtFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud);
	bool writeTxtFile      (const string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, int subsample_ratio);  //Randomly Subsample for saving @Overload

	// dxf IO;
    
    // pose/imu IO;
	bool readposes(const string &fileName, std::vector<Eigen::Matrix4f> &poses);
    bool readimudata(const string &fileName, std::vector<std::vector<IMU_data> > &imu_datas);
    
    // Output Pose Graph Information;
	bool writePoseGraph(const string &output_folder, Transaction &transaction);
    bool writeOdomPose(const string &output_folder, Transaction &transaction);

	// Read active object bounding boxs for active object filtering

	// Batch read filename from folder
	bool batchReadFileNamesInFolders(const std::string &folderName, const std::string & extension, std::vector<std::string> &fileNames);
	bool batchReadFileNamesInSubFolders(const std::string &folderName, const std::string & extension, std::vector<std::vector<std::string> > &fileNames);
	bool batchReadFileNamesInFoldersAndSubFolders(const std::string &folderName, const std::string & extension, std::vector<std::string> &fileNames);
	// Batch read multi-source filename from folder in one line
	void batchReadMultiSourceFileNamesInDataFolders(const std::string &ALS_folder, const std::string &TLS_folder, const std::string &MLS_folder, const std::string &BPLS_folder,
		std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files);

	// Display point cloud via VTK;
	void display1cloud        (const typename pcl::PointCloud<PointT>::Ptr &cloud, std::string displayname, int display_downsample_ratio);
	void display2clouds       (const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, std::string displayname, int display_downsample_ratio);
	void displaymulticlouds   (const typename pcl::PointCloud<PointT>::Ptr &cloud0, const typename pcl::PointCloud<PointT>::Ptr &cloud1, const typename pcl::PointCloud<PointT>::Ptr &cloud2, std::string displayname);
	void displaynclouds       (const typename std::vector<pcl::PointCloud<PointT> > & clouds, std::string displayname, int display_downsample_ratio);
    void display_submap       (const SubMap & submap, std::string displayname, color_type color_mode, int display_downsample_ratio);
	
	//Display of Graph 
	void display2Dboxes (const vector<CloudBlock> &blocks);
	void display2Dcons  (const vector<Constraint> &cons);
    void display_hdmap_edges (const vector<Edge_between_2Frames> &cons);
	void display_hdmap_edges (const vector<Edge_between_2Submaps> &cons);

	// For ALS Division
	void ALS_block_by_time(const typename pcl::PointCloud<PointT>::Ptr &pointCloud, typename std::vector<pcl::PointCloud<PointT> > &CloudBlocks, float time_step_in_second);
	bool batchWriteBlockInColor(const string &fileName, typename std::vector<pcl::PointCloud<PointT> > &CloudBlocks, bool automatic_shift_or_not);

	// For coordinate transformation
	void pcXYZ2XY(const typename pcl::PointCloud<PointT>::Ptr &pointcloudxyz, pcl::PointCloud<pcl::PointXY>::Ptr &pointcloudxy);
	bool readindiceslist(std::vector<int> & indicesA, std::vector<int> & indicesB);
	bool readindiceslist(const typename pcl::PointCloud<PointT>::Ptr &CloudA, const typename pcl::PointCloud<PointT>::Ptr &CloudB, std::vector <std::vector<double> > & coordinatesA, std::vector <std::vector<double> > & coordinatesB);
	bool read_XYZ_XYZlist(std::vector <std::vector<double> > & coordinatesA, std::vector <std::vector<double> > & coordinatesB);
	bool read_XYZ_BLHlist(std::vector <std::vector<double> > & coordinatesSC_XYZ, std::vector <std::vector<double> > & coordinatesUTM_XYZ);
	bool tran_eng2utm(float centerlong_eng_proj);
	bool tran_wgs2eng(float centerlong_eng_proj, float proj_surface_h_eng);
	bool XYZ_4DOFCSTran(std::vector<double> &transpara); //4 parameters transform (plane)
	bool XYZ_7DOFCSTran(std::vector<double> &transpara); //7 parameters transform (space)
	bool lasfileGK2UTM(const string &fileName); //Gauss Projection to UTM projection
	bool lasfileshift(const string &fileName, vector<double> &shift);  // translation
	double cal_cor_RMSE(std::vector <std::vector<double> > & coordinatesA, std::vector <std::vector<double> > & coordinatesB);

	//Read in blocks
	bool readLasBlock(const string &fileName, CloudBlock & block);                                   //Without translation
	bool readLasBlock(const string &fileName, int data_type_, int strip_num_, int num_in_strip_, CloudBlock & block);//@Overload
	bool readLasBlock(const string &fileName, CloudBlock & block, bool automatic_shift_or_not);      //With translation @Overload
	void batchReadMultiSourceLasBlock(std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files,
		std::vector<std::vector<CloudBlock> > &ALS_strip_blocks, std::vector<CloudBlock> &TLS_blocks, std::vector<CloudBlock> &MLS_blocks, std::vector<CloudBlock> &BPLS_blocks, std::vector<CloudBlock> &All_blocks);

	//Read point cloud pair from constraint
	void readLasCloudPairfromCon(const Constraint &this_con, std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files, 
		string &Filename1, string &Filename2, typename pcl::PointCloud<PointT>::Ptr &cloud1, typename pcl::PointCloud<PointT>::Ptr &cloud2);

	//Batch down-sample point cloud pair
	void batchdownsamplepair(const Constraint &this_con, typename pcl::PointCloud<PointT>::Ptr &cloud1, typename pcl::PointCloud<PointT>::Ptr &cloud2, typename pcl::PointCloud<PointT>::Ptr &subcloud1, typename pcl::PointCloud<PointT>::Ptr &subcloud2,
		float ALS_radius, float TLS_radius, float MLS_radius, float BPLS_radius);

	//Batch output final point clouds
	void batchwritefinalcloud(vector<CloudBlock> &All_blocks, std::vector<std::vector<std::string> > &ALS_strip_files, std::vector<std::string> &TLS_files, std::vector<std::string> &MLS_files, std::vector<std::string> &BPLS_files);

	
 

protected:

private:
	vector<double> global_shift;
	
};


#endif //DATAIO_H 