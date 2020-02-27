#include <iostream>
#include <fstream>
#include "dataio.hpp"
#include "filter.hpp"
#include "keypoint_detect.hpp"
#include "binary_feature_extraction.hpp"
#include "cloud_viewer.hpp"
#include "ghicp_reg.h"
#include "utility.h"

using namespace ghicp;

typedef pcl::PointXYZI Point_T;

bool match_feature_type(char use_feature, FeatureType &Ft)
{
	switch (use_feature)
	{
	case 'B':
		Ft = BSC;
		break;
	case 'F':
		Ft = FPFH;
		break;
	case 'R':
		Ft = RoPS;
		break;
	case 'N':
		Ft = None;
		break;
	default:
		return 0;
	}
	return 1;
}

bool match_corres_type(char corres_estimation, CorrespondenceType &Ct)
{
	switch (corres_estimation)
	{
	case 'K':
		Ct = KM;
		break;
	case 'N':
		Ct = NN;
		break;
	case 'R':
		Ct = NNR;
		break;
	default:
		return 0;
	}
	return 1;
}

int main(int argc, char **argv)
{
	//Import configuration
	//Data path (4 formats are available: *.pcd, *.las, *.ply, *.txt)
	std::string filenameT = argv[1];                //Target pointcloud file path
	std::string filenameS = argv[2];                //Source pointcloud file path
	std::string filenameR = argv[3];                //Registered source pointcloud file path

	std::string use_feature = argv[4];				//BSC(B), FPFH(F), RoPS(R), None(N)
	std::string corres_estimation_method = argv[5]; //KM(K), NN(N), NNR(R)
	FeatureType Ft;
	CorrespondenceType Ct;
	match_feature_type(use_feature.c_str()[0], Ft);
	match_corres_type(corres_estimation_method.c_str()[0], Ct);

	float resolution = atof(argv[6]);               //control the subsampling scale (unit:m);
	float neighborhood_radius = atof(argv[7]);      //Curvature estimation / feature encoding radius
	float curvature_non_max_radius = atof(argv[8]); //Keypoint extraction based on curvature: non max suppression radius 
	float weight_adjustment_ratio = atof(argv[9]);  //Weight would be adjusted if the IoU between expected value and calculated value is beyond this value)
	float weight_adjustment_step = atof(argv[10]);  //Weight adjustment for one iteration)
	int reg_dof = atoi(argv[11]);                   //Degree of freedom of the transformation (4 (with leveling) or 6 (arbitary))
	float estimated_IoU = atof(argv[12]);           //Estimated approximate overlapping ratio of two point cloud
	bool launch_viewer = atoi(argv[13]);            //Launch the real-time viewer or not

	//Import the data
	DataIo<Point_T> dataio;
	pcl::PointCloud<Point_T>::Ptr pointCloudT(new pcl::PointCloud<Point_T>()), pointCloudS(new pcl::PointCloud<Point_T>());
	dataio.readCloudFile(filenameT, pointCloudT);
	dataio.readCloudFile(filenameS, pointCloudS);

	//Downsampling
	CFilter<Point_T> cfilter;
	pcl::PointCloud<Point_T>::Ptr pointCloudT_down(new pcl::PointCloud<Point_T>()), pointCloudS_down(new pcl::PointCloud<Point_T>());
	cfilter.voxelfilter(pointCloudT, pointCloudT_down, resolution);
	cfilter.voxelfilter(pointCloudS, pointCloudS_down, resolution);
	Bounds s_cloud_bbx;
	cfilter.getCloudBound(*pointCloudS_down, s_cloud_bbx);
	float bbx_magnitude = s_cloud_bbx.max_x - s_cloud_bbx.min_x + s_cloud_bbx.max_y - s_cloud_bbx.min_y + s_cloud_bbx.max_z - s_cloud_bbx.min_z;

	//Extract keypoints
	float non_stable_ratio_threshold= 0.65;
	CKeypointDetect<Point_T> ckpd(neighborhood_radius, non_stable_ratio_threshold, 20, curvature_non_max_radius);
	pcl::PointIndicesPtr keyPointIndicesT, keyPointIndicesS;
	ckpd.keypointDetectionBasedOnCurvature(pointCloudT_down, keyPointIndicesT);
	ckpd.keypointDetectionBasedOnCurvature(pointCloudS_down, keyPointIndicesS);
	int nkps = keyPointIndicesS->indices.size();
	int nkpt = keyPointIndicesT->indices.size();
	Eigen::MatrixX3d kpSXYZ, kpTXYZ;
	dataio.savecoordinates(pointCloudS_down, pointCloudT_down, keyPointIndicesS, keyPointIndicesT, kpSXYZ, kpTXYZ);
	Keypoints Kp;
	Kp.setCoordinate(kpSXYZ, kpTXYZ);

	//Feature encoding
	switch (Ft)
	{
	case BSC: //Use BSC feature
	{														 
		BSCEncoder<Point_T> bsc(curvature_non_max_radius, 7);
		doubleVectorSBF bscT, bscS;
		bsc.extractBinaryFeatures(pointCloudT_down, keyPointIndicesT, 0, bscT);       //Fixed feature(1 feature/CS for one keypoint)
		bsc.extractBinaryFeatures(pointCloudS_down, keyPointIndicesS, reg_dof, bscS); //6DOF feature (4 features/CSs for one keypoints)
		Kp.setBSCfeature(bscS, bscT);
		break;
	}
	case FPFH: //Use FPFH feature
	{
		FPFHfeature<Point_T> fpfh(curvature_non_max_radius);
		fpfhFeaturePtr fpfhT(new fpfhFeature), fpfhS(new fpfhFeature), fpfhT_k(new fpfhFeature), fpfhS_k(new fpfhFeature);
		fpfh.compute_fpfh_feature(pointCloudT_down, fpfhT);
		fpfh.compute_fpfh_feature(pointCloudS_down, fpfhS);
		fpfh.keyfpfh(fpfhS, fpfhT, keyPointIndicesS, keyPointIndicesT, fpfhS_k, fpfhT_k);
		Kp.setFPFHfeature(fpfhS_k, fpfhT_k);
		break;
	}
	case RoPS:  //Use RoPS feature
	{
		std::cout << "Not passed yet." << std::endl;
		break;
	}
	default:    //Do not use feature
	{
		std::cout << "Wrong feature input." << std::endl;
		return 0;
	}
	}

	//Apply registration
	Energyfunction Ef;
	Ef.init(nkps, nkpt, bbx_magnitude);
    Eigen::Matrix4d Rt_final; //transformation matrix from Source to Target Point Cloud
	pcl::PointCloud<Point_T>::Ptr pointCloudS_reg(new pcl::PointCloud<Point_T>());

	GHRegistration ghreg(Kp, Ef, Ft, Ct, curvature_non_max_radius, weight_adjustment_ratio, weight_adjustment_step,reg_dof, estimated_IoU);
	ghreg.set_raw_pointcloud(pointCloudS_down, pointCloudT_down);
	ghreg.set_viewer(launch_viewer);
	ghreg.ghicp_reg(Rt_final);
    
	pcl::transformPointCloud(*pointCloudS, *pointCloudS_reg, Rt_final.template cast<float>());
	dataio.writeCloudFile(filenameR, pointCloudS_reg); //Write out the transformed Source Point Cloud
    
    CloudViewer<Point_T> viewer;
	viewer.Dispaly2Cloud(pointCloudS_reg,pointCloudT,"Registration Result",5); //Show the registration result

	return 0;
}