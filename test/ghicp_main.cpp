#include <iostream>
#include <fstream>
#include "dataio.hpp"
#include "filter.hpp"
#include "keypoint_detect.hpp"
#include "binary_feature_extraction.hpp"
#include "registration.h"
#include "utility.h"

using namespace ghicp;

typedef pcl::PointXYZ Point_T;

bool match_feature_type(char use_feature, FeatureType Ft)
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

bool match_corres_type(char corres_estimation, CorrespondenceType Ct)
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
	std::string filenameT = argv[1]; //Target pointcloud file path
	std::string filenameS = argv[2]; //Source pointcloud file path
	std::string filenameR = argv[3]; //Registered source pointcloud file path

	std::string use_feature = argv[4]; //BSC(B), FPFH(F), RoPS(R), None(N)
	std::string corres_estimation_method = argv[5]; //KM(K), NN(N), NNR(R)
	FeatureType Ft;
	CorrespondenceType Ct;
    match_feature_type(use_feature.c_str()[0], Ft);
    match_corres_type(corres_estimation_method.c_str()[0], Ct);

	float resolution = atof(argv[6]); //control the subsampling scale (unit:m) ; //0.04f for TLS //0.00025f for bunny
	float neighborhood_radius = atof(argv[7]);
	float curvature_non_max_radius = atof(argv[8]);
	float weight_adjustment_ratio = atof(argv[9]); //(Weight would be adjusted if the IoU between expected value and calculated value is beyond this value)
	float weight_adjustment_step = atof(argv[10]); //(Weight adjustment for one iteration)

	float estimated_IoU = atof(argv[11]);

	DataIo<Point_T> dataio;
	pcl::PointCloud<Point_T>::Ptr pointCloudT(new pcl::PointCloud<Point_T>()), pointCloudS(new pcl::PointCloud<Point_T>());

	dataio.readCloudFile(filenameT, pointCloudT);
	dataio.readCloudFile(filenameS, pointCloudS);

	CFilter<Point_T> cfilter;
	pcl::PointCloud<Point_T>::Ptr pointCloudT_down(new pcl::PointCloud<Point_T>()), pointCloudS_down(new pcl::PointCloud<Point_T>());
	cfilter.voxelfilter(pointCloudT, pointCloudT_down, resolution);
	cfilter.voxelfilter(pointCloudS, pointCloudS_down, resolution);
	Bounds s_cloud_bbx;
	cfilter.getCloudBound(*pointCloudS_down, s_cloud_bbx);
	float bbx_magnitude = s_cloud_bbx.max_x - s_cloud_bbx.min_x + s_cloud_bbx.max_y - s_cloud_bbx.min_y + s_cloud_bbx.max_z - s_cloud_bbx.min_z;

	CKeypointDetect<Point_T> ckpd(neighborhood_radius, 0.8, 20, curvature_non_max_radius);

	pcl::PointIndicesPtr keyPointIndicesT, keyPointIndicesS;
	ckpd.keypointDetectionBasedOnCurvature(pointCloudT_down, keyPointIndicesT);
	ckpd.keypointDetectionBasedOnCurvature(pointCloudS_down, keyPointIndicesS);
	int nkps = keyPointIndicesS->indices.size();
	int nkpt = keyPointIndicesT->indices.size();

	Eigen::MatrixX3d kpSXYZ, kpTXYZ;
	dataio.savecoordinates(pointCloudS_down, pointCloudT_down, keyPointIndicesS, keyPointIndicesT, kpSXYZ, kpTXYZ);

	Keypoints Kp;
	Kp.setCoordinate(kpSXYZ, kpTXYZ);

	switch (Ft)
	{
	case BSC:
	{														  //Use BSC feature
		BSCEncoder<Point_T> bsc(curvature_non_max_radius, 7); //Create BSC object.
		doubleVectorSBF bscT, bscS;
		bsc.extractBinaryFeatures(pointCloudT_down, keyPointIndicesT, bscT);
		bsc.extractBinaryFeatures(pointCloudS_down, keyPointIndicesS, bscS);
		Kp.setBSCfeature(bscS, bscT);
		break;
	}
	case FPFH: //Use FPFH feature
	{		  //fm.detectFeaturesORB(frames[i], feature_extract_parameter);
		std::cout << "Not ready yet" << std::endl;
		break;
	}
	default:
	{
		std::cout << "Wrong feature input." << std::endl;
		return 0;
	}
	}

	Energyfunction Ef;
	Ef.init(nkps, nkpt, bbx_magnitude);

	GHRegistration ghreg(Kp,Ef,Ft,Ct,curvature_non_max_radius, weight_adjustment_ratio, weight_adjustment_step, estimated_IoU);
    Eigen::Matrix4d Rt_final;
	ghreg.ghicp_reg(Rt_final);

    return 0;
}