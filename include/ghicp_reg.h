#ifndef _INCLUDE_GHICP_REG_H_
#define _INCLUDE_GHICP_REG_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

#include "utility.h"
#include "km.h"
#include "stereo_binary_feature.h"
#include "fpfh.hpp"

namespace ghicp
{
struct Energyfunction
{
	std::vector<std::vector<double>> ED, FD, CD;
	int weight_changing_rate; //weight function rate
	double penalty, para1_penalty, para2_penalty, penalty_initial;
	int min_cor;
	double KM_eps;
	float scale;

	Energyfunction() { ; }

	void init(int kps_num, int kpt_num, float bbx_magnitude)
	{
		ED.resize(kps_num, vector<double>(kpt_num));
		FD.resize(kps_num, vector<double>(kpt_num));
		CD.resize(kps_num, vector<double>(kpt_num));

		penalty_initial = 2.0; //initial penalty for FD (mean-k*std)
		para1_penalty = 1.0;   //for ED penalty estimation
		para2_penalty = 1.0;   //for FD penalty estimation

		min_cor = 10;			  //min keypoint pairs for registration
		weight_changing_rate = 6; //ED,FD weight changing parameter
		KM_eps = 0.01;			  //KM's parameter (Smaller eps, longer consuming time)

		scale = 0.005 * bbx_magnitude;
	}
};

struct Keypoints
{
	int kps_num, kpt_num;
	Eigen::MatrixX3d kpSXYZ, kpTXYZ;
	doubleVectorSBF bscS, bscT;
	fpfhFeaturePtr fpfhS, fpfhT;

	Keypoints() { ; }

	void setCoordinate(Eigen::MatrixX3d &kps, Eigen::MatrixX3d &kpt)
	{
		kpSXYZ = kps;
		kpTXYZ = kpt;
		kps_num = kpSXYZ.rows();
		kpt_num = kpTXYZ.rows();
	}

	void setBSCfeature(const doubleVectorSBF &bsc_S, const doubleVectorSBF &bsc_T)
	{
		bscS = bsc_S;
		bscT = bsc_T;
	}

	void setFPFHfeature(const fpfhFeaturePtr &fpfh_S, const fpfhFeaturePtr &fpfh_T)
	{
		fpfhS = fpfh_S;
		fpfhT = fpfh_T;
	}
};

class GHRegistration
{
  public:
	GHRegistration(Keypoints Kp, Energyfunction Ef, FeatureType Ft, CorrespondenceType Ct,
				   float radiusNonMax, float weight_adjustment_ratio, float weight_adjustment_step,
				   int dof_type, float estimated_IoU,
				   float converge_tran = 0.02, float converge_rot = 0.02,
				   int ite = 0, int ite2 = 0)
	{

		KP = Kp;
		EF = Ef;
		Ft_ = Ft;
		Ct_ = Ct;

		iteration_number = ite;
		iteration_k = ite2;
		nonmax = radiusNonMax;
		gt_maxdis = nonmax / 3;

		Rt_tillnow = Eigen::Matrix4d::Identity();
		converge = 0;

		index_output = 0;
		RMS = 99999;

		matchlist.resize(KP.kps_num, std::vector<int>(200));

		//Weight adjustment and convergence condition
		adjustweight_ratio_ = weight_adjustment_ratio;
		adjustweight_step_ = weight_adjustment_step;
		converge_t_ = converge_tran;
		converge_r_ = converge_rot;
		estimated_IoU_ = estimated_IoU;

		if (dof_type == 6)
			use_6dof_case_ = 1;
		else
			use_6dof_case_ = 0;

		pointCloudS_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
		pointCloudStemp_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
		pointCloudT_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
	}

	void set_raw_pointcloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloudS,
							const pcl::PointCloud<pcl::PointXYZI>::Ptr &pointCloudT)
	{
		pointCloudS_ = pointCloudS;
		pointCloudT_ = pointCloudT;
	}

	void set_viewer(bool launch_viewer)
	{
		launch_viewer_ = launch_viewer;
	}

	//Main entrance
	bool ghicp_reg(Eigen::Matrix4d &Rt_final);

	bool cal_gt_match(Eigen::Matrix4Xd &SKP, Eigen::Matrix4Xd &TKP0);
	bool cal_recall_precision();
	double calCloudFeatureDistance(int cor_num);

	double gt_maxdis;						 //ground truth max distance for correspondence keypoint pair
	double PCFD;							 //Pairwise Cloud Feature Distance (0-1) used for multi-view registration as weight of MST
	double RMS;								 //Temp root mean square error
	Eigen::Matrix4d Rt_tillnow;				 //Temp transformation matrix
	Eigen::Matrix4d Rt_gt;					 //ground truth transformation matrix
	std::vector<std::vector<int>> matchlist; //Source Point Cloud match result for every iteration
	std::vector<int> gtmatchlist;			 //Ground truth match result for Source Point Cloud

	//save energy,RMS before & after transformation, correspondence number, correspondence precision and recall of each iteration for displaying
	std::vector<double> energy, rmse, rmseafter, pre, rec;
	std::vector<int> cor;

	//save Target point cloud coordinates
	Eigen::Matrix4Xd skP, sfP, sP; //Source keypoints, Source downsampled points, Source full points (after temp transformation)
	Eigen::Matrix4Xd tkP;		   //Target keypoints

  protected:
  private:
	bool calED();

	bool calFD_BSC();
	bool calFD_FPFH();

	bool calCD_NF();
	bool calCD_BSC();
	bool calCD_FPFH();

	bool findcorrespondenceKM();
	bool findcorrespondenceNNR();
	bool findcorrespondenceNN();

	bool transformestimation(Eigen::Matrix4d &Rt);
	bool update(Eigen::Matrix4Xd &TKP, Eigen::Matrix4Xd &TFP, Eigen::Matrix4d &Rt);
	bool adjustweight();

	Keypoints KP;
	Energyfunction EF;
	FeatureType Ft_;
	CorrespondenceType Ct_;
	Eigen::MatrixX3d Spoint, Tpoint;

	pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloudS_, pointCloudStemp_, pointCloudT_;

	int iteration_number; //real iteration number from 1
	int iteration_k;	  //iteration number determined for weight

	double FDM;	//correspondence feature distance mean value
	double FDstd;  //correspondence feature distance standard deviation
	double IoU;	//Intersection over Union ratio of registration (Similarly to overlapping rate)
	double nonmax; //The radius of keypoint neighborhood curvature non-maximum suppression in meter

	bool converge;
	bool index_output;

	bool use_6dof_case_; //The ground truth transformation is 6 DOF (for the case of TLS with leveling, you can use 4 DOF case, this value would be 0)

	double converge_t_; //convergence condition in translation (unit:m)
	double converge_r_; //convergence condition in rotation    (unit:degree)

	float adjustweight_step_;  //Weight adjustment for one iteration (0.1)
	float adjustweight_ratio_; //Weight would be adjusted if the IoU between expected value and calculated value is beyond this value (1.1)

	float estimated_IoU_;

	bool launch_viewer_;
};

} // namespace ghicp

#endif //_INCLUDE_GHICP_REG_H_

#if 0
    bool displayCorrespondence(const pcXYZIPtr &cloudS, Eigen::Matrix4Xd &TP);

	// Display in different color [ you need to use template here, it's stupid to do like that ]
	void displayPCrb(const pcXYZIPtr &cloudT, Eigen::Matrix4Xd &cloudS, Eigen::Matrix4Xd &KpT, Eigen::Matrix4Xd &kpS);
	void displayPCyb(const pcXYZIPtr &cloudT, Eigen::Matrix4Xd &cloudS, Eigen::Matrix4Xd &KpT, Eigen::Matrix4Xd &kpS);
	void displayPCvb(const pcXYZIPtr &cloudT, Eigen::Matrix4Xd &cloudS, Eigen::Matrix4Xd &KpT, Eigen::Matrix4Xd &kpS);

	//pcl::PointCloud<pcl::PointXYZI> ::Ptr output(const pcXYZIPtr &cloud, Eigen::Matrix4d &Rt);
	void output(Eigen::Matrix4Xd &TP);
	void save(const pcXYZIPtr &cloudfS, const pcXYZIPtr &cloudS, Eigen::MatrixX3d &kpSXYZ_0, Eigen::MatrixX3d &kpTXYZ_0);
	void energyRMSoutput();
    
	void readGTRT();

	//Comparison with other methods
	void Reg_3DNDT(pcXYZIPtr CloudS, pcXYZIPtr CloudT);
	void Reg_FPFHSAC(pcXYZIPtr CloudS, pcXYZIPtr CloudT);
	void Reg_ClassicICP(pcXYZIPtr CloudS, pcXYZIPtr CloudT);
#endif