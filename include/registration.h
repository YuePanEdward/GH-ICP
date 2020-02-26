#ifndef _INCLUDE_REGISTRATION_H_
#define _INCLUDE_REGISTRATION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

#include "utility.h"
#include "km.h"
#include "stereo_binary_feature.h"

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
    
    Energyfunction(){;}
    

	void init(int kps_num, int kpt_num, float bbx_magnitude)
	{
		ED.resize(kps_num, vector<double>(kpt_num));
		FD.resize(kps_num, vector<double>(kpt_num));
		CD.resize(kps_num, vector<double>(kpt_num));

		penalty_initial = 2.5; //initial penalty for FD (mean-k*std)
		para1_penalty = 1.0;   //for ED penalty estimation
		para2_penalty = 1.0;   //for FD penalty estimation

		min_cor = 10;			  //min keypoint pairs for registration
		weight_changing_rate = 6; //ED,FD weight changing parameter
		KM_eps = 0.01;			  //KM's parameter (Smaller eps, longer consuming time)

		scale = 0.01 * bbx_magnitude;
	}
};

struct Keypoints
{
	int kps_num, kpt_num;
	Eigen::MatrixX3d kpSXYZ, kpTXYZ;

	Keypoints(){;}

    void init(int nkps, int nkpt, Eigen::MatrixX3d &kps, Eigen::MatrixX3d &kpt)
	{
		kps_num=nkps; 
		kpt_num=nkpt;
		kpSXYZ=kps;
		kpTXYZ=kpt; 
	}
	
};

class GHRegistration
{
  public:
	GHRegistration(int nkps, int nkpt, Eigen::MatrixX3d &kpSXYZ, Eigen::MatrixX3d &kpTXYZ,
				   float bbx_magnitude, float radiusNonMax,
				   float weight_adjustment_ratio, float weight_adjustment_step,
				   float converge_tran = 0.005, float converge_rot = 0.005,
				   int ite = 0, int ite2 = 0)
	{
		
		KP.init(nkps, nkpt, kpSXYZ, kpTXYZ);
		EF.init(nkps, nkpt,bbx_magnitude);

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
		adjustweight_ratio = weight_adjustment_ratio;
		adjustweight_step = weight_adjustment_step;
		converge_t = converge_tran;
		converge_r = converge_rot;
	}

	//Main entrance
	bool ghicp_reg();

	bool calED();

	bool calFD_BSC(const doubleVectorSBF &bscS, const doubleVectorSBF &bscT);
	bool calFD_FPFH(const fpfhFeaturePtr &fpfhS, const fpfhFeaturePtr &fpfhT);

	bool calCD_NF();
	bool calCD_BSC();
	bool calCD_FPFH();

	bool findcorrespondenceKM();
	bool findcorrespondenceNNR();
	bool findcorrespondenceNN();

	bool transformestimation(Eigen::Matrix4d &Rt);
	bool update(Eigen::Matrix4Xd &TKP, Eigen::Matrix4Xd &TFP, Eigen::Matrix4d &Rt);
	bool adjustweight(double estimated_overlap_ratio);

	bool calGTmatch(Eigen::Matrix4Xd &SKP, Eigen::Matrix4Xd &TKP0);
	bool cal_recall_precision();

	double calCloudFeatureDistance(int cor_num);

	int iteration_number; //real iteration number from 1
	int iteration_k;	  //iteration number determined for weight
	double RMS;
	double FDM;	//correspondence feature distance mean value
	double FDstd;  //correspondence feature distance standard deviation
	double IoU;	//Intersection over Union ratio of registration (Similarly to overlapping rate)
	double nonmax; //The radius of keypoint neighborhood curvature non-maximum suppression in meter

	bool converge;
	bool index_output;

	double converge_t; //convergence condition in translation (unit:m)
	double converge_r; //convergence condition in rotation    (unit:degree)

	float adjustweight_step;  //Weight adjustment for one iteration (0.1)
	float adjustweight_ratio; //Weight would be adjusted if the IoU between expected value and calculated value is beyond this value (1.2)

	double gt_maxdis; //ground truth max distance for correspondence keypoint pair

	double PCFD; //Pairwise Cloud Feature Distance (0-1)  used for multi-view registration as weight of MST

	Eigen::Matrix4d Rt_tillnow;
	Eigen::Matrix4d Rt_gt; //ground truth Rt matrix

	std::vector<std::vector<int>> matchlist; //Source Point Cloud match result for every iteration
	std::vector<int> gtmatchlist;			 //Ground truth match result for Source Point Cloud

	//save Target point cloud coordinates
	Eigen::Matrix4Xd skP, sfP, sP; //Source keypoints, Source downsampled points, Source full points (after temp transformation)
	Eigen::Matrix4Xd tkP;		   //Target keypoints

	//save energy,RMS before & after transformation, correspondence number, correspondence precision and recall of each iteration for displaying
	std::vector<double> energy, rmse, rmseafter, pre, rec;
	std::vector<int> cor;

  protected:
  private:
	Keypoints KP;
	Energyfunction EF;
	Eigen::MatrixX3d Spoint, Tpoint;
};

} // namespace ghicp

#endif //_INCLUDE_REGISTRATION_H_

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