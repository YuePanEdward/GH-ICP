#ifndef RG
#define RG

#include<pcl\point_cloud.h>
#include<pcl\point_types.h>
#include <Eigen/dense>
#include <vector>
#include "utility.h"
#include "KM.h"
#include "StereoBinaryFeature.h"

using namespace utility;
using namespace std;
using namespace KMSpace;
using namespace Eigen;

namespace Reg
{
	struct Energyfunction
	{
		std::vector<vector<double>> ED;
		std::vector<vector<double>> FD;
		std::vector<vector<double>> CD;
		int m;//weight function rate
		double penalty;
		int min_cor;
		double para1_penalty;
		double para2_penalty;
		double penalty_initial;
		double KM_eps;
		int k;
	};

	struct Keypoints
	{
		int kps_num;
		int kpt_num;
		//vector<vector<double>> kpSXYZ;
		//vector<vector<double>> kpTXYZ;
		Eigen::Matrix<double, Dynamic, 3> kpSXYZ;
		Eigen::Matrix<double, Dynamic, 3> kpTXYZ;
	};

	
	class Registration
	{
	public:

		Registration(Keypoints KP_m , Energyfunction EF_m , int ite , int ite2 ){
			KP = KP_m;
			EF = EF_m;
			iteration_number = ite;
			iteration_k = ite2;
		}
		
		void calED();
		void calFD(const doubleVectorSBF &bscS, const doubleVectorSBF &bscT);
		void calCD();
		void findcorrespondence();
		void transformestimation(Eigen::Matrix4d &Rt);
		void update(Eigen::Matrix4Xd &TKP, Eigen::Matrix4Xd &TFP, Eigen::Matrix4d &Rt);
		void displayCorrespondence(const pcXYZIPtr &cloudS, Eigen::Matrix4Xd &TP);
		void displayPC(const pcXYZIPtr &cloudS, Eigen::Matrix4Xd &cloudT, Eigen::Matrix4Xd &kpS, Eigen::Matrix4Xd &kpT);
		//pcl::PointCloud<pcl::PointXYZI> ::Ptr output(const pcXYZIPtr &cloud, Eigen::Matrix4d &Rt);
		void output(Eigen::Matrix4Xd &TP);
		void save(const pcXYZIPtr &cloudfT, const pcXYZIPtr &cloudT, Eigen::MatrixX3d &kpTXYZ_0, Eigen::MatrixX3d &kpSXYZ_0);
		void energyRMSoutput();
		void calGTmatch(Eigen::Matrix4Xd &SKP, Eigen::Matrix4Xd &TKP0);
		void cal_recall_precision();
		void readGTRT();

		double calCloudFeatureDistance(int cor_num);
		
		double PCFD;  //Pairwise Cloud Feature Distance (0-1)  used for multi-view registration as weight of MST

		int iteration_number; //real iteration number from 1
		int iteration_k;      //iteration number determined for weight
		double RMS;
		double FDM;           //correspondence feature distance mean value
		double FDstd;         //correspondence feature distance standard diviation
		//double CDmax;
		bool converge;
		bool index_output;

		double converge_t;    //convergence condition in translation (unit:m)
		double converge_r;    //convergence condition in rotation    (unit:degree)

		double gt_maxdis;     //ground truth max distance for correspondence keypoint pair

		Eigen::Matrix4d Rt_tillnow;
		Eigen::Matrix4d Rt_gt; //ground truth Rt matrix

		vector<vector<int>> matchlist; //Source Point Cloud match result for every iteration 
		vector<int> gtmatchlist;  // ground truth match result for Source Point Cloud

		//save Target point cloud coordinates
		Eigen::Matrix4Xd tfP;
		Eigen::Matrix4Xd tP;
		Eigen::Matrix4Xd tkP;
		Eigen::Matrix4Xd skP;

		//save energy ,RMS and correspondence number of each iteration for displaying
		vector<double> energy; 
		vector<double> rmse;
		vector<int> cor;

	protected:

	private:
		Keypoints KP;
		Energyfunction EF;
		Eigen::Matrix<double, Dynamic, 3> Spoint;
		Eigen::Matrix<double, Dynamic, 3> Tpoint;

		
	};

	
}

















#endif