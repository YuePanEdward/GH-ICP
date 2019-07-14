#ifndef RG
#define RG

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
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
		float k;
		int usefeature;
	};

	struct Keypoints
	{
		int kps_num;
		int kpt_num;
		Eigen::Matrix<double, Dynamic, 3> kpSXYZ;
		Eigen::Matrix<double, Dynamic, 3> kpTXYZ;
		
		/*
		Keypoints(int nkps, int nkpt, Eigen::Matrix<double, Dynamic, 3> & kps, Eigen::Matrix<double, Dynamic, 3> & kpt)
		{
			kps_num = nkps;
			kpt_num = nkpt;
			kpSXYZ = kps;
			kpTXYZ = kpt;
		}
		*/
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

		void calFD_BSC(const doubleVectorSBF &bscS, const doubleVectorSBF &bscT);
		void calFD_FPFH(const fpfhFeaturePtr &fpfhS, const fpfhFeaturePtr &fpfhT);

		void calCD_NF();
		void calCD_BSC();
		void calCD_FPFH();

		void findcorrespondenceKM();
		void findcorrespondenceNNR();
		void findcorrespondenceNN();

		void transformestimation(Eigen::Matrix4d &Rt);
		void update(Eigen::Matrix4Xd &TKP, Eigen::Matrix4Xd &TFP, Eigen::Matrix4d &Rt);
		void adjustweight(double estimated_overlap);
		
		void displayCorrespondence(const pcXYZIPtr &cloudS, Eigen::Matrix4Xd &TP);

		// Display in different color [ you need to use template here, it's stupid to do like that ]
		void displayPCrb(const pcXYZIPtr &cloudT, Eigen::Matrix4Xd &cloudS, Eigen::Matrix4Xd &KpT, Eigen::Matrix4Xd &kpS);
		void displayPCyb(const pcXYZIPtr &cloudT, Eigen::Matrix4Xd &cloudS, Eigen::Matrix4Xd &KpT, Eigen::Matrix4Xd &kpS);
		void displayPCvb(const pcXYZIPtr &cloudT, Eigen::Matrix4Xd &cloudS, Eigen::Matrix4Xd &KpT, Eigen::Matrix4Xd &kpS);

		//pcl::PointCloud<pcl::PointXYZI> ::Ptr output(const pcXYZIPtr &cloud, Eigen::Matrix4d &Rt);
		void output(Eigen::Matrix4Xd &TP);
		void save(const pcXYZIPtr &cloudfS, const pcXYZIPtr &cloudS, Eigen::MatrixX3d &kpSXYZ_0, Eigen::MatrixX3d &kpTXYZ_0);
		void energyRMSoutput();
		void calGTmatch(Eigen::Matrix4Xd &SKP, Eigen::Matrix4Xd &TKP0);
		void cal_recall_precision();
		void readGTRT();

		double calCloudFeatureDistance(int cor_num);
		
		//Comparison with other methods
		void Reg_3DNDT(pcXYZIPtr CloudS, pcXYZIPtr CloudT);
		void Reg_FPFHSAC(pcXYZIPtr CloudS, pcXYZIPtr CloudT);
		void Reg_ClassicICP(pcXYZIPtr CloudS, pcXYZIPtr CloudT);

		double PCFD;          //Pairwise Cloud Feature Distance (0-1)  used for multi-view registration as weight of MST

		int iteration_number; //real iteration number from 1
		int iteration_k;      //iteration number determined for weight
		double RMS;
		double FDM;           //correspondence feature distance mean value
		double FDstd;         //correspondence feature distance standard deviation
		double IoU;           //Intersection over Union ratio of registration (Similarly to overlapping rate)
		double nonmax;        //The radius of keypoint neighborhood curvature non-maximum suppression in meter


		//double CDmax;
		bool converge;
		bool index_output;

		double converge_t;    //convergence condition in translation (unit:m)
		double converge_r;    //convergence condition in rotation    (unit:degree)

		float adjustweight_step;   //Weight adjustment for one iteration (0.1)
		float adjustweight_ratio;  //Weight would be adjusted if the IoU between expected value and calculated value is beyond this value (1.2)

		double gt_maxdis;     //ground truth max distance for correspondence keypoint pair

		Eigen::Matrix4d Rt_tillnow;
		Eigen::Matrix4d Rt_gt; //ground truth Rt matrix

		vector<vector<int>> matchlist; //Source Point Cloud match result for every iteration 
		vector<int> gtmatchlist;  // ground truth match result for Source Point Cloud

		//save Target point cloud coordinates
		Eigen::Matrix4Xd sfP;
		Eigen::Matrix4Xd sP;
		Eigen::Matrix4Xd tkP;
		Eigen::Matrix4Xd skP;

		//save energy ,RMS and correspondence number of each iteration for displaying
		vector<double> energy; 
		vector<double> rmse;
		vector<double> rmseafter; //RMSE after transformation
		vector<int> cor;
		vector<double> pre;  //precision
		vector<double> rec;  //recall


	protected:

	private:
		Keypoints KP;
		Energyfunction EF;
		Eigen::Matrix<double, Dynamic, 3> Spoint;
		Eigen::Matrix<double, Dynamic, 3> Tpoint;

		
	};

	
}

















#endif