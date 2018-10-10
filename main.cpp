#include "dataIo.h"
#include "utility.h"
#include "voxelFilter.h"
#include "keypointDetection.h"
#include "BinaryFeatureExtraction.h"
#include "StereoBinaryFeature.h"
#include "KM.h"
#include "Registration.h"
#include <pcl\filters\extract_indices.h>
#include <pcl/console/parse.h>  
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/visualization/pcl_visualizer.h>  
#include <pcl/segmentation/supervoxel_clustering.h>  
#include <vtkPolyLine.h> 
#include <Eigen\dense>
#include <wtypes.h>
#include <pcl/registration/icp.h> 
#include <math.h>




using namespace std;
using namespace keypoint;
using namespace utility;
using namespace KMSpace;
using namespace Reg;
using namespace Eigen;


//Pan Yue WHU-SGG
int main()
{
	//timing
	DWORD t1, t2, t3, t4, t5, t6, t7, t8, t9, ts1 ,ts2 ,ts3 ,ts4;

	cout << "!----------------------------------------------------------------!" << endl;
	cout << "!           IGSP-Iterative Global Similarity Points              !" << endl;
	cout << "!             by Yue et al.                                      !" << endl;
	cout << "!----------------------------------------------------------------!" << endl;
	/*------ 1. data io------*/
	string filenameS, filenameT, paralistfile;
	cout << "input source file" << endl;
	cin >> filenameS;
	cout << "input target file" << endl;
	cin >> filenameT;
	cout << "input paralist file" << endl;
	cin >> paralistfile;


	pcXYZIPtr pointCloudS(new pcXYZI()), pointCloudT(new pcXYZI());
	//pcXYZPtr  pointclouds(new pcXYZ());
	DataIo io;
	t1 = GetTickCount();
	io.readPcdFile(filenameS, pointCloudS);
	io.readPcdFile(filenameT, pointCloudT);
	//pcl::copyPointCloud(*pointclouds, *pointCloudS); //from XYZI to XYZ
	cout << "Data loaded" << endl;
	cout << "Raw point number:" << pointCloudS->size() << " , " << pointCloudT->size()<<endl;
	io.readParalist(paralistfile);
	t2 = GetTickCount();
	cout << "Time for importing data: " << (t2 - t1)*1.0 / 1000 << " s" << endl
		 << "-----------------------------------------------------------------------------" << endl;
	
	
	//------------------Directly use ICP--------------------------------------
	/*cout << "Directly use ICP to register" << endl;
	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp0;
	icp0.setInputTarget(pointCloudS);
	icp0.setInputSource(pointCloudT);
	pcl::PointCloud<pcl::PointXYZI> DICP;
	ofstream ofs;
	ofs.open("ICP_RMSE.txt");
	if (ofs.is_open())
	{
		ofs << "iteration No.\t" << "RMSE" << endl;
		for (int i = 0; i < 60; i++)
		{
			//icp0.setTransformationEpsilon(1e-6);
			//icp0.setEuclideanFitnessEpsilon(0.01);
			icp0.setMaximumIterations(1);
			//icp0.setUseReciprocalCorrespondences(true);
			icp0.align(DICP);
			ofs << i + 1 << "\t" << sqrt(icp0.getFitnessScore()) << endl;
			icp0.setInputSource(DICP.makeShared());	
		}
		ofs.close();
	}
	//std::cout << icp0.getFinalTransformation() << std::endl;
	io.display(pointCloudS, DICP.makeShared());
	io.writePcdFile("DirectICP.pcd", DICP.makeShared());
	cout << "-----------------------------------------------------------------------------" << endl;*/
	//------------------Directly use ICP--------------------------------------
	

	/*------ 2. voxelization------*/

	float resolution = io.paralist.downsample_resolution;//control the subsampling scale (unit:m) ;//0.04f for TLS//0.00025f for bunny
	VoxelFilter<pcl::PointXYZI> filter(resolution);
	pcXYZIPtr filterPointCloudS(new pcXYZI()), filterPointCloudT(new pcXYZI());
	filterPointCloudS = filter.filter(pointCloudS);
	filterPointCloudT = filter.filter(pointCloudT);
	//filter.outputSimplifiedPCD("filterPointCloudS.pcd", filterPointCloudS);
	cout << "Point cloud filtered," <<"number:"<< filterPointCloudS->size()<<" , "<<filterPointCloudT->size()<<endl;
	t3 = GetTickCount();
	cout << "Time for filtering: " << (t3 - t2)*1.0 / 1000 << " s" << endl
		 << "-----------------------------------------------------------------------------" << endl;
	
	/*------ 3. keypoint detection------*/
	keypointOption kpOption;
	kpOption.radiusNonMax = pow(filter.V_boundingbox/io.paralist.num_point_bb,1.0/3); 
	cout << "BB: "<<filter.V_boundingbox<<"   Nonmax:  " << kpOption.radiusNonMax << endl;
	kpOption.radiusFeatureCalculation = io.paralist.feature_r;
	kpOption.ratioMax = io.paralist.keypoint_max_ratio;                    //0.85f
	kpOption.minPtNum = io.paralist.keypoint_min_num;                      //15 - 25
	//kpOption.radiusFeatureCalculation = 0.6f;    //0.5f for TLS and 0.0025f for bunny
	//kpOption.radiusNonMax =1.8f;                 //0.8f for TLS and 0.005f for bunny
	CkeypointDetection kpd(kpOption);
	
	//for ALS
	/*
	keypointOption kpOption2;
	kpOption2.radiusFeatureCalculation = 1.0f;   //1.0f for ALS 
	kpOption2.ratioMax = 0.85f;                  //0.85f
	kpOption2.minPtNum = 10;                     //10 - 15
	kpOption2.radiusNonMax = 1.0f;               //1.0f for ALS 
	CkeypointDetection kpd2(kpOption2);
	*/

	pcl::PointIndicesPtr keyPointIndicesS, keyPointIndicesT;
	kpd.keypointDetectionBasedOnCurvature(filterPointCloudS, keyPointIndicesS);
	kpd.outputKeypoints("keyPointIndicesS.txt", keyPointIndicesS, filterPointCloudS);
	cout << keyPointIndicesS->indices.size() << " Key points detected for Source Point Cloud " << endl;
	
	//for ALS
	//kpd2.keypointDetectionBasedOnCurvature(filterPointCloudT, keyPointIndicesT);
	//kpd2.outputKeypoints("keyPointIndicesT.txt", keyPointIndicesT, filterPointCloudT);

	kpd.keypointDetectionBasedOnCurvature(filterPointCloudT, keyPointIndicesT);
	kpd.outputKeypoints("keyPointIndicesT.txt", keyPointIndicesT, filterPointCloudT);
	cout << keyPointIndicesT->indices.size() << " Key points detected for Target Point Cloud " << endl;
	
	Eigen::MatrixX3d kpSXYZ;
	Eigen::MatrixX3d kpTXYZ;
	kpd.savecoordinates(filterPointCloudS, filterPointCloudT, keyPointIndicesS, keyPointIndicesT, kpSXYZ, kpTXYZ);
	t4 = GetTickCount();
	cout << "Time for keypoint extraction: " << (t4 - t3)*1.0 / 1000 << " s" << endl
		 << "-----------------------------------------------------------------------------" << endl;

	/*------ 4. feature calculation------*/
	float extract_radius = kpOption.radiusFeatureCalculation;	         //feature extract radius;//0.5 for TLS and  0.0025f for bunny
	unsigned int voxel_side_num = 7;     //每一维度的格子个数,建议取值为奇数,可以减少边缘效应对特征计算的影响;
	StereoBinaryFeatureExtractor bsc(extract_radius, voxel_side_num);  //创建对象;
	
	//for ALS
	//float extract_radius2 = 1.0;	     //特征提取半径;//1.0 for ALS 
	//unsigned int voxel_side_num2 = 7;  //每一维度的格子个数,建议取值为奇数,可以减少边缘效应对特征计算的影响;
	//StereoBinaryFeatureExtractor bsc2(extract_radius2, voxel_side_num2);  //创建对象;

	doubleVectorSBF bscS,bscT;

	bsc.extractBinaryFeatures(filterPointCloudS, keyPointIndicesS, bscS);
	cout << "Source Cloud BSC extracted" << endl;
	bsc.extractBinaryFeatures(filterPointCloudT, keyPointIndicesT, bscT);
    
	//for ALS
	//bsc2.extractBinaryFeatures(filterPointCloudT, keyPointIndicesT, bscT);
	cout << "Target Cloud BSC extracted" << endl;
	t5 = GetTickCount();
	cout << "Time for feature calculation: " << (t5 - t4)*1.0 / 1000 << " s" << endl
		 << "-----------------------------------------------------------------------------" << endl;

	/*------- 5. Registration----------*/
	Keypoints Kp;
	Kp.kps_num = keyPointIndicesS->indices.size();
	Kp.kpt_num = keyPointIndicesT->indices.size();
	Kp.kpSXYZ = kpSXYZ;
	Kp.kpTXYZ = kpTXYZ;
	Energyfunction Ef;
	Ef.ED.resize(Kp.kps_num, vector<double>(Kp.kpt_num));
	Ef.FD.resize(Kp.kps_num, vector<double>(Kp.kpt_num));
	Ef.CD.resize(Kp.kps_num, vector<double>(Kp.kpt_num));
	
	//parameter setting  //these can be fixed, what we need to do is to change the scale of ED according to filtered resolution
	//Ef.penalty_threshod = 1.6;//2.0//1.95
	
	//for penalty
	//Ef.penalty = 50;         // initial penalty for FD
	Ef.penalty_initial = io.paralist.p_pre;  // initial penalty for FD (mean-k*std)
	Ef.para1_penalty =io.paralist.p_ED;     // for ED penalty estimation
	Ef.para2_penalty =io.paralist.p_FD;     // for FD penalty estimation
	//others
	Ef.min_cor = 30;           // min keypoint pairs for registration
	Ef.m = io.paralist.m;                  // ED,FD weight parameter //m=10，8
	Ef.KM_eps = io.paralist.kmeps;          // KM's parameter (Smaller eps, longer consuming time)  //-----------originally 0.001
	Ef.usefeature = io.paralist.feature;
	//------------------------------------
	//Ef.k = io.paralist.scale / kpOption.radiusNonMax;             // EF's unit //10: dm //100 :cm //1 : m
	Ef.k = io.paralist.scale;
	

	Registration Reg(Kp, Ef, 0, 0);
	//initialization
	Reg.Rt_tillnow = Eigen::Matrix4d::Identity();
	Reg.converge = 0;
	Reg.index_output = 0;
	Reg.RMS = 1000.0;
	Reg.iteration_number = 0;    //if this begins from 1, which means ED !=0 for the first iteration
	Reg.matchlist.resize(Kp.kps_num, vector<int>(200));
	Reg.gt_maxdis = kpOption.radiusNonMax/3;
	//convergence condition
	Reg.converge_t = io.paralist.converge_t;
	Reg.converge_r = io.paralist.converge_r;
	//save TP ,TFP ,TKP and SKP
	Reg.save(filterPointCloudT, pointCloudT,kpTXYZ,kpSXYZ);
	//display original pc
	ts1 = GetTickCount();
	Reg.displayPCrb(filterPointCloudS, Reg.tfP, Reg.skP, Reg.tkP);
	ts2 = GetTickCount();

	//Read ground truth RT
	Reg.readGTRT();

	//cal FD(constant)
	Reg.calFD(bscS, bscT);

	t6 = GetTickCount();
	cout << "Time for registration preparation: " << (t6 - t5- ts2 + ts1)*1.0 / 1000 << " s" << endl;
	
	//output parameters
	cout << "parameter list:  " << endl << "ED,FD weight parameter m = " << Ef.m << "  minimum correspondence pairs = " << Ef.min_cor <<endl
		<< "original penalty for FD = " << Ef.penalty<<"   ED penalty estimation parameter = "<< Ef.para1_penalty
		<< "   FD penalty estimation parameter = " << Ef.para2_penalty<<endl
		<< "KM's parameter eps = " << Ef.KM_eps <<"   ED scalar weight = "<<Ef.k<< endl
		<< "-----------------------------------------------------------------------------" << endl;

	//iteration

	//for (int i = 0; i < 120; i++) // fixed iteration number
	while (!Reg.converge)    //till convergence
	{
		Eigen::Matrix4d Rt;
		cout << Reg.iteration_number << " : " << endl;
		DWORD ts,te1,te2,te3;
		ts = GetTickCount();
		Reg.calED();
		Reg.calCD();
		Reg.findcorrespondence(); //using  Kuhn–Munkres algorithm  (KM)   or  MCMF
        //Maybe we can add some geometric consistency terms here
		te1 = GetTickCount();
		//Reg.displayCorrespondence(filterPointCloudS, Reg.tfP); // display matching result
		Reg.transformestimation(Rt);//using  SVD
		Reg.Rt_tillnow = Rt*Reg.Rt_tillnow;
		cout <<"RT till now"<<endl<< Reg.Rt_tillnow << endl;
		te2 = GetTickCount();
		if (io.paralist.output==1) Reg.output(Reg.tfP);  //output result per iteration
		te3 = GetTickCount();
		cout << "Time for finding correspondence（KM) : " << (te1 - ts)*1.0 / 1000 << " s" << endl
			<< "Time for transform estimation and updating : " << (te2 - te1)*1.0 / 1000 << " s" << endl
			<< "Time for output temporal result : " << (te3 - te2)*1.0 / 1000 << " s" << endl
			<< "Time for one iteration : " << (te3 - ts)*1.0 / 1000 << " s" << endl
			<< "-----------------------------------------------------------------------------" << endl;
		Reg.iteration_number++;
		//Reg.iteration_k++;
	}

	Reg.energyRMSoutput();
	t7 = GetTickCount();
	
	cout << "converge_break" << endl;
	cout <<"final Rt without classic ICP" << endl<< Reg.Rt_tillnow << endl;
	cout << "IGSP Time: " << (t7 - t1- ts2 + ts1)*1.0 / 1000 << " s" << endl
	<< "-----------------------------------------------------------------------------" << endl;
	//display IGSP pc
	ts3 = GetTickCount();
	Reg.displayPCvb(filterPointCloudS, Reg.tfP, Reg.skP, Reg.tkP);
	ts4 = GetTickCount();
	//output IGSP temporal result
	Reg.output(Reg.tfP);
	

	//---------------------------classic ICP refinement--------------------------------//

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	
	//io.readPcdFileXYZ(filenameS, cloud_in);
	pcl::copyPointCloud(*filterPointCloudS, *cloud_in); //Copy pointcloud (format transfer) input: XYZI output:XYZ
	//cout << "source ok" << endl;
	Eigen::Matrix<double, 4, Dynamic> TransPC;
	TransPC.resize(4, Reg.tfP.cols());
	TransPC = Reg.Rt_tillnow*Reg.tfP;
	cloud_out->points.resize(TransPC.cols());
	
	for (size_t i = 0; i < TransPC.cols(); ++i)
	{
		cloud_out->points[i].x = TransPC(0, i);
		cloud_out->points[i].y = TransPC(1, i);
		cloud_out->points[i].z = TransPC(2, i);
	}
	//cout << "target ok" << endl;
	t8 = GetTickCount();
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_out);
	icp.setInputTarget(cloud_in);
	icp.setMaxCorrespondenceDistance(10*kpOption.radiusNonMax); 
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(0.01);
	icp.setMaximumIterations(20);
	icp.setUseReciprocalCorrespondences(true);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << endl;
	cout << "Classic ICP's Transfrom"<<endl<< icp.getFinalTransformation() << endl;
	t9 = GetTickCount();

	Reg.Rt_tillnow = icp.getFinalTransformation().cast<double>() *Reg.Rt_tillnow;  //use cast<type>() tp do the type transfer for eigen
	cout << "final Rt " << endl << Reg.Rt_tillnow << endl;
	io.writePcdFileXYZ("FinalRegistered.pcd", Final.makeShared());
	
	
    //io.displaymulti(pointCloudS, DICP.makeShared(),Final.makeShared());   //-------------change here-----------//
	//cout << "Classic ICP Time: " << (t9 - t8- ts4 + ts3)*1.0 / 1000 << " s" << endl;
	//cout << "Total Time: " << (t9 - t8 + t7 - t1- ts2 + ts1 -ts4 +ts3)*1.0 / 1000 << " s" << endl
		//<< "-----------------------------------------------------------------------------" << endl;
	//----------------------------------------------------------------------------------------//
	//display final pc
	//Reg.displayPC(filterPointCloudS, Reg.tfP, Reg.skP, Reg.tkP);
	
	//output 
	Reg.output(Reg.tP);
	Reg.index_output = 1;
	//Reg.output(Reg.tkP);
	//
	Reg.calGTmatch(Reg.skP, Reg.tkP);
	Reg.cal_recall_precision();

	cout << "Output for experiment done" << endl 
		 << "-----------------------------------------------------------------------------" << endl
		 << "Thanks for using this software." << endl << "By Yue Pan et al. @ Wuhan University & University of Alberta" << endl;
	//end
	int endok;
	cin >> endok;
	return 1;

	//for bunny,please lay the target cloud (xyz) horizontally because BSC is a 4DOF feature
	//You can try to use the 6 DOF feature like FPFH or SHOT
}

