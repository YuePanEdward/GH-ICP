#include "utility.h"
#include "keypointDetection.h"
#include "BinaryFeatureExtraction.h"
#include "StereoBinaryFeature.h"
#include "KM.h"
#include "fpfh.h"
#include "dataio.cpp"
#include "Registration.h"
#include "voxelFilter.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/features/fpfh.h>
#include <pcl/console/parse.h>  
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/segmentation/supervoxel_clustering.h>  
#include <Eigen/dense>
#include <wtypes.h>
#include <pcl/registration/icp.h> 
#include <math.h>

using namespace std;
using namespace keypoint;
using namespace utility;
using namespace KMSpace;
using namespace Reg;
using namespace Eigen;

typedef pcl::PointXYZ Point_T; 


//By Yue Pan et al. @WHU
//Transform source cloud into target cloud's reference system

int main(int argc, char** argv)
{	
	google::InitGoogleLogging("Mylog");
	google::SetLogDestination(google::GLOG_INFO, "./logfile/MyLogInfo");
	LOG(INFO) << "Launch the program!";
	
	if (argc != 5) {
       printf("Input error, should be: binary  traget_cloud_filename  source_cloud_filename  config_filename  estiamted_overlapping_rate\n");
       return -1;
    }

    string filenameT = argv[1];
	string filenameS = argv[2];
	string paralistfile = argv[3];
	double estimated_IoU = atof(argv[4]);

	cout << "!---------------------------GH-ICP------------------------------!" << endl;
    
	//timing
	clock_t t1, t2, t3, t4, t5, t6, t7, t8, t9, ts1, ts2, ts3, ts4;

	/*----------------------- 1. Data input-------------------------*/
    
	DataIo<Point_T> io;
	pcl::PointCloud<Point_T>::Ptr pointCloudT(new pcl::PointCloud<Point_T>()), pointCloudS(new pcl::PointCloud<Point_T>());
	
	t1 = clock();
	io.readCloudFile(filenameT,pointCloudT);
	io.readCloudFile(filenameS,pointCloudS);

	cout << "Data loaded" << endl;
	cout << "Raw point number: [ S:  " << pointCloudS->size() << "  , T:  " << pointCloudT->size()<<" ]"<<endl;
	io.readParalist(paralistfile);
	io.displayparameter();
	t2 = clock();
	//cout << "Time for importing data: " << (t2 - t1)*1.0 / 1000 << " s" << endl
	//	 << "-----------------------------------------------------------------------------" << endl;
	

	/*------------------ 2. Voxelization filtering------------------------*/

	float resolution = io.paralist.downsample_resolution;//control the subsampling scale (unit:m) ;//0.04f for TLS//0.00025f for bunny
	VoxelFilter<pcl::PointXYZI> filter(resolution);
	pcXYZIPtr filterPointCloudS(new pcXYZI()), filterPointCloudT(new pcXYZI());
	filterPointCloudS = filter.filter(pointCloudS);
	filterPointCloudT = filter.filter(pointCloudT);
	cout << "Down-sampled point cloud number: [ S:  " << filterPointCloudS->size()<<" , T:  "<<filterPointCloudT->size()<<" ]"<<endl;
	t3 =clock();
	//cout << "Time for filtering: " << float(t3 - t2)/CLOCKS_PER_SEC  << " s" << endl
	//	 << "-----------------------------------------------------------------------------" << endl;
	
	//------------------Directly use ICP--------------------------------------
	/*cout << "Directly use ICP to register" << endl;
	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp0;
	icp0.setInputTarget(filterPointCloudT);
	icp0.setInputSource(filterPointCloudS);
	pcl::PointCloud<pcl::PointXYZI> DICP;
	ofstream ofs;
	ofs.open("ICP_RMSE.txt");
	if (ofs.is_open())
	{
	ofs << "iteration No.\t" << "RMSE" << endl;
	for (int i = 0; i < 50; i++)
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
	io.display(filterPointCloudT, DICP.makeShared());
	io.writePcdFile("DirectICP.pcd", DICP.makeShared());
	cout << "-----------------------------------------------------------------------------" << endl;*/
	//------------------Directly use ICP--------------------------------------

	//-------------Directly use FPFH-SAC or FPFH-SAC+ICP---------------------------------
	/*FPFHfeature fpfh0;
	fpfhFeaturePtr source_fpfh = fpfh0.compute_fpfh_feature(filterPointCloudS);
	fpfhFeaturePtr target_fpfh = fpfh0.compute_fpfh_feature(filterPointCloudT);
	//fpfh.displayhistogram(source_fpfh, 1);
	pcXYZIPtr DFPFHSAC(new pcXYZI());
	DFPFHSAC = fpfh0.fpfhalign(filterPointCloudS, filterPointCloudT, source_fpfh, target_fpfh);
	fpfh0.displaycorrespondence(filterPointCloudS, filterPointCloudT, DFPFHSAC, source_fpfh, target_fpfh);
	io.writePcdFile("DirectFPFHSAC.pcd", DFPFHSAC);
	io.display(filterPointCloudT, DFPFHSAC);
	*/
	//-------------Directly use FPFH-SAC or FPFH-SAC+ICP---------------------------------
	

	/*----------------------- 3. Keypoint detection--------------------------*/
	keypointOption kpOption;
	kpOption.minPtNum = io.paralist.keypoint_min_num;
	kpOption.radiusFeatureCalculation = io.paralist.feature_r;
	kpOption.radiusNonMax = io.paralist.keypoint_non_max;
	kpOption.ratioMax = io.paralist.keypoint_max_ratio;
	//kpOption.radiusNonMax = pow(filter.V_boundingbox/io.paralist.num_point_bb,1.0/3); 
	//cout << "BoundingBox: "<<filter.V_boundingbox<<" m  , Nonmax: " << kpOption.radiusNonMax << " m"<< endl;
	CkeypointDetection kpd(kpOption);
	
	pcl::PointIndicesPtr keyPointIndicesS, keyPointIndicesT;
	kpd.keypointDetectionBasedOnCurvature(filterPointCloudS, keyPointIndicesS);
	//kpd.outputKeypoints("keyPointIndicesS.txt", keyPointIndicesS, filterPointCloudS);
	cout << keyPointIndicesS->indices.size() << " Key points detected for Source Point Cloud " << endl;	

	kpd.keypointDetectionBasedOnCurvature(filterPointCloudT, keyPointIndicesT);
	//kpd.outputKeypoints("keyPointIndicesT.txt", keyPointIndicesT, filterPointCloudT);
	cout << keyPointIndicesT->indices.size() << " Key points detected for Target Point Cloud " << endl;
	
	Eigen::MatrixX3d kpSXYZ,kpTXYZ;  
	kpd.savecoordinates(filterPointCloudS, filterPointCloudT, keyPointIndicesS, keyPointIndicesT, kpSXYZ, kpTXYZ);
	
	t4 = clock();
	//cout << "Time for keypoint extraction: " << float(t4 - t3) / CLOCKS_PER_SEC << " s" << endl
	//	 << "-----------------------------------------------------------------------------" << endl;

	/*---------------------- 4.(1) Feature calculation BSC--------------------------*/
	
	StereoBinaryFeatureExtractor bsc(io.paralist.feature_r, 7);    //Create BSC object.
    doubleVectorSBF bscS, bscT;
	if (io.paralist.feature == 1){
		bsc.extractBinaryFeatures(filterPointCloudS, keyPointIndicesS, bscS);
		cout << "Source Cloud BSC extracted" << endl;
		bsc.extractBinaryFeatures(filterPointCloudT, keyPointIndicesT, bscT);
		cout << "Target Cloud BSC extracted" << endl;

		t5 = clock();
		//cout << "Time for BSC feature calculation: " << float(t5 - t4) / CLOCKS_PER_SEC << " s" << endl
		//		<< "-----------------------------------------------------------------------------" << endl;
	}

	/*---------------------- 4.(2) Feature calculation FPFH--------------------------*/
	FPFHfeature fpfh;
	fpfhFeaturePtr source_fpfh(new fpfhFeature), target_fpfh(new fpfhFeature);
	fpfh.radius = io.paralist.feature_r;
	if (io.paralist.feature == 2){
		fpfhFeaturePtr source_fpfh_all = fpfh.compute_fpfh_feature(filterPointCloudS);
		cout << "Source Cloud FPFH extracted" << endl;
		fpfhFeaturePtr target_fpfh_all = fpfh.compute_fpfh_feature(filterPointCloudT);
		cout << "Target Cloud FPFH extracted" << endl;
		fpfh.keyfpfh(source_fpfh_all, target_fpfh_all, keyPointIndicesS, keyPointIndicesT, source_fpfh, target_fpfh);
		//cout << "Keypoints' FPFH extracted" << endl;
		t5 = clock();
		//cout << "Time for FPFH feature calculation: " << float(t5 - t4) / CLOCKS_PER_SEC << " s" << endl
		// 	<< "-----------------------------------------------------------------------------" << endl;
	}

	/*----------------------- 5. Registration-------------------------*/
	Keypoints Kp;
	Kp.kps_num = keyPointIndicesS->indices.size();
	Kp.kpt_num = keyPointIndicesT->indices.size();
	Kp.kpSXYZ = kpSXYZ;
	Kp.kpTXYZ = kpTXYZ;

	Energyfunction Ef;
	Ef.ED.resize(Kp.kps_num, vector<double>(Kp.kpt_num));
	Ef.FD.resize(Kp.kps_num, vector<double>(Kp.kpt_num));
	Ef.CD.resize(Kp.kps_num, vector<double>(Kp.kpt_num));
	
	//Define penalty parameters
	Ef.penalty_initial = io.paralist.p_pre;    // initial penalty for FD (mean-k*std)
	Ef.para1_penalty   = io.paralist.p_ED;     // for ED penalty estimation
	Ef.para2_penalty   = io.paralist.p_FD;     // for FD penalty estimation
	
	//other parameters
	Ef.min_cor    = 10;                        // min keypoint pairs for registration
	Ef.m          = io.paralist.iter_speed;    // ED,FD weight changing parameter 
	Ef.KM_eps     = io.paralist.kmeps;         // KM's parameter (Smaller eps, longer consuming time) 
	Ef.usefeature = io.paralist.feature;
	Ef.k          = io.paralist.scale;

	Registration Reg(Kp, Ef, 0, 0);
	// FPFH_SAC
	//Reg.Reg_FPFHSAC(filterPointCloudS, filterPointCloudT);
	// 3D NDT
	//Reg.Reg_3DNDT(filterPointCloudS, filterPointCloudT);

	//Initialization
	Reg.Rt_tillnow = Eigen::Matrix4d::Identity();
    Reg.converge = 0;
	Reg.nonmax = kpOption.radiusNonMax;
	Reg.index_output = 0;
	Reg.RMS = 1000.0;
	Reg.iteration_number = 0; 
    Reg.matchlist.resize(Kp.kps_num, vector<int>(200));
	Reg.gt_maxdis = kpOption.radiusNonMax/3;
	
	//Weight adjustment and convergence condition
	Reg.adjustweight_ratio = io.paralist.weight_adjustment_ratio;
	Reg.adjustweight_step = io.paralist.weight_adjustment_step;
	Reg.converge_t = io.paralist.converge_t;
	Reg.converge_r = io.paralist.converge_r;

	//save SP ,SFP ,SKP and TKP
	Reg.save(filterPointCloudS, pointCloudS, kpSXYZ, kpTXYZ);
	
	//Display Original Point Cloud
	ts1 = clock();
	Reg.displayPCrb(filterPointCloudT, Reg.sfP, Reg.tkP, Reg.skP);
	ts2 = clock();

	//Read ground truth RT
	Reg.readGTRT();

	//Calculate FD
	if (Ef.usefeature == 1) Reg.calFD_BSC (bscS, bscT);
	if (Ef.usefeature == 2) Reg.calFD_FPFH(source_fpfh, target_fpfh);
	
	//t6 = clock();
	//cout << "Time for registration preparation: " << float(t6 - t5 - ts2 + ts1) / CLOCKS_PER_SEC << " s" << endl;
	
	//Output parameters
	/*cout << "parameter list:" << endl << "ED,FD weight parameter m = " << Ef.m << "  minimum correspondence pairs = " << Ef.min_cor <<endl
		<< "original penalty for FD = " << Ef.penalty<<"   ED penalty estimation parameter = "<< Ef.para1_penalty
		<< "  FD penalty estimation parameter = " << Ef.para2_penalty<<endl
		<< "KM's parameter eps = " << Ef.KM_eps <<"   ED scalar weight = "<<Ef.k<< endl
		<< "-----------------------------------------------------------------------------" << endl;*/

	//Iterative process

	while (!Reg.converge)    
	{
		Eigen::Matrix4d Rt;
		cout << Reg.iteration_number << " : " << endl;
		clock_t ts, te1, te2, te3;
		ts = clock();
		//Calculate ED
		Reg.calED();
		//Calculate CD
		if (Ef.usefeature == 0) Reg.calCD_NF();
		else if (Ef.usefeature == 1) Reg.calCD_BSC();
		else if (Ef.usefeature == 2) Reg.calCD_FPFH();
		
		//if (Reg.RMS > 10000)
		//{
			//Reg.findcorrespondenceKM();
		//}//Use Kuhn�CMunkres algorithm(KM) or MCMF
		//else Reg.findcorrespondenceNN();
		
		if (io.paralist.correspondence_type == 0) Reg.findcorrespondenceNN();
		else if (io.paralist.correspondence_type == 1) Reg.findcorrespondenceKM();

		te1 = clock();
		//Reg.displayCorrespondence(filterPointCloudS, Reg.tfP);  //Display matching result
		Reg.transformestimation(Rt);//using  SVD
		Reg.adjustweight(estimated_IoU);
		Reg.Rt_tillnow = Rt*Reg.Rt_tillnow;
		cout <<"RT till now"<<endl<< Reg.Rt_tillnow << endl;
		te2 = clock();
		if (io.paralist.output==1) Reg.output(Reg.sfP);  //Output result per iteration
		te3 = clock();
		Reg.iteration_number++;
		
		/*cout << "Time for finding correspondence : " << float(te1 - ts) / CLOCKS_PER_SEC << " s" << endl
			 << "Time for transform estimation and updating : " << float(te2 - te1) / CLOCKS_PER_SEC << " s" << endl
			 << "Time for output temporal result : " << float(te3 - te2) / CLOCKS_PER_SEC << " s" << endl
			 << "Time for one iteration : " << float(te3 - ts) / CLOCKS_PER_SEC << " s" << endl
			 << "-----------------------------------------------------------------------------" << endl;*/
	}

	//Reg.energyRMSoutput();
	t7 = clock();
	
	//cout << "converge_break" << endl;
	cout << "final Rt without classic ICP" << endl<< Reg.Rt_tillnow << endl;
	cout << "IGSP Time: " << float(t7 - t1 - ts2 + ts1) / CLOCKS_PER_SEC << " s" << endl
	<< "-----------------------------------------------------------------------------" << endl;
	
	//display and output IGSP Registered Point Cloud
	ts3 = clock();
	Reg.output(Reg.sfP);
	Reg.displayPCvb(filterPointCloudT, Reg.sfP, Reg.tkP, Reg.skP);
	ts4 = clock();
	

	//---------------------------classic ICP refinement--------------------------------//
	/*cout << "ICP Refinement." << endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	
	//io.readPcdFileXYZ(filenameS, cloud_in);
	pcl::copyPointCloud(*pointCloudT, *cloud_in); //Copy pointcloud (format transfer) input: XYZI output:XYZ
	cout << "source ok" << endl;
	Eigen::Matrix<double, 4, Dynamic> TransPC;
	TransPC.resize(4, Reg.sP.cols());
	TransPC = Reg.Rt_tillnow*Reg.sP;
	cloud_out->points.resize(TransPC.cols());
	
	for (size_t i = 0; i < TransPC.cols(); ++i)
	{
		cloud_out->points[i].x = TransPC(0, i);
		cloud_out->points[i].y = TransPC(1, i);
		cloud_out->points[i].z = TransPC(2, i);
	}
	cout << "target ok" << endl;
	t8 = clock();

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_out);
	icp.setInputTarget(cloud_in);
	icp.setMaxCorrespondenceDistance(4*kpOption.radiusNonMax); 
	icp.setTransformationEpsilon(1e-6);
	icp.setEuclideanFitnessEpsilon(0.01);
	icp.setMaximumIterations(15);
	icp.setUseReciprocalCorrespondences(true);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	cout << "has converged:" << icp.hasConverged() << " score: " <<
		icp.getFitnessScore() << endl;
	cout << "Classic ICP's Transfrom"<<endl<< icp.getFinalTransformation() << endl;
	t9 = clock();
	
	Reg.Rt_tillnow = icp.getFinalTransformation().cast<double>() *Reg.Rt_tillnow;  //use cast<type>() tp do the type transfer for eigen
	cout << "final Rt " << endl << Reg.Rt_tillnow << endl;
	io.writePcdFileXYZ("FinalRegistered.pcd", Final.makeShared());*/
	
	//---------------------------classic ICP refinement--------------------------------//
	
    //io.displaymulti(pointCloudS, DICP.makeShared(),Final.makeShared());   //-------------change here-----------//
	//cout << "Classic ICP Time: " << (t9 - t8- ts4 + ts3)*1.0 / 1000 << " s" << endl;
	//cout << "Total Time: " << (t9 - t8 + t7 - t1- ts2 + ts1 -ts4 +ts3)*1.0 / 1000 << " s" << endl
		//<< "-----------------------------------------------------------------------------" << endl;
	//----------------------------------------------------------------------------------------//
	//display final pc
	//Reg.displayPC(filterPointCloudS, Reg.tfP, Reg.skP, Reg.tkP);
	
	//output 
	//Reg.output(Reg.sP);
	//Reg.index_output = 1;
	//Reg.output(Reg.tkP);
	//
	//Reg.calGTmatch(Reg.skP, Reg.tkP);
	//Reg.cal_recall_precision();

	cout << "Output for experiment completed" << endl 
		 << "----------------------------------------------------------------------------------" << endl
		 << "Thanks for using this software." << endl << "By Yue Pan et al. @ Wuhan University" << endl;
	//end
	
	int endok;
	cin >> endok;
	return 1;

	//for bunny,please lay the target cloud (xyz) horizontally because BSC is a 4DOF feature
	//You can try to use the 6 DOF feature like FPFH or SHOT
}

