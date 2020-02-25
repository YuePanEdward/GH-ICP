#include <iostream>
#include <fstream>
#include "dataio.hpp"
#include "filter.hpp"
#include "keypointdetect.hpp"
#include "BinaryFeatureExtraction.hpp"

using namespace ghicp;

typedef pcl::PointXYZ Point_T;

int main(int argc, char **argv)
{
    std::string filenameT = argv[1]; //Target pointcloud file path
    std::string filenameS = argv[2]; //Source pointcloud file path
    std::string filenameR = argv[3]; //Registered source pointcloud file path

    std::string use_feature = argv[4]; //BSC(B), FPFH(F)
    char using_feature = use_feature.c_str()[0];

    float resolution = atof(argv[5]); //control the subsampling scale (unit:m) ; //0.04f for TLS //0.00025f for bunny
    float neighborhood_radius = atof(argv[6]);
    float curvature_non_max_radius = atof(argv[7]);

    float estimated_IoU = atof(argv[8]);

    DataIo<Point_T> dataio;
    pcl::PointCloud<Point_T>::Ptr pointCloudT(new pcl::PointCloud<Point_T>()), pointCloudS(new pcl::PointCloud<Point_T>());

    dataio.readCloudFile(filenameT, pointCloudT);
    dataio.readCloudFile(filenameS, pointCloudS);

    CFilter<Point_T> cfilter;
    pcl::PointCloud<Point_T>::Ptr pointCloudT_down(new pcl::PointCloud<Point_T>()), pointCloudS_down(new pcl::PointCloud<Point_T>());
    cfilter.voxelfilter(pointCloudT, pointCloudT_down, resolution);
    cfilter.voxelfilter(pointCloudS, pointCloudS_down, resolution);

    CKeypointDetect<Point_T> ckpd(neighborhood_radius, 0.8, 20, curvature_non_max_radius);

    pcl::PointIndicesPtr keyPointIndicesT, keyPointIndicesS;
    ckpd.keypointDetectionBasedOnCurvature(pointCloudT_down, keyPointIndicesT);
    ckpd.keypointDetectionBasedOnCurvature(pointCloudS_down, keyPointIndicesS);

    switch (using_feature)
    {
    case 'B':
    {                                                         //Use BSC feature
        BSCEncoder<Point_T> bsc(curvature_non_max_radius, 7); //Create BSC object.
        doubleVectorSBF bscT, bscS;
        bsc.extractBinaryFeatures(pointCloudT_down, keyPointIndicesT, bscT);
        bsc.extractBinaryFeatures(pointCloudS_down, keyPointIndicesS, bscS);

        break;
    }
    case 'F': //Use FPFH feature
    {         //fm.detectFeaturesORB(frames[i], feature_extract_parameter);
        std::cout << "Not ready yet" << std::endl;
        break;
    }
    default:
    {
        std::cout << "Wrong feature input." << std::endl;
        return 0;
    }
    }

#if 0
    if (io.paralist.feature == 1)
    {

        //t5 = clock();
        //cout << "Time for BSC feature calculation: " << float(t5 - t4) / CLOCKS_PER_SEC << " s" << endl
        //		<< "-----------------------------------------------------------------------------" << endl;
    }

    /*---------------------- 4.(2) Feature calculation FPFH--------------------------*/
    FPFHfeature fpfh;
    fpfhFeaturePtr source_fpfh(new fpfhFeature), target_fpfh(new fpfhFeature);
    fpfh.radius = io.paralist.feature_r;
    if (io.paralist.feature == 2)
    {
        fpfhFeaturePtr source_fpfh_all = fpfh.compute_fpfh_feature(filterPointCloudS);
        cout << "Source Cloud FPFH extracted" << endl;
        fpfhFeaturePtr target_fpfh_all = fpfh.compute_fpfh_feature(filterPointCloudT);
        cout << "Target Cloud FPFH extracted" << endl;
        fpfh.keyfpfh(source_fpfh_all, target_fpfh_all, keyPointIndicesS, keyPointIndicesT, source_fpfh, target_fpfh);
        //cout << "Keypoints' FPFH extracted" << endl;
        //t5 = clock();
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
	//ts1 = clock();
	Reg.displayPCrb(filterPointCloudT, Reg.sfP, Reg.tkP, Reg.skP);
	//ts2 = clock();

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
		//clock_t ts, te1, te2, te3;
		//ts = clock();
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

		//te1 = clock();
		//Reg.displayCorrespondence(filterPointCloudS, Reg.tfP);  //Display matching result
		Reg.transformestimation(Rt);//using  SVD
		Reg.adjustweight(estimated_IoU);
		Reg.Rt_tillnow = Rt*Reg.Rt_tillnow;
		cout <<"RT till now"<<endl<< Reg.Rt_tillnow << endl;
		//te2 = clock();
		if (io.paralist.output==1) Reg.output(Reg.sfP);  //Output result per iteration
		//te3 = clock();
		Reg.iteration_number++;
		
		/*cout << "Time for finding correspondence : " << float(te1 - ts) / CLOCKS_PER_SEC << " s" << endl
			 << "Time for transform estimation and updating : " << float(te2 - te1) / CLOCKS_PER_SEC << " s" << endl
			 << "Time for output temporal result : " << float(te3 - te2) / CLOCKS_PER_SEC << " s" << endl
			 << "Time for one iteration : " << float(te3 - ts) / CLOCKS_PER_SEC << " s" << endl
			 << "-----------------------------------------------------------------------------" << endl;*/
	}
#endif

    return 0;
}