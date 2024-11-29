#define _USE_MATH_DEFINES

#include <pcl/registration/icp.h> 
#include <pcl/registration/gicp.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/features/from_meshes.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h> 
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> 
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/ia_ransac.h>


#include <glog/logging.h>

#include <cmath>

#include "common_reg.h"
#include "pca.h"

namespace ghicp
{

// This program accomplish the fine registration between ALS block and TLS point clouds. [Use TLS as the control cloud (alignment target)]
// It also provides various methods to achieve the same goal. You can do comprehensive experiment based on this program.
// It is part of the ALS Refinement Project for Highway Expansion and Reconstruction Engineering

/**
* \brief Point-to-Point metric ICP
* \param[in]  SourceCloud : A pointer of the Source Point Cloud (Each point of it is used to find the nearest neighbor as correspondence)
* \param[in]  TargetCloud : A pointer of the Target Point Cloud
* \param[out] TransformedSource : A pointer of the Source Point Cloud after registration [ Transformed ]
* \param[out] transformationS2T : The transformation matrix (4*4) of Source Point Cloud for this registration
* \param[in]  max_iter : A parameter controls the max iteration number for the registration
* \param[in]  use_reciprocal_correspondence : A parameter controls whether use reciprocal correspondence or not (bool)
* \param[in]  use_trimmed_rejector : A parameter controls whether use trimmed correspondence rejector or not (bool)
* \param[in]  thre_dis : A parameter used to estimate the approximate overlap ratio of Source Point Cloud. It acts as the search radius of overlapping estimation.
*/
template<typename PointT>
bool CRegistration<PointT>::icp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	Eigen::Matrix4f & transformationS2T,
	int max_iter,
	bool use_reciprocal_correspondence,
	bool use_trimmed_rejector,
	float thre_dis,
	float min_overlap_for_reg)
{
	clock_t t0, t1;
	t0=clock();
	
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	
	// Use Reciprocal Correspondences or not? [a -> b && b -> a]
	icp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);
	
	// Trimmed or not? [ Use a predefined overlap ratio to trim part of the correspondence with bigger distance ]
	if (use_trimmed_rejector){
		pcl::registration::CorrespondenceRejectorTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorTrimmed);
		float overlap_ratio = calOverlap(SourceCloud, TargetCloud, thre_dis);
		if (overlap_ratio < min_overlap_for_reg) {
			LOG(WARNING) << "The overlap ratio is too small. This registration would not be done.";
			return false;
		}
		else{
			trimmed_cr->setOverlapRatio(overlap_ratio);
			icp.addCorrespondenceRejector(trimmed_cr);
		}
	}

	icp.setInputSource(SourceCloud);
	icp.setInputTarget(TargetCloud);

	//icp.setMaxCorrespondenceDistance(thre_dis);

	// Converge criterion ( 'Or' Relation ) 
	// Set the maximum number of iterations [ n>x ] (criterion 1)
	icp.setMaximumIterations(max_iter);     //Most likely to happen
	// Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	icp.setTransformationEpsilon(1e-8);     //Quite hard to happen
	// Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	icp.setEuclideanFitnessEpsilon(1e-5);   //Quite hard to happen
	
	icp.align(*TransformedSource);  //Use closed-form SVD to estimate transformation for each iteration [You can switch to L-M Optimization]
	transformationS2T = icp.getFinalTransformation().template cast<float>();
	
	t1=clock();

	// Commented these out if you don't want to output the registration log
	LOG(INFO) << "Point-to-Point ICP done in  " << float(t1 - t0) / CLOCKS_PER_SEC << "s";
	LOG(INFO) << transformationS2T;
	LOG(INFO) << "The fitness score of this registration is " << icp.getFitnessScore();
	if (icp.getFitnessScore() > 5000) LOG(WARNING) << "The fitness score of this registration is a bit too large";
	LOG(INFO) << "-----------------------------------------------------------------------------";

	//
	cout << "Point-to-Point ICP done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl << transformationS2T << endl;
	cout << "The fitness score of this registration is " <<icp.getFitnessScore()<<endl;
	cout << "-----------------------------------------------------------------------------" << endl;
	return true;
}

// Key
/**
* \brief Point-to-Plane metric ICP
* \param[in]  SourceCloud : A pointer of the Source Point Cloud (Each point of it is used to find the nearest neighbor as correspondence)
* \param[in]  TargetCloud : A pointer of the Target Point Cloud
* \param[out] TransformedSource : A pointer of the Source Point Cloud after registration [ Transformed ]
* \param[out] transformationS2T : The transformation matrix (4*4) of Source Point Cloud for this registration
* \param[in]  max_iter : A parameter controls the max iteration number for the registration
* \param[in]  use_reciprocal_correspondence : A parameter controls whether use reciprocal correspondence or not (bool)
* \param[in]  use_trimmed_rejector : A parameter controls whether use trimmed correspondence rejector or not (bool)
* \param[in]  thre_dis : A parameter used to estimate the approximate overlap ratio of Source Point Cloud. It acts as the search radius of overlapping estimation.
*/
// Tips: The Source and Target Point Cloud must have normal (You need to calculated it outside this method). In this case, PointT should be something like PointXYZ**Normal
template<typename PointT>
bool CRegistration<PointT>::ptplicp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	Eigen::Matrix4f & transformationS2T,
	int max_iter,
	bool use_reciprocal_correspondence,
	bool use_trimmed_rejector,
	float thre_dis,
	int covariance_K,  // You can switch to radius search 
	float min_overlap_for_reg)
{
	clock_t t0, t1;
	t0=clock();
	
	// In this case, The Cloud's Normal hasn't been calculated yet. 
	
	pcl::PointCloud<pcl::PointNormal>::Ptr SourceNormal(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr TargetNormal(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::PointNormal>::Ptr TransformedSourceN(new pcl::PointCloud<pcl::PointNormal>());

	PrincipleComponentAnalysis<PointT> pca_estimator;
	//Radius search
	//pca_estimator.CalculatePointCloudWithNormal_Radius(SourceCloud, radius, SourceNormal); //To correct the potential bug: PointT should be PointXYZ or the PointNormal(XYZNormal) would go wrong
	//pca_estimator.CalculatePointCloudWithNormal_Radius(TargetCloud, radius, TargetNormal);
	
	//KNN search
	pca_estimator.CalculatePointCloudWithNormal_KNN(SourceCloud, covariance_K, SourceNormal);
	pca_estimator.CalculatePointCloudWithNormal_KNN(TargetCloud, covariance_K, TargetNormal);

	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> ptplicp;
	
	ptplicp.setInputSource(SourceNormal);
	ptplicp.setInputTarget(TargetNormal);
	
	// Use Reciprocal Correspondences or not? [a -> b && b -> a]
	ptplicp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

	// Trimmed or not? [ Use a predefined overlap ratio to trim part of the correspondence with bigger distance ]
	if (use_trimmed_rejector){
		pcl::registration::CorrespondenceRejectorTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorTrimmed);
		float overlap_ratio = calOverlap(SourceCloud, TargetCloud, thre_dis);
		if (overlap_ratio < min_overlap_for_reg) {
			LOG(WARNING) << "The overlap ratio is too small. This registration would not be done.";
			return false;
		}
		else{
			trimmed_cr->setOverlapRatio(overlap_ratio);
			ptplicp.addCorrespondenceRejector(trimmed_cr);
		}
	}
	
	// Converge criterion ( 'Or' Relation ) 
	// Set the maximum number of iterations [ n>x ] (criterion 1)
	ptplicp.setMaximumIterations(max_iter);     //Most likely to happen
	// Set the transformation difference threshold [delta_t<sqrt(x) or delta_ang<arccos(1-x)] (criterion 2)
	ptplicp.setTransformationEpsilon(1e-8);     //Quite hard to happen
	// Set the relative RMS difference between two consecutive iterations [ RMS(n)-RMS(n+1)<x*RMS(n) ] (criterion 3)
	ptplicp.setEuclideanFitnessEpsilon(1e-5);   //Quite hard to happen
	
	ptplicp.align(*TransformedSourceN);  //Use Linear Least Square Method to estimate transformation for each iteration

	transformationS2T = ptplicp.getFinalTransformation();

	t1=clock();

	// Commented these out if you don't want to output the registration log
	LOG(INFO) << "Point-to-Plane ICP done in  " << float(t1 - t0) / CLOCKS_PER_SEC << "s";
	// LOG(INFO) << "Transform Matrix" << endl << transformationS2T;
	LOG(INFO) << "The fitness score of this registration is " << ptplicp.getFitnessScore();
	if (ptplicp.getFitnessScore() > 5000) LOG(WARNING) << "The fitness score of this registration is a bit too large";
	// LOG(INFO) << "-----------------------------------------------------------------------------";

	// Commented these out if you don't want to output the registration log
	cout << "Point-to-Plane ICP done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl << transformationS2T << endl; //This one is with perturbation;
	cout << "The fitness score of this registration is " << ptplicp.getFitnessScore() << endl;
	//cout << "-----------------------------------------------------------------------------" << endl;
}

/**
* \brief Generalized (Plane-to-Plane) metric ICP
* \param[in]  SourceCloud : A pointer of the Source Point Cloud (Each point of it is used to find the nearest neighbor as correspondence)
* \param[in]  TargetCloud : A pointer of the Target Point Cloud
* \param[out] TransformedSource : A pointer of the Source Point Cloud after registration [ Transformed ]
* \param[out] transformationS2T : The transformation matrix (4*4) of Source Point Cloud for this registration
* \param[in]  k_neighbor_covariance : A parameter controls the number of points used to calculate the approximate covariance of points, which is used to accomplish the General ICP
* \param[in]  max_iter : A parameter controls the max iteration number for the registration
* \param[in]  use_reciprocal_correspondence : A parameter controls whether use reciprocal correspondence or not (bool)
* \param[in]  use_trimmed_rejector : A parameter controls whether use trimmed correspondence rejector or not (bool)
* \param[in]  thre_dis : A parameter used to estimate the approximate overlap ratio of Source Point Cloud. It acts as the search radius of overlapping estimation.
*/
// Attention please.
// Problem : It is found that the G-ICP codes in pcl does not support the correspondence rejector because its computeTransformation method is completely different from classic icp's though gicp is inherited from icp.
// In my opinion, this is the main reason for G-ICP's ill behavior. I am working on the G-ICP with trimmed property now.
template<typename PointT>
bool CRegistration<PointT>::gicp_reg(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	Eigen::Matrix4f & transformationS2T,
	int max_iter,
	bool use_reciprocal_correspondence,
	bool use_trimmed_rejector,
	float thre_dis,
	int covariance_K,
	float min_overlap_for_reg)
{	
	clock_t t0, t1;
	t0=clock();
	
	pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
	
	// Set the number of points used to calculated the covariance of a point
	gicp.setCorrespondenceRandomness(covariance_K);
	
	// Use Reciprocal Correspondences or not? [a -> b && b -> a]
	gicp.setUseReciprocalCorrespondences(use_reciprocal_correspondence);

	// Trimmed or not? [ Use a predefined overlap ratio to trim part of the correspondence with bigger distance ]
	if (use_trimmed_rejector){
		pcl::registration::CorrespondenceRejectorTrimmed::Ptr trimmed_cr(new pcl::registration::CorrespondenceRejectorTrimmed);
		float overlap_ratio = calOverlap(SourceCloud, TargetCloud, thre_dis);
		if (overlap_ratio < min_overlap_for_reg) {
			LOG(WARNING) << "The overlap ratio is too small. This registration would not be done.";
			return false;
		}
		else{
			trimmed_cr->setOverlapRatio(overlap_ratio);
			gicp.addCorrespondenceRejector(trimmed_cr);
		}
	}
	
	gicp.setMaxCorrespondenceDistance(1e6); //A large value

	gicp.setInputSource(SourceCloud);
	gicp.setInputTarget(TargetCloud);

	// Converge criterion ( 'Or' Relation ) 
	// According to gicp.hpp, the iteration terminates when one of the three happens
	// Set the maximum number of iterations  (criterion 1)
	gicp.setMaximumIterations(max_iter);     //Most likely to happen
	// Set the transformation difference threshold (criterion 2)
	gicp.setTransformationEpsilon(1e-8);     //Hard to happen
	// Set the rotation difference threshold (criterion 3) 
	gicp.setRotationEpsilon(1e-6);           //Hard to happen

	gicp.align(*TransformedSource);

	transformationS2T = gicp.getFinalTransformation();

	t1=clock();

	// Commented these out if you don't want to output the registration log
	LOG(INFO) << "GICP done in  " << float(t1 - t0) / CLOCKS_PER_SEC << "s";
	LOG(INFO) << transformationS2T;
	LOG(INFO) << "The fitness score of this registration is " << gicp.getFitnessScore();
	if (gicp.getFitnessScore() > 5000) LOG(WARNING) << "The fitness score of this registration is a bit too large";
	LOG(INFO) << "-----------------------------------------------------------------------------";

	// Commented these out if you don't want to output the registration log
	cout << "GICP done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl << transformationS2T << endl;
	cout << "The fitness score of this registration is " << gicp.getFitnessScore() << endl;
	cout << "-----------------------------------------------------------------------------" << endl;
}


/**
* \brief Estimated the approximate overlapping ratio for Cloud1 considering Cloud2
* \param[in]  Cloud1 : A pointer of the Point Cloud used for overlap ratio calculation
* \param[in]  Cloud2 : A pointer of the Point Cloud overlapped with Cloud1
* \param[out] thre_dis : It acts as the search radius of overlapping estimation
* \return : The estimated overlap ratio [from 0 to 1]
*/
template<typename PointT>
float CRegistration<PointT>::calOverlap(const typename pcl::PointCloud<PointT>::Ptr & Cloud1,
	const typename pcl::PointCloud<PointT>::Ptr & Cloud2,
	float thre_dis)
{
	int overlap_point_num=0;
	float overlap_ratio;

	pcl::KdTreeFLANN<PointT> kdtree;
	
	if(!Cloud2->empty())
		kdtree.setInputCloud(Cloud2);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	
	for (int i = 0; i < Cloud1->size(); i++){
		if (kdtree.radiusSearch(Cloud1->points[i], thre_dis, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) overlap_point_num++;
	}
	
	overlap_ratio = (0.01 + overlap_point_num) / Cloud1->size();
	//cout << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio << endl;
	LOG(INFO) << "The estimated approximate overlap ratio of Cloud 1 is " << overlap_ratio;

	return overlap_ratio;
}

/**
* \brief Transform a Point Cloud using a given transformation matrix
* \param[in]  Cloud : A pointer of the Point Cloud before transformation
* \param[out] TransformedCloud : A pointer of the Point Cloud after transformation
* \param[in]  transformation : A 4*4 transformation matrix
*/
template<typename PointT>
void CRegistration<PointT>::transformcloud(typename pcl::PointCloud<PointT>::Ptr & Cloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedCloud,
	Eigen::Matrix4f & transformation)
{
	Eigen::Matrix4Xf PC;
	Eigen::Matrix4Xf TPC; 
	PC.resize(4, Cloud->size());
	TPC.resize(4, Cloud->size());
	for (int i = 0; i < Cloud->size(); i++)
	{
		PC(0,i)= Cloud->points[i].x;
		PC(1,i)= Cloud->points[i].y;
		PC(2,i)= Cloud->points[i].z;
		PC(3,i)= 1;
	}
	TPC = transformation * PC;
	for (int i = 0; i < Cloud->size(); i++)
	{
		PointT pt;
		pt.x = TPC(0, i);
		pt.y = TPC(1, i);
		pt.z = TPC(2, i);
		TransformedCloud->points.push_back(pt);
	}
	cout << "Transform done ..." << endl;
}

/**
* \brief Get the Inverse (Not Mathimatically) of a giving 4*4 Transformation Matrix
* \param[in]  transformation : A 4*4 transformation matrix ( from Cloud A to Cloud A' )
* \param[out] invtransformation : Inversed 4*4 transformation matrix ( from Cloud A' to Cloud A )
*/
template<typename PointT>
void CRegistration<PointT>::invTransform(const Eigen::Matrix4f & transformation,
	Eigen::Matrix4f & invtransformation)
{
	invtransformation.block<3, 3>(0, 0) = (transformation.block<3, 3>(0, 0)).transpose();
	invtransformation(0, 3) = -transformation(0, 3);
	invtransformation(1, 3) = -transformation(1, 3);
	invtransformation(2, 3) = -transformation(2, 3);
	invtransformation(3, 0) = 0;
	invtransformation(3, 1) = 0;
	invtransformation(3, 2) = 0;
	invtransformation(3, 3) = 1;
}

template<typename PointT>
void CRegistration<PointT>::compute_fpfh_feature(const typename pcl::PointCloud<PointT>::Ptr &input_cloud, fpfhFeaturePtr &cloud_fpfh, float search_radius){
	
	// Calculate the Point Normal
	NormalsPtr cloud_normal(new Normals);
	calNormal(input_cloud, cloud_normal, search_radius);

	// Estimate FPFH Feature
	pcl::FPFHEstimationOMP<PointT, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4);
	if(!input_cloud->empty())
		est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(cloud_normal);
	typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	est_fpfh.setSearchMethod(tree);
	//est_fpfh.setKSearch(20);
	est_fpfh.setRadiusSearch(2.0*search_radius);
	est_fpfh.compute(*cloud_fpfh);
}


template<typename PointT>
void CRegistration<PointT>::Coarsereg_FPFHSAC(const typename pcl::PointCloud<PointT>::Ptr & SourceCloud,
	const typename pcl::PointCloud<PointT>::Ptr & TargetCloud,
	typename pcl::PointCloud<PointT>::Ptr & TransformedSource,
	Eigen::Matrix4f & transformationS2T,
	float search_radius)
{
	clock_t t0, t1;
	t0 = clock();

	fpfhFeaturePtr source_fpfh(new fpfhFeature());
	fpfhFeaturePtr target_fpfh(new fpfhFeature());

	compute_fpfh_feature(SourceCloud, source_fpfh, search_radius);
	compute_fpfh_feature(TargetCloud, target_fpfh, search_radius);
    
	pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(SourceCloud);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(TargetCloud);
	sac_ia.setTargetFeatures(target_fpfh);
	//sac_ia.setNumberOfSamples(20);       
	sac_ia.setCorrespondenceRandomness(15); 
	sac_ia.align(*TransformedSource);
	transformationS2T = sac_ia.getFinalTransformation();
	
	t1 = clock();
	// Commented these out if you don't want to output the registration log
	cout << "FPFH-SAC done in " << float(t1 - t0) / CLOCKS_PER_SEC << " s" << endl << transformationS2T << endl;
	cout << "The fitness score of this registration is " << sac_ia.getFitnessScore() << endl;
	cout << "-----------------------------------------------------------------------------" << endl;
}

template<typename PointT>
bool CRegistration<PointT>::CSTRAN_4DOF(const std::vector <std::vector<double>> & coordinatesA, const std::vector <std::vector<double>> & coordinatesB, std::vector<double> &transpara, int cp_number)
{
	double tx,ty,a,b;                   // 4 parameters 
	double s, rot_rad, rot_degree;
	double RMSE_check, sum_squaredist;
	int pointnumberA, pointnumberB, pointnumbercheck;
	Eigen::Vector4d transAB;
	transpara.resize(5);

	pointnumberA = coordinatesA.size();
	pointnumberB = coordinatesB.size();

	sum_squaredist = 0;

	if (cp_number < 3)
	{
		cout << "Error ! Not enough control point number ..." << endl;
		return 0;
	}

	Eigen::MatrixXd A_;
	Eigen::VectorXd b_;

	A_.resize(cp_number * 2, 4);
	b_.resize(cp_number * 2, 1);

	for (int j = 0; j < cp_number; j++)
	{
		// A Matrix
		A_(j * 2, 0) = 1;
		A_(j * 2, 1) = 0;
		A_(j * 2, 2) = coordinatesA[j][0];
		A_(j * 2, 3) = -coordinatesA[j][1];

		A_(j * 2 + 1, 0) = 0;
		A_(j * 2 + 1, 1) = 1;
		A_(j * 2 + 1, 2) = coordinatesA[j][1];
		A_(j * 2 + 1, 3) = coordinatesA[j][0];

		//b Vector
		b_(j * 2, 0) = coordinatesB[j][0];
		b_(j * 2 + 1, 0) = coordinatesB[j][1];
	}
	transAB = ((A_.transpose()*A_).inverse())*A_.transpose()*b_;
		
	tx = transAB(0, 0);
	ty = transAB(1, 0);
	a = transAB(2, 0);
	b = transAB(3, 0);
	s = sqrt(a*a + b*b);
	
	transpara[0] = tx;
	transpara[1] = ty;
	transpara[2] = s;
	transpara[3] = b / s; //sin (ang)
	transpara[4] = a / s; //cos (ang)
	
	cout.setf(ios::showpoint);  //��С�����Ⱥ����0��ʾ����;
	cout.precision(12);         //����������ȣ�������Ч����;

	cout<< "Estimated Transformation From A to B" << endl
		<< "tx: " << tx << " m" << endl
		<< "ty: " << ty << " m" << endl
		<< "scale: " << s << endl;

	// Checking
	if (pointnumberA >= pointnumberB) pointnumbercheck = pointnumberB;
	else pointnumbercheck = pointnumberA;

	if (pointnumbercheck <= cp_number) cout << "Not enough points for check ..." << endl;
	else
	{
		pointnumbercheck -= cp_number;
		for (int j = 0; j < pointnumbercheck; j++)
		{
			double X_tran, Y_tran, squaredist;
			X_tran = transpara[2] * transpara[4] * coordinatesA[j + cp_number][0] - transpara[2] * transpara[3] * coordinatesA[j + cp_number][1] + transpara[0];
			Y_tran = transpara[2] * transpara[3] * coordinatesA[j + cp_number][0] + transpara[2] * transpara[4] * coordinatesA[j + cp_number][1] + transpara[1];
			squaredist = (X_tran - coordinatesB[j + cp_number][0])*(X_tran - coordinatesB[j + cp_number][0]) +
				(Y_tran - coordinatesB[j + cp_number][1])*(Y_tran - coordinatesB[j + cp_number][1]);
			sum_squaredist += squaredist;
		}

		RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

		cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
	}

	return 1;
}

template<typename PointT>
bool CRegistration<PointT>::CSTRAN_7DOF(const std::vector <std::vector<double>> & coordinatesA, const std::vector <std::vector<double>> & coordinatesB, std::vector<double> &transpara, int cp_number)
{
	double RMSE_check, sum_squaredist;
	int pointnumberA, pointnumberB, pointnumbercheck;
	Eigen::VectorXd transAB;
	transAB.resize(7);
	transpara.resize(7);

	pointnumberA = coordinatesA.size();
	pointnumberB = coordinatesB.size();

	sum_squaredist = 0;

	if (cp_number < 4)
	{
		cout << "Error ! Not enough control point number ..." << endl;
		return 0;
	}

	Eigen::MatrixXd A_;
	Eigen::VectorXd b_;

	A_.resize(cp_number * 3, 7);
	b_.resize(cp_number * 3, 1);

	for (int j = 0; j < cp_number; j++)
	{
		// A Matrix   tx ty tz rx ry rz s
		A_(j * 3, 0) = 1;
		A_(j * 3, 1) = 0;
		A_(j * 3, 2) = 0;
		A_(j * 3, 3) = 0;
		A_(j * 3, 4) = -coordinatesA[j][2];
		A_(j * 3, 5) = coordinatesA[j][1];
		A_(j * 3, 6) = coordinatesA[j][0];

		A_(j * 3 + 1, 0) = 0;
		A_(j * 3 + 1, 1) = 1;
		A_(j * 3 + 1, 2) = 0;
		A_(j * 3 + 1, 3) = coordinatesA[j][2];
		A_(j * 3 + 1, 4) = 0;
		A_(j * 3 + 1, 5) = -coordinatesA[j][0];
		A_(j * 3 + 1, 6) = coordinatesA[j][1];

		A_(j * 3 + 2, 0) = 0;
		A_(j * 3 + 2, 1) = 0;
		A_(j * 3 + 2, 2) = 1;
		A_(j * 3 + 2, 3) = -coordinatesA[j][1];
		A_(j * 3 + 2, 4) = coordinatesA[j][0];
		A_(j * 3 + 2, 5) = 0;
		A_(j * 3 + 2, 6) = coordinatesA[j][2];

		//b Vector
		b_(j * 3, 0) = coordinatesB[j][0];
		b_(j * 3 + 1, 0) = coordinatesB[j][1];
		b_(j * 3 + 2, 0) = coordinatesB[j][2];
	}
	transAB = ((A_.transpose()*A_).inverse())*A_.transpose()*b_;

	transpara[0] = transAB(0);transpara[1] = transAB(1);transpara[2] = transAB(2);transpara[3] = transAB(3); transpara[4] = transAB(4); transpara[5] = transAB(5);transpara[6] = transAB(6);

	cout.setf(ios::showpoint);  //��С�����Ⱥ����0��ʾ����;
	cout.precision(10);         //����������ȣ�������Ч����;

	cout << "Estimated Transformation From A to B" << endl
		<< "tx: " << transpara[0] << " m" << endl
		<< "ty: " << transpara[1] << " m" << endl
		<< "tz: " << transpara[2] << " m" <<endl
		<< "rx: " << transpara[3] << endl
		<< "ry: " << transpara[4] << endl
		<< "rz: " << transpara[5] << endl
		<< "scale: " << transpara[6] << endl;

	// Checking
	if (pointnumberA >= pointnumberB) pointnumbercheck = pointnumberB;
	else pointnumbercheck = pointnumberA;

	if (pointnumbercheck <= cp_number) cout << "Not enough points for check ..." << endl;
	else
	{
		pointnumbercheck -= cp_number;
		for (int j = 0; j < pointnumbercheck; j++)
		{
			double X_tran, Y_tran, Z_tran, squaredist;
			X_tran = transpara[0] + transpara[6] * coordinatesA[j + cp_number][0] + transpara[5] * coordinatesA[j + cp_number][1] - transpara[4] * coordinatesA[j + cp_number][2];
			Y_tran = transpara[1] + transpara[6] * coordinatesA[j + cp_number][1] - transpara[5] * coordinatesA[j + cp_number][0] + transpara[3] * coordinatesA[j + cp_number][2];
			Z_tran = transpara[2] + transpara[6] * coordinatesA[j + cp_number][2] + transpara[4] * coordinatesA[j + cp_number][0] - transpara[3] * coordinatesA[j + cp_number][1];
			squaredist = (X_tran - coordinatesB[j + cp_number][0])*(X_tran - coordinatesB[j + cp_number][0]) +
				(Y_tran - coordinatesB[j + cp_number][1])*(Y_tran - coordinatesB[j + cp_number][1])+
				(Z_tran - coordinatesB[j + cp_number][2])*(Z_tran - coordinatesB[j + cp_number][2]);
			sum_squaredist += squaredist;
		}

		RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

		cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
	}

	return 1;
}


//Brief: Using the Gauss-Newton Least Square Method to solve 4 Degree of Freedom Transformation from no less than 2 points
//cp_number is the control points' number for LLS calculation, the rest of points are used to check the accuracy;
template<typename PointT>
bool CRegistration<PointT>::LLS_4DOF(const std::vector <std::vector<double>> & coordinatesA, const std::vector <std::vector<double>> & coordinatesB, Eigen::Matrix4d & TransMatrixA2B, int cp_number, double theta0_degree)
{
	Eigen::Vector4d transAB;
	Eigen::Vector4d temp_trans;

	int iter_num = 0;

	double theta, theta0, dtheta, eps;  // in rad
	double yaw_degree;   // in degree
	double tx, ty, tz;                  // in meter
	double RMSE_check, sum_squaredist;
	int pointnumberA, pointnumberB, pointnumbercheck;
	//std::vector <std::vector<double>> coordinatesAT;

	//cout << "Input the approximate yaw angle in degree" << endl;
	//cin >> theta0_degree;

	dtheta = 9999;
	eps = 1e-9;

	theta0 = theta0_degree / 180 * M_PI; //Original Guess

	pointnumberA = coordinatesA.size();
	pointnumberB = coordinatesB.size();

	sum_squaredist = 0;

	if (cp_number < 2)
	{
		cout << "Error ! Not enough control point number ..." << endl;
		return 0;
	}

	while (abs(dtheta)>eps)
	{

		Eigen::MatrixXd A_;
		Eigen::VectorXd b_;

		A_.resize(cp_number * 3, 4);
		b_.resize(cp_number * 3, 1);

		for (int j = 0; j < cp_number; j++)
		{
			// A Matrix
			A_(j * 3, 0) = -coordinatesA[j][0] * sin(theta0) - coordinatesA[j][1] * cos(theta0);
			A_(j * 3, 1) = 1;
			A_(j * 3, 2) = 0;
			A_(j * 3, 3) = 0;

			A_(j * 3 + 1, 0) = coordinatesA[j][0] * cos(theta0) - coordinatesA[j][1] * sin(theta0);
			A_(j * 3 + 1, 1) = 0;
			A_(j * 3 + 1, 2) = 1;
			A_(j * 3 + 1, 3) = 0;

			A_(j * 3 + 2, 0) = 0;
			A_(j * 3 + 2, 1) = 0;
			A_(j * 3 + 2, 2) = 0;
			A_(j * 3 + 2, 3) = 1;

			//b Vector
			b_(j * 3, 0) = coordinatesB[j][0] - coordinatesA[j][0] * cos(theta0) + coordinatesA[j][1] * sin(theta0);
			b_(j * 3 + 1, 0) = coordinatesB[j][1] - coordinatesA[j][0] * sin(theta0) - coordinatesA[j][1] * cos(theta0);
			b_(j * 3 + 2, 0) = coordinatesB[j][2] - coordinatesA[j][2];
		}

		//x=(ATA)-1(ATb)
		temp_trans = ((A_.transpose()*A_).inverse())*A_.transpose()*b_;
		dtheta = temp_trans(0, 0);

		theta0 += dtheta;

		iter_num++;

		//cout << "Result for iteration " << iter_num << " is " << endl
		//<< temp_trans(1, 0) << " , " << temp_trans(2, 0) << " , " << temp_trans(3, 0) << " , " << theta0 * 180 / M_PI <<endl;
	}

	transAB = temp_trans;

	theta = theta0;
	yaw_degree = theta * 180 / M_PI;

	tx = transAB(1, 0);
	ty = transAB(2, 0);
	tz = transAB(3, 0);


	cout.setf(ios::showpoint);  
	cout.precision(12);        


	cout << "Calculated by Linear Least Square"<<endl
		 << "Converged in " << iter_num << " iterations ..." << endl
		 << "Station B 's Coordinate and Orientation in A's System is:" << endl
		 << "X: " << tx << " m" << endl
		 << "Y: " << ty << " m" << endl
		 << "Z: " << tz << " m" << endl
		 << "yaw: " << yaw_degree << " degree" << endl;


	TransMatrixA2B(0, 0) = cos(theta);
	TransMatrixA2B(0, 1) = -sin(theta);
	TransMatrixA2B(0, 2) = 0;
	TransMatrixA2B(0, 3) = tx;

	TransMatrixA2B(1, 0) = sin(theta);
	TransMatrixA2B(1, 1) = cos(theta);
	TransMatrixA2B(1, 2) = 0;
	TransMatrixA2B(1, 3) = ty;

	TransMatrixA2B(2, 0) = 0;
	TransMatrixA2B(2, 1) = 0;
	TransMatrixA2B(2, 2) = 1;
	TransMatrixA2B(2, 3) = tz;

	TransMatrixA2B(3, 0) = 0;
	TransMatrixA2B(3, 1) = 0;
	TransMatrixA2B(3, 2) = 0;
	TransMatrixA2B(3, 3) = 1;

	cout << "The Transformation Matrix from Coordinate System A to B is: " << endl
		<< TransMatrixA2B << endl;


	// Checking
	if (pointnumberA >= pointnumberB) pointnumbercheck = pointnumberB;
	else pointnumbercheck = pointnumberA;

	if (pointnumbercheck <= cp_number) cout << "Not enough points for check ..." << endl;
	else
	{
		pointnumbercheck -= cp_number;
		for (int j = 0; j < pointnumbercheck; j++)
		{
			double X_tran, Y_tran, Z_tran, squaredist;
			X_tran = cos(theta)*coordinatesA[j + cp_number][0] - sin(theta)*coordinatesA[j + cp_number][1] + tx;
			Y_tran = sin(theta)*coordinatesA[j + cp_number][0] + cos(theta)*coordinatesA[j + cp_number][1] + ty;
			Z_tran = coordinatesA[j + cp_number][2] + tz;

			squaredist = (X_tran - coordinatesB[j + cp_number][0])*(X_tran - coordinatesB[j + cp_number][0]) +
				(Y_tran - coordinatesB[j + cp_number][1])*(Y_tran - coordinatesB[j + cp_number][1]) +
				(Z_tran - coordinatesB[j + cp_number][2])*(Z_tran - coordinatesB[j + cp_number][2]);
			sum_squaredist += squaredist;
		}

		RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

		cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
	}

	return 1;
}

template<typename PointT>
bool CRegistration<PointT>::SVD_6DOF(const std::vector <std::vector<double>> & coordinatesA, const std::vector <std::vector<double>> & coordinatesB, Eigen::Matrix4d & TransMatrixA2B, int cp_number)
{
	Eigen::Matrix4f transAB2D;
	pcl::PointCloud<PointT> Points2D_A, Points2D_B;
	double ZAB_mean, ZAB_sum;
	int pointnumberA, pointnumberB, pointnumbercheck;
	double RMSE_check, sum_squaredist;
	
	pointnumberA = coordinatesA.size();
	pointnumberB = coordinatesB.size();
	ZAB_sum = 0;
	sum_squaredist = 0;

	for (size_t i = 0; i < cp_number; i++)
	{
		PointT PtA,PtB;
		PtA.x = coordinatesA[i][0];
		PtA.y = coordinatesA[i][1];
		PtA.z = coordinatesA[i][2];
		
		PtB.x = coordinatesB[i][0];
		PtB.y = coordinatesB[i][1];
		PtB.z = coordinatesB[i][2];

		Points2D_A.push_back(PtA);
		Points2D_B.push_back(PtB);
		ZAB_sum += (coordinatesB[i][2] - coordinatesA[i][2]);
	}

	ZAB_mean = ZAB_sum / cp_number;

	if (cp_number < 2)
	{
		cout << "Error ! Not enough control point number ..." << endl;
		return 0;
	}

	pcl::registration::TransformationEstimationSVD<PointT, PointT> svd_estimator;
	svd_estimator.estimateRigidTransformation(Points2D_A, Points2D_B, transAB2D);

	TransMatrixA2B = transAB2D.cast<double>();

	/*
	TransMatrixA2B(0, 0) = transAB2D(0, 0);
	TransMatrixA2B(0, 1) = transAB2D(0, 1);
	TransMatrixA2B(0, 2) = 0;
	TransMatrixA2B(0, 3) = transAB2D(0, 3);

	TransMatrixA2B(1, 0) = transAB2D(1, 0);
	TransMatrixA2B(1, 1) = transAB2D(1, 1);
	TransMatrixA2B(1, 2) = 0;
	TransMatrixA2B(1, 3) = transAB2D(1, 3);

	TransMatrixA2B(2, 0) = 0;
	TransMatrixA2B(2, 1) = 0;
	TransMatrixA2B(2, 2) = 1;
	TransMatrixA2B(2, 3) = ZAB_mean;

	TransMatrixA2B(3, 0) = 0;
	TransMatrixA2B(3, 1) = 0;
	TransMatrixA2B(3, 2) = 0;
	TransMatrixA2B(3, 3) = 1;
	*/

	double tx, ty, tz, yaw_rad, yaw_degree;

	tx = TransMatrixA2B(0, 3);
	ty = TransMatrixA2B(1, 3);
	tz = TransMatrixA2B(2, 3);
	yaw_rad = acos(TransMatrixA2B(0, 0));
	if (TransMatrixA2B(1, 0) < 0) yaw_rad = -yaw_rad;
	yaw_degree = yaw_rad / M_PI * 180;

	cout << "Calculated by SVD" << endl
		 << "Station B 's Coordinate and Orientation in A's System is:" << endl
		 << "X: " << tx << " m" << endl
		 << "Y: " << ty << " m" << endl
		 << "Z: " << tz << " m" << endl
		 << "yaw: " << yaw_degree << " degree" << endl;


	cout << "The Transformation Matrix from Coordinate System A to B is: " << endl
		 << TransMatrixA2B << endl;


	// Checking
	if (pointnumberA >= pointnumberB) pointnumbercheck = pointnumberB;
	else pointnumbercheck = pointnumberA;

	if (pointnumbercheck <= cp_number) cout << "Not enough points for check ..." << endl;
	else
	{
		pointnumbercheck -= cp_number;
		for (int j = 0; j < pointnumbercheck; j++)
		{
			double X_tran, Y_tran, Z_tran, squaredist;
			X_tran = TransMatrixA2B(0, 0)*coordinatesA[j + cp_number][0] + TransMatrixA2B(0, 1)*coordinatesA[j + cp_number][1] + TransMatrixA2B(0, 2)*coordinatesA[j + cp_number][2]+ tx;
			Y_tran = TransMatrixA2B(1, 0)*coordinatesA[j + cp_number][0] + TransMatrixA2B(1, 1)*coordinatesA[j + cp_number][1] + TransMatrixA2B(1, 2)*coordinatesA[j + cp_number][2]+ ty;
			Z_tran = TransMatrixA2B(2, 0)*coordinatesA[j + cp_number][0] + TransMatrixA2B(2, 1)*coordinatesA[j + cp_number][1] + TransMatrixA2B(2, 2)*coordinatesA[j + cp_number][2]+ tz;

			squaredist = (X_tran - coordinatesB[j + cp_number][0])*(X_tran - coordinatesB[j + cp_number][0]) +
				(Y_tran - coordinatesB[j + cp_number][1])*(Y_tran - coordinatesB[j + cp_number][1]) +
				(Z_tran - coordinatesB[j + cp_number][2])*(Z_tran - coordinatesB[j + cp_number][2]);
			sum_squaredist += squaredist;
		}

		RMSE_check = sqrt(sum_squaredist / pointnumbercheck);

		cout << "Calculated from " << pointnumbercheck << " points, the RMSE is " << RMSE_check << endl;
	}
}

}