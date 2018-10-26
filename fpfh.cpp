#include "utility.h"
#include "fpfh.h"
#include <pcl\PointIndices.h>


using namespace  std;
using namespace  utility;

fpfhFeaturePtr FPFHfeature::compute_fpfh_feature(const pcXYZIPtr &input_cloud){
	//计算法向量
	NormalsPtr point_normal(new Normals);
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> est_normal;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(10);
	//est_normal.setRadiusSearch(0.02);
	est_normal.compute(*point_normal);
	
	//fpfh 估计
	fpfhFeaturePtr fpfh(new fpfhFeature());
	
	pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4);
	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	//est_fpfh.setRadiusSearch(0.03);
	est_fpfh.compute(*fpfh);

	return fpfh;
}

fpfhFeaturePtr FPFHfeature::compute_fpfh_keypoint(const pcXYZIPtr &input_cloud, const pcl::PointIndicesPtr &indices){
	
	pcXYZIPtr inputkp(new pcXYZI());
	for (size_t i = 0; i < indices->indices.size(); i++){
		inputkp->points[i] = input_cloud->points[indices->indices[i]];
	}
	
	NormalsPtr point_normal(new Normals);
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> est_normal;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
	est_normal.setInputCloud(inputkp);
	est_normal.setSearchMethod(tree);
	//est_normal.setKSearch(100);
	est_normal.setRadiusSearch(radius);
	est_normal.compute(*point_normal);

	//fpfh 估计
	fpfhFeaturePtr fpfh(new fpfhFeature());

	pcl::FPFHEstimationOMP<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(4);
	est_fpfh.setInputCloud(inputkp);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	//est_fpfh.setKSearch(50);
	est_fpfh.setRadiusSearch(radius/2);
	est_fpfh.compute(*fpfh);

	return fpfh;


}

void FPFHfeature::keyfpfh(const fpfhFeaturePtr &source_fpfh, const fpfhFeaturePtr &target_fpfh, const pcl::PointIndicesPtr &sindices, const pcl::PointIndicesPtr &tindices,
	fpfhFeaturePtr &source_kfpfh, fpfhFeaturePtr &target_kfpfh)
{
	source_kfpfh->width = sindices->indices.size();
	source_kfpfh->height = 1;
	target_kfpfh->width = tindices->indices.size();
	target_kfpfh->height = 1;
	source_kfpfh->points.resize(source_kfpfh->width * source_kfpfh->height);
	target_kfpfh->points.resize(target_kfpfh->width * target_kfpfh->height);

	for (size_t i = 0; i < sindices->indices.size();i++){
		source_kfpfh->points[i] = source_fpfh->points[sindices->indices[i]];
	}

	for (size_t i = 0; i < tindices->indices.size();i++){
		target_kfpfh->points[i] = target_fpfh->points[tindices->indices[i]];
	}
	
}



pcXYZIPtr FPFHfeature::fpfhalign(const pcXYZIPtr sourcecloud,const pcXYZIPtr targetcloud, const fpfhFeaturePtr source_fpfh, const fpfhFeaturePtr target_fpfh){
	//FPFH-SAC配准
	clock_t start, end, time;
	start = clock();

	pcl::SampleConsensusInitialAlignment<pcl::PointXYZI, pcl::PointXYZI, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(sourcecloud);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(targetcloud);
	sac_ia.setTargetFeatures(target_fpfh);
	pcXYZIPtr alignedcloud(new pcXYZI());
	//sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	sac_ia.setCorrespondenceRandomness(6); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	sac_ia.align(*alignedcloud);
	cout << "Has converged:" << sac_ia.hasConverged() << "score" << sac_ia.getFitnessScore() << endl;

	end = clock();
	cout << "FPFH-SAC Registration completed" << endl;
	cout << "calculate time is: " << float(end - start) / CLOCKS_PER_SEC << endl;

	return alignedcloud;

}

float FPFHfeature::compute_fpfh_distance(float his1[33] ,float his2[33]){
	//how to calculate the distance/similarity of histogram
	// First method : Correlation 相关系数法 
	// Correlation= COV(x,y)/sqrt(D(x)*D(y))
	
	float d_correlation = 0;
	float d_correlation_up=0;
	float d_correlation_down1 = 0;
	float d_correlation_down2 = 0;
	float mean_his1 = 0;
	float mean_his2 = 0;

	for (int i = 0; i < 33; i++){
		mean_his1 += his1[i];
		mean_his2 += his2[i];
	}
	mean_his1 /= 33;
	mean_his2 /= 33;

	for (int i = 0; i < 33; i++){
		d_correlation_up += (his1[i] - mean_his1)*(his2[i] - mean_his2);
		d_correlation_down1 += (his1[i] - mean_his1)*(his1[i] - mean_his1);
		d_correlation_down2 += (his2[i] - mean_his2)*(his2[i] - mean_his2);
	}
	d_correlation = d_correlation_up / sqrt(d_correlation_down1*d_correlation_down2);

	return abs(d_correlation);
}

void FPFHfeature::displayhistogram(const fpfhFeaturePtr fpfh, int index){
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	pcl::visualization::PCLPlotter plotter;
	plotter.addFeatureHistogram<pcl::FPFHSignature33>(*fpfh, "fpfh", index);
}

void FPFHfeature::displaycorrespondence(const pcXYZIPtr sourcecloud, const pcXYZIPtr targetcloud, const pcXYZIPtr aligncloud, const fpfhFeaturePtr source_fpfh, const fpfhFeaturePtr target_fpfh)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("Fpfh test"));
    
	int v1 = 0;
    int v2 = 1;
    view->createViewPort(0, 0, 0.5, 1, v1);
    view->createViewPort(0.5, 0, 1, 1, v2);
    view->setBackgroundColor(0, 0, 0, v1);
    view->setBackgroundColor(0.05, 0, 0, v2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> source_cloud_color(sourcecloud, 255, 0, 0);
	view->addPointCloud(sourcecloud, source_cloud_color, "source_cloud_v1", v1);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> target_cloud_color(targetcloud, 0, 255, 0);
	view->addPointCloud(targetcloud, target_cloud_color, "target_cloud_v1", v1);

    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source_cloud_v1", v1);

    //
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> aligend_cloud_color(aligncloud, 255, 0, 0);
    view->addPointCloud(aligncloud, aligend_cloud_color, "aligend_cloud_v2", v2);

	view->addPointCloud(targetcloud, target_cloud_color, "target_cloud_v2", v2);

    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "aligend_cloud_v2");
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud_v2");

    //对应关系可视化
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> crude_cor_est;
    boost::shared_ptr<pcl::Correspondences> cru_correspondences(new pcl::Correspondences);
    crude_cor_est.setInputSource(source_fpfh);
    crude_cor_est.setInputTarget(target_fpfh);
    //crude_cor_est.determineCorrespondences(*cru_correspondences);
    crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
    cout << "crude size is " << cru_correspondences->size() << endl;
    view->addCorrespondences<pcl::PointXYZI>(sourcecloud, targetcloud, *cru_correspondences, "correspondence", v1);
    view->initCameraParameters();
    while (!view->wasStopped())
    {
	   view->spinOnce(100);
	   boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

    system("pause");
   
}


