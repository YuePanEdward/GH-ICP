#ifndef FPFH
#define FPFH
#include <pcl/io/pcd_io.h>
#include <ctime>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/fpfh.h>
#include <pcl/PointIndices.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/visualization/histogram_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/features/fpfh_omp.h> //����fpfh���ټ����omp(��˲��м���)
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_features.h> //�����Ĵ����Ӧ��ϵȥ��
#include <pcl/registration/correspondence_rejection_sample_consensus.h> //�������һ����ȥ��
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <Eigen/Dense>
#include "utility.h"
#include "keypointDetection.h"


using namespace utility;
using namespace std;
using namespace keypoint;

class FPFHfeature{
public:
	fpfhFeaturePtr compute_fpfh_feature(const pcXYZIPtr &input_cloud);
	fpfhFeaturePtr compute_fpfh_keypoint(const pcXYZIPtr &input_cloud, const pcl::PointIndicesPtr &indices);
	
	void keyfpfh(const fpfhFeaturePtr &source_fpfh, const fpfhFeaturePtr &target_fpfh, const pcl::PointIndicesPtr &sindices, const pcl::PointIndicesPtr &tindices,
		fpfhFeaturePtr &source_kfpfh, fpfhFeaturePtr &target_kfpfh);

	pcXYZIPtr fpfhalign(const pcXYZIPtr sourcecloud, const pcXYZIPtr targetcloud, const fpfhFeaturePtr source_fpfh, const fpfhFeaturePtr target_fpfh);
	float compute_fpfh_distance(float his1[33], float his2[33]);
	void displayhistogram(const fpfhFeaturePtr fpfh,int index);
	void displaycorrespondence(const pcXYZIPtr sourcecloud, const pcXYZIPtr targetcloud, const pcXYZIPtr aligncloud, const fpfhFeaturePtr source_fpfh,const fpfhFeaturePtr target_fpfh);

	double radius;
protected:

private:

};

#endif