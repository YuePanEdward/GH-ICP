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

    // if (io.paralist.feature == 1)
    // {

    //     //t5 = clock();
    //     //cout << "Time for BSC feature calculation: " << float(t5 - t4) / CLOCKS_PER_SEC << " s" << endl
    //     //		<< "-----------------------------------------------------------------------------" << endl;
    // }

    // /*---------------------- 4.(2) Feature calculation FPFH--------------------------*/
    // FPFHfeature fpfh;
    // fpfhFeaturePtr source_fpfh(new fpfhFeature), target_fpfh(new fpfhFeature);
    // fpfh.radius = io.paralist.feature_r;
    // if (io.paralist.feature == 2)
    // {
    //     fpfhFeaturePtr source_fpfh_all = fpfh.compute_fpfh_feature(filterPointCloudS);
    //     cout << "Source Cloud FPFH extracted" << endl;
    //     fpfhFeaturePtr target_fpfh_all = fpfh.compute_fpfh_feature(filterPointCloudT);
    //     cout << "Target Cloud FPFH extracted" << endl;
    //     fpfh.keyfpfh(source_fpfh_all, target_fpfh_all, keyPointIndicesS, keyPointIndicesT, source_fpfh, target_fpfh);
    //     //cout << "Keypoints' FPFH extracted" << endl;
    //     //t5 = clock();
    //     //cout << "Time for FPFH feature calculation: " << float(t5 - t4) / CLOCKS_PER_SEC << " s" << endl
    //     // 	<< "-----------------------------------------------------------------------------" << endl;
    // }

    return 0;
}