#ifndef BINARY_FEATRUE_EXTRACTION
#define BINARY_FEATRUE_EXTRACTION


#include "StereoBinaryFeature.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>
#include <xmmintrin.h>
#include <limits>

#include <boost/tuple/tuple.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>

#include <Eigen/Eigenvalues>
#include "utility.h"


//�����
#define SHOW_PROCESS	//�Ƿ���ʾ��ǰ��ȡ����
//#define DISTANCE_WEIGHT	//�Ƿ���о����Ȩ

using namespace std;


class StereoBinaryFeatureExtractor:public StereoBinaryFeature
{
public:
	/************************************************************************/
	/*               Ŀǰ���õ�ʹ�������������ֵ��Ϊ0,1������                 */
	/************************************************************************/
	float extract_radius_;				//���������İ뾶��С
	unsigned int voxel_side_num_;		//���������ĸ������� voxel_side_num_=N ���������N*N*N
	float unit_side_length_;			//ÿ�����ӵĸ����߳�
	vector<float> side_length_thresh_;	//�洢�ߵ��ٽ�ֵ


	int gridFeatureDimension_;
	int compairFeatureDimensionInEachPlane_;//ÿһ��ͶӰ���ϱȽ�������ά��;
	int compairFeatureDimension_;

	///////////////////////////��������ԵĴ洢//////////////////////////
	vector<pair<int, int>> grid_index_pairs_2d_;
	
	/*�洢�����Ϣ�ĸ���*/
	struct GridVoxel
	{
		double point_num;
		float point_weight;
		float intensity;
		float density;
		float normalized_point_weight;
		float average_depth;
		GridVoxel() :point_num(0.0), point_weight(0.0f), intensity(0.0f), density(0.0f), 
			         normalized_point_weight(0.0f), average_depth(0.0f)
		{

		}
	};

	/*���캯��*/
	StereoBinaryFeatureExtractor(float extract_radius,			//������ȡ�뾶
								 unsigned int voxel_side_num,   //���ָ�����Ŀ ������Ŀ���Ϊ�����������ԵЧӦ
								 bool build_sample_pattern = false
								 ):	extract_radius_(extract_radius),voxel_side_num_(voxel_side_num)
	{

		gridFeatureDimension_ = 3 * voxel_side_num_*voxel_side_num_;
		compairFeatureDimensionInEachPlane_ = voxel_side_num_*voxel_side_num_;
		compairFeatureDimension_ = 6 * compairFeatureDimensionInEachPlane_;
		

		//���㵥λ�߳�
		unit_side_length_=2*extract_radius_/voxel_side_num_;

		
		//�����ٽ�ֵ
		side_length_thresh_.resize(voxel_side_num_);
		for(int i=0;i<voxel_side_num_;i++)
		{
			side_length_thresh_[i]=-extract_radius_+i*unit_side_length_;
		}

		if (build_sample_pattern)
		{
			//�Ƚϵ��;
			for (int i = 0; i < compairFeatureDimensionInEachPlane_; i++)
			{
				int pair1, pair2;
				do 
				{
					pair1 = rand() % (voxel_side_num_*voxel_side_num_);
					pair2 = rand() % (voxel_side_num_*voxel_side_num_);
				} while (pair1 == pair2 || contain2DPair(pair1, pair2));
				
				grid_index_pairs_2d_.push_back(pair<int, int>(pair1, pair2));
			}

			ofstream fout("sample_pattern.txt");
			cout << "����������---------------------------------" << endl;
			for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
			{
				fout << grid_index_pairs_2d_[i].first << " " << grid_index_pairs_2d_[i].second << endl;
				//cout << grid_index_pairs_2d_[i].first << " " << grid_index_pairs_2d_[i].second << endl;

			}
			fout.flush();
			fout.close();
		}

		else
		{
			ifstream fin("sample_pattern.txt");
			grid_index_pairs_2d_.resize(compairFeatureDimensionInEachPlane_);
			for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
			{
				fin >> grid_index_pairs_2d_[i].first >> grid_index_pairs_2d_[i].second;
			}
			fin.close();
		}
	}

	

	/*����ÿ���ؼ���ľֲ�����ϵ��������ΪZ������,�ڶ�������ΪX������,�����任���þֲ�����ϵ�µĵ�ѹ��result_cloud��;*/
	bool computeLocalCoordinateSystem(const pcXYZIPtr & input_cloud,//�������
								      int test_index,    //�ؼ����������
									  const vector<int> & search_indices, //�ؼ���������������
								      CoordinateSystem &localCoordinateSystem);	//�ֲ�����ϵ�ķ���;

	void transformPointCloudToLocalSystem(const pcXYZIPtr & input_cloud,//�������
								      int test_index,    //�ؼ����������
									  const vector<int> & search_indices, //�ؼ���������������  
									  const CoordinateSystem &localCoordinateSystem,
									  pcXYZIPtr & result_cloud);//������Ѿ������ת�����Եĵ���;
	
	/*���յ��ܶȼ���ĵ��С�������������*/
	void constructCubicGrid(const pcXYZIPtr & rotated_cloud,		//����ת�����μ���õ��ľֲ�����
						    vector<GridVoxel> & grid);									//�������


	//�������������������ͶӰ���ͶӰ����;
	StereoBinaryFeature  computeFeatureProjectedGrid(const vector<GridVoxel> & grid);

	//�������������������ͶӰ���ͶӰ�����������Ƚϵ�����;
	StereoBinaryFeature  computeFeatureProjectedGridAndCompareFeature(const vector<GridVoxel> & grid);

	//�������������������ͶӰ���ͶӰ�����������Ƚϵ�����(ÿ��ͶӰ��ֱ�Ƚ�);
	StereoBinaryFeature  computeFeatureProjectedGridAndCompareFeature2D(const vector<GridVoxel> & grid);

	/*�����ɵĸ�����ֵ����Ϊ��������*/
	StereoBinaryFeature computeFeatureBinarizeGrid(const vector<GridVoxel> & grid);	//�������,�������������;

	//��ά�������������;
	void randomSamplePointPairs();

	/*��ȡ��������,��ȡʧ���򷵻ص�vectorΪ��*/
	void extractBinaryFeatures(const pcXYZIPtr & input_cloud,	//����ĵ���
		                       const pcl::PointIndicesPtr &indices,
		                       doubleVectorSBF & features);

	bool extractBinaryFeatureOfKeypoint(const pcXYZIPtr & input_cloud,	//����ĵ���
		                                size_t ptIndex, const std::vector<int> &searchIndexes,
										vectorSBF & feature);
  
private:
	
	inline int getVoxelNum(float coor)
	{
		for(int i=0;i<voxel_side_num_;++i)
		{
			if(coor<side_length_thresh_[i])
				return i-1;
		}
		return voxel_side_num_-1;
	}

	/*
	�õ���x,y,z�����ӵ�����ֵ ��0��ʼ
	*/
	inline int getVoxelIndex(int x, int y, int z)
	{
		return x + y*voxel_side_num_ + z*voxel_side_num_*voxel_side_num_;
	}


	bool contain2DPair(int pair1, int pair2)
	{
		if (grid_index_pairs_2d_.empty() == true)
		{
			return false;
		}
		else
		{
			for (int i = 0; i < grid_index_pairs_2d_.size(); i++)
			{
				if (((pair1 == grid_index_pairs_2d_[i].first) && (pair2 == grid_index_pairs_2d_[i].second))
					|| ((pair1 == grid_index_pairs_2d_[i].second) && (pair2 == grid_index_pairs_2d_[i].first)))
				{
					return true;
				}	
			}
			return false;
		}
		
	}

	//����2D��pca����ֲ�����ֲ�����������;
	void computeEigenVectorsBy2Dpca(const pcXYZIPtr & input_cloud,//�������
		                            const vector<int> & search_indices, //�ؼ���������������
									int test_index,
								    Eigen::Vector3f  &principalDirection);

	bool computeEigenVectorsByWeightPCA(const pcXYZIPtr & input_cloud,//�������
		                                const vector<int> & search_indices, //�ؼ���������������
								        int test_index,
								        Eigen::Vector3f  &principalDirection,
								        Eigen::Vector3f  &middleDirection,
								        Eigen::Vector3f  &normalDirection);

	bool computeEigenVectorsByPCA(const pcXYZIPtr & input_cloud,//�������
								const vector<int> & search_indices, //�ؼ���������������
								int test_index,
								Eigen::Vector3f  &principalDirection,
								Eigen::Vector3f  &middleDirection,
								Eigen::Vector3f  &normalDirection);

	//����ֲ�����ϵ�ͳ�������ϵ����ת����;
	void computeTranformationMatrixBetweenCoordinateSystems(const CoordinateSystem & coordinate_src,
		                                                    const CoordinateSystem & coordinate_traget,
															Eigen::Matrix4f  & tragetToSource);

	int getNumof1InFeature(const StereoBinaryFeature &feature)
	{
		int count(0);
		for (int i = 0; i < feature.size_; i++)
		{
			int bit_num = i % 8;
			int byte_num = i / 8;
			char test_num = 1 << bit_num;
			if (feature.feature_[byte_num] & test_num)
			{
				count++;
			}

		}
		return count;
	}


	//����������֮���3ά�ռ����;
	float Comput3DDistanceBetweenPoints(const pcl::PointXYZI &pt1, const pcl::PointXYZI &pt2)
	{
		float dertax, dertay, dertaz, dis;
		dertax = pt1.x - pt2.x;
		dertay = pt1.y - pt2.y;
		dertaz = pt1.z - pt2.z;
		dis = (dertax)*(dertax)+(dertay)*(dertay)+(dertaz)*(dertaz);
		dis = sqrt(dis);
		return dis;
	}

};

#endif