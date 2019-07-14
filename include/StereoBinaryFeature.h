#ifndef STEREO_BINARY_FEATRUE
#define STEREO_BINARY_FEATRUE

#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <boost/shared_array.hpp>

#include <Eigen/Eigen>

using namespace std;

class StereoBinaryFeature
{
public:
	char * feature_;
	unsigned int size_;	//��ʾbit��Ŀ;
	unsigned int byte_;	//��ʾ�ֽ���Ŀ;
	
	int bscVisualWordsIndex_;

	vector<int> bscVisualWordsIndexV_;

	size_t keypointIndex_;

	struct CoordinateSystem
	{
		Eigen::Vector3f  xAxis;
		Eigen::Vector3f  yAxis;
		Eigen::Vector3f  zAxis;
		Eigen::Vector3f  origin;
	};
	CoordinateSystem localSystem_;//�ֲ�����ϵ;

	//���캯��;
	StereoBinaryFeature(unsigned int size = 0) :size_(size), byte_(0)
	{
		if (size != 0)
		{
			byte_ = static_cast<unsigned int>(ceil(float(size_) / 8.f));
			feature_ = new char[byte_];

			for (unsigned int i = 0; i < byte_; i++)
			{
				feature_[i] = 0;
			}
			localSystem_.xAxis.x() = 0.0f;
			localSystem_.xAxis.y() = 0.0f;
			localSystem_.xAxis.z() = 0.0f;

			localSystem_.yAxis.x() = 0.0f;
			localSystem_.yAxis.y() = 0.0f;
			localSystem_.yAxis.z() = 0.0f;

			localSystem_.zAxis.x() = 0.0f;
			localSystem_.zAxis.y() = 0.0f;
			localSystem_.zAxis.z() = 0.0f;

			localSystem_.origin.x() = 0.0f;
			localSystem_.origin.y() = 0.0f;
			localSystem_.origin.z() = 0.0f;

			bscVisualWordsIndex_ = 0;
		}
		else
			feature_ = nullptr;
	}

	//��������
	~StereoBinaryFeature()
	{
		if (feature_ != nullptr)
		{
			delete[] feature_;
		}
	}

	//��ֵ���캯��
	StereoBinaryFeature & operator=(const StereoBinaryFeature & sbf)
	{
		this->size_ = sbf.size_;
		if (size_ > 0)
		{
			this->byte_ = sbf.byte_;
			this->feature_ = new char[byte_];
			this->localSystem_ = sbf.localSystem_;
			this->bscVisualWordsIndex_ = sbf.bscVisualWordsIndex_;
			this->bscVisualWordsIndexV_ = sbf.bscVisualWordsIndexV_;
			for (size_t i = 0; i < byte_; i++)
				this->feature_[i] = sbf.feature_[i];
		}
		else
			this->feature_ = nullptr;
		return *this;
	}

	//���ƹ��캯��
	StereoBinaryFeature(const StereoBinaryFeature& sbf)
	{
		this->size_ = sbf.size_;
		if (size_ > 0)
		{
			this->byte_ = sbf.byte_;
			this->feature_ = new char[byte_];
			this->localSystem_ = sbf.localSystem_;

			this->bscVisualWordsIndex_ = sbf.bscVisualWordsIndex_;
			this->bscVisualWordsIndexV_ = sbf.bscVisualWordsIndexV_;
			for (size_t i = 0; i < byte_; i++)
				this->feature_[i] = sbf.feature_[i];
		}
		else
			this->feature_ = nullptr;
	}


	//�õ�nλbit�ϵ�ֵΪ1��0     �������±���������
	bool getNthBitValue(int n)
	{
		int bit_num = n % 8;
		int byte_num = n / 8;
		char test_num = 1 << bit_num;
		return (feature_[byte_num] & test_num) != 0;
	}

	//����nλbit�ϵ�ֵΪ1	�������±���������
	void setNthBitValue(int n)
	{
		int bit_num = n % 8;
		int byte_num = n / 8;
		char test_num = 1 << bit_num;
		feature_[byte_num] |= test_num;
	}

	//��������ж�;
	bool operator==(const StereoBinaryFeature & sbf) const
	{
		//��С������򷵻�false
		if (sbf.size_ != size_)
			return false;
		//����һ��bit�ϲ�ͬ�򷵻�false
		for (int i = 0; i<byte_; i++)
		{
			//������0���ʾ����bit��ͬ
			if ((feature_[i] ^ sbf.feature_[i])>0)
				return false;
		}
		return true;
	}

	//���shared_array ת��Ϊchar������
	boost::shared_array<unsigned char> make_shared();

	/*������������������֮���hammingDistance����ֵΪ-1ʱ��ʾ����������С��һ���޷�����*/
	int hammingDistance(const StereoBinaryFeature & sbf1, const StereoBinaryFeature & sbf2);

	/*��������ʽ��������*/
	void readFeatures(vector<StereoBinaryFeature>& features, const string& path);

	/*��������ʽ�������*/
	void writeFeatures(const vector<StereoBinaryFeature>& features, const string& path);
	

private:
	unsigned char byteBitsLookUp(unsigned char b);

};

typedef StereoBinaryFeature  SBF;
typedef vector<SBF>  vectorSBF;
typedef vector<vectorSBF>  doubleVectorSBF;
typedef vector<doubleVectorSBF>  TribleVectorSBF;

#endif