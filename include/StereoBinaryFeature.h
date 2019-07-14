#ifndef STEREO_BINARY_FEATRUE
#define STEREO_BINARY_FEATRUE

#include <iostream>
#include <vector>
#include <cmath>
#include <ctime>
#include <fstream>
#include <string>
#include <sstream>

#include <pcl\kdtree\kdtree_flann.h>
#include <pcl\point_cloud.h>
#include <pcl\point_types.h>
#include <pcl\common\distances.h>
#include <boost\shared_array.hpp>

#include <Eigen/Eigen>

using namespace std;

class StereoBinaryFeature
{
public:
	char * feature_;
	unsigned int size_;	//表示bit数目;
	unsigned int byte_;	//表示字节数目;
	
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
	CoordinateSystem localSystem_;//局部坐标系;

	//构造函数;
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

	//析构函数
	~StereoBinaryFeature()
	{
		if (feature_ != nullptr)
		{
			delete[] feature_;
		}
	}

	//赋值构造函数
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

	//复制构造函数
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


	//得到n位bit上的值为1或0     不进行下标数量检验
	bool getNthBitValue(int n)
	{
		int bit_num = n % 8;
		int byte_num = n / 8;
		char test_num = 1 << bit_num;
		return (feature_[byte_num] & test_num) != 0;
	}

	//设置n位bit上的值为1	不进行下标数量检验
	void setNthBitValue(int n)
	{
		int bit_num = n % 8;
		int byte_num = n / 8;
		char test_num = 1 << bit_num;
		feature_[byte_num] |= test_num;
	}

	//重载相等判定;
	bool operator==(const StereoBinaryFeature & sbf) const
	{
		//大小不相等则返回false
		if (sbf.size_ != size_)
			return false;
		//任意一个bit上不同则返回false
		for (int i = 0; i<byte_; i++)
		{
			//异或大于0则表示存在bit不同
			if ((feature_[i] ^ sbf.feature_[i])>0)
				return false;
		}
		return true;
	}

	//输出shared_array 转换为char的数组
	boost::shared_array<unsigned char> make_shared();

	/*计算两个二进制特征之间的hammingDistance返回值为-1时表示两个特征大小不一致无法计算*/
	int hammingDistance(const StereoBinaryFeature & sbf1, const StereoBinaryFeature & sbf2);

	/*二进制形式读入特征*/
	void readFeatures(vector<StereoBinaryFeature>& features, const string& path);

	/*二进制形式输出特征*/
	void writeFeatures(const vector<StereoBinaryFeature>& features, const string& path);
	

private:
	unsigned char byteBitsLookUp(unsigned char b);

};

typedef StereoBinaryFeature  SBF;
typedef vector<SBF>  vectorSBF;
typedef vector<vectorSBF>  doubleVectorSBF;
typedef vector<doubleVectorSBF>  TribleVectorSBF;

#endif