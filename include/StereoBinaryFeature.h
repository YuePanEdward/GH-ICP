#ifndef _INCLUDE_STEREO_BINARY_FEATRUE_H
#define _INCLUDE_STEREO_BINARY_FEATRUE_H

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

namespace ghicp
{

class StereoBinaryFeature
{
  public:
	char *feature_;
	unsigned int size_; //number of bit;
	unsigned int byte_; //number of byte;

	int bscVisualWordsIndex_;

	vector<int> bscVisualWordsIndexV_;

	size_t keypointIndex_;

	struct CoordinateSystem
	{
		Eigen::Vector3f xAxis;
		Eigen::Vector3f yAxis;
		Eigen::Vector3f zAxis;
		Eigen::Vector3f origin;
	};
	CoordinateSystem localSystem_; //Local Coordinate System (LCS)

	//Constructor
	StereoBinaryFeature(unsigned int size = 0) : size_(size), byte_(0)
	{
		if (size != 0)
		{
			byte_ = static_cast<unsigned int>(ceil(float(size_) / 8.f));
			feature_ = new char[byte_];

			for (unsigned int i = 0; i < byte_; i++)
			{
				feature_[i] = 0;
			}

			//initialization
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

	//Destructor
	~StereoBinaryFeature()
	{
		if (feature_ != nullptr)
		{
			delete[] feature_;
		}
	}

	//Operator overloading (Assignment)
	StereoBinaryFeature &operator=(const StereoBinaryFeature &sbf)
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

	//Copy
	StereoBinaryFeature(const StereoBinaryFeature &sbf)
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

	//Get the 0/1 value on n th position without subscription number checking
	bool getNthBitValue(int n)
	{
		int bit_num = n % 8;
		int byte_num = n / 8;
		char test_num = 1 << bit_num;
		return (feature_[byte_num] & test_num) != 0;
	}

	//Set the 0/1 value on n th position without subscription number checking
	void setNthBitValue(int n)
	{
		int bit_num = n % 8;
		int byte_num = n / 8;
		char test_num = 1 << bit_num;
		feature_[byte_num] |= test_num;
	}

	//Operator overloading (equal)
	bool operator==(const StereoBinaryFeature &sbf) const
	{
		//different size -> false
		if (sbf.size_ != size_)
			return false;
		//any difference at any position -> false
		for (int i = 0; i < byte_; i++)
		{
			//XOR > 0 -> there's some difference
			if ((feature_[i] ^ sbf.feature_[i]) > 0)
				return false;
		}
		return true;
	}

	boost::shared_array<unsigned char> make_shared();

	/*Calculate the hamming distance of two binary feature, if the size is different, return -1*/
	int hammingDistance(const StereoBinaryFeature &sbf1, const StereoBinaryFeature &sbf2);

	/*Read the binary feature*/
	void readFeatures(vector<StereoBinaryFeature> &features, const string &path);

	/*Write the binary feature*/
	void writeFeatures(const vector<StereoBinaryFeature> &features, const string &path);

  private:
	unsigned char byteBitsLookUp(unsigned char b);
};

typedef StereoBinaryFeature SBF;
typedef vector<SBF> vectorSBF;
typedef vector<vectorSBF> doubleVectorSBF;
typedef vector<doubleVectorSBF> TribleVectorSBF;

} // namespace ghicp
#endif //_INCLUDE_STEREO_BINARY_FEATRUE_H