#ifndef _INCLUDE_DATA_IO_HPP
#define _INCLUDE_DATA_IO_HPP

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

//liblas
#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

#include <string>
#include <fstream>
#include <vector>

#include "utility.h"

namespace ghicp
{

template <typename PointT>
class DataIo : public CloudUtility<PointT>
{
  public:
	bool readCloudFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
	{
		std::string extension;
		extension = fileName.substr(fileName.find_last_of('.') + 1); //Get the suffix of the file;

		if (!strcmp(extension.c_str(), "pcd"))
		{
			readPcdFile(fileName, pointCloud);
			std::cout << "A pcd file has been imported" << std::endl;
		}
		else if (!strcmp(extension.c_str(), "las"))
		{
			bool global_shift_or_not = 0;
			std::cout << "Would you like to do a global shift ?  0. No  1. Yes [default 0]" << std::endl;
			std::cin >> global_shift_or_not;
			if (!global_shift_or_not)
			{
				readLasFile(fileName, pointCloud);
			}
			else
			{
				bool use_automatic_drift = 0;
				std::cout << "Using the automatic shift or enter the global shift yourself ? " << std::endl
						  << "0. Read a global shift file  1.Use the automatic shift [default 0]" << std::endl;
				std::cin >> use_automatic_drift;
				readLasFile(fileName, pointCloud, use_automatic_drift);
			}
			std::cout << "A las file has been imported" << std::endl;
		}
		else if (!strcmp(extension.c_str(), "ply"))
		{
			readPlyFile(fileName, pointCloud);
			std::cout << "A ply file has been imported" << std::endl;
		}
		else if (!strcmp(extension.c_str(), "txt"))
		{
			readTxtFile(fileName, pointCloud);
			std::cout << "A txt file has been imported" << std::endl;
		}
		else
		{
			std::cout << "Undefined Point Cloud Format." << std::endl;
			return 0;
		}

		std::cout << "Data loaded (" << pointCloud->points.size() << " points)" << std::endl;
		return 1;
	}

	bool writeCloudFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
	{
		std::string extension;
		extension = fileName.substr(fileName.find_last_of('.') + 1); //Get the suffix of the file;

		if (!strcmp(extension.c_str(), "pcd"))
		{
			writePcdFile(fileName, pointCloud);
			std::cout << "A pcd file has been exported" << std::endl;
		}
		else if (!strcmp(extension.c_str(), "las"))
		{
			bool global_shift_or_not = 0;
			std::cout << "Would you like to do a global shift ?  0. No  1. Yes [default 0]" << std::endl;
			std::cin >> global_shift_or_not;
			if (!global_shift_or_not)
			{
				writeLasFile(fileName, pointCloud);
			}
			else
			{
				bool use_automatic_drift = 0;
				std::cout << "Using the automatic shift or enter the global shift yourself ? " << std::endl
						  << "0. Read a global shift file  1.Use the automatic shift [default 0]" << std::endl;
				std::cin >> use_automatic_drift;
				writeLasFile(fileName, pointCloud, use_automatic_drift);
			}
			std::cout << "A las file has been exported" << std::endl;
		}
		else if (!strcmp(extension.c_str(), "ply"))
		{
			writePlyFile(fileName, pointCloud);
			std::cout << "A ply file has been exported" << std::endl;
		}
		else if (!strcmp(extension.c_str(), "txt"))
		{
			writeTxtFile(fileName, pointCloud);
			std::cout << "A txt file has been exported" << std::endl;
		}
		else
		{
			std::cout << "Undefined Point Cloud Format." << std::endl;
			return 0;
		}
	}

	bool readPcdFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
	{
		if (pcl::io::loadPCDFile<PointT>(fileName, *pointCloud) == -1)
		{
			PCL_ERROR("Couldn't read file\n");
			return false;
		}
		return true;
	}

	bool writePcdFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
	{
		if (pcl::io::savePCDFileBinary(fileName, *pointCloud) == -1)
		{
			PCL_ERROR("Couldn't write file\n");
			return false;
		}
		return true;
	}

	bool readLasFileHeader(const std::string &fileName, liblas::Header &header)
	{
		if (fileName.substr(fileName.rfind('.')).compare(".las"))
		{
			return 0;
		}
		else
		{
			std::ifstream ifs;
			ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
			if (ifs.bad())
			{
				return 0;
			}

			liblas::ReaderFactory f;
			liblas::Reader reader = f.CreateWithStream(ifs);

			header = reader.GetHeader();
		}
		return 1;
	}

	bool readLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud) //Without translation
	{
		//cout << "A global translation or gravitization should be done to keep the precision of point cloud when adopting pcl to do las file point cloud processing" << endl;

		if (fileName.substr(fileName.rfind('.')).compare(".las"))
		{
			return 0;
		}

		std::ifstream ifs;
		ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			std::cout << "Matched Terms are not found." << std::endl;
		}
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);
		liblas::Header const &header = reader.GetHeader();

		//Bounding box Information
		double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
		Xmin = header.GetMinX();
		Ymin = header.GetMinY();
		Zmin = header.GetMinZ();
		Xmax = header.GetMaxX();
		Ymax = header.GetMaxY();
		Zmax = header.GetMaxZ();

		while (reader.ReadNextPoint())
		{
			const liblas::Point &p = reader.GetPoint();
			PointT pt;
			pt.x = p.GetX();
			pt.y = p.GetY();
			pt.z = p.GetZ();

			//------------------------------------------------Assign Intensity--------------------------------------------------//
			//If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
			//If the Point template PointT is without intensity, you should comment the line.
			//pt.intensity = p.GetIntensity();
			//pt.intensity = p.GetTime();
			//pt.intensity = p.GetScanAngleRank();
			//pt.intensity = p.GetNumberOfReturns();
			//pt.intensity = p.GetScanDirection();

			//---------------------------------------------------Assign Color--------------------------------------------------//
			//If the Point template PointT has RGB, you can assign the Color according to the point cloud in las.
			//If the Point template PointT is without RGB, you should comment the line.
			//liblas::Color lasColor;
			//lasColor= p.GetColor();
			//pt.r = lasColor.GetRed();
			//pt.g = lasColor.GetGreen();
			//pt.b = lasColor.GetBlue();

			pointCloud->points.push_back(pt);
		}
		return 1;
	}

	bool writeLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud) //Without translation
	{

		Bounds bound;
		this->getCloudBound(*pointCloud, bound);

		std::ofstream ofs;
		ofs.open(fileName.c_str(), std::ios::out | std::ios::binary);
		if (ofs.is_open())
		{
			liblas::Header header;
			header.SetDataFormatId(liblas::ePointFormat2);
			header.SetVersionMajor(1);
			header.SetVersionMinor(2);
			header.SetMin(bound.min_x, bound.min_y, bound.min_z);
			header.SetMax(bound.max_x, bound.max_y, bound.max_z);
			header.SetOffset((bound.min_x + bound.max_x) / 2.0, (bound.min_y + bound.max_y) / 2.0, (bound.min_z + bound.max_z) / 2.0);
			header.SetScale(0.01, 0.01, 0.01);
			header.SetPointRecordsCount(pointCloud->points.size());

			liblas::Writer writer(ofs, header);
			liblas::Point pt(&header);

			for (int i = 0; i < pointCloud->points.size(); i++)
			{
				pt.SetCoordinates(double(pointCloud->points[i].x), double(pointCloud->points[i].y), double(pointCloud->points[i].z));

				//If the Point template PointT is without intensity, you should comment the line.
				//pt.SetIntensity(pointCloud->points[i].intensity);

				//If the Point template PointT is without RGB, you should comment the line.
				//liblas::Color lasColor;
				//lasColor.SetRed(pointCloud->points[i].r);
				//lasColor.SetGreen(pointCloud->points[i].g);
				//lasColor.SetBlue(pointCloud->points[i].b);
				//pt.SetColor(lasColor);

				writer.WritePoint(pt);
			}
			ofs.flush();
			ofs.close();
		}
		return 1;
	}

	bool readLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not) //With translation
	{
		global_shift.resize(3);
		//std::cout << "A global translation or gravitization should be done to keep the precision of point cloud when adopting pcl to do las file point cloud processing" << endl;

		if (fileName.substr(fileName.rfind('.')).compare(".las"))
		{
			return 0;
		}

		std::ifstream ifs;
		ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			std::cout << "Matched Terms are not found." << std::endl;
		}
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);
		liblas::Header const &header = reader.GetHeader();

		//Bounding box Information
		double Xmin, Ymin, Zmin, Xmax, Ymax, Zmax;
		Xmin = header.GetMinX();
		Ymin = header.GetMinY();
		Zmin = header.GetMinZ();
		Xmax = header.GetMaxX();
		Ymax = header.GetMaxY();
		Zmax = header.GetMaxZ();

		if (automatic_shift_or_not)
		{
			// Automatic Gloabl Shift Value;
			global_shift[0] = -Xmin;
			global_shift[1] = -Ymin;
			global_shift[2] = -Zmin;

			std::ofstream out("GlobalShift.txt", std::ios::out);
			out << setiosflags(std::ios::fixed) << std::setprecision(8) << global_shift[0] << std::endl;
			out << setiosflags(std::ios::fixed) << std::setprecision(8) << global_shift[1] << std::endl;
			out << setiosflags(std::ios::fixed) << std::setprecision(8) << global_shift[2] << std::endl;
			out.close();

			std::cout << "A txt File named GlobalShift.txt is saved in current Folder" << std::endl;
		}
		else
		{
			std::string fileGlobalShift;
			std::cout << "Please enter or drag in the Global Shift File" << std::endl
					  << "Example [GlobalShift.txt] :" << std::endl
					  << "-366370.90" << std::endl
					  << "-3451297.82" << std::endl
					  << "-14.29" << std::endl;

			std::cin >> fileGlobalShift;

			std::ifstream in(fileGlobalShift.c_str(), std::ios::in);
			in >> global_shift[0];
			in >> global_shift[1];
			in >> global_shift[2];
			in.close();
		}

		while (reader.ReadNextPoint())
		{
			const liblas::Point &p = reader.GetPoint();
			PointT pt;

			//A translation to keep the precision
			pt.x = p.GetX() + global_shift[0];
			pt.y = p.GetY() + global_shift[1];
			pt.z = p.GetZ() + global_shift[2];

			//------------------------------------------------Assign Intensity--------------------------------------------------//
			//If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
			//If the Point template PointT is without intensity, you should comment the line.
			//pt.intensity = p.GetIntensity();
			//pt.intensity = p.GetTime();
			//pt.intensity = p.GetScanAngleRank();
			//pt.intensity = p.GetNumberOfReturns();
			//pt.intensity = p.GetScanDirection();

			//---------------------------------------------------Assign Color--------------------------------------------------//
			//If the Point template PointT has RGB, you can assign the Color according to the point cloud in las.
			//If the Point template PointT is without RGB, you should comment the line.
			//liblas::Color lasColor;
			//lasColor= p.GetColor();
			//pt.r = lasColor.GetRed();
			//pt.g = lasColor.GetGreen();
			//pt.b = lasColor.GetBlue();

			pointCloud->points.push_back(pt);
		}
		return 1;
	}

	bool writeLasFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, bool automatic_shift_or_not) //With translation
	{
		global_shift.resize(3);

		Bounds bound;
		this->getCloudBound(*pointCloud, bound);

		if (!automatic_shift_or_not)
		{
			std::cout << "Use default (last input) global shift or not" << std::endl
					  << "1. Yes  0. No" << std::endl;
			bool use_last_shift;
			std::cin >> use_last_shift;
			if (!use_last_shift)
			{
				std::string fileGlobalShift;
				std::cout << "Please enter or drag in the Global Shift File" << std::endl
						  << "Example [GlobalShift.txt] :" << std::endl
						  << "-366370.90" << std::endl
						  << "-3451297.82" << std::endl
						  << "-14.29" << std::endl;

				std::cin >> fileGlobalShift;

				std::ifstream in(fileGlobalShift, std::ios::in);
				in >> global_shift[0];
				in >> global_shift[1];
				in >> global_shift[2];
				in.close();
			}
		}

		std::ofstream ofs;
		ofs.open(fileName.c_str(), std::ios::out | std::ios::binary);
		if (ofs.is_open())
		{
			liblas::Header header;
			header.SetDataFormatId(liblas::ePointFormat2);
			header.SetVersionMajor(1);
			header.SetVersionMinor(2);
			header.SetMin(bound.min_x - global_shift[0], bound.min_y - global_shift[1], bound.min_z - global_shift[2]);
			header.SetMax(bound.max_x - global_shift[0], bound.max_y - global_shift[1], bound.max_z - global_shift[2]);
			header.SetOffset((bound.min_x + bound.max_x) / 2.0 - global_shift[0], (bound.min_y + bound.max_y) / 2.0 - global_shift[1], (bound.min_z + bound.max_z) / 2.0 - global_shift[2]);
			header.SetScale(0.01, 0.01, 0.01);
			header.SetPointRecordsCount(pointCloud->points.size());

			liblas::Writer writer(ofs, header);
			liblas::Point pt(&header);

			for (size_t i = 0; i < pointCloud->points.size(); i++)
			{
				pt.SetCoordinates(double(pointCloud->points[i].x) - global_shift[0], double(pointCloud->points[i].y) - global_shift[1], double(pointCloud->points[i].z) - global_shift[2]);

				bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
				if (intensity_available)
				{
					pt.SetIntensity(pointCloud->points[i].intensity);
				}

				//If the Point template PointT is without RGB, you should comment the line.
				//liblas::Color lasColor;
				//lasColor.SetRed(pointCloud->points[i].r);
				//lasColor.SetGreen(pointCloud->points[i].g);
				//lasColor.SetBlue(pointCloud->points[i].b);
				//pt.SetColor(lasColor);

				writer.WritePoint(pt);
			}
			ofs.flush();
			ofs.close();
		}
		return 1;
	}

	bool readLasFileLast(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
	{
		if (fileName.substr(fileName.rfind('.')).compare(".las"))
		{
			return 0;
		}

		std::ifstream ifs;
		ifs.open(fileName.c_str(), std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			std::cout << "Matched Terms are not found." << std::endl;
		}
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);
		liblas::Header const &header = reader.GetHeader();

		while (reader.ReadNextPoint())
		{
			const liblas::Point &p = reader.GetPoint();
			PointT pt;

			//A translation to keep the precision
			pt.x = p.GetX() + global_shift[0];
			pt.y = p.GetY() + global_shift[1];
			pt.z = p.GetZ() + global_shift[2];

			//------------------------------------------------Assign Intensity--------------------------------------------------//
			//If the Point template PointT has intensity, you can assign the intensity with any feature of the point cloud in las.
			bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
			if (intensity_available)
			{
				pt.intensity = p.GetIntensity();
			}
			//pt.intensity = p.GetTime();
			//pt.intensity = p.GetScanAngleRank();
			//pt.intensity = p.GetNumberOfReturns();
			//pt.intensity = p.GetScanDirection();

			//---------------------------------------------------Assign Color--------------------------------------------------//
			//If the Point template PointT has RGB, you can assign the Color according to the point cloud in las.
			//If the Point template PointT is without RGB, you should comment the line.
			//liblas::Color lasColor;
			//lasColor= p.GetColor();
			//pt.r = lasColor.GetRed();
			//pt.g = lasColor.GetGreen();
			//pt.b = lasColor.GetBlue();

			pointCloud->points.push_back(pt);
		}
		return 1;
	}

	bool readPlyFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
	{
		if (pcl::io::loadPLYFile<PointT>(fileName, *pointCloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file \n");
			return (-1);
		}
	}

	bool writePlyFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
	{
		if (pcl::io::savePLYFile<PointT>(fileName, *pointCloud) == -1) //* load the file
		{
			PCL_ERROR("Couldn't write file \n");
			return (-1);
		}
	}

	bool readTxtFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
	{
		std::ifstream in(fileName.c_str(), std::ios::in);
		if (!in)
		{
			return 0;
		}
		double x_ = 0, y_ = 0, z_ = 0;
		int i = 0;
		while (!in.eof())
		{
			in >> x_ >> y_ >> z_;
			if (in.fail())
			{
				break;
			}
			PointT Pt;
			Pt.x = x_;
			Pt.y = y_;
			Pt.z = z_;
			pointCloud->points.push_back(Pt);
			++i;
		}
		in.close();
		//std::cout << "Import finished ... ..." << std::endl;
		return 1;
	}

	bool writeTxtFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
	{
		std::ofstream ofs;
		ofs.open(fileName.c_str());
		if (ofs.is_open())
		{
			for (size_t i = 0; i < pointCloud->size(); ++i)
			{
				ofs << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].x << "  "
					<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].y << "  "
					<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].z
					//<<"  "<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].intensity
					<< std::endl;
			}
			ofs.close();
		}
		else
		{
			return 0;
		}
		//std::cout << "Output finished ... ..." << std::endl;
		return 1;
	}

	bool writeTxtFile(const std::string &fileName, const typename pcl::PointCloud<PointT>::Ptr &pointCloud, int subsample_ratio)
	{
		std::ofstream ofs;
		ofs.open(fileName.c_str());
		if (ofs.is_open())
		{
			for (size_t i = 0; i < pointCloud->size(); ++i)
			{
				if (i % subsample_ratio == 0) //Subsampling;
				{
					ofs << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].x << "  "
						<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].y << "  "
						<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].z
						//<<"  "<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[i].intensity
						<< std::endl;
				}
			}
			ofs.close();
		}
		else
		{
			return 0;
		}
		//std::cout << "Output finished ... ..." << std::endl;
		return 1;
	}

	bool outputKeypoints(const std::string &filename, const pcl::PointIndicesPtr &indices, const typename pcl::PointCloud<PointT>::Ptr &pointCloud)
	{
		std::ofstream ofs;
		ofs.open(filename);

		if (ofs.is_open())
		{
			for (int i = 0; i < indices->indices.size(); ++i)
			{
				ofs << setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[indices->indices[i]].x << "\t"
					<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[indices->indices[i]].y << "\t"
					<< setiosflags(std::ios::fixed) << std::setprecision(6) << pointCloud->points[indices->indices[i]].z << std::endl;
			}
			ofs.close();
		}
		else
		{
			return 0;
		}
		return 1;
	}

	bool savecoordinates(const typename pcl::PointCloud<PointT>::Ptr &Source_FPC, const typename pcl::PointCloud<PointT>::Ptr &Target_FPC,
						 pcl::PointIndicesPtr &Source_KPI, pcl::PointIndicesPtr &Target_KPI,
						 Eigen::MatrixX3d &SXYZ, Eigen::MatrixX3d &TXYZ)
	{

		SXYZ.resize(Source_KPI->indices.size(), 3);
		TXYZ.resize(Target_KPI->indices.size(), 3);
		for (int i = 0; i < Source_KPI->indices.size(); ++i)
		{
			SXYZ.row(i) << Source_FPC->points[Source_KPI->indices[i]].x, Source_FPC->points[Source_KPI->indices[i]].y, Source_FPC->points[Source_KPI->indices[i]].z;
		}
		for (int i = 0; i < Target_KPI->indices.size(); ++i)
		{
			TXYZ.row(i) << Target_FPC->points[Target_KPI->indices[i]].x, Target_FPC->points[Target_KPI->indices[i]].y, Target_FPC->points[Target_KPI->indices[i]].z;
		}
		std::cout << "Key points saved." << std::endl;
		return 1;
	}

  private:
	std::vector<double> global_shift;
};

} // namespace ghicp

#endif // _INCLUDE_DATA_IO_HPP
