#include "filter.h"

template<typename PointT>
bool CFilter<PointT>::voxelfilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, float voxel_size)
{
    float inverse_voxel_size = 1.0f / voxel_size;

	Eigen::Vector4f min_p, max_p;
	pcl::getMinMax3D(*cloud_in, min_p, max_p);

	Eigen::Vector4f gap_p;  //boundingbox gap;
	gap_p = max_p - min_p;
		
	unsigned long long max_vx = ceil(gap_p.coeff(0)*inverse_voxel_size)+1;
	unsigned long long max_vy = ceil(gap_p.coeff(1)*inverse_voxel_size)+1;
	unsigned long long max_vz = ceil(gap_p.coeff(2)*inverse_voxel_size)+1;
		
	if (max_vx*max_vy*max_vz >= std::numeric_limits<unsigned long long>::max())
	{
		std::cout << "Filtering Failed: The number of box exceed the limit."<<endl;
		return 0;
	}

	unsigned long long mul_vx = max_vy*max_vz;
	unsigned long long mul_vy = max_vz;
	unsigned long long mul_vz = 1;

	std::vector<IDPair> id_pairs(cloud_in->size());
	unsigned int idx = 0;
	for (typename pcl::PointCloud<PointT>::iterator it = cloud_in->begin(); it != cloud_in->end(); it++)
	{

	   unsigned long long vx = floor((it->x - min_p.coeff(0))*inverse_voxel_size);
	   unsigned long long vy = floor((it->y - min_p.coeff(1))*inverse_voxel_size);
	   unsigned long long vz = floor((it->z - min_p.coeff(2))*inverse_voxel_size);

	   unsigned long long voxel_idx = vx*mul_vx + vy*mul_vy + vz*mul_vz;

	   IDPair pair;
	   pair.idx = idx;
	   pair.voxel_idx = voxel_idx;
	   id_pairs.push_back(pair);
	   idx++;
	}

	//Do sorting
	std::sort(id_pairs.begin(), id_pairs.end());

	unsigned int begin_id = 0;

	while (begin_id < id_pairs.size())
	{
		cloud_out->push_back(cloud_in->points[id_pairs[begin_id].idx]);

		unsigned int compare_id = begin_id + 1;
		while (compare_id < id_pairs.size() && id_pairs[begin_id].voxel_idx == id_pairs[compare_id].voxel_idx) compare_id++;
		begin_id = compare_id;
	}
	return 1;
}


//SOR (Statisics Outliers Remover);
template<typename PointT>
bool CFilter<PointT>::SORFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in,  typename pcl::PointCloud<PointT>::Ptr & cloud_out, int MeanK, double std)  
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<PointT> sor;
         
	sor.setInputCloud(cloud_in);
	sor.setMeanK(MeanK);         //50
	sor.setStddevMulThresh(std); //2.0
	sor.filter(*cloud_out);

    return 1;  
}

//Filter the point cloud according to the horizontal and vertical distance to the lidar center
template<typename PointT>
bool CFilter<PointT>::DisFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, double xy_dis_max, double z_min, double z_max)  
{
	double dis_square;
	for (int i=0; i< cloud_in->points.size();i++)
	{
		dis_square=cloud_in->points[i].x*cloud_in->points[i].x + cloud_in->points[i].y + cloud_in->points[i].y;
		if (dis_square < xy_dis_max*xy_dis_max && cloud_in->points[i].z<z_max && cloud_in->points[i].z>z_min)
		{cloud_out->points.push_back(cloud_in->points[i]);}
	}
    return 1;
}

template<typename PointT>
bool CFilter<PointT>::ActiveObjectFilter(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_out, std::vector<Bounds> & active_bbxs)  
{
    vector<bool> is_static(cloud_in->points.size(),1);
    for (int i=0; i<cloud_in->points.size();i++)
    {
       for (int j=0; j<active_bbxs.size();j++)
       {
           //In the bounding box
           if(cloud_in->points[i].x > active_bbxs[j].min_x && cloud_in->points[i].x < active_bbxs[j].max_x &&
              cloud_in->points[i].y > active_bbxs[j].min_y && cloud_in->points[i].y < active_bbxs[j].max_y &&
              cloud_in->points[i].z > active_bbxs[j].min_z && cloud_in->points[i].z < active_bbxs[j].max_z )
           {is_static[i]=0; break;}
       }
       if(is_static[i]) cloud_out->points.push_back(cloud_in->points[i]);   
    }

	return 1;
}

template<typename PointT>
bool CFilter<PointT>::ExtractGroundPoint(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_ground, typename pcl::PointCloud<PointT>::Ptr & cloud_unground ,float grid_resolution,float max_height_difference)
{
    Bounds bounds;
    CenterPoint center_pt;
    getBoundAndCenter(cloud_in , bounds , center_pt);
    
    int row, col, num_voxel;  
	row = ceil((bounds.max_y - bounds.min_y) / grid_resolution);
	col = ceil((bounds.max_x - bounds.min_x) / grid_resolution);
	num_voxel = row*col;

	Voxel* grid = new Voxel[num_voxel];
	for (int i = 0; i < num_voxel; i++) {grid[i].min_z = FLT_MAX;}
    
    //Preprocessing
	preprocessing(cloud_in, bounds.max_x, bounds.max_y, bounds.min_x, bounds.min_y, row, col, num_voxel, grid, grid_resolution);
    
    //Processing
	processing(cloud_in, cloud_ground, cloud_unground, grid, num_voxel, grid_resolution, max_height_difference);
    
    //Post processing
	postprocessing(cloud_in, cloud_ground, cloud_unground);

	delete[]grid;

    return 1;
}

template<typename PointT>
void CFilter<PointT>::preprocessing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, double max_x, double max_y, double min_x, double min_y, int row, int col, int num_voxel, Voxel* grid, float grid_resolution)
{
    int temp_num_voxel,ExpandGrid_List, ExpandGrid_Row;
	ExpandGrid_List = col + 2;
	ExpandGrid_Row = row + 2;
	temp_num_voxel = ExpandGrid_Row*ExpandGrid_List;
	Voxel *temp_grid = new Voxel[temp_num_voxel];

	for (int i = 0; i < cloud_in->points.size(); i++)
	{
		int temp_row, temp_list, temp_num;
		temp_list = floor((cloud_in->points[i].x - min_x) / grid_resolution);
		temp_row = floor((cloud_in->points[i].y - min_y) / grid_resolution);
		temp_num = temp_row*col + temp_list;
		if (temp_num >= 0 && temp_num < num_voxel)
		{
			grid[temp_num].point_id.push_back(i);
			grid[temp_num].PointsNumber++;
			if (cloud_in->points[i].z < grid[temp_num].min_z)
			{
				grid[temp_num].min_z = cloud_in->points[i].z;
				grid[temp_num].NeighborMin_z = cloud_in->points[i].z;
			}
		}
	}

	for (int i = 0; i < num_voxel; i++)
	{
		int ExpandGrid_TempRow, ExpandGrid_TempList, ExpandGrid_TempNum;
		ExpandGrid_TempRow = i / col + 1;
		ExpandGrid_TempList =i % col + 1;
		ExpandGrid_TempNum = ExpandGrid_TempRow*ExpandGrid_List + ExpandGrid_TempList;
		temp_grid[ExpandGrid_TempNum].min_z = grid[i].min_z;
		if (ExpandGrid_TempList == 1 || ExpandGrid_TempRow == 1 || ExpandGrid_TempList == col || ExpandGrid_TempRow == row)
		{
			if (ExpandGrid_TempList == 1)
			{
				temp_grid[ExpandGrid_TempNum - 1].min_z = grid[i].min_z;
				if (ExpandGrid_TempRow == 1)
					temp_grid[ExpandGrid_TempNum - 1 - ExpandGrid_TempList].min_z = grid[i].min_z;
			}
			else
			{
				if (ExpandGrid_TempList == col)
				{
					temp_grid[ExpandGrid_TempNum + 1].min_z = grid[i].min_z;
					if (ExpandGrid_TempRow == col)
						temp_grid[ExpandGrid_TempNum + 1 + ExpandGrid_TempList].min_z = grid[i].min_z;
				}
			}
			if (ExpandGrid_TempRow == 1)
			{
				temp_grid[ExpandGrid_TempNum - ExpandGrid_List].min_z = grid[i].min_z;
				if (ExpandGrid_TempList == col)
					temp_grid[ExpandGrid_TempNum + 1 - ExpandGrid_TempList].min_z = grid[i].min_z;
			}
			else
			{
				if (ExpandGrid_TempRow == row)
				{
					temp_grid[ExpandGrid_TempNum + ExpandGrid_List].min_z = grid[i].min_z;
					if (ExpandGrid_TempList == 1)
						temp_grid[ExpandGrid_TempNum - 1 + ExpandGrid_TempList].min_z = grid[i].min_z;

				}
			}
		}
	}
	for (int i = 0; i < num_voxel; i++)
	{
		int ExpandGrid_TempRow, ExpandGrid_TempList, ExpandGrid_TempNum;
		ExpandGrid_TempRow = i / col + 1;
		ExpandGrid_TempList = i % col + 1;
		ExpandGrid_TempNum = ExpandGrid_TempRow*ExpandGrid_List + ExpandGrid_TempList;
		for (int j = -1; j < 2; j++)
		{
			for (int k = -1; k<2; k++)
			{
				if (grid[i].NeighborMin_z > temp_grid[ExpandGrid_TempNum + j*ExpandGrid_List + k].min_z && (j != 0 || k != 0))
					grid[i].NeighborMin_z = temp_grid[ExpandGrid_TempNum + j*ExpandGrid_List + k].min_z;
			}
		}
	}
	delete[] temp_grid;
}

template<typename PointT>
void CFilter<PointT>::processing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in, typename pcl::PointCloud<PointT>::Ptr & cloud_ground, typename pcl::PointCloud<PointT>::Ptr & cloud_unground,
			Voxel* grid, int num_voxel, float grid_resolution, float max_height_difference)
{
    int n = 0, m = 0;
	for (int i = 0; i < num_voxel; i++)
	{
		for (int j = 0; j < grid[i].point_id.size(); j++)
		{
			if (cloud_in->points[grid[i].point_id[j]].z - grid[i].min_z < max_height_difference && grid[i].min_z - grid[i].NeighborMin_z < 2 * grid_resolution) //Add to ground points
			{
				cloud_ground->points.push_back(cloud_in->points[grid[i].point_id[j]]);
			}
			else //Add to nonground points
			{
				cloud_unground->points.push_back(cloud_in->points[grid[i].point_id[j]]);
			}
		}
	}
}


template<typename PointT>
void CFilter<PointT>::postprocessing(const typename pcl::PointCloud<PointT>::Ptr & cloud_in,
			typename pcl::PointCloud<PointT>::Ptr & cloud_ground,
			typename pcl::PointCloud<PointT>::Ptr & cloud_unground)

{
    typename pcl::PointCloud<PointT>::Ptr temp_ground_cloud(new typename pcl::PointCloud<PointT>());
	std::vector<int> is_ground_points;
	
    PointT groundpointsTotempgroudpoints;

	for (int i = 0; i < cloud_ground->points.size(); i++)
	{
		groundpointsTotempgroudpoints = cloud_ground->points[i];
		temp_ground_cloud->push_back(groundpointsTotempgroudpoints);
		is_ground_points.push_back(i);
	}
	cloud_ground->clear();
	
    std::set<int, less<int> > Unsearch;
	std::set<int, less<int> >::iterator iterUnsearch;

	std::vector<int>isearse;
	
    for (int i = 0; i < temp_ground_cloud->points.size(); i++)
	{
		isearse.push_back(i);
		Unsearch.insert(i);
	}

	pcl::KdTreeFLANN<PointT> Kdtree_search_ground_cloud;
	pcl::KdTreeFLANN<PointT> Kdtree_search_cloud;

	Kdtree_search_ground_cloud.setInputCloud(temp_ground_cloud);
	Kdtree_search_cloud.setInputCloud(cloud_in);
	
    //Radius search
    float radius = 1.0;
	std::vector<int>PointIdSearch_ground_cloud;
	std::vector<float>PointDistanceSearch_ground_cloud;
	std::vector<int>PointIdSearch_cloud;
	std::vector<float>PointDistanceSearch_cloud;
	
    PointT Searchpoint;
	int Pointsub;

	while (!Unsearch.empty()) //Still some points not searched
	{
		iterUnsearch = Unsearch.begin();
		Pointsub = *iterUnsearch;
		Searchpoint = temp_ground_cloud->points[Pointsub];
		Unsearch.erase(Pointsub);
		Kdtree_search_ground_cloud.radiusSearch(Searchpoint, radius, PointIdSearch_ground_cloud, PointDistanceSearch_ground_cloud); //Radius search
		
        //Empricial Settings
        if (PointIdSearch_ground_cloud.size()<5) 
		{
			if (isearse[Pointsub] != -1)
			{
				is_ground_points[Pointsub] = -2; 
				isearse[Pointsub] = -1;
			}
		}    
		else
		{
			if (PointIdSearch_ground_cloud.size()>10)
			{
				for (int i = 0; i < PointIdSearch_ground_cloud.size(); i++)
				{
					if (isearse[PointIdSearch_ground_cloud[i]] != -1)
					{
						Unsearch.erase(PointIdSearch_ground_cloud[i]);
						isearse[PointIdSearch_ground_cloud[i]] = -1;
					}
				}
			}
			else
			{
				Kdtree_search_cloud.radiusSearch(Searchpoint, radius, PointIdSearch_cloud, PointDistanceSearch_cloud);
				if (PointIdSearch_cloud.size() > 2 * PointIdSearch_ground_cloud.size())
				{
					for (int i = 0; i < PointIdSearch_ground_cloud.size(); i++)
					{
						if (isearse[PointIdSearch_ground_cloud[i]] != -1)
						{
							is_ground_points[PointIdSearch_ground_cloud[i]] = -1;
							Unsearch.erase(PointIdSearch_ground_cloud[i]);
							isearse[PointIdSearch_ground_cloud[i]] = -1;
						}
					}
				}
				else
				{
					if (isearse[Pointsub] != -1)
					{
						Unsearch.erase(Pointsub);
						isearse[Pointsub] = -1;
					}
				}
            }
		}
	}
    //Free the memory
	isearse.clear();
	vector<int>().swap(isearse);
	PointIdSearch_cloud.clear();
	vector<int>().swap(PointIdSearch_cloud);
	PointDistanceSearch_cloud.clear();
	vector<float>().swap(PointDistanceSearch_cloud);
	PointIdSearch_ground_cloud.clear();
	vector<int>().swap(PointIdSearch_ground_cloud);
	PointDistanceSearch_ground_cloud.clear();
	vector<float>().swap(PointDistanceSearch_ground_cloud);
	
    for (int i = 0; i < temp_ground_cloud->points.size(); i++)
	{
		if (is_ground_points[i] != -1)
		{
			if (is_ground_points[i] != -2)
			{
				cloud_ground->push_back(temp_ground_cloud->points[i]);
			}
		}
		else
		{
			cloud_unground->points.push_back(temp_ground_cloud->points[i]);
		}
	}
	temp_ground_cloud->clear();
	pcl::PointCloud<pcl::PointXYZI>().swap(*temp_ground_cloud);
	is_ground_points.clear();
	vector<int>().swap(is_ground_points);
}