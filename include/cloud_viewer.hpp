#ifndef _INCLUDE_CLOUD_VIEWER_H
#define _INCLUDE_CLOUD_VIEWER_H

#include <string>
#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <memory>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "utility.h"

using namespace std;

namespace ghicp
{

// Visualization color scale
enum color_type
{
    INTENSITY,
    HEIGHT,
    SINGLE,
    FRAME
};

template <typename PointT>
class CloudViewer
{
  public:
    //Constructor
    CloudViewer()
    {
        is_frist_epoch_ = 1;
    };
    ~CloudViewer(){};

    void Dispaly2Cloud(const typename pcl::PointCloud<PointT>::Ptr &Cloud1, const typename pcl::PointCloud<PointT>::Ptr &Cloud2,
                       std::string displayname, int display_downsample_ratio)
    {
        std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));

        viewer->setBackgroundColor(255, 255, 255);
        char t[256];
        std::string s;
        int n = 0;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < Cloud1->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud1->points[i].x;
                pt.y = Cloud1->points[i].y;
                pt.z = Cloud1->points[i].z;
                pt.r = 255;
                pt.g = 215;
                pt.b = 0;
                pointcloud1->points.push_back(pt);
            }
        } // Golden

        if (!viewer->updatePointCloud(pointcloud1, "pointcloudT"))
        {
            viewer->addPointCloud(pointcloud1, "pointcloudT");
        }

        for (size_t i = 0; i < Cloud2->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud2->points[i].x;
                pt.y = Cloud2->points[i].y;
                pt.z = Cloud2->points[i].z;
                pt.r = 233;
                pt.g = 233;
                pt.b = 216;
                pointcloud2->points.push_back(pt);
            }
        } // Silver

        if (!viewer->updatePointCloud(pointcloud2, "pointcloudS"))
        {
            viewer->addPointCloud(pointcloud2, "pointcloudS");
        }

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::microseconds(100000));
        }
    }

    void DisplayNClouds(const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds1,
                        const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds2,
                        std::string displayname, color_type color_mode, int display_downsample_ratio)
    {
        std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);
        char t[256];
        std::string s;

        //Create two vertically separated viewports
        int v1(0);
        int v2(1);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

        // Set camera position and orientation
        float x_position = clouds1[0]->points[0].x;
        float y_position = clouds1[0]->points[0].y;
        float z_position = clouds1[0]->points[0].z;

        viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 0);
        viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 1);
        viewer->setSize(1600, 900); // Visualiser window size

        _DisplayNClouds(clouds1, viewer, "1-", color_mode, display_downsample_ratio, v1);
        _DisplayNClouds(clouds2, viewer, "2-", color_mode, display_downsample_ratio, v2);

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::microseconds(100000));
        }
    }

    void DisplayNClouds(const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
                        std::string displayname, color_type color_mode, int display_downsample_ratio)
    {
        std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(displayname));
        viewer->setBackgroundColor(0, 0, 0);
        //Create two vertically separated viewports
        int v1(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        // Set camera position and orientation
        float x_position = clouds[0]->points[0].x;
        float y_position = clouds[0]->points[0].y;
        float z_position = clouds[0]->points[0].z;

        viewer->setCameraPosition(x_position, y_position, z_position, 1, 0, 0, 0, 0, 1, 0);

        _DisplayNClouds(clouds, viewer, "prefix", color_mode, display_downsample_ratio, 0);

        cout << "Click X(close) to continue..." << endl;
        while (!viewer->wasStopped())
        {
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::microseconds(100000));
        }
    }

    bool displayRegistration_on_fly(std::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                                    const typename pcl::PointCloud<PointT>::Ptr &Cloud_S,
                                    const typename pcl::PointCloud<PointT>::Ptr &Cloud_T,
                                    int display_downsample_ratio,
                                    int display_time_ms)
    {
        if (!is_frist_epoch_) // You need to remove the original camera and point cloud first
        {
            viewer->removePointCloud("pointcloudS");
            viewer->removePointCloud("pointcloudT");
        }
        else
        {
            is_frist_epoch_ = 0;
        }

        bool intensity_available = pcl::traits::has_field<PointT, pcl::fields::intensity>::value;
        if (Cloud_T->points[0].intensity < 0.001)
            intensity_available = 0;

        char t[256];
        std::string s;
        int n = 0;
        float intensity_ratio;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_S_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud_T_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < Cloud_T->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud_T->points[i].x;
                pt.y = Cloud_T->points[i].y;
                pt.z = Cloud_T->points[i].z;

                if (intensity_available)
                    intensity_ratio = 1.0 * Cloud_T->points[i].intensity;
                else
                    intensity_ratio = 1.0;

                pt.r = 233 * intensity_ratio;
                pt.g = 233 * intensity_ratio;
                pt.b = 216 * intensity_ratio;
                Cloud_T_rgb->points.push_back(pt);
            }
        } // Silver

        if (!viewer->updatePointCloud(Cloud_T_rgb, "pointcloudT"))
        {
            viewer->addPointCloud(Cloud_T_rgb, "pointcloudT");
        }

        for (size_t i = 0; i < Cloud_S->points.size(); ++i)
        {
            if (i % display_downsample_ratio == 0)
            {
                pcl::PointXYZRGB pt;
                pt.x = Cloud_S->points[i].x;
                pt.y = Cloud_S->points[i].y;
                pt.z = Cloud_S->points[i].z;

                if (intensity_available)
                    intensity_ratio = 1.0 * Cloud_S->points[i].intensity;
                else
                    intensity_ratio = 1.0;

                pt.r = 255 * intensity_ratio;
                pt.g = 215 * intensity_ratio;
                pt.b = 0 * intensity_ratio;
                Cloud_S_rgb->points.push_back(pt);
            }
        } // Golden

        if (!viewer->updatePointCloud(Cloud_S_rgb, "pointcloudS"))
        {
            viewer->addPointCloud(Cloud_S_rgb, "pointcloudS");
        }

        //std::cout << "Update the viewer done." << std::endl;

        viewer->spinOnce(display_time_ms);
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
    }

  private:
    bool is_frist_epoch_;

    void _DisplayNClouds(const std::vector<typename pcl::PointCloud<PointT>::Ptr> &clouds,
                         std::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
                         std::string prefix, color_type color_mode, int display_downsample_ratio, int viewport)
    {
        char ch_t[256];
        std::string str;

        float maxz, minz, maxz2, minz2, c_value;
        maxz2 = 20.0;
        minz2 = -20.0;
        float frame_color_r, frame_color_g, frame_color_b;

        for (int j = 0; j < clouds.size(); j++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            switch (j)
            {
            case 0:
            {
                frame_color_r = 255;
                frame_color_g = 0;
                frame_color_b = 0;
                break;
            }
            case 1:
            {
                frame_color_r = 0;
                frame_color_g = 255;
                frame_color_b = 0;
                break;
            }
            case 2:
            {
                frame_color_r = 0;
                frame_color_g = 0;
                frame_color_b = 255;
                break;
            }
            case 3:
            {
                frame_color_r = 255;
                frame_color_g = 255;
                frame_color_b = 0;
                break;
            }
            default:
            {
                frame_color_r = 255 * (rand() / (1.0 + RAND_MAX));
                frame_color_g = 255 * (rand() / (1.0 + RAND_MAX));
                frame_color_b = 255 * (rand() / (1.0 + RAND_MAX));
                break;
            }
            }

            for (size_t i = 0; i < clouds[j]->points.size(); ++i)
            {
                if (i % display_downsample_ratio == 0) //Downsample for display
                {
                    pcl::PointXYZRGB pt;
                    pt.x = clouds[j]->points[i].x;
                    pt.y = clouds[j]->points[i].y;
                    pt.z = clouds[j]->points[i].z;

                    switch (color_mode)
                    {
                    case SINGLE: //Single Color for all the points: Golden
                    {
                        pt.r = 255;
                        pt.g = 215;
                        pt.b = 0;
                        break;
                    }
                    case HEIGHT: //Height ramp color scalar
                    {
                        c_value = min_(max_(pt.z - minz2, 0.0) / (maxz2 - minz2), 1.0);
                        pt.r = 255 * c_value;
                        pt.g = 255 * (1.0 - c_value);
                        pt.b = 50 + 150 * c_value;
                        break;
                    }
                    case FRAME: //Random color for each frame
                    {
                        pt.r = frame_color_r * clouds[j]->points[i].intensity / 255;
                        pt.g = frame_color_g * clouds[j]->points[i].intensity / 255;
                        pt.b = frame_color_b * clouds[j]->points[i].intensity / 255;
                        break;
                    }
                    case INTENSITY: //Fix it later
                    {
                        //float color_intensity= 255.0 * (submap.frames[j].pointcloud_odom_down->points[i].intensity - mini)/(maxi-mini);
                        pt.r = min_(1.1 * clouds[j]->points[i].intensity, 255);
                        pt.g = pt.r;
                        pt.b = pt.r;
                        break;
                    }
                    default: //RED
                    {
                        pt.r = 255;
                        pt.g = 0;
                        pt.b = 0;
                        break;
                    }
                    }

                    rgbcloud->points.push_back(pt);
                }
            }
            sprintf(ch_t, "%d", j);
            str = prefix + ch_t;
            viewer->addPointCloud(rgbcloud, str, viewport);
        }
    }
    //    vector<double> global_shift;
};
} // namespace ghicp

#endif //_INCLUDE_CLOUD_VIEWER_H