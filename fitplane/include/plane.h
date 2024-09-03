/**
 *  Created by Qingchen Bi on 2022/11/05
 *  Several functions refer to PUTN
 */

#ifndef PLANE_H
#define PLANE_H

#include "World.h"
// #include "fitplane/PlaneMap.h"
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>

namespace FitPlane
{

struct Plane
{
public: 
    Eigen::Vector3d init_coord; // 2D 
    std::vector<Eigen::Vector3d> plane_pts;
    Eigen::Vector3d normal_vector;
    float traversability = 100;
    float plane_height;
    float plane_angle;
}; 

class PlaneMap
{
public:
    PlaneMap(World* world, const float resolution);
    ~PlaneMap();
    bool InitPlaneMap();
    void clearPlaneMap();
    void PointCloudMapCallback(const sensor_msgs::PointCloud2& PointCloud_Map);

    bool init();
    bool getPlaneMap();
    Plane FitPlane(Eigen::Vector3d& p_surface,World* world,const double &radius);

    void visSurf(const PlaneMap &planemap, ros::Publisher* surf_vis_pub);
    bool pubPlaneGridMap(const PlaneMap &planemap);

    double getAngle(Eigen::Vector3d &plane_vector);

    Plane** Plane_Map_=NULL; 
    // fitplane::PlaneMap PlaneGridMap_; 
    bool has_PlaneMap_=false;
    float resolution_;
    Eigen::Vector3i index_num_;
    Eigen::Vector3d leftdownbound_;
    Eigen::Vector3d rightupbound_;
    World* world_ = NULL;

    // For flat terrain
    // float max_angle_ = 60.0; 
    // float max_flatness_ = 0.7;
    // float w1_ = 0.9;
    
    // For uneven terrain
    float max_angle_ = 40.0; 
    float max_flatness_ = 0.01;
    float w1_ = 0.8;

    ros::Publisher plane_grid_map_pub_;

    int width_;
    int height_;
    nav_msgs::OccupancyGrid plane_Occmap_;
    nav_msgs::OccupancyGrid Original_plane_Occmap_;
    ros::Publisher plane_OccMap_pub_;
    ros::Publisher trav_cloud_pub;

    int occThre_ = 100;
    int flatThre_ = 99;
    int nobs_ = 120;
    
    Eigen::Vector3d index2coord(const Eigen::Vector3i &index)
    {
        Eigen::Vector3d coord = resolution_*index.cast<double>() + leftdownbound_+ 0.5*resolution_*Eigen::Vector3d::Ones();
        return coord;
    }
    Eigen::Vector3i coord2index(const Eigen::Vector3d &coord)
    {
        Eigen::Vector3i index = ( (coord-leftdownbound_)/resolution_).cast<int>();            
        return index;
    }
    bool isInBorder(const int& x, const int& y)
    {
        return x >= 0 && y >= 0 && x < width_ && y < height_;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    ros::Publisher pubExploredArea;
    double exploredAreaVoxelSize = 0.1;
    pcl::VoxelGrid<pcl::PointXYZ> exploredAreaDwzFilter;
};

}

#endif