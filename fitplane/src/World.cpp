/**
 *  This file contains classes and methods to construct the world for the robot.
 *  It contains classes to store points, lines, world width and height, and obstacles.
 *  Author: jianzhuozhu
 *  Date: 2021-7-24
 * 
 *  Modified by Qingchen Bi on 2022/11/05
 */

#include "plane.h"

using namespace std;
using namespace Eigen;

namespace FitPlane
{

World::World(const float &resolution, const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
{
    lowerbound_=INF*Vector3d::Ones(); 
    upperbound_=-INF*Vector3d::Ones();
    idx_count_=Vector3i::Zero(); 
    resolution_ = resolution;
    nh_ = nh;
    nh_private_ = nh_private;

    nh_private_.getParam("PointCloud_Map_topic", PointCloud_Map_topic);
    nh_private_.getParam("Grid_Map_topic", Grid_Map_topic);
    nh_private_.getParam("PointCloud_topic", PointCloud_topic);
    
    nh_private_.getParam("use_ex_range", use_ex_range_);
    nh_private_.getParam("ex_robot_back", ex_robot_back_);
    nh_private_.getParam("ex_robot_front", ex_robot_front_);
    nh_private_.getParam("ex_robot_right", ex_robot_right_);
    nh_private_.getParam("ex_robot_left", ex_robot_left_);
    
    Grid_Map_pub = nh_.advertise<sensor_msgs::PointCloud2>(Grid_Map_topic, 1);
}

World::~World()
{
    clearMap();
}

void World::clearMap()
{
    if(has_map_)
    {
        for(int i=0;i < idx_count_(0);i++)
        {
            for(int j=0;j < idx_count_(1);j++)
            {
                delete[] grid_map_[i][j]; 
                grid_map_[i][j]=NULL; 
            }
            delete[] grid_map_[i];
            grid_map_[i]=NULL;
        }
        delete[] grid_map_;
        grid_map_=NULL;
    }
}

void World::initGridMap(const Vector3d &lowerbound,const Vector3d &upperbound)
{
    lowerbound_=lowerbound;
    upperbound_=upperbound;
    idx_count_=((upperbound_-lowerbound_)/resolution_).cast<int>()+Eigen::Vector3i::Ones(); 
    grid_map_=new bool**[idx_count_(0)];
    for(int i=0;i < idx_count_(0);i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j=0;j < idx_count_(1);j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool)); 
        }
    }
    has_map_=true; 
}

void World::initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud)
{   
    if(cloud.points.empty())
    {
        ROS_ERROR("Can not initialize the map with an empty point cloud!");
        return;
    }
    clearMap();
    cloud_near_.clear();
    GetRobotPosition();
    for(const auto&pt:cloud.points)
    {
        if(use_ex_range_)
        {
            if(pt.x < ex_robot_back_ || pt.x > ex_robot_front_ || pt.y < ex_robot_right_ || pt.y > ex_robot_left_)
                continue;            
        }
        if(abs(pt.x - ego_position_.x) > minrange_ || abs(pt.y - ego_position_.y) > minrange_)
            continue;
        else
        {
            cloud_near_.points.push_back(pt);        
        }
    }
    float minx = 10000;
    float miny = 10000;
    float minz = 10000; 
    float maxx = -10000;
    float maxy = -10000;
    float maxz = -10000;    
    for(const auto&pt:cloud_near_.points)
    {
        if(pt.x < minx)
        {
            minx = pt.x;
            lowerbound_(0)=minx;
        } 
        if(pt.y < miny)
        {
            miny = pt.y;
            lowerbound_(1)=miny;
        } 
        if(pt.z < minz)
        {
            minz = pt.z;
            lowerbound_(2)=minz;
        } 

        if(pt.x > maxx)
        {
            maxx = pt.x;
            upperbound_(0)=maxx;
        } 
        if(pt.y > maxy)
        {
            maxy = pt.y;
            upperbound_(1)=maxy;
        } 
        if(pt.z + 1.0 > maxz)
        {
            maxz = pt.z + 1;
            upperbound_(2)=maxz;
        } 
    }

    idx_count_ = ((upperbound_-lowerbound_)/resolution_).cast<int>() + Eigen::Vector3i::Ones();

    grid_map_=new bool**[idx_count_(0)];
    for(int i = 0 ; i < idx_count_(0) ; i++)
    {
        grid_map_[i]=new bool*[idx_count_(1)];
        for(int j = 0 ; j < idx_count_(1) ; j++)
        {
            grid_map_[i][j]=new bool[idx_count_(2)];
            memset(grid_map_[i][j],true,idx_count_(2)*sizeof(bool));
        }
    }
    has_map_=true;
}

void World::setObs(const Vector3d &point)
{   
    Vector3i idx=coord2index(point);
    grid_map_[idx(0)][idx(1)][idx(2)]=false; 
}

bool World::isFree(const Vector3d &point)
{
    Vector3i idx = coord2index(point);
    bool is_free = isInsideBorder(idx) && grid_map_[idx(0)][idx(1)][idx(2)];
    return is_free;
}

Vector3d World::coordRounding(const Vector3d & coord)
{
    return index2coord(coord2index(coord));
}

bool World::project2surface(const float &x,const float &y,Vector3d* p_surface)
{
    bool ifsuccess=false;

    if(x>=lowerbound_(0) && x<=upperbound_(0) && y>=lowerbound_(1) && y<=upperbound_(1))
    {
        for(float z = lowerbound_(2) ; z < upperbound_(2) ; z+=resolution_)
        {
            if( !isFree(x,y,z) && isFree(x,y,z+resolution_) )
            {
                *p_surface=Vector3d(x,y,z); 
                ifsuccess=true;
                break;
            }
        }
    }
    return ifsuccess;
}

bool World::isInsideBorder(const Vector3i &index)
{
    return index(0) >= 0 &&
           index(1) >= 0 &&
           index(2) >= 0 && 
           index(0) < idx_count_(0)&&
           index(1) < idx_count_(1)&&
           index(2) < idx_count_(2);
}
}

