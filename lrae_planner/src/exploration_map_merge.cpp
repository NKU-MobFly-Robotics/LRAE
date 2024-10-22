/**
 *  Created by Qingchen Bi on 2023/4/11
 */
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include "Eigen/Eigen"
#include <std_msgs/Int8.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

struct mapUpdateData
{
    Eigen::Vector3d robot_coord;
    bool updated = false;
}; 
mapUpdateData** mapUpdate = NULL;
nav_msgs::OccupancyGrid mapData;
nav_msgs::OccupancyGrid globalMapData;
bool hasMap = false;
bool initSucessed = false;

geometry_msgs::Point ego_position_;
geometry_msgs::Point ego_position_last;

float mapUpdateRes;
int mapUpdateWidth;
int mapUpdateHeight;

int map_w = 1800;
int map_h = 1800;
double mapinitox = -90.0;
double mapinitoy = -90.0;
double merge_size = 10.0;
double safe_obs_dis = 20.0;

std_msgs::Int8 isMoveInited;

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    mapData=*msg;
    hasMap = true;
}

void SetBround(nav_msgs::OccupancyGrid& globalMapData)
{
    for(int i = 0; i < globalMapData.info.height; i++)
    {
        globalMapData.data[0 + i * globalMapData.info.width] = 100;  
        globalMapData.data[globalMapData.info.width - 1 + i * globalMapData.info.width] = 100;  
    }
    for(int j = 0; j < globalMapData.info.width; j++)
    {
        globalMapData.data[j + 0 * globalMapData.info.width] = 100;
        globalMapData.data[((globalMapData.info.width - 1) + (globalMapData.info.height - 1)  * globalMapData.info.width) - j] = 100;
    }
}

void initGlobalMap()
{
    globalMapData.header.frame_id = mapData.header.frame_id;
    globalMapData.header.stamp = mapData.header.stamp;
    globalMapData.info.origin.position.x = mapinitox;
    globalMapData.info.origin.position.y = mapinitoy;
    globalMapData.info.origin.position.z = -0.5;

    globalMapData.info.resolution = mapData.info.resolution;
    globalMapData.info.width = map_w; // globalMapData.info.origin.position.x * (-2) / globalMapData.info.resolution;
    globalMapData.info.height = map_h; //globalMapData.info.origin.position.y * (-2) / globalMapData.info.resolution;
    std::vector<int8_t> tmp(globalMapData.info.width * globalMapData.info.height, -1);
    globalMapData.data = tmp;

    mapUpdateRes = mapData.info.resolution * 2;
    mapUpdateWidth = (int)(globalMapData.info.width * mapData.info.resolution / mapUpdateRes);
    mapUpdateHeight = (int)(globalMapData.info.height * mapData.info.resolution / mapUpdateRes);
    mapUpdate = new mapUpdateData*[mapUpdateWidth];
    for(int i = 0; i < mapUpdateWidth; i++)
    {
        mapUpdate[i] = new mapUpdateData[mapUpdateHeight];
    }
    
    SetBround(globalMapData);
    initSucessed = true;
}

void mapMerge()
{
    int rx = mapinitox;
    int ry = mapinitox;
    if(ego_position_last.x != mapinitox)
    {
        rx = CONTXY2DISC(ego_position_last.x - globalMapData.info.origin.position.x, mapUpdateRes); 
        ry = CONTXY2DISC(ego_position_last.y - globalMapData.info.origin.position.y, mapUpdateRes);
    }

    for(int i = 0; i < mapData.info.width; i++)
    {
        for(int j = 0; j < mapData.info.height; j++)
        {
            double px = DISCXY2CONT(i, mapData.info.resolution) + mapData.info.origin.position.x;
            double py = DISCXY2CONT(j, mapData.info.resolution) + mapData.info.origin.position.y;
            if(abs(px - ego_position_.x) > merge_size || abs(py - ego_position_.y) > merge_size)
                continue;
            else
            {
                int ix = CONTXY2DISC(px - globalMapData.info.origin.position.x, globalMapData.info.resolution); 
                int iy = CONTXY2DISC(py - globalMapData.info.origin.position.y, globalMapData.info.resolution);

                int ux = CONTXY2DISC(px - globalMapData.info.origin.position.x, mapUpdateRes); 
                int uy = CONTXY2DISC(py - globalMapData.info.origin.position.y, mapUpdateRes);

                if(rx == ux && ry == uy && ego_position_last.x != mapinitox)
                {
                    if(mapUpdate[rx][ry].updated)
                        continue;
                    else
                    {
                        if(globalMapData.data[ix + iy * globalMapData.info.width] == -1)
                            globalMapData.data[ix + iy * globalMapData.info.width] = mapData.data[i + j * mapData.info.width];
                        else if(globalMapData.data[ix + iy * globalMapData.info.width] < 90 && globalMapData.data[ix + iy * globalMapData.info.width] >=0)
                        {
                            if(mapData.data[i + j * mapData.info.width] != -1)
                                globalMapData.data[ix + iy * globalMapData.info.width] = 
                                    (int)(0.2 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                    0.8 * (double)(mapData.data[i + j * mapData.info.width]));
                        }
                        else
                        {
                            if(abs(px - ego_position_.x) >= safe_obs_dis || abs(py - ego_position_.y) >= safe_obs_dis)
                            {
                                if(mapData.data[i + j * mapData.info.width] < 119 && globalMapData.data[ix + iy * globalMapData.info.width] < 119) 
                                {
                                    if(mapData.data[i + j * mapData.info.width] != -1)
                                        globalMapData.data[ix + iy * globalMapData.info.width] = 
                                            (int)(0.2 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                            0.8 * (double)(mapData.data[i + j * mapData.info.width]));
                                }
                                else
                                    globalMapData.data[ix + iy * globalMapData.info.width] = 100;
                            }
                            else
                            {
                                if(mapData.data[i + j * mapData.info.width] < 119 && globalMapData.data[ix + iy * globalMapData.info.width] < 119) 
                                {
                                    if(mapData.data[i + j * mapData.info.width] != -1)
                                        globalMapData.data[ix + iy * globalMapData.info.width] = 
                                            (int)(0.96 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                            0.04 * (double)(mapData.data[i + j * mapData.info.width]));
                                }
                                else
                                    globalMapData.data[ix + iy * globalMapData.info.width] = 100;
                            }
                        }
                        mapUpdate[rx][ry].updated = true;
                    }
                }
                else
                {
                    if(mapUpdate[ux][uy].updated)
                        continue;
                    else
                    {
                        if(globalMapData.data[ix + iy * globalMapData.info.width] == -1)
                            globalMapData.data[ix + iy * globalMapData.info.width] = mapData.data[i + j * mapData.info.width];
                        else if(globalMapData.data[ix + iy * globalMapData.info.width] < 90 && globalMapData.data[ix + iy * globalMapData.info.width] >=0)
                        {
                            if(mapData.data[i + j * mapData.info.width] != -1)
                                globalMapData.data[ix + iy * globalMapData.info.width] = 
                                    (int)(0.2 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                    0.8 * (double)(mapData.data[i + j * mapData.info.width]));
                        }
                        else
                        {
                            if(abs(px - ego_position_.x) >= safe_obs_dis || abs(py - ego_position_.y) >= safe_obs_dis)
                            {
                                if(mapData.data[i + j * mapData.info.width] < 119 && globalMapData.data[ix + iy * globalMapData.info.width] < 119) 
                                {
                                    if(mapData.data[i + j * mapData.info.width] != -1)
                                        globalMapData.data[ix + iy * globalMapData.info.width] = 
                                            (int)(0.2 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                            0.8 * (double)(mapData.data[i + j * mapData.info.width]));
                                }
                                else
                                    globalMapData.data[ix + iy * globalMapData.info.width] = 100;
                            }
                            else
                            {
                                if(mapData.data[i + j * mapData.info.width] < 119 && globalMapData.data[ix + iy * globalMapData.info.width] < 119) 
                                {
                                    if(mapData.data[i + j * mapData.info.width] != -1)
                                        globalMapData.data[ix + iy * globalMapData.info.width] = 
                                            (int)(0.96 * (double)(globalMapData.data[ix + iy * globalMapData.info.width]) + 
                                            0.04 * (double)(mapData.data[i + j * mapData.info.width]));
                                }
                                else
                                    globalMapData.data[ix + iy * globalMapData.info.width] = 100;
                            }
                        }
                    }
                }
            }                                         
        }
    }
} 

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration_map_merge");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");
    tf::TransformListener listener_;
    tf::StampedTransform transform_;
    node_handle.getParam("/exploration_map_merge/map_w", map_w);
    node_handle.getParam("/exploration_map_merge/map_h", map_h);
    node_handle.getParam("/exploration_map_merge/mapinitox", mapinitox);
    node_handle.getParam("/exploration_map_merge/mapinitoy", mapinitoy);
    node_handle.getParam("/exploration_map_merge/merge_size", merge_size);
    node_handle.getParam("/exploration_map_merge/safe_obs_dis", safe_obs_dis);
    ros::Subscriber map_sub = node_handle.subscribe("/plane_OccMap", 1000, mapCallBack);
    ros::Publisher globalMap_pub = node_handle.advertise<nav_msgs::OccupancyGrid>("/globalMap", 10);
    ros::Publisher move_inited_pub = node_handle.advertise<std_msgs::Int8>("/MoveInited", 1);
    ros::Publisher robot_position_pub = node_handle.advertise<geometry_msgs::Point>("/RobotPosition", 1);
    // ros::Publisher trav_cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("local_traversibility_ponit_cloud", 1);

    ego_position_last.x = mapinitox;
    ego_position_last.y = mapinitox;
    isMoveInited.data = 0;
    ros::Rate rate(2);
    while(ros::ok())
    {
        if(hasMap && !initSucessed)
        {
            initGlobalMap();
        }
        geometry_msgs::Pose current_pose_ros;
        geometry_msgs::TransformStamped transform_pose;
        tf::Quaternion quat; 
        double roll,pitch,yaw;
        try
        {
            listener_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
            listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform_);
            tf::transformStampedTFToMsg(transform_, transform_pose);
            ego_position_.x = transform_.getOrigin().x();
            ego_position_.y = transform_.getOrigin().y();
            tf::quaternionMsgToTF(transform_pose.transform.rotation,quat); 
            tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
            ego_position_.z = transform_.getOrigin().z();
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        if(abs(ego_position_last.x - ego_position_.x) >= 0.1 || abs(ego_position_last.y - ego_position_.y) >= 0.1 || abs(ego_position_last.z - ego_position_.z) >= 0.1)
        {
            if(initSucessed)
            {
                mapMerge();
                isMoveInited.data = 1;
                move_inited_pub.publish(isMoveInited);
            }
            ego_position_last = ego_position_;
        }

        if(initSucessed)
        {
            globalMap_pub.publish(globalMapData);          
        }
        robot_position_pub.publish(ego_position_);
        ros::spinOnce();  
        rate.sleep();      
    }
    return 0;
}
