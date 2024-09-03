//
// Created by hjl on 2021/9/18.
// Modified by Qingchen Bi on 2022/11/05
//
#ifndef TOPO_PLANNER_WS_SLAM_OUTPUT_H
#define TOPO_PLANNER_WS_SLAM_OUTPUT_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/Float32.h>
class SlamOutput {
public:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener tf_listener;
    ros::Publisher odom_pub;
    ros::Publisher reg_pub;
    ros::Publisher dwz_cloud_pub;
    ros::Subscriber odom_sub;
    ros::Subscriber scan_sub;
    ros::Timer global_down_timer;

    tf::StampedTransform transform;
    ros::Rate rate;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyLocalCloudOdom;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> local_cloud_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> local_odom_sub_;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyLocalCloudOdom>> SynchronizerLocalCloudOdom;
    SynchronizerLocalCloudOdom sync_local_cloud_odom_;

    std::string frame_id;
    std::string child_frame_id;

    bool is_get_first;

    tf::Transform T_B_W ;

    std::vector<tf::StampedTransform> ST_B_Bi_vec;
    double vec_length;

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    float down_voxel_size = 0.1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    double exploredAreaVoxelSize = 0.1;
    pcl::VoxelGrid<pcl::PointXYZ> exploredAreaDwzFilter;
    ros::Timer execution_timer_;

    void execute(const ros::TimerEvent&); 

    sensor_msgs::PointCloud2ConstPtr scanIn_;
    tf::StampedTransform ST_B_Bi_;
    
    SlamOutput(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

    void pointCloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud,const nav_msgs::OdometryConstPtr& input);

    float exploredVolume_ = 0;
    ros::Publisher explored_volume_pub;
};



#endif //TOPO_PLANNER_WS_SLAM_OUTPUT_H
