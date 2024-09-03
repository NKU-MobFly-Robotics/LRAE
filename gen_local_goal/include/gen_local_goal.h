/**
 *  Created by Qingchen Bi on 2022/3/21
 */

#ifndef GEN_LOCAL_GOAL_H
#define GEN_LOCAL_GOAL_H

#include <ros/ros.h>
#include <string>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include <utils.h>
#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <ros/publisher.h>
#include <tf/transform_listener.h>

using namespace bezier_local_planner;

class GenLocalGoal
{  
public:
  GenLocalGoal(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
  ~GenLocalGoal();
private:
  void execute(const ros::TimerEvent&);
  void Initialize();
  void MapCallBack(const nav_msgs::OccupancyGrid::Ptr& msg);
  void PathCallBack(const nav_msgs::Path::Ptr& msg);
  void GetGlobalPlan();
  void velCallBack(const geometry_msgs::Twist::Ptr& msg);

  ros::Subscriber map_sub_;
  ros::Subscriber global_path_sub_;
  ros::Subscriber subOdom_;
  ros::NodeHandle nh_;
  ros::NodeHandle nh_p_;
  ros::Publisher pubwaypoint_;
  std::string map_topic_ = "/plane_OccMap";
  std::string path_topic_ = "/exporation_path";

  nav_msgs::OccupancyGrid map_;
  nav_msgs::Path path_;
  nav_msgs::Path tmp_path_;

  tf::TransformListener listener_;
  tf::StampedTransform transform_;

  ros::Timer execution_timer_;

  double tmp_path_length_;
  double path_resolution_;
  geometry_msgs::PointStamped last_waypoint_;
  ros::Publisher pubSpeed_;
  int no_path_count_ = 0;
  
  int has_path_no_vel_count_ = 0;
  ros::Subscriber subSpeed_;
  geometry_msgs::Twist cmd_vel_;
  bool pub_vel_ing_ = false;
  int pub_vel_count_ = 0;

};
#endif 