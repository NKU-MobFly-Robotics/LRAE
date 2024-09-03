/**
 *  Created by Qingchen Bi on 2022/3/21
 */

#include "gen_local_goal.h"
#include "geometry_msgs/PointStamped.h"

GenLocalGoal::GenLocalGoal(ros::NodeHandle& nh, ros::NodeHandle& nh_p)
{
  nh_ = nh;
  nh_p_ = nh_p;
  tmp_path_length_ = 3;
  path_resolution_ = 0.3;
  Initialize();
}

void GenLocalGoal::Initialize()
{
  execution_timer_ = nh_.createTimer(ros::Duration(0.1), &GenLocalGoal::execute, this);
  map_sub_ = nh_.subscribe(map_topic_, 10, &GenLocalGoal::MapCallBack, this);
  global_path_sub_ = nh_.subscribe(path_topic_, 100, &GenLocalGoal::PathCallBack, this);
  pubwaypoint_ = nh_.advertise<geometry_msgs::PointStamped>("look_ahead_goal", 1);
  pubSpeed_ = nh_.advertise<geometry_msgs::Twist> ("/cmd_vel", 5);
  subSpeed_ = nh_.subscribe("/cmd_vel", 100, &GenLocalGoal::velCallBack, this);
  last_waypoint_.point.x = -1;
}

void GenLocalGoal::MapCallBack(const nav_msgs::OccupancyGrid::Ptr& msg)
{
  map_ = *msg;
}

void GenLocalGoal::PathCallBack(const nav_msgs::Path::Ptr& msg)
{
  if(msg->poses.size() > 2)
    path_ = *msg;
}

GenLocalGoal::~GenLocalGoal()
{
}

void GenLocalGoal::GetGlobalPlan()
{
  if(map_.info.resolution != 0.0)
    path_resolution_ = map_.info.resolution;
  int local_path_point_num = (int)(tmp_path_length_ / path_resolution_);

  tmp_path_.poses.clear();
  tmp_path_.header.frame_id = "map";
  int ps = path_.poses.size();
  int head_num = std::min(local_path_point_num, ps);
  {
    if(head_num > 2)
    {
      for (int i = 0; i < head_num; i++)
      {
        double x = path_.poses[i].pose.position.x;
        double y = path_.poses[i].pose.position.y;
        geometry_msgs::PoseStamped tmp_p;
        tmp_p.pose.position.x = x;
        tmp_p.pose.position.y = y;
        tmp_p.header.frame_id = "map";
        tmp_path_.poses.push_back(tmp_p);
      }       
    }
 
  }
}

void GenLocalGoal::velCallBack(const geometry_msgs::Twist::Ptr& msg)
{
  cmd_vel_ = *msg;
}

void GenLocalGoal::execute(const ros::TimerEvent&)
{
  GetGlobalPlan();

  geometry_msgs::PointStamped waypoint;

  if(!tmp_path_.poses.empty())
  {
    waypoint.point.x = tmp_path_.poses.back().pose.position.x;
    waypoint.point.y = tmp_path_.poses.back().pose.position.y;
    waypoint.header.frame_id = "map";
    pubwaypoint_.publish(waypoint);
    last_waypoint_ = waypoint;
    no_path_count_ = 0;

    if(fabs(cmd_vel_.linear.x) <= 1e-4 && fabs(cmd_vel_.angular.z) <= 1e-4)
    {
      has_path_no_vel_count_++;
    }
    else if(!pub_vel_ing_ && (fabs(cmd_vel_.linear.x) > 1e-4 || fabs(cmd_vel_.angular.z) > 1e-4))
    {
      has_path_no_vel_count_ = 0;
    }
    else if(pub_vel_ing_ && (fabs(cmd_vel_.linear.x) > 1e-4 || fabs(cmd_vel_.angular.z) > 1e-4))
    {
      pub_vel_count_++;
      if(pub_vel_count_ > 30)
      {
        pub_vel_ing_ = false;
        pub_vel_count_ = 0;
      }
    }
    if(has_path_no_vel_count_ > 30)
    {
      pub_vel_ing_ = true;
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0.5;
      cmd_vel.angular.z = 0.0; 
      pubSpeed_.publish(cmd_vel);
    }

  }
  else
  {
    if(last_waypoint_.point.x != -1)
    {
      pubwaypoint_.publish(last_waypoint_);
      no_path_count_++;
    }
    if(no_path_count_ > 30)
    {
      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0.5;
      cmd_vel.angular.z = 0.0; 
      pubSpeed_.publish(cmd_vel);
    }
  }
}
