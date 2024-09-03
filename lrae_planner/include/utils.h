
#ifndef __UTILS_H_
#define __UTILS_H_
#include <ros/ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include "Eigen/Eigen"
namespace utils_ns
{
struct ViewPoint
{
    geometry_msgs::Point position;
    int indexX = 0;
    int indexY = 0;
    int fartherCentroidX = 0;
    int fartherCentroidY = 0;
    float score = 0.0;
    int disgridnum = 10000;
    int traver_degree = 100;
    double rv_path_cost = 0.0;
};

struct PointInt
{
  PointInt(int _x, int _y)
  {
    x = _x;
    y = _y;
  }

  int x;
  int y;
};

template <typename T>
T getParam(ros::NodeHandle& nh, const std::string& name, const T default_val)
{
  T val;
  bool success = nh.getParam(name, val);
  if (!success)
  {
    ROS_ERROR_STREAM("Cannot read parameter: " << name);
    return default_val;
  }
  return val;
}

struct Index2
{
  int x = 0;
  int y = 0;
};

double getAngleRobot(geometry_msgs::Point& p, geometry_msgs::Point& robot);
double getAngleVector(geometry_msgs::Point& p1, geometry_msgs::Point& p2, geometry_msgs::Point& robot);
bool Bresenham(int x1, int y1, int x2, int y2, std::vector<Eigen::Vector2i> &visited_grid, nav_msgs::OccupancyGrid& map);
}
#endif