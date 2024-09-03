#ifndef _PLANNER_INTERFACE_H_
#define _PLANNER_INTERFACE_H_

#include <cstdlib>
#include <vector>
#include "utils.h"

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)
#define DISTANCE2(X1, Y1, X2, Y2) pow((X1 - X2), 2) + pow((Y1 - Y2), 2)

class PlannerInterface
{
public:
  PlannerInterface()
  {
  }

  virtual ~PlannerInterface()
  {
  }

  virtual bool Search(int startX, int startY, int goalX, int goalY, unsigned char** occMap, int width, int height,
                      unsigned char obsthresh, std::vector<utils_ns::PointInt>& path, std::vector<utils_ns::PointInt>& expands, int& pathCost) = 0;
  virtual bool SearchforCost(int startX, int startY, int goalX, int goalY, nav_msgs::OccupancyGrid occMap, int width, int height,
                     unsigned char obsthresh, int& pathCost, std::vector<utils_ns::PointInt>& path) = 0;
  virtual bool SearchforNearCen(int startX, int startY, int goalX, int goalY, nav_msgs::OccupancyGrid occMap, int width, int height,
                     unsigned char obsthresh, int& pathCost, std::vector<utils_ns::PointInt>& path) = 0;
};

#endif  // _PLANNER_INTERFACE_H_