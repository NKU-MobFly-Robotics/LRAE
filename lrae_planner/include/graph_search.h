#ifndef _GRAPH_SEARCH_H_
#define _GRAPH_SEARCH_H_

#include "planner_interface.h"
#include "heap.h"
#include <cmath>

class Grid2DSearchState : public AbstractSearchState
{
public:
  int x, y;
  int g;
  int iterationaccessed;

  Grid2DSearchState* predecessor; 

public:
  Grid2DSearchState()
  {
    iterationaccessed = 0; 
  }
  ~Grid2DSearchState()
  {
  }
};

class GraphSearch : public PlannerInterface
{
public:
  GraphSearch();
  virtual ~GraphSearch();

  bool Search(int startX, int startY, int goalX, int goalY, unsigned char** occMap, int width, int height,
              unsigned char obsthresh, std::vector<utils_ns::PointInt>& path, std::vector<utils_ns::PointInt>& expands, int& pathCost);
  bool SearchforCost(int startX, int startY, int goalX, int goalY, nav_msgs::OccupancyGrid occMap, int width, int height,
                     unsigned char obsthresh, int& pathCost, std::vector<utils_ns::PointInt>& path);
  bool SearchforNearCen(int startX, int startY, int goalX, int goalY, nav_msgs::OccupancyGrid occMap, int width, int height,
                     unsigned char obsthresh, int& pathCost, std::vector<utils_ns::PointInt>& path);

private:
  int heuristic(int x, int y, int goalX, int goalY) const
  {
    return 10 * std::max(abs(x - goalX), abs(y - goalY));
  }

  bool withinMap(int x, int y)
  {
    return (x >= 0 && y >= 0 && x < width_ && y < height_);
  }

  void computedxy();
  bool createSearchStates2D();
  void initializeSearchState2D(Grid2DSearchState* state2D);

private:
  CIntHeap* openList2D_;
  Grid2DSearchState** searchStates2D_;
  int dx_[8];
  int dy_[8];
  int dx0intersects_[8];
  int dx1intersects_[8];
  int dy0intersects_[8];
  int dy1intersects_[8];
  int dxy_cost_[8];
  int iteration_;
  int width_, height_;
  // int occThre_ = 100;
  int flatThre_ = 95;

  CIntHeap* forCostopenList2D_;
  Grid2DSearchState** forCostsearchStates2D_;
  int forCostiteration_;
  bool forCostcreateSearchStates2D();
  void forCostinitializeSearchState2D(Grid2DSearchState* state2D);

};

#endif  // _GRAPH_SEARCH_H_