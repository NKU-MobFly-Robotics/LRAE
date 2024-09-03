#include <algorithm>
#include "graph_search.h"
#include "utils.h"

using namespace utils_ns;
int fd_x8[8] = { -1, 0, 0, 1, 1, -1, 1, -1};
int fd_y8[8] = { 0, 1, -1, 0, 1, -1, -1, 1};
int d_x24[24] = {-2,-2,-2,-2,-2,-1,-1,0,0,1,1,2,2,2,2,2, -1, 0, 0, 1, 1, -1, 1, -1};
int d_y24[24] = {-2,-1,0,1,2,-2,2,-2,2,2,-2,-2,-1,0,2,1, 0, 1, -1, 0, 1, -1, -1, 1};

GraphSearch::GraphSearch() : openList2D_(NULL), searchStates2D_(NULL), iteration_(0), forCostiteration_(0), forCostopenList2D_(NULL), forCostsearchStates2D_(NULL)
{
  computedxy();
}

GraphSearch::~GraphSearch()
{
  if (openList2D_ != NULL)
  {
    openList2D_->makeemptyheap();
    delete openList2D_;
    openList2D_ = NULL;
  }

  if (searchStates2D_ != NULL)
  {
    for (int x = 0; x < width_; x++)
    {
      delete[] searchStates2D_[x];
    }
    delete[] searchStates2D_;
    searchStates2D_ = NULL;
  }

  if (forCostopenList2D_ != NULL)
  {
    forCostopenList2D_->makeemptyheap();
    delete forCostopenList2D_;
    forCostopenList2D_ = NULL;
  }

  if (forCostsearchStates2D_ != NULL)
  {
    for (int x = 0; x < width_; x++)
    {
      delete[] forCostsearchStates2D_[x];
    }
    delete[] forCostsearchStates2D_;
    forCostsearchStates2D_ = NULL;
  }
}

void GraphSearch::computedxy()
{
  dx_[0] = 1;
  dy_[0] = 1;
  dx0intersects_[0] = -1;
  dy0intersects_[0] = -1;
  dx_[1] = 1;
  dy_[1] = 0;
  dx0intersects_[1] = -1;
  dy0intersects_[1] = -1;
  dx_[2] = 1;
  dy_[2] = -1;
  dx0intersects_[2] = -1;
  dy0intersects_[2] = -1;
  dx_[3] = 0;
  dy_[3] = 1;
  dx0intersects_[3] = -1;
  dy0intersects_[3] = -1;
  dx_[4] = 0;
  dy_[4] = -1;
  dx0intersects_[4] = -1;
  dy0intersects_[4] = -1;
  dx_[5] = -1;
  dy_[5] = 1;
  dx0intersects_[5] = -1;
  dy0intersects_[5] = -1;
  dx_[6] = -1;
  dy_[6] = 0;
  dx0intersects_[6] = -1;
  dy0intersects_[6] = -1;
  dx_[7] = -1;
  dy_[7] = -1;
  dx0intersects_[7] = -1;
  dy0intersects_[7] = -1;

  for (int dind = 0; dind < 8; dind++)
  {
    if (dx_[dind] != 0 && dy_[dind] != 0)
      dxy_cost_[dind] = 14;
    else
      dxy_cost_[dind] = 10;
  }
}

bool GraphSearch::createSearchStates2D()
{
  int x, y;

  if (searchStates2D_ != NULL)
  {
    printf("ERROR: We already have a non-NULL search states array\n");
    return false;
  }

  searchStates2D_ = new Grid2DSearchState*[width_]; 
  for (x = 0; x < width_; x++)
  {
    searchStates2D_[x] = new Grid2DSearchState[height_];
    for (y = 0; y < height_; y++)
    {
      searchStates2D_[x][y].iterationaccessed = iteration_;
      searchStates2D_[x][y].x = x;
      searchStates2D_[x][y].y = y;
      initializeSearchState2D(&searchStates2D_[x][y]);
    }
  }
  return true;
}

bool GraphSearch::forCostcreateSearchStates2D()
{
  int x, y;

  if (forCostsearchStates2D_ != NULL)
  {
    printf("ERROR: We already have a non-NULL search states array\n");
    return false;
  }

  forCostsearchStates2D_ = new Grid2DSearchState*[width_]; 
  for (x = 0; x < width_; x++)
  {
    forCostsearchStates2D_[x] = new Grid2DSearchState[height_];
    for (y = 0; y < height_; y++)
    {
      forCostsearchStates2D_[x][y].iterationaccessed = forCostiteration_;
      forCostsearchStates2D_[x][y].x = x;
      forCostsearchStates2D_[x][y].y = y;
      forCostinitializeSearchState2D(&forCostsearchStates2D_[x][y]);
    }
  }
  return true;
}

void GraphSearch::initializeSearchState2D(Grid2DSearchState* state2D)
{
  state2D->g = INFINITECOST;
  state2D->heapindex = 0;
  state2D->iterationaccessed = iteration_;
  state2D->predecessor = NULL;
}
void GraphSearch::forCostinitializeSearchState2D(Grid2DSearchState* state2D)
{
  state2D->g = INFINITECOST;
  state2D->heapindex = 0;
  state2D->iterationaccessed = forCostiteration_;
  state2D->predecessor = NULL;
}

bool GraphSearch::Search(int startX, int startY, int goalX, int goalY, unsigned char** occMap, int width, int height,
                         unsigned char obsthresh, std::vector<PointInt>& path, std::vector<PointInt>& expands, int& pathCost)
{
  width_ = width;
  height_ = height;
  if (openList2D_ == NULL) 
  {
    openList2D_ = new CIntHeap(width * height);
  }
  if (searchStates2D_ == NULL) 
  {
    if (!createSearchStates2D())
    {
      throw SBPL_Exception("ERROR: failed to create searchstatespace2D");
    }
  }

  Grid2DSearchState* searchExpState = NULL; 
  Grid2DSearchState* searchPredState = NULL;
  int numofExpands = 0;
  int key;
  iteration_++;
  openList2D_->makeemptyheap();
  if (!withinMap(startX, startY) || !withinMap(goalX, goalY))
  {
    printf("ERROR: grid2Dsearch is called on invalid start (%d %d) or goal(%d %d)\n", startX, startY, goalX, goalY);
    return false;
  }
  searchExpState = &searchStates2D_[startX][startY]; 
  initializeSearchState2D(searchExpState); 
  Grid2DSearchState* search2DGoalState = &searchStates2D_[goalX][goalY];
  initializeSearchState2D(search2DGoalState);
  searchExpState->g = 0;
  key = searchExpState->g + heuristic(startX, startY, goalX, goalY) +  0 * occMap[startX][startY]; 

  openList2D_->insertheap(searchExpState, key);
  char* pbClosed = (char*)calloc(1, width_ * height_);
  expands.clear();
  while (!openList2D_->emptyheap() && std::min(INFINITECOST, search2DGoalState->g) > openList2D_->getminkeyheap())
  {
    searchExpState = dynamic_cast<Grid2DSearchState*>(openList2D_->deleteminheap()); 
    numofExpands++;
    int exp_x = searchExpState->x;
    int exp_y = searchExpState->y;
    expands.push_back(PointInt(exp_x, exp_y));
    pbClosed[exp_x + width_ * exp_y] = 1;
    unsigned char expcost = occMap[exp_x][exp_y];
    for (int dir = 0; dir < 8; dir++)
    {
      int newx = exp_x + dx_[dir];
      int newy = exp_y + dy_[dir];
      if (!withinMap(newx, newy)) 
        continue;

      if (pbClosed[newx + width_ * newy] == 1) 
        continue;
      int mapcost = std::max(occMap[newx][newy], expcost); 
      if (mapcost >= obsthresh) 
        continue;
      if(abs(dx_[dir]) + abs(dy_[dir]) == 2)
      {
        int newx1 = exp_x + dx_[dir];
        int newy1 = exp_y + 0;
        int newx2 = exp_x + 0;
        int newy2 = exp_y + dy_[dir];  
        if(!withinMap(newx1, newy1))
          continue;
        if(!withinMap(newx2, newy2))
          continue; 
        if(occMap[newx1][newy1] >= obsthresh && occMap[newx2][newy2] >= obsthresh)
          continue;
      }

      bool obs_flag = false;
      for(int m = 0; m < 24; m++)
      {
        int newnewx = newx + d_x24[m];
        int newnewy = newy + d_y24[m];
        if (!withinMap(newnewx, newnewy)) 
          continue;
        if(occMap[newnewx][newnewy] >= flatThre_)
        {
          obs_flag = true;
          break;
        }
      }
      if(obs_flag)
        continue;

      int cost = dxy_cost_[dir]; 
      searchPredState = &searchStates2D_[newx][newy];
      if (searchPredState->iterationaccessed != iteration_ || searchPredState->g > cost + searchExpState->g)
      {
        searchPredState->iterationaccessed = iteration_;
        searchPredState->g = std::min(INFINITECOST, cost + searchExpState->g);
        key = searchPredState->g + heuristic(searchPredState->x, searchPredState->y, goalX, goalY) + 0 * occMap[newx][newy];
        if (searchPredState->heapindex == 0)
          openList2D_->insertheap(searchPredState, key);
        else
          openList2D_->updateheap(searchPredState, key);

        searchPredState->predecessor = searchExpState;
      }
      else{
      }
    }  
  }   

  free(pbClosed);  
  // printf("Path cost=%d\n", search2DGoalState->g);
  pathCost = search2DGoalState->g;
  path.clear();
  searchPredState = search2DGoalState;
  path.push_back(PointInt(searchPredState->x, searchPredState->y)); 
  while (searchPredState->predecessor != NULL) 
  {
    searchPredState = searchPredState->predecessor;
    path.push_back(PointInt(searchPredState->x, searchPredState->y));
  }
  std::reverse(path.begin(), path.end()); 
  return true;
}

bool GraphSearch::SearchforCost(int startX, int startY, int goalX, int goalY, nav_msgs::OccupancyGrid occMap, int width, int height,
                         unsigned char obsthresh, int& pathCost, std::vector<PointInt>& path)
{
  width_ = width;
  height_ = height;
  if (forCostopenList2D_ == NULL) 
  {
    forCostopenList2D_ = new CIntHeap(width * height);
  }
  if (forCostsearchStates2D_ == NULL) 
  {
    if (!forCostcreateSearchStates2D())
    {
      throw SBPL_Exception("ERROR: failed to create searchstatespace2D");
    }
  }

  Grid2DSearchState* searchExpState = NULL; 
  Grid2DSearchState* searchPredState = NULL; 
  int numofExpands = 0;
  int key;
  forCostiteration_++;
  forCostopenList2D_->makeemptyheap();

  if (!withinMap(startX, startY) || !withinMap(goalX, goalY))
  {
    printf("ERROR: grid2Dsearch is called on invalid start (%d %d) or goal(%d %d)\n", startX, startY, goalX, goalY);
    return false;
  }

  searchExpState = &forCostsearchStates2D_[startX][startY]; 
  forCostinitializeSearchState2D(searchExpState); 
  Grid2DSearchState* search2DGoalState = &forCostsearchStates2D_[goalX][goalY];
  forCostinitializeSearchState2D(search2DGoalState);

  searchExpState->g = 0;
  key = searchExpState->g + heuristic(startX, startY, goalX, goalY); 
  forCostopenList2D_->insertheap(searchExpState, key);
  char* pbClosed = (char*)calloc(1, width_ * height_);

  while (!forCostopenList2D_->emptyheap() && std::min(INFINITECOST, search2DGoalState->g) > forCostopenList2D_->getminkeyheap())
  {
    searchExpState = dynamic_cast<Grid2DSearchState*>(forCostopenList2D_->deleteminheap()); 
    numofExpands++;
    int exp_x = searchExpState->x;
    int exp_y = searchExpState->y;
    pbClosed[exp_x + width_ * exp_y] = 1;
    int expcost = occMap.data[exp_x + exp_y * width_];
    for (int dir = 0; dir < 8; dir++)
    {
      int newx = exp_x + dx_[dir];
      int newy = exp_y + dy_[dir];
      if (!withinMap(newx, newy)) 
        continue;
      if (pbClosed[newx + width_ * newy] == 1) 
        continue;
      int mapcost;
      if(expcost > occMap.data[newx + newy * width_])
        mapcost = expcost;
      else
        mapcost = occMap.data[newx + newy * width_];
      if (mapcost >= obsthresh)  
        continue;
      int cost = dxy_cost_[dir]; 
      searchPredState = &forCostsearchStates2D_[newx][newy];
      if (searchPredState->iterationaccessed != forCostiteration_ || searchPredState->g > cost + searchExpState->g)
      {
        searchPredState->iterationaccessed = forCostiteration_;
        searchPredState->g = std::min(INFINITECOST, cost + searchExpState->g);
        key = searchPredState->g + heuristic(searchPredState->x, searchPredState->y, goalX, goalY);
        if (searchPredState->heapindex == 0)
          forCostopenList2D_->insertheap(searchPredState, key);
        else
          forCostopenList2D_->updateheap(searchPredState, key);

        searchPredState->predecessor = searchExpState;
      }
      else{
      }
    }  
  }  

  free(pbClosed); 
  pathCost = search2DGoalState->g;
  return true;
}

bool GraphSearch::SearchforNearCen(int startX, int startY, int goalX, int goalY, nav_msgs::OccupancyGrid occMap, int width, int height,
                         unsigned char obsthresh, int& pathCost, std::vector<PointInt>& path)
{
  width_ = width;
  height_ = height;
  if (forCostopenList2D_ == NULL) 
  {
    forCostopenList2D_ = new CIntHeap(width * height);
  }
  if (forCostsearchStates2D_ == NULL) 
  {
    if (!forCostcreateSearchStates2D())
    {
      throw SBPL_Exception("ERROR: failed to create searchstatespace2D");
    }
  }

  Grid2DSearchState* searchExpState = NULL;
  Grid2DSearchState* searchPredState = NULL; 
  int numofExpands = 0;
  int key;
  forCostiteration_++;
  forCostopenList2D_->makeemptyheap();

  if (!withinMap(startX, startY) || !withinMap(goalX, goalY))
  {
    printf("ERROR: grid2Dsearch is called on invalid start (%d %d) or goal(%d %d)\n", startX, startY, goalX, goalY);
    return false;
  }

  searchExpState = &forCostsearchStates2D_[startX][startY]; 
  forCostinitializeSearchState2D(searchExpState); 
  Grid2DSearchState* search2DGoalState = &forCostsearchStates2D_[goalX][goalY];
  forCostinitializeSearchState2D(search2DGoalState);

  searchExpState->g = 0;
  key = searchExpState->g + heuristic(startX, startY, goalX, goalY); 
  forCostopenList2D_->insertheap(searchExpState, key);
  char* pbClosed = (char*)calloc(1, width_ * height_);

  while (!forCostopenList2D_->emptyheap() && std::min(INFINITECOST, search2DGoalState->g) > forCostopenList2D_->getminkeyheap())
  {
    searchExpState = dynamic_cast<Grid2DSearchState*>(forCostopenList2D_->deleteminheap());
    numofExpands++;
    int exp_x = searchExpState->x;
    int exp_y = searchExpState->y;
    pbClosed[exp_x + width_ * exp_y] = 1;
    int expcost = occMap.data[exp_x + exp_y * width_];
    for (int dir = 0; dir < 8; dir++)
    {
      int newx = exp_x + dx_[dir];
      int newy = exp_y + dy_[dir];
      if (!withinMap(newx, newy)) 
        continue;
      if (pbClosed[newx + width_ * newy] == 1) 
        continue;
      int mapcost;
      if(expcost > occMap.data[newx + newy * width_])
        mapcost = expcost;
      else
        mapcost = occMap.data[newx + newy * width_];
      if (mapcost >= obsthresh)  
        continue;
      int cost = dxy_cost_[dir]; 
      searchPredState = &forCostsearchStates2D_[newx][newy];
      if (searchPredState->iterationaccessed != forCostiteration_ || searchPredState->g > cost + searchExpState->g)
      {
        searchPredState->iterationaccessed = forCostiteration_;
        searchPredState->g = std::min(INFINITECOST, cost + searchExpState->g);
        key = searchPredState->g + heuristic(searchPredState->x, searchPredState->y, goalX, goalY);
        if (searchPredState->heapindex == 0)
          forCostopenList2D_->insertheap(searchPredState, key);
        else
          forCostopenList2D_->updateheap(searchPredState, key);

        searchPredState->predecessor = searchExpState;
      }
      else{
      }
    }  
  }   

  free(pbClosed); 
  pathCost = search2DGoalState->g;

  path.clear();
  searchPredState = search2DGoalState;
  path.push_back(PointInt(searchPredState->x, searchPredState->y)); 
  while (searchPredState->predecessor != NULL) 
  {
    searchPredState = searchPredState->predecessor;
    path.push_back(PointInt(searchPredState->x, searchPredState->y));
  }
  std::reverse(path.begin(), path.end()); 

  return true;
}