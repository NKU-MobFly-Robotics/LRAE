/**
 *  Created by Qingchen Bi on 2023/10/25
 */
#ifndef _PATH_PLANNING_H_
#define _PATH_PLANNING_H_

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "planner_interface.h"
#include "graph_search.h"
#include <nav_msgs/Path.h>

namespace lrae_planner_ns
{
class PathPlanning
{
public:
    PathPlanning();
    ~PathPlanning();
    void setViewPointSet(std::vector<utils_ns::ViewPoint> viewpointset1, std::vector<utils_ns::ViewPoint> viewpointset2, bool viewpoint1has, bool viewpoint2has);
    void setCentroidPairIndex(utils_ns::Index2 centroidone, utils_ns::Index2 centroidtwo);
    std::vector<utils_ns::ViewPoint> getViewPointSet1();
    std::vector<utils_ns::ViewPoint> getViewPointSet2();
    void setPlanMap(const nav_msgs::OccupancyGrid& map);
    bool getExplorationPath(nav_msgs::Path& exporation_path);
    void setRobotPosition(geometry_msgs::Point robot_position);
    bool finePath(nav_msgs::Path& OriPath);
    nav_msgs::Path getPositionPath(const std::vector<utils_ns::PointInt>& path);

private:
    void clearMap();
    void clearViewPointSet();
    void setStart(int startx, int starty);
    void setGoal(int goalx, int goaly);
    bool makePlan(nav_msgs::Path& guiPath, int& pathCost);

    std::vector<utils_ns::ViewPoint> viewpointset1_;
    std::vector<utils_ns::ViewPoint> viewpointset2_;
    utils_ns::Index2 centroidone_;
    utils_ns::Index2 centroidtwo_;
    nav_msgs::OccupancyGrid recmap_;
    unsigned char** planmap_ = NULL;
    int width_, height_;
    double origin_x_, origin_y_;
    float resolution_;
    int start_x_, start_y_, goal_x_, goal_y_, robot_x_, robot_y_;
    bool has_start_ = false;
    bool has_goal_ = false;
    PlannerInterface* planner_;
    geometry_msgs::Point robot_position_;

    int occThre_ = 95;

    bool viewpoint1has_ = false;
    bool viewpoint2has_ = false;
    nav_msgs::Path last_RVpath_;
    nav_msgs::Path last_VVpath_;
    utils_ns::Index2 last_goal1_;
    utils_ns::Index2 last_goal2_;
    bool first_planning_ = true;
};
}
#endif
