/**
 *  Created by Qingchen Bi on 2023/10/24
 */
#ifndef _EXPLORATION_PLANNING_H_
#define _EXPLORATION_PLANNING_H_

#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"
#include <tf/transform_listener.h>
#include "visiual.h"
#include "utils.h"
#include "path_planning.h"
#include "two_opt.h"

#define CONTXY2DISC(X, CELLSIZE) (((X) >= 0) ? ((int)((X) / (CELLSIZE))) : ((int)((X) / (CELLSIZE)) - 1))
#define DISCXY2CONT(X, CELLSIZE) ((X) * (CELLSIZE) + (CELLSIZE) / 2.0)

using namespace std;

namespace lrae_planner_ns
{
struct LastState
{
    bool findCenis = false;
    bool findNearCenPathis = false;
};

struct ExactWindow
{
    ExactWindow() = default;
    ExactWindow(const int& x, const int& y, const int& w, const int& h)
    {
        oriindx = x;
        oriindy = y;
        winwithd = w;
        winheight = h;
    };
    int winwithd;
    int winheight;
    int oriindx;
    int oriindy;
    typedef std::shared_ptr<ExactWindow> Ptr;
};

struct CellPoint
{
    geometry_msgs::Point position;
    int indexX;
    int indexY;
    int cellstate; // -1 unknown 0 free 1 occupancied
};

struct Centroid
{
    geometry_msgs::Point centroidposition;
    utils_ns::Index2 cenindex;
    int unknownnum = 0;
    int windowsoder = 0;
    bool isUnknown;
    std::vector<utils_ns::ViewPoint> viewpoint_fir;
    std::vector<utils_ns::ViewPoint> viewpoint_sec;
    std::vector<utils_ns::ViewPoint> viewpoint_fin;
    std::multimap<float, utils_ns::ViewPoint*> evaluated_viewpoints;
};

struct CandiCentroidPair
{
    Centroid centroidone;
    Centroid centroidtwo;
    int oneindexincentroids = 0;
    int twoindexincentroids = 0;
    int distance = 0;
    int informationnum = 0;
};

class ExplorationPlanning
{
public:
    ExplorationPlanning(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
    bool initialize();
    ~ExplorationPlanning();
    ros::NodeHandle nh_;
    geometry_msgs::Point ego_position_;
private:
    void getRobotPosition();
    void execute(const ros::TimerEvent&); 
    void traversibilityMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void globalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    bool getCentroid(const ExactWindow& ew);
    bool getCentroidHelper(const int ori_x, const int& ori_y, const int& w, const int& h, const int& winoder);
    bool assessUnknownRegion(Centroid* choosecentroid);
    void purgeViewpoint(std::vector<utils_ns::ViewPoint> &purgedViewpoints, Centroid &tmpCentroid);
    std::vector<int> getRouteOrder();

    ros::NodeHandle nh_p_;
    ros::Subscriber traversibility_map_sub_;
    ros::Subscriber global_map_sub_;
    ros::Subscriber robots_position_sub_;
    ros::Publisher exploration_path_pub_;
    ros::Publisher exploration_route_pub_;

    string traversibility_map_topic_ = "plane_OccMap";
    string global_map_topic_ = "globalMap";
    nav_msgs::OccupancyGrid traversibility_map_; 
    nav_msgs::OccupancyGrid global_map_; 
    ros::Timer execution_timer_;

    string ns_;

    tf::TransformListener listener_;
    tf::StampedTransform transform_;

    double resolution_;
    double ori_x_;
    double ori_y_;
    int width_;
    int height_;

    ExactWindow ExactWindow_;
    utils_ns::Index2 min_left_index_;
    utils_ns::Index2 max_right_index_;
    std::vector<CellPoint> in_win_Unpoints_;
    std::vector<Centroid> centroids_;
    std::vector<Centroid> last_centroids_;
    std::vector<Centroid> all_outwin_centroids_;
 
    visual::Marker::Ptr robot_position_marker_;
    visual::Marker::Ptr centroids_marker_;
    visual::Marker::Ptr windows_marker_;
    visual::Marker::Ptr viewpoints_marker_;
    visual::Marker::Ptr choose_centroids_marker_;
    visual::Marker::Ptr choose_viewpoints_marker_;
    
    CandiCentroidPair first_two_cen_;

    PathPlanning path_planner_;
	
    bool has_robot_position_;   
    bool has_map_;    
    bool has_tra_map_; 
    int update_cen_count_ = 5; 
    bool use_viewpoint_plan_ = false;

    // TODO parameters
    double angle_pen_ = 0.45; 
    int update_cen_thre_ = 6; // 3 s = 6 * 0.5        
    int unknown_num_thre_ = 80; 
    double minrange_ = 20.0; 
    bool limit_max_square_ = true; 
    bool use_go_end_nearest_ = true;
    double end_neacen_disthre_ = 10.0; 
    double end_cur_disrate_ = 2.0; 

    PlannerInterface* cal_cost_planner_;

    LastState last_ex_state_;
    Centroid last_near_cen_;
    int pushcen_ = 0;
    std::vector<Centroid> no_path_cens_;
    bool findCenIs_ = true;
    bool go_home_ = false;
    int go_home_count_ = 0;

    // function 
    bool isInBorder(const int& x, const int& y)
    {
        return x >= 0 && y >= 0 && x < width_ && y < height_;
    }
    geometry_msgs::Point index2coord(const int& ix, const int & iy)
    {
        geometry_msgs::Point coord;
        coord.x = resolution_ * double(ix) + ori_x_ + 0.5 * resolution_;
        coord.y = resolution_ * double(iy) + ori_y_ + 0.5 * resolution_;
        return coord;
    }
    utils_ns::Index2 coord2index(const geometry_msgs::Point& coord)
    {
        utils_ns::Index2 index;
        index.x = int((coord.x - ori_x_) / resolution_);
        index.y = int((coord.y - ori_y_) / resolution_);
        return index;
    }
};
}
#endif  