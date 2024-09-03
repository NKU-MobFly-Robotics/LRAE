/**
 *  Created by Qingchen Bi on 2023/10/25
 */
#include "path_planning.h"

int zd_x16[16] = {-1, 0, 1, 1, 1, 0, -1, -1, 0, 0, -2, 2, 2, 2, -2, -2};
int zd_y16[16] = {1, 1, 1, 0, -1, -1, -1, 0, -2, 2, 0, 0, -2, 2, 2, -2};

namespace lrae_planner_ns
{
    PathPlanning::PathPlanning()
    {
        planner_ = new GraphSearch();
        start_x_ = 0;
        start_y_ = 0;
        goal_x_ = 0;
        goal_y_ = 0;
        first_planning_ = true;
    }
    PathPlanning::~PathPlanning()
    {
        if (planner_)
            delete planner_;
        clearMap();       
    }

    void PathPlanning::setViewPointSet(std::vector<utils_ns::ViewPoint> viewpointset1, std::vector<utils_ns::ViewPoint> viewpointset2, bool viewpoint1has, bool viewpoint2has)
    {
        viewpointset1_ = viewpointset1;
        viewpointset2_ = viewpointset2;
        viewpoint1has_ = viewpoint1has;
        viewpoint2has_ = viewpoint2has;
    }
    void PathPlanning::clearViewPointSet()
    {
        viewpointset1_.clear();
        viewpointset2_.clear();
    }

    void PathPlanning::setCentroidPairIndex(utils_ns::Index2 centroidone, utils_ns::Index2 centroidtwo)
    {
        centroidone_ = centroidone;
        centroidtwo_ = centroidtwo;
    }

    std::vector<utils_ns::ViewPoint> PathPlanning::getViewPointSet1()
    {
        return viewpointset1_;
    }

    std::vector<utils_ns::ViewPoint> PathPlanning::getViewPointSet2()
    {
        return viewpointset2_;
    }

    void PathPlanning::setRobotPosition(geometry_msgs::Point robot_position)
    {
        robot_position_ = robot_position;
        robot_x_ = CONTXY2DISC(robot_position_.x - origin_x_, resolution_);
        robot_y_ = CONTXY2DISC(robot_position_.y - origin_y_, resolution_);
    }
  
    void PathPlanning::clearMap()
    {
        if (planmap_)
        {
            for (int i = 0; i < width_; i++)
            {
                delete[] planmap_[i];
            }
            delete[] planmap_;
        }
    }

    void PathPlanning::setPlanMap(const nav_msgs::OccupancyGrid& map)
    {
        recmap_ = map;
        clearMap();
        width_ = map.info.width;
        height_ = map.info.height;
        resolution_ = map.info.resolution;
        origin_x_ = map.info.origin.position.x;
        origin_y_ = map.info.origin.position.y;
        planmap_ = new unsigned char*[width_];
        for (int i = 0; i < width_; i++)
        {
            planmap_[i] = new unsigned char[height_];
        }
        for (int i = 0; i < width_; i++)
        {
            for (int j = 0; j < height_; j++)
            {
                if(map.data[i + j * width_] == -1)
                    planmap_[i][j] = 0;
                else
                    planmap_[i][j] = map.data[i + j * width_];
            }
        }
    }

    void PathPlanning::setStart(int startx, int starty)
    {
        if(height_ <= starty || starty < 0 || width_ <= startx || startx < 0)
        {
            ROS_INFO("Start Out of map");
            has_start_ = false;
        }
        else if(planmap_[startx][starty] >= occThre_)
        {
            // ROS_INFO("Start is Occupancy");
            has_start_ = false;
        }
        else{
            start_x_ = startx;
            start_y_ = starty;
            has_start_ = true;
        }
    }

    void PathPlanning::setGoal(int goalx, int goaly)
    {
        if(height_ <= goaly || goaly < 0 || width_ <= goalx || goalx < 0)
        {
            ROS_INFO("Goal Out of map");
            has_goal_ = false;
        }
        else if(planmap_[goalx][goaly] >= occThre_)
        {
            // ROS_INFO("Goal is Occupancy");
            has_goal_ = false;
        }
        else{
            goal_x_ = goalx;
            goal_y_ = goaly;
            has_goal_ = true;
        }
    }

    bool PathPlanning::makePlan(nav_msgs::Path& guiPath, int& pathCost)
    {
        if(has_goal_ && has_start_)
        {
            std::vector<utils_ns::PointInt> path;
            std::vector<utils_ns::PointInt> expands; 
            planner_->Search(start_x_, start_y_, goal_x_, goal_y_, planmap_, width_, height_, occThre_, path, expands, pathCost);  
            if(path.size() >= 2)
            {
                guiPath = getPositionPath(path);
                has_goal_ = false;
                has_start_ = false;
                return true;
            }
            else{
                // ROS_INFO("No Search Path");
                has_goal_ = false;
                has_start_ = false;
                return false;
            }
        }
        else{
            // ROS_INFO("No goal || start");
            has_goal_ = false;
            has_start_ = false;
            return false;
        }
    }

    nav_msgs::Path PathPlanning::getPositionPath(const std::vector<utils_ns::PointInt>& path)
    {
        nav_msgs::Path gui_path;
        gui_path.header.frame_id = "map";
        gui_path.header.stamp = ros::Time::now();
        gui_path.poses.resize(path.size());
        for (int i = 0; i < (int)path.size(); i++)
        {
            if(height_ > path[i].y && path[i].y >= 0 && width_ > path[i].x && path[i].x >= 0)
            {
                double x = DISCXY2CONT(path[i].x, resolution_) + origin_x_;
                double y = DISCXY2CONT(path[i].y, resolution_) + origin_y_;
                gui_path.poses[i].pose.position.x = x;
                gui_path.poses[i].pose.position.y = y; 
                if(i + 1 < (int)path.size())
                {
                    double nx = DISCXY2CONT(path[i + 1].x, resolution_) + origin_x_;
                    double ny = DISCXY2CONT(path[i + 1].y, resolution_) + origin_y_;
                    gui_path.poses[i].pose.orientation.w = atan2((ny - y),(nx -x));
                } 
                else  
                    gui_path.poses[i].pose.orientation.w = gui_path.poses[i - 1].pose.orientation.w;         
            }

        }
        return gui_path;
    }

    bool PathPlanning::finePath(nav_msgs::Path& OriRVPath)
    {
        if(first_planning_ && OriRVPath.poses.size() > 2)
        {
            first_planning_ = false;
            last_RVpath_ = OriRVPath;
            last_goal1_ = centroidone_;
            last_goal2_ = centroidtwo_; 
            return true;
        }
       
        else if(!first_planning_ && last_RVpath_.poses.size() > 2)
        { 
            if(OriRVPath.poses.size() <= 2)
            {
                if(planmap_[last_goal1_.x][last_goal1_.y] < occThre_)
                {
                    OriRVPath = last_RVpath_;
                    // ROS_INFO("Last path is empty");
                    return true;
                }
            }
            else if(OriRVPath.poses.size() > 3)
            {
                double robotOriAngel = utils_ns::getAngleRobot(OriRVPath.poses[3].pose.position, robot_position_);
                double robotLasAngel = utils_ns::getAngleRobot(last_RVpath_.poses[3].pose.position, robot_position_);
                if(recmap_.data[last_goal1_.x + width_ * last_goal1_.y] == -1)
                {
                    if((robotOriAngel - robotLasAngel) > 1.1)
                    {
                        OriRVPath = last_RVpath_;
                        // ROS_INFO("Follow last path");

                        return true;
                    }
                    else{
                        last_RVpath_ = OriRVPath;
                        last_goal1_ = centroidone_;
                        last_goal2_ = centroidtwo_; 
                        return true;
                    }

                }
                else{
                    last_RVpath_ = OriRVPath;
                    last_goal1_ = centroidone_;
                    last_goal2_ = centroidtwo_; 
                    return true;
                }
            }
        }
        return true;
    }

    bool PathPlanning::getExplorationPath(nav_msgs::Path& exporation_path)
    {   
            setStart(robot_x_, robot_y_);
            if((height_ > centroidone_.y && centroidone_.y >= 0 && width_ > centroidone_.x && centroidone_.x >= 0 ))
                setGoal(centroidone_.x, centroidone_.y);
            else
            {
                setGoal(robot_x_, robot_y_);
                ROS_ERROR("The goal is out of map");
            }
            int rvcost;
            nav_msgs::Path rvpath;
            makePlan(rvpath, rvcost);

            exporation_path = rvpath;

            int count_plan = 0;
            bool findPath = true;
            if(exporation_path.poses.size() < 2)
                findPath = false;
            while(!findPath && count_plan < 16)
            {
                for(int i = 0; i < 16; i++)
                {
                    int zx = centroidone_.x + zd_x16[i];
                    int zy = centroidone_.y + zd_y16[i];
                
                    if(recmap_.data[zx + zy * recmap_.info.width] < occThre_)
                    {
                        setStart(robot_x_, robot_y_);
                        if (height_ > zy && zy >= 0 && width_ > zx && zx >= 0 )
                           setGoal(zx, zy);
                        else
                        {
                            ROS_ERROR("The goal is out of map !");
                            continue;
                        }                        
                        makePlan(rvpath, rvcost);
                        exporation_path = rvpath;
                        if(exporation_path.poses.size() >= 2)
                        {
                            findPath = true;
                            break;
                        }
                    }
                    count_plan++;
                }
            }
            // viewpoint1has_ = false;
            // viewpoint2has_ = false;
            clearViewPointSet();
            if(exporation_path.poses.size() < 2)
            {
                // ROS_INFO("No path ! \n");
                return false;
            }
            else
                return true;
    }
}
