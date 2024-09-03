#include "ros/ros.h"
#include "utils.h"
namespace utils_ns
{
double getAngleRobot(geometry_msgs::Point& p, geometry_msgs::Point& robot)
{
    double angle_p = atan2(p.y - robot.y, p.x - robot.x);
    double angle = (robot.z - angle_p);
    if (angle > M_PI)
        angle -= 2 * M_PI;
    if (angle < -M_PI)
        angle += 2 * M_PI;
    return fabs(angle);
}

double getAngleVector(geometry_msgs::Point& p1, geometry_msgs::Point& p2, geometry_msgs::Point& robot)
{
    double theta = atan2(p1.y - robot.y, p1.x - robot.x) - atan2(p2.y - robot.y, p2.x - robot.x);
    if (theta > M_PI)
        theta -= 2 * M_PI;
    if (theta < -M_PI)
        theta += 2 * M_PI;
    return fabs(theta);
}

bool Bresenham(int x1, int y1, int x2, int y2, std::vector<Eigen::Vector2i> &visited_grid, nav_msgs::OccupancyGrid& map)
{
 
	int dx = abs(x2 - x1);
	int dy = abs(y2 - y1);
	bool is_great_than_45_degree = false;
 
	if (dx <= dy)
	{
		is_great_than_45_degree = true;
	}
	int fx = (x2 - x1) > 0 ? 1 : -1;
	int fy = (y2 - y1) > 0 ? 1 : -1;
	if (dy == 0)
		fy = 0;
	if (dx == 0)
		fx = 0;
 
	int ix = x1;
	int iy = y1;
	int p1 = 2 * dy - dx; 
	int p2 = 2 * dx - dy; 
    Eigen::Vector2i c_grid;
    c_grid[0] = ix;
    c_grid[1] = iy;
    visited_grid.push_back(c_grid); 

    if (is_great_than_45_degree){
		while (iy != y2){
			if (p2 > 0){
				p2 += 2 * (dx - dy);
				ix += fx;
			}
			else{
				p2 += 2*dx;
			}
			iy += fy;
            c_grid[0] = ix;
            c_grid[1] = iy;
            visited_grid.push_back(c_grid); 
            if(map.data[c_grid[0] + c_grid[1] * map.info.width] >= 90)
                return false;
		}
	}
	else{
		while (ix != x2){
			if (p1 > 0){
				p1 += 2 * (dy - dx);
				iy += fy;
			}
			else{
				p1 += 2 * dy;
			}
			ix += fx;
            c_grid[0] = ix;
            c_grid[1] = iy;
            visited_grid.push_back(c_grid);
            if(map.data[c_grid[0] + c_grid[1] * map.info.width] >= 90)
                return false;

		}
	}
    if(visited_grid.size() >= 10)
        return true;
    else
        return false;
}
}