/**
 *  Created by Qingchen Bi on 2022/11/05
 */
#include <ros/ros.h>
#include "vector"
#include "backward.hpp"
#include <plane.h>

namespace backward
{
backward::SignalHandling sh;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Traversibility_mapping");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Rate rate(2);
    FitPlane::World world(0.1, nh, nh_private);
    float plane_size = 0.3; // 0.5
    FitPlane::PlaneMap planemap(&world, plane_size);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();        
    }

    return 0;
}