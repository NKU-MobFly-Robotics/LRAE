/**
 *  Created by Qingchen Bi on 2022/3/21
 */
#include <ros/ros.h>
#include "gen_local_goal.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gen_local_goal_node");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_node_handle("~");

    GenLocalGoal gen_local_goal(node_handle, private_node_handle);
    ros::spin();
    return 0;
}