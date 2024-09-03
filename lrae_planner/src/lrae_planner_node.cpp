/**
 *  Created by Qingchen Bi on 2023/10/24
 */
#include <ros/ros.h>
#include "exploration_planning.h"
#include "backward.hpp"

namespace backward
{
backward::SignalHandling sh;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "lrae_planner_node");
  ros::NodeHandle node_handle;
  ros::NodeHandle private_node_handle("~");

  lrae_planner_ns::ExplorationPlanning exploration_planner(node_handle, private_node_handle);

  ros::spin();

  return 0;
}