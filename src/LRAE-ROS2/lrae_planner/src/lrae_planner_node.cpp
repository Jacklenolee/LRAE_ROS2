/**
 *  Created by Qingchen Bi on 2023/10/24
 */
#include <rclcpp/rclcpp.hpp>
#include "exploration_planning.h"
#include "backward.hpp"

namespace backward
{
backward::SignalHandling sh;
}
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node_handle = rclcpp::Node::make_shared("lrae_planner_node");
  lrae_planner_ns::ExplorationPlanning exploration_planner(node_handle);

  rclcpp::spin_some(node_handle);

  return 0;
}