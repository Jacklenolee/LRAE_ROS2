/**
 *  Created by Qingchen Bi on 2022/3/21
 */
#include "rclcpp/rclcpp.hpp"
#include "gen_local_goal.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node_handle = rclcpp::Node::make_shared("gen_local_goal_node");

    GenLocalGoal gen_local_goal(node_handle);
    rclcpp::spin(node_handle);  // 启动 ROS 2 的回调处理循环
    rclcpp::shutdown();  // 退出时关闭 ROS 2 环境
    return 0;
}