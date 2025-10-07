/**
 *  Created by Qingchen Bi on 2022/3/21
 */

#ifndef GEN_LOCAL_GOAL_H
#define GEN_LOCAL_GOAL_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <utils.h>
#include <algorithm>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <rclcpp/timer.hpp>
#include <functional>

using namespace bezier_local_planner;

class GenLocalGoal
{  
public:
  GenLocalGoal(rclcpp::Node::SharedPtr nh);
  ~GenLocalGoal();
private:
  void execute();
  void Initialize();
  void MapCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void PathCallBack(const nav_msgs::msg::Path::SharedPtr msg);
  void GetGlobalPlan();
  void velCallBack(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subOdom_;
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pubwaypoint_;
  std::string map_topic_ = "/plane_OccMap";
  std::string path_topic_ = "/exporation_path";

  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path tmp_path_;

  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  std::shared_ptr<tf2_ros::Buffer> buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
  std::shared_ptr< tf2_ros::TransformListener> listener_;

  rclcpp::TimerBase::SharedPtr execution_timer_;

  double tmp_path_length_;
  double path_resolution_;
  geometry_msgs::msg::PointStamped last_waypoint_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubSpeed_;
  int no_path_count_ = 0;
  
  int has_path_no_vel_count_ = 0;
  rclcpp::Subscription< geometry_msgs::msg::Twist>::SharedPtr subSpeed_;
  geometry_msgs::msg::Twist cmd_vel_;
  bool pub_vel_ing_ = false;
  int pub_vel_count_ = 0;

};
#endif 