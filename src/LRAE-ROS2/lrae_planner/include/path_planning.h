/**
 *  Created by Qingchen Bi on 2023/10/25
 */
#ifndef _PATH_PLANNING_H_
#define _PATH_PLANNING_H_

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/point.hpp"  // ROS2 中的 Point 消息
#include "planner_interface.h"      // 自定义头文件（如果是ROS2自定义包）
#include "graph_search.h"             // 自定义头文件（如果是ROS2自定义包）
#include "nav_msgs/msg/path.hpp"       // ROS2 中的 Path 消息

namespace lrae_planner_ns
{
class PathPlanning;

class PathPlanning
{
public:
    PathPlanning();
    ~PathPlanning();
    void setViewPointSet(std::vector<utils_ns::ViewPoint> viewpointset1, std::vector<utils_ns::ViewPoint> viewpointset2, bool viewpoint1has, bool viewpoint2has);
    void setCentroidPairIndex(utils_ns::Index2 centroidone, utils_ns::Index2 centroidtwo);
    std::vector<utils_ns::ViewPoint> getViewPointSet1();
    std::vector<utils_ns::ViewPoint> getViewPointSet2();
    void setPlanMap(const nav_msgs::msg::OccupancyGrid& map);
    bool getExplorationPath(nav_msgs::msg::Path& exporation_path);
    void setRobotPosition(geometry_msgs::msg::Point robot_position);
    bool finePath(nav_msgs::msg::Path& OriPath);
    nav_msgs::msg::Path getPositionPath(const std::vector<utils_ns::PointInt>& path);

private:
    void clearMap();
    void clearViewPointSet();
    void setStart(int startx, int starty);
    void setGoal(int goalx, int goaly);
    bool makePlan(nav_msgs::msg::Path& guiPath, int& pathCost);

    std::vector<utils_ns::ViewPoint> viewpointset1_;
    std::vector<utils_ns::ViewPoint> viewpointset2_;
    utils_ns::Index2 centroidone_;
    utils_ns::Index2 centroidtwo_;
    nav_msgs::msg::OccupancyGrid recmap_;
    unsigned char** planmap_ = NULL;
    int width_, height_;
    double origin_x_, origin_y_;
    float resolution_;
    int start_x_, start_y_, goal_x_, goal_y_, robot_x_, robot_y_;
    bool has_start_ = false;
    bool has_goal_ = false;
    PlannerInterface* planner_;
    geometry_msgs::msg::Point robot_position_;

    int occThre_ = 95;

    bool viewpoint1has_ = false;
    bool viewpoint2has_ = false;
    nav_msgs::msg::Path last_RVpath_;
    nav_msgs::msg::Path last_VVpath_;
    utils_ns::Index2 last_goal1_;
    utils_ns::Index2 last_goal2_;
    bool first_planning_ = true;
};
}
#endif
