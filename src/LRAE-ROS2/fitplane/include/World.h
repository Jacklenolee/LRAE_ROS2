#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point.hpp>
#include <chrono>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/clock.hpp>

namespace FitPlane
{
    class PlaneMap;

struct FitPlaneArg
{
    double w_total_;
    double w_flatness_;
    double w_slope_;
    double w_sparsity_;
    double ratio_max_;
    double ratio_min_;
    double conv_thre_;
};
const float INF = std::numeric_limits<float>::max();
const float PI = 3.14151f;

class World;

namespace visualization { void visWorld(World* world, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> world_vis_pub); }

/**
 * @brief Class for storing obstacles and world dimension.
 *        The information of obstacle is stored in a three-dimensional bool array.
 *        Before using the PF-RRT* algorithm, a suitable grid map must be built.
 */
class World
{
public:
    friend void visualization::visWorld(World* world, std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> world_vis_pub);

    bool has_map_ = false;  // Indicate whether the range of the grid map has been determined

    World(const float &resolution, rclcpp::Node::SharedPtr nh); //, FitPlaneArg fitarg)
    ~World();

    void initGridMap(const pcl::PointCloud<pcl::PointXYZ> &cloud);
    void initGridMap(const Eigen::Vector3d &lowerbound, const Eigen::Vector3d &upperbound);
    void setObs(const Eigen::Vector3d &point);
    Eigen::Vector3d coordRounding(const Eigen::Vector3d &coord);

    bool isFree(const Eigen::Vector3d &point);
    bool isFree(const float &coord_x, const float &coord_y, const float &coord_z) { return isFree(Eigen::Vector3d(coord_x, coord_y, coord_z)); }

    bool project2surface(const float &x, const float &y, Eigen::Vector3d* p_surface);
    bool project2surface(const Eigen::Vector3d &p_original, Eigen::Vector3d* p_surface) { return project2surface(p_original(0), p_original(1), p_surface); }

    bool isInsideBorder(const Eigen::Vector3i &index);
    bool isInsideBorder(const Eigen::Vector3d &point) { return isInsideBorder(coord2index(point)); }

    Eigen::Vector3d getLowerBound() { return lowerbound_; }
    Eigen::Vector3d getUpperBound() { return upperbound_; }
    float getResolution() { return resolution_; }

    Eigen::Vector3d index2coord(const Eigen::Vector3i &index)
    {
        Eigen::Vector3d coord = resolution_ * index.cast<double>() + lowerbound_ + 0.5 * resolution_ * Eigen::Vector3d::Ones();
        return coord;
    }

    Eigen::Vector3i coord2index(const Eigen::Vector3d &coord)
    {
        Eigen::Vector3i index = ((coord - lowerbound_) / resolution_).cast<int>();
        return index;
    }

    bool ***grid_map_ = NULL;

    float resolution_;
    Eigen::Vector3i idx_count_;
    Eigen::Vector3d lowerbound_;
    Eigen::Vector3d upperbound_;

    // ROS 2
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr PointCloud_Map_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr PointCloud_sub;
    std::string PointCloud_topic = "/velodyne_points";
    std::string PointCloud_Map_topic = "/laser_cloud_map";
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr Grid_Map_pub;
    std::string Grid_Map_topic = "/grid_map";
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr Plane_Map_pub;
    std::string Plane_Map_topic = "/plane_map";

    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    std::shared_ptr<tf2_ros::Buffer> buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
    std::shared_ptr< tf2_ros::TransformListener> listener_;
    // tf2_ros::TransformListener listener_;
    geometry_msgs::msg::Point ego_position_;
    pcl::PointCloud<pcl::PointXYZ> cloud_near_;
    float minrange_ = 30.0;
    bool use_ex_range_ = false;
    double ex_robot_front_ = 90; 
    double ex_robot_back_ = -10;
    double ex_robot_left_ = 90;
    double ex_robot_right_ = -10;
    void clearMap();

    void GetRobotPosition()
    {
        geometry_msgs::msg::Pose current_pose_ros;
        geometry_msgs::msg::TransformStamped transform_pose;
        tf2::Quaternion quat;
        double roll, pitch, yaw;

        try
        {
            // Wait for the transform and get the transform from /map to /base_link
            transform_pose = buffer_->lookupTransform("/map", "/base_link", tf2::TimePointZero);

            // Extract position and orientation from the transform
            ego_position_.x = transform_pose.transform.translation.x;
            ego_position_.y = transform_pose.transform.translation.y;
            ego_position_.z = transform_pose.transform.translation.z;

            quat.setValue(transform_pose.transform.rotation.x, transform_pose.transform.rotation.y, transform_pose.transform.rotation.z, transform_pose.transform.rotation.w);
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(nh_->get_logger(), "Could not get transform: %s", ex.what());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

};

inline Eigen::Vector2d project2plane(const Eigen::Vector3d &p) { return Eigen::Vector2d(p(0), p(1)); }
inline Eigen::Vector2d project2plane(const float &x, const float &y) { return Eigen::Vector2d(x, y); }

template <typename T>
void clean_vector(std::vector<T*> &vec)
{
    for (auto &element : vec)
    {
        delete element;
        element = NULL;
    }
    vec.clear();
}

}

#endif
