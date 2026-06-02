#ifndef TOPO_PLANNER_WS_SLAM_OUTPUT_HPP
#define TOPO_PLANNER_WS_SLAM_OUTPUT_HPP

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Core>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/msg/float32.hpp>

class SlamOutput : public rclcpp::Node
{
public:
    explicit SlamOutput(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
private:
    // TF相关
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr reg_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dwz_cloud_pub;
    
    // 消息过滤器和同步器
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2, 
        nav_msgs::msg::Odometry> SyncPolicyLocalCloudOdom;
    
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> local_cloud_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> local_odom_sub_;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyLocalCloudOdom>> SynchronizerLocalCloudOdom;
    SynchronizerLocalCloudOdom sync_local_cloud_odom_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr execution_timer_;
    
    // 参数
    std::string frame_id;
    std::string child_frame_id;
    bool is_get_first;
    tf2::Transform T_B_W;
    double vec_length;
    
    // 点云处理相关
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    float down_voxel_size = 0.1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    double exploredAreaVoxelSize = 0.1;
    pcl::VoxelGrid<pcl::PointXYZ> exploredAreaDwzFilter;
    
    // 存储数据
    sensor_msgs::msg::PointCloud2::ConstSharedPtr scanIn_;
    tf2::Transform ST_B_Bi_;
    
    // 回调函数
    void pointCloudOdomCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scanIn,
        const nav_msgs::msg::Odometry::ConstSharedPtr & input);
    
    void execute();
};

#endif //TOPO_PLANNER_WS_SLAM_OUTPUT_HPP