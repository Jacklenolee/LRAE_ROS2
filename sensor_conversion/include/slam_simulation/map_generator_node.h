#ifndef MAP_GENERATOR_NODE_H
#define MAP_GENERATOR_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

class MapGeneratorNode : public rclcpp::Node {
public:
    MapGeneratorNode();

private:
    // 发布器
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    
    // 订阅器
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    
    // TF监听器
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // 点云处理相关
    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter_;
    pcl::VoxelGrid<pcl::PointXYZ> exploredAreaDwzFilter_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud_;
    
    // 参数
    std::string frame_id_;
    std::string child_frame_id_;
    double down_voxel_size_;
    double exploredAreaVoxelSize_;
    
    // 回调函数
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud);
    
    // 辅助函数
    Eigen::Matrix4f transformToEigenMatrix4f(const geometry_msgs::msg::Transform &transform);
};

#endif // MAP_GENERATOR_NODE_H