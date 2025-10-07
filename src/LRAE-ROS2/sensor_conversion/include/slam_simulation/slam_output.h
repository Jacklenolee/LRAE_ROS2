#ifndef TOPO_PLANNER_WS_SLAM_OUTPUT_H
#define TOPO_PLANNER_WS_SLAM_OUTPUT_H

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include <Eigen/Core>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


class SlamOutput :public rclcpp::Node {
public:
    rclcpp::Node::SharedPtr nh_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr reg_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dwz_cloud_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub;
    rclcpp::TimerBase::SharedPtr global_down_timer;

    std::shared_ptr<tf2_ros::TransformBroadcaster>  broadcaster;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::Rate rate;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> SyncPolicyLocalCloudOdom;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> local_cloud_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>> local_odom_sub_;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyLocalCloudOdom>> SynchronizerLocalCloudOdom;
    SynchronizerLocalCloudOdom sync_local_cloud_odom_;

    std::string frame_id;
    std::string child_frame_id;

    bool is_get_first;

    tf2::Transform T_B_W ;

    std::vector<geometry_msgs::msg::TransformStamped> ST_B_Bi_vec;
    double vec_length;

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    float down_voxel_size = 0.1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr exploredAreaCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    double exploredAreaVoxelSize = 0.1;
    pcl::VoxelGrid<pcl::PointXYZ> exploredAreaDwzFilter;
    rclcpp::TimerBase::SharedPtr execution_timer_;

    void execute(); 

    sensor_msgs::msg::PointCloud2::SharedPtr scanIn_;
    geometry_msgs::msg::TransformStamped ST_B_Bi_;
    
    SlamOutput(const rclcpp::Node::SharedPtr nh);

    void pointCloudOdomCallback(const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud,const nav_msgs::msg::Odometry::SharedPtr input);

    float exploredVolume_ = 0;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr explored_volume_pub;
};



#endif //TOPO_PLANNER_WS_SLAM_OUTPUT_H