#include "../include/slam_simulation/slam_output.h"

SlamOutput::SlamOutput(const rclcpp::NodeOptions & options)
: rclcpp::Node("slam_output", options),
  is_get_first(false),
  T_B_W(tf2::Transform::getIdentity()),
  vec_length(2.0)
{
    // 获取参数
    frame_id = this->declare_parameter<std::string>("frame_id", "map");
    child_frame_id = this->declare_parameter<std::string>("child_frame_id", "sensor");
    down_voxel_size = this->declare_parameter<float>("down_voxel_size", 0.1f);

    // 创建发布者
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odometry_init", 1);
    reg_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 1);
    
    // 创建TF广播器
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 创建消息过滤器和同步器
    rclcpp::QoS qos_profile(1);
    local_cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, "point_cloud", qos_profile.get_rmw_qos_profile());
    
    rclcpp::QoS qos_profile_odom(100);
    local_odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
        this, "odometry", qos_profile_odom.get_rmw_qos_profile());
    
    sync_local_cloud_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyLocalCloudOdom>>(
        SyncPolicyLocalCloudOdom(100), *local_cloud_sub_, *local_odom_sub_);
    
    sync_local_cloud_odom_->registerCallback(
        std::bind(&SlamOutput::pointCloudOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
    
}

void SlamOutput::pointCloudOdomCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & scanIn,
    const nav_msgs::msg::Odometry::ConstSharedPtr & input)
{
    scanIn_ = scanIn;
    
    // 从里程计消息中提取四元数和位置
    tf2::Quaternion quaternion;
    tf2::fromMsg(input->pose.pose.orientation, quaternion);
    
    tf2::Vector3 vector3(
        input->pose.pose.position.x, 
        input->pose.pose.position.y, 
        input->pose.pose.position.z);
    
    tf2::Transform T_W_Bi(quaternion, vector3);
    
    if (!is_get_first) {
        T_B_W = T_W_Bi.inverse();
        is_get_first = true;
    }
    
    // 创建变换
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = scanIn->header.stamp;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    
    // 计算变换
    tf2::Transform transform_result = T_B_W * T_W_Bi;
    ST_B_Bi_ = transform_result;
    
    // 填充变换消息
    transform_stamped.transform.translation.x = transform_result.getOrigin().x();
    transform_stamped.transform.translation.y = transform_result.getOrigin().y();
    transform_stamped.transform.translation.z = transform_result.getOrigin().z();
    transform_stamped.transform.rotation = tf2::toMsg(transform_result.getRotation());
    
    // 广播变换
    tf_broadcaster_->sendTransform(transform_stamped);
    
    // 创建并发布里程计消息
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    odom_msg->child_frame_id = child_frame_id;
    odom_msg->header.frame_id = frame_id;
    odom_msg->header.stamp = scanIn->header.stamp;
    
    odom_msg->pose.pose.orientation = tf2::toMsg(transform_result.getRotation());
    odom_msg->pose.pose.position.x = transform_result.getOrigin().x();
    odom_msg->pose.pose.position.y = transform_result.getOrigin().y();
    odom_msg->pose.pose.position.z = transform_result.getOrigin().z();
    
    odom_msg->twist = input->twist;
    
    odom_pub->publish(std::move(odom_msg));
    execute();
}

void SlamOutput::execute()
{
    if (!is_get_first || !scanIn_) {
        return;
    }
    // 点云处理
    pcl::PointCloud<pcl::PointXYZI> scan;
    pcl::fromROSMsg(*scanIn_,scan);
    
    // 获取变换矩阵
    Eigen::Matrix4f pose;
    pose << ST_B_Bi_.getBasis()[0][0], ST_B_Bi_.getBasis()[0][1], ST_B_Bi_.getBasis()[0][2], ST_B_Bi_.getOrigin()[0],
            ST_B_Bi_.getBasis()[1][0], ST_B_Bi_.getBasis()[1][1], ST_B_Bi_.getBasis()[1][2], ST_B_Bi_.getOrigin()[1],
            ST_B_Bi_.getBasis()[2][0], ST_B_Bi_.getBasis()[2][1], ST_B_Bi_.getBasis()[2][2], ST_B_Bi_.getOrigin()[2],
            0, 0, 0, 1;
    
    // 点云变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (auto &point: scan.points) {
        pcl::PointXYZ reg_point;
        reg_point.x = point.x * pose(0, 0) + point.y * pose(0, 1) + point.z * pose(0, 2) + pose(0, 3);
        reg_point.y = point.x * pose(1, 0) + point.y * pose(1, 1) + point.z * pose(1, 2) + pose(1, 3);
        reg_point.z = point.x * pose(2, 0) + point.y * pose(2, 1) + point.z * pose(2, 2) + pose(2, 3);
        registered_scan->points.push_back(reg_point);
    }
     
    // 发布点云消息
    sensor_msgs::msg::PointCloud2 scan_data_msg;
    pcl::toROSMsg(*registered_scan, scan_data_msg);
    scan_data_msg.header.stamp = scanIn_->header.stamp;
    scan_data_msg.header.frame_id = frame_id;
    reg_pub->publish(scan_data_msg);
}
