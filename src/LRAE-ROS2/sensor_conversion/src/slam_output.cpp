//
// Created by hjl on 2021/9/18.
// Modified by Qingchen Bi on 2022/11/05
//

#include "slam_simulation/slam_output.h"
#include <Transform.hpp>

SlamOutput::SlamOutput(const rclcpp::Node::SharedPtr nh) : Node("slam_sim_output"),nh_(nh),rate(100), vec_length(2.0),
is_get_first(false) {
 // Default values
        frame_id = "map";
        child_frame_id = "sensor";
        down_voxel_size = 0.1;

        // Retrieve parameters with a fallback to default values
        this->declare_parameter<std::string>("frame_id", frame_id);
        this->declare_parameter<std::string>("child_frame_id", child_frame_id);
        this->declare_parameter<double>("down_voxel_size", down_voxel_size);

        // Fetch the parameters
        this->get_parameter("frame_id", frame_id);
        if (!this->has_parameter("frame_id"))
        {
            RCLCPP_WARN(this->get_logger(), "No frame_id specified. Using default: %s.", frame_id.c_str());
        }

        this->get_parameter("child_frame_id", child_frame_id);
        if (!this->has_parameter("child_frame_id"))
        {
            RCLCPP_WARN(this->get_logger(), "No child_frame_id specified. Using default: %s.", child_frame_id.c_str());
        }

        this->get_parameter("down_voxel_size", down_voxel_size);
        if (!this->has_parameter("down_voxel_size"))
        {
            RCLCPP_WARN(this->get_logger(), "No down_voxel_size specified. Using default: %f.", down_voxel_size);
        }

    T_B_W = tf2::Transform::getIdentity();

    downSizeFilter.setLeafSize(down_voxel_size, down_voxel_size, down_voxel_size);
    exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
    odom_pub = nh_->create_publisher<nav_msgs::msg::Odometry>("odometry_init", 1);
    reg_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 1);
    dwz_cloud_pub = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("dwz_scan_cloud", 1);

    // 创建订阅器
    local_cloud_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(nh_, "point_cloud");
    local_odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(nh_, "odometry");

    // 使用同步器同步订阅的数据
    sync_local_cloud_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyLocalCloudOdom>>(
    SyncPolicyLocalCloudOdom(10), *local_cloud_sub_, *local_odom_sub_);
    sync_local_cloud_odom_->registerCallback(
    std::bind(&SlamOutput::pointCloudOdomCallback, this, std::placeholders::_1, std::placeholders::_2));

    // 创建定时器，周期为0.2秒
    execution_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(200), std::bind(&SlamOutput::execute, this));
}

void SlamOutput::pointCloudOdomCallback(const sensor_msgs::msg::PointCloud2::SharedPtr scanIn,
   const nav_msgs::msg::Odometry::SharedPtr input) {

    scanIn_ = scanIn;
    tf2::Quaternion quaternion(input->pose.pose.orientation.x, input->pose.pose.orientation.y,
                              input->pose.pose.orientation.z, input->pose.pose.orientation.w);
    tf2::Vector3 vector3(input->pose.pose.position.x, input->pose.pose.position.y, input->pose.pose.position.z);
    tf2::Transform T_W_Bi(quaternion, vector3);
    if (!is_get_first) {
        T_B_W = T_W_Bi.inverse();
        is_get_first = true;
    }
    // 创建变换Stamped消息
    geometry_msgs::msg::TransformStamped ST_B_Bi;

    // 设置父框架、子框架和时间戳
    ST_B_Bi.header.stamp = scanIn->header.stamp;
    ST_B_Bi.header.frame_id = frame_id;  // 替换为你的父框架
    ST_B_Bi.child_frame_id = child_frame_id;  // 替换为你的子框架

    // 填充变换信息
    ST_B_Bi.transform.translation.x = (T_B_W * T_W_Bi).getOrigin().x();
    ST_B_Bi.transform.translation.y = (T_B_W * T_W_Bi).getOrigin().y();
    ST_B_Bi.transform.translation.z = (T_B_W * T_W_Bi).getOrigin().z();

    ST_B_Bi.transform.rotation.x = (T_B_W * T_W_Bi).getRotation().x();
    ST_B_Bi.transform.rotation.y = (T_B_W * T_W_Bi).getRotation().y();
    ST_B_Bi.transform.rotation.z = (T_B_W * T_W_Bi).getRotation().z();
    ST_B_Bi.transform.rotation.w = (T_B_W * T_W_Bi).getRotation().w();
    ST_B_Bi_ = ST_B_Bi;    

    broadcaster->sendTransform(ST_B_Bi); 

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.child_frame_id = child_frame_id;
    odom_msg.header.frame_id = frame_id;
    odom_msg.header.stamp = scanIn->header.stamp; 
    odom_msg.pose.pose.orientation.x = ST_B_Bi.transform.rotation.x;
    odom_msg.pose.pose.orientation.y = ST_B_Bi.transform.rotation.y;
    odom_msg.pose.pose.orientation.z = ST_B_Bi.transform.rotation.z;
    odom_msg.pose.pose.orientation.w = ST_B_Bi.transform.rotation.w;
    odom_msg.pose.pose.position.x = ST_B_Bi.transform.translation.x;
    odom_msg.pose.pose.position.y = ST_B_Bi.transform.translation.y;
    odom_msg.pose.pose.position.z = ST_B_Bi.transform.translation.z;

    // 设置速度信息（如果需要）
    odom_msg.twist = input->twist; 

    odom_pub->publish(odom_msg); // laser_odom_init
}

void SlamOutput::execute()
{   
    if(is_get_first)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*scanIn_, *scan);

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_data = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        std::vector<int> scan_index;
        pcl::removeNaNFromPointCloud(*scan, *scan_data, scan_index);

        downSizeFilter.setInputCloud(scan_data);
        pcl::PointCloud<pcl::PointXYZI> scan_dwz;
        downSizeFilter.filter(scan_dwz);

        geometry_msgs::msg::TransformStamped transform_stamped = ST_B_Bi_;

        // 将 TransformStamped 转换为 tf2::Transform
        tf2::Quaternion q(
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z,
        transform_stamped.transform.rotation.w
        );
        tf2::Vector3 t(
        transform_stamped.transform.translation.x,
        transform_stamped.transform.translation.y,
        transform_stamped.transform.translation.z
        );
        tf2::Transform T_b_bi(q, t);

        Eigen::Matrix4f pose;
        pose << T_b_bi.getBasis()[0][0], T_b_bi.getBasis()[0][1], T_b_bi.getBasis()[0][2], T_b_bi.getOrigin()[0],
                T_b_bi.getBasis()[1][0], T_b_bi.getBasis()[1][1], T_b_bi.getBasis()[1][2], T_b_bi.getOrigin()[1],
                T_b_bi.getBasis()[2][0], T_b_bi.getBasis()[2][1], T_b_bi.getBasis()[2][2], T_b_bi.getOrigin()[2],
                0, 0, 0, 1;

        pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        for (auto &point: scan_dwz.points) {
            pcl::PointXYZ reg_point;
            reg_point.x = point.x * pose(0, 0) + point.y * pose(0, 1) + point.z * pose(0, 2) + pose(0, 3);
            reg_point.y = point.x * pose(1, 0) + point.y * pose(1, 1) + point.z * pose(1, 2) + pose(1, 3);
            reg_point.z = point.x * pose(2, 0) + point.y * pose(2, 1) + point.z * pose(2, 2) + pose(2, 3);
            registered_scan->points.push_back(reg_point);
        }

        *exploredAreaCloud += *registered_scan;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInDwz(new pcl::PointCloud<pcl::PointXYZ>());
        exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
        exploredAreaDwzFilter.filter(*cloudInDwz);
        
        sensor_msgs::msg::PointCloud2 scan_data_msg;
        pcl::toROSMsg(*cloudInDwz, scan_data_msg);
        scan_data_msg.header.stamp = scanIn_->header.stamp;
        scan_data_msg.header.frame_id = frame_id;

        reg_pub->publish(scan_data_msg);
    }
}
