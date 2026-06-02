#include "../include/slam_simulation/map_generator_node.h"

MapGeneratorNode::MapGeneratorNode() : Node("map_generator_node") {
    // 声明参数
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<std::string>("child_frame_id", "base_link");
    this->declare_parameter<double>("down_voxel_size", 0.1);
    this->declare_parameter<double>("explored_area_voxel_size", 0.1);
    
    // 获取参数
    frame_id_ = this->get_parameter("frame_id").as_string();
    child_frame_id_ = this->get_parameter("child_frame_id").as_string();
    down_voxel_size_ = this->get_parameter("down_voxel_size").as_double();
    exploredAreaVoxelSize_ = this->get_parameter("explored_area_voxel_size").as_double();
    
    // 初始化点云处理
    downSizeFilter_.setLeafSize(down_voxel_size_, down_voxel_size_, down_voxel_size_);
    exploredAreaDwzFilter_.setLeafSize(exploredAreaVoxelSize_, exploredAreaVoxelSize_, exploredAreaVoxelSize_);
    exploredAreaCloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    
    // 创建发布器
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 1);
    
    
    // 创建订阅器
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "transformed_cloud", 10, std::bind(&MapGeneratorNode::cloudCallback, this, std::placeholders::_1));

}

void MapGeneratorNode::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr scanIn) {
    // 转换点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*scanIn, *scan);
    
    // 移除NaN点
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_data = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    std::vector<int> scan_index;
    pcl::removeNaNFromPointCloud(*scan, *scan_data, scan_index);
    
    // 降采样
    downSizeFilter_.setInputCloud(scan_data);
    pcl::PointCloud<pcl::PointXYZ> scan_dwz;
    downSizeFilter_.filter(scan_dwz);

    // 累积点云
    *exploredAreaCloud_ += scan_dwz;
    
    // 对累积点云进行降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInDwz(new pcl::PointCloud<pcl::PointXYZ>());
    exploredAreaDwzFilter_.setInputCloud(exploredAreaCloud_);
    exploredAreaDwzFilter_.filter(*cloudInDwz);
    
    // 发布地图
    sensor_msgs::msg::PointCloud2 scan_data_msg;
    pcl::toROSMsg(*cloudInDwz, scan_data_msg);
    scan_data_msg.header.stamp = scanIn->header.stamp;
    scan_data_msg.header.frame_id = frame_id_;
    map_pub_->publish(scan_data_msg);
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}