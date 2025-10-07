cmake_minimum_required(VERSION 3.8)
project(sensor_conversion)

# 使用 C++17
add_compile_options(-std=c++17)

# 查找 ROS 2 组件
find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 头文件目录
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

# 创建库
add_library(slam_output SHARED src/slam_output.cpp)
ament_target_dependencies(slam_output 
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    pcl_ros
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_filters
)

# 创建可执行节点
add_executable(slam_sim_output_node src/slam_output_node.cpp)
ament_target_dependencies(slam_sim_output_node 
    rclcpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    pcl_ros
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    message_filters
)
target_link_libraries(slam_sim_output_node slam_output)

# 安装目标文件
install(TARGETS 
 slam_sim_output_node
    DESTINATION lib/${PROJECT_NAME}
)

# 使 package 可用
ament_package()
