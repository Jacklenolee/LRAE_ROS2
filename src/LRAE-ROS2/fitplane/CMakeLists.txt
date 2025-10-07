cmake_minimum_required(VERSION 3.5)
project(fitplane)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# 找到 ROS2 依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)

# 找到自定义消息定义的路径
find_package(rosidl_default_generators REQUIRED)

# 消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Plane.msg"
  "msg/PlaneMap.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# 将 include 目录添加到编译路径
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# 创建可执行文件
add_executable(Traversibility_mapping
  src/FitPlane.cpp
  src/World.cpp
  src/plane.cpp
)

# 链接库
ament_target_dependencies(Traversibility_mapping
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  PCL
  pcl_conversions
  tf2_ros
)

# 安装目标文件
install(TARGETS
  Traversibility_mapping
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/
)
# 包的安装
ament_package()

