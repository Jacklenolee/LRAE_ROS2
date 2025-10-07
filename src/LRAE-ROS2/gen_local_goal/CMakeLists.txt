cmake_minimum_required(VERSION 3.5)
project(gen_local_goal)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找ament_cmake宏
find_package(ament_cmake REQUIRED)

# 查找ROS 2组件
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED) # 假设您使用的是nav2_msgs而不是nav_msgs，因为nav_msgs在ROS 2中没有直接对应
find_package(tf2 REQUIRED)       # tf在ROS 2中被tf2取代
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
# 查找Eigen3
find_package(Eigen3 REQUIRED)

# 声明ament_cmake包
ament_package()

# 包含目录
include_directories(
  include
  ${ament_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(gen_local_goal_node src/gen_local_goal_node.cpp src/gen_local_goal.cpp)

# 链接库
ament_target_dependencies(
  gen_local_goal_node
  rclcpp
  nav_msgs # 或者您可能需要使用其他具体的消息包
  tf2
  std_msgs
  tf2_ros
)
install(TARGETS
  gen_local_goal_node
  DESTINATION lib/${PROJECT_NAME}
)
target_link_libraries(gen_local_goal_node ${catkin_LIBRARIES} )
