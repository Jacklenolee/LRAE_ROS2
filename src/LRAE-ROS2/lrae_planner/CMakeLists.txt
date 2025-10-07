cmake_minimum_required(VERSION 3.5)
project(lrae_planner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置构建类型
set(CMAKE_BUILD_TYPE Release)

set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# 查找ament和ROS 2的依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories( 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
  "${PROJECT_SOURCE_DIR}/include"
  ${ament_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  /opt/ros/humble/include
)

# 添加库
add_library(visual src/visiual.cpp)
ament_target_dependencies(visual rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs visualization_msgs pcl_conversions)
target_link_libraries(visual ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES})  # 根据需要调整链接的库
 
add_library(utils src/utils.cpp)
ament_target_dependencies(utils rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs Eigen3 nav_msgs visualization_msgs pcl_conversions)
target_link_libraries(utils ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES}) 

add_library(path_planning src/path_planning.cpp src/graph_search.cpp src/heap.cpp)
ament_target_dependencies(path_planning rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs Eigen3 nav_msgs visualization_msgs pcl_conversions)
target_link_libraries(path_planning ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES}) 

add_library(exploration_planning src/exploration_planning.cpp)
ament_target_dependencies(exploration_planning rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs Eigen3 nav_msgs visualization_msgs pcl_conversions   )
target_link_libraries(exploration_planning ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES} utils visual path_planning)
# 添加可执行文件
add_executable(lrae_planner_node src/lrae_planner_node.cpp)
ament_target_dependencies(lrae_planner_node rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs Eigen3 nav_msgs visualization_msgs pcl_conversions )
target_link_libraries(lrae_planner_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES} exploration_planning )

add_executable(exploration_map_merge src/exploration_map_merge.cpp)
ament_target_dependencies(exploration_map_merge rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs Eigen3 nav_msgs visualization_msgs pcl_conversions )
target_link_libraries(exploration_map_merge ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBRARIES} exploration_planning )

# 安装目标
install(TARGETS
  lrae_planner_node
  exploration_map_merge
  DESTINATION lib/${PROJECT_NAME})
 
# 生成ament包文件
ament_package()