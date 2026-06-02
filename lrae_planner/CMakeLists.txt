cmake_minimum_required(VERSION 3.8)
project(lrae_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(ROS_DEPENDENCIES
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  visualization_msgs
  pcl_conversions
)

add_library(visual src/visiual.cpp)
add_library(utils src/utils.cpp)
add_library(path_planning src/path_planning.cpp src/graph_search.cpp src/heap.cpp)
add_library(exploration_planning src/exploration_planning.cpp)

add_executable(lrae_planner_node src/lrae_planner_node.cpp)
add_executable(exploration_map_merge src/exploration_map_merge.cpp)

# 为你的目标可执行文件添加针对 ARM/Eigen 的修复宏 (-D 后缀在 CMake 中转换为定义)
target_compile_definitions(lrae_planner_node PRIVATE
  EIGEN_DONT_VECTORIZE
  EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
)
# 为你的目标可执行文件添加针对 ARM/Eigen 的修复宏 (-D 后缀在 CMake 中转换为定义)
target_compile_definitions(exploration_map_merge PRIVATE
  EIGEN_DONT_VECTORIZE
  EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
)

foreach(target
  visual
  utils
  path_planning
  exploration_planning
  lrae_planner_node
  exploration_map_merge
)
  target_include_directories(${target} PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
  )
  ament_target_dependencies(${target} ${ROS_DEPENDENCIES})
  target_link_libraries(${target}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
  )
endforeach()

target_link_libraries(exploration_planning
  utils
  visual
  path_planning
)
target_link_libraries(lrae_planner_node exploration_planning)
target_link_libraries(exploration_map_merge exploration_planning)

install(TARGETS
  lrae_planner_node
  exploration_map_merge
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)
 
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
