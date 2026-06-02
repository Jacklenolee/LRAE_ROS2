cmake_minimum_required(VERSION 3.5)
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()
project(sensor_conversion)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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
find_package(PCL REQUIRED)

set(ROS_DEPENDENCIES
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)

add_library(slam_output src/slam_output.cpp)
target_include_directories(slam_output PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(slam_output ${ROS_DEPENDENCIES})
target_link_libraries(slam_output ${PCL_LIBRARIES})

add_executable(slam_sim_output_node src/slam_output_node.cpp)
target_include_directories(slam_sim_output_node PRIVATE include)
ament_target_dependencies(slam_sim_output_node ${ROS_DEPENDENCIES})
target_link_libraries(slam_sim_output_node slam_output)

add_executable(map_generator_node src/map_generator_node.cpp)
target_include_directories(map_generator_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
target_compile_definitions(slam_sim_output_node PRIVATE
  EIGEN_DONT_VECTORIZE
  EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
)

ament_target_dependencies(map_generator_node ${ROS_DEPENDENCIES})
target_link_libraries(map_generator_node ${PCL_LIBRARIES})

install(
  TARGETS slam_output
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS slam_sim_output_node map_generator_node
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
