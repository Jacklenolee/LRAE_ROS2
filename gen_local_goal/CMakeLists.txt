cmake_minimum_required(VERSION 3.8)
project(gen_local_goal)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(gen_local_goal_node src/gen_local_goal_node.cpp src/gen_local_goal.cpp)
target_compile_definitions(gen_local_goal_node PRIVATE
  EIGEN_DONT_VECTORIZE
  EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
)
target_include_directories(gen_local_goal_node PRIVATE include)
ament_target_dependencies(
  gen_local_goal_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
)
target_link_libraries(gen_local_goal_node Eigen3::Eigen)

install(TARGETS
  gen_local_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
