cmake_minimum_required(VERSION 3.8)
project(fitplane)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Plane.msg"
  "msg/PlaneMap.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(Traversibility_mapping
  src/FitPlane.cpp
  src/World.cpp
  src/plane.cpp
)

# 为你的目标可执行文件添加针对 ARM/Eigen 的修复宏 (-D 后缀在 CMake 中转换为定义)
target_compile_definitions(Traversibility_mapping PRIVATE
  EIGEN_DONT_VECTORIZE
  EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT
)

target_include_directories(Traversibility_mapping PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(Traversibility_mapping
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  PCL
  pcl_conversions
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(Traversibility_mapping ${PCL_LIBRARIES})

install(TARGETS
  Traversibility_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
