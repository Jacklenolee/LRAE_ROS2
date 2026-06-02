cmake_minimum_required(VERSION 3.8)
project(simworld)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
