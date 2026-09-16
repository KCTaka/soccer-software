cmake_minimum_required(VERSION 3.22)
project(humanoid_bringup VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/../../model/generated/robot.urdf"
  DESTINATION share/${PROJECT_NAME}/config
  RENAME robot_description.urdf)

ament_package()
