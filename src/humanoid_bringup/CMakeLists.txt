cmake_minimum_required(VERSION 3.22)
project(humanoid_bringup VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/generated/robot.urdf"
    DESTINATION share/${PROJECT_NAME}/config
    RENAME robot_description.urdf)

install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/generated/safety_manifest.yaml"
    DESTINATION share/${PROJECT_NAME}/config)

install(PROGRAMS nodes/trajectory_player.py
    DESTINATION lib/${PROJECT_NAME})

ament_package()
