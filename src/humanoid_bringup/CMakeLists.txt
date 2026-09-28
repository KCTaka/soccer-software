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

# SIL model. The MJCF locates its meshes through a relative meshdir
# (../source/menagerie/unitree_g1/assets), so both are installed with the
# same relative layout under share/${PROJECT_NAME}/model.
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/generated/robot.mjcf"
    DESTINATION share/${PROJECT_NAME}/model/generated)
install(DIRECTORY
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/source/menagerie/unitree_g1/assets"
    DESTINATION share/${PROJECT_NAME}/model/source/menagerie/unitree_g1)

install(PROGRAMS
  nodes/trajectory_player.py
  nodes/sil_stand_verifier.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
