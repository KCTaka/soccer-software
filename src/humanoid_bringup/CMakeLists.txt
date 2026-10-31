cmake_minimum_required(VERSION 3.22)
project(humanoid_bringup VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch policies
  DESTINATION share/${PROJECT_NAME})

install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/unitree_g1/generated/robot.urdf"
    DESTINATION share/${PROJECT_NAME}/config
    RENAME robot_description.urdf)

install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/unitree_g1/generated/safety_manifest.yaml"
    DESTINATION share/${PROJECT_NAME}/config)

# Bench dial board (bench_2s_10m): generated description for the slider/real-bench launch, plus the
# generated transport wiring and safety manifest the real-hardware mode loads. controllers.yaml
# ships with the install(DIRECTORY config ...) above.
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/bench_2s_10m/generated/robot.urdf"
    DESTINATION share/${PROJECT_NAME}/config
    RENAME bench_2s_10m_description.urdf)
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/bench_2s_10m/generated/stm32_wiring.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/bench_2s_10m/generated/safety_manifest.yaml"
    DESTINATION share/${PROJECT_NAME}/config/bench_2s_10m)

# Bench dial board (bench_1s_5m): generated description for RViz/mock-sim, plus the generated
# transport wiring and safety manifest the real-hardware modes load. controllers.yaml ships with
# the install(DIRECTORY config ...) above.
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/bench_1s_5m/generated/robot.urdf"
    DESTINATION share/${PROJECT_NAME}/config
    RENAME bench_1s_5m_description.urdf)
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/bench_1s_5m/generated/stm32_wiring.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/bench_1s_5m/generated/safety_manifest.yaml"
    DESTINATION share/${PROJECT_NAME}/config/bench_1s_5m)

# SIL model. The MJCF locates its meshes through a relative meshdir
# (../source/menagerie/unitree_g1/assets), so both are installed with the
# same relative layout under share/${PROJECT_NAME}/model.
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/unitree_g1/generated/robot.mjcf"
    DESTINATION share/${PROJECT_NAME}/model/robots/unitree_g1/generated)
install(DIRECTORY
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/unitree_g1/source/menagerie/unitree_g1/assets"
    DESTINATION share/${PROJECT_NAME}/model/robots/unitree_g1/source/menagerie/unitree_g1)

install(PROGRAMS
  nodes/trajectory_player.py
  nodes/sil_stand_verifier.py
  nodes/bench_player.py
  nodes/bench_trajectories_core.py
  nodes/walk_policy.py
  nodes/walk_policy_core.py
  DESTINATION lib/${PROJECT_NAME})

# The reference-trajectory node, installed without the .py so it runs as
# `ros2 run humanoid_bringup bench_trajectories`. It imports bench_trajectories_core (installed
# beside it above) at runtime.
install(PROGRAMS nodes/bench_trajectories.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bench_trajectories)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(walk_policy_core_test test/test_walk_policy_core.py)
  ament_add_pytest_test(bench_trajectories_core_test test/test_bench_trajectories_core.py)
endif()

ament_package()
