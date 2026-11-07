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

# Leg prototype: the generated description for both modes, the safety manifest and STM32 wiring the
# hardware component and transport load, and the MJCF SIL simulates. The meshes keep their
# repo-relative layout, which both robot.xml's meshdir and the URDF's package:// URIs assume.
# controllers.yaml, sil.yaml and hil.yaml ship with the install(DIRECTORY config ...) above.
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/leg_prototype/generated/robot.urdf"
    DESTINATION share/${PROJECT_NAME}/config
    RENAME leg_prototype_description.urdf)
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/leg_prototype/generated/stm32_wiring.yaml"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/leg_prototype/generated/safety_manifest.yaml"
    DESTINATION share/${PROJECT_NAME}/config/leg_prototype)
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/leg_prototype/generated/scene.xml"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/leg_prototype/generated/robot.xml"
    DESTINATION share/${PROJECT_NAME}/model/robots/leg_prototype/generated)
install(DIRECTORY
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/leg_prototype/source/meshes"
    DESTINATION share/${PROJECT_NAME}/model/robots/leg_prototype/source)

# SIL model: scene.xml includes robot.xml from its own directory, and the MJCF
# locates its meshes through a relative meshdir (../source/menagerie/unitree_g1/assets),
# so all are installed with the same relative layout under share/${PROJECT_NAME}/model.
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/unitree_g1/generated/scene.xml"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/unitree_g1/generated/robot.xml"
    DESTINATION share/${PROJECT_NAME}/model/robots/unitree_g1/generated)
install(DIRECTORY
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/unitree_g1/source/menagerie/unitree_g1/assets"
    DESTINATION share/${PROJECT_NAME}/model/robots/unitree_g1/source/menagerie/unitree_g1)

install(PROGRAMS
  nodes/trajectory_player.py
  nodes/sil_stand_verifier.py
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
  ament_add_pytest_test(sil_stand_config_test test/test_sil_stand_config.py)
  ament_add_pytest_test(leg_config_test test/test_leg_config.py)
endif()

ament_package()
