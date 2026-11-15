cmake_minimum_required(VERSION 3.22)
project(humanoid_bringup VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch policies
  DESTINATION share/${PROJECT_NAME})

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
