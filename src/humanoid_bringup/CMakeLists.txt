cmake_minimum_required(VERSION 3.22)
project(humanoid_bringup VERSION 0.1.0 LANGUAGES NONE)

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
  # One pytest run over test/, which collects every test_*.py and reports each test case.
  ament_add_pytest_test(${PROJECT_NAME}_pytest test)
endif()

ament_package()
