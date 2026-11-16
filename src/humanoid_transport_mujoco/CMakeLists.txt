cmake_minimum_required(VERSION 3.22)
project(humanoid_transport_mujoco VERSION 0.1.0 LANGUAGES CXX)

# ADR-001: the real-time exchange() path stays ROS-free. rclcpp, geometry_msgs and tf2_ros serve
# only the parameter node and the non-real-time GroundTruthPublisher thread.
set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  humanoid_transport
  pluginlib
  rclcpp
  tf2_ros
)

find_package(ament_cmake REQUIRED)
find_package(humanoid_cmake REQUIRED)
humanoid_set_compiler_options()
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(mujoco REQUIRED)  # humanoid_cmake's Findmujoco.cmake

add_library(${PROJECT_NAME} SHARED
  src/mujoco_actuator_transport.cpp
  src/ground_truth_publisher.cpp
  src/mujoco_disturbance.cpp
  src/mujoco_imu.cpp
  src/mujoco_placement.cpp
  src/sim_node.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME} PUBLIC
  humanoid_transport::humanoid_transport
  pluginlib::pluginlib
  rclcpp::rclcpp
  geometry_msgs::geometry_msgs
  tf2_ros::tf2_ros
  mujoco::mujoco)

# Export plugin for the humanoid_transport base class package.
pluginlib_export_plugin_description_file(humanoid_transport mujoco_transport_plugin.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(humanoid_test_support REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The transport as SIL runs it, built once per robot generated with an MJCF: configure() reads
  # its parameters from rclcpp's global overrides, which a process sets once (test/sim_fixture.hpp).
  #   sim   the ADR-001 conformance suite, and what only the simulator shows
  #   hold  the SIL start state, with hold_until_commanded and an initial pose
  humanoid_generated_robots(simulated_robots WITH mjcf/scene.xml)
  foreach(robot IN LISTS simulated_robots)
    ament_add_gmock(${PROJECT_NAME}_sim_${robot}_test
      test/conformance_test.cpp test/transport_test.cpp test/sim_test_main.cpp
      SKIP_LINKING_MAIN_LIBRARIES)
    ament_add_gmock(${PROJECT_NAME}_hold_${robot}_test test/hold_test.cpp
      SKIP_LINKING_MAIN_LIBRARIES)
    foreach(target IN ITEMS ${PROJECT_NAME}_sim_${robot}_test ${PROJECT_NAME}_hold_${robot}_test)
      target_link_libraries(${target}
        ${PROJECT_NAME} humanoid_test_support::humanoid_test_support rclcpp::rclcpp)
      humanoid_test_robots_dir(${target})
      target_compile_definitions(${target} PRIVATE HUMANOID_TEST_ROBOT="${robot}")
    endforeach()
  endforeach()

  # The simulator's pure parts, on synthetic in-memory models; placement also stands every
  # generated floating robot on the ground. The robots directory is a compile definition
  # (humanoid_test_robots_dir), never the environment.
  foreach(name IN ITEMS disturbance imu placement)
    ament_add_gmock(${PROJECT_NAME}_${name}_test test/${name}_test.cpp)
    target_link_libraries(${PROJECT_NAME}_${name}_test
      ${PROJECT_NAME} humanoid_test_support::humanoid_test_support)
  endforeach()
  humanoid_test_robots_dir(${PROJECT_NAME}_placement_test)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
# The exported library links mujoco::mujoco, which MuJoCo does not export: the extras define it.
ament_package(CONFIG_EXTRAS cmake/humanoid_transport_mujoco-extras.cmake)
