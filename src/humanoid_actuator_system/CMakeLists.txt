cmake_minimum_required(VERSION 3.22)
project(humanoid_actuator_system VERSION 0.1.0 LANGUAGES CXX)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_index_cpp
  hardware_interface
  humanoid_safety
  humanoid_transport
  pluginlib
  rclcpp
  rclcpp_lifecycle
  yaml-cpp
)

find_package(ament_cmake REQUIRED)
find_package(humanoid_cmake REQUIRED)
humanoid_set_compiler_options()
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/drive_layout.cpp
  src/four_bar_linkage.cpp
  src/hardware_contract.cpp
  src/humanoid_actuator_system.cpp
  src/safety_manifest_parser.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# Namespaced targets, matching what ros2_control 4.47.0 itself links against.
target_link_libraries(${PROJECT_NAME} PUBLIC
  ament_index_cpp::ament_index_cpp
  hardware_interface::hardware_interface
  humanoid_safety::humanoid_safety
  humanoid_transport::humanoid_transport
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  yaml-cpp)

pluginlib_export_plugin_description_file(hardware_interface humanoid_actuator_system_plugin.xml)

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

  # The component's pure rules, on hand-built inputs.
  foreach(name IN ITEMS feedback_validation tuple_ownership hardware_contract safety_manifest_parser)
    ament_add_gmock(${PROJECT_NAME}_${name}_test test/${name}_test.cpp)
    target_link_libraries(${PROJECT_NAME}_${name}_test
      ${PROJECT_NAME} humanoid_test_support::humanoid_test_support)
  endforeach()

  # Against every robot humanoid_description installs, the files the component loads at launch,
  # so an emitter change this package does not accept fails here rather than there. The robots
  # directory is a compile definition (humanoid_test_robots_dir), never the environment.
  #   four_bar_mujoco  each linkage against the generated MJCF SIL simulates
  find_package(mujoco REQUIRED)  # humanoid_cmake's Findmujoco.cmake
  foreach(name IN ITEMS generated_description drive_layout four_bar_linkage four_bar_mujoco)
    ament_add_gmock(${PROJECT_NAME}_${name}_test test/${name}_test.cpp)
    target_link_libraries(${PROJECT_NAME}_${name}_test
      ${PROJECT_NAME} humanoid_test_support::humanoid_test_support)
    humanoid_test_robots_dir(${PROJECT_NAME}_${name}_test)
  endforeach()
  target_link_libraries(${PROJECT_NAME}_four_bar_mujoco_test mujoco::mujoco)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
