cmake_minimum_required(VERSION 3.22)
project(humanoid_actuator_system VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wsuggest-override -Werror)
endif()

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
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_feedback_validation_test test/feedback_validation_test.cpp)
  target_include_directories(${PROJECT_NAME}_feedback_validation_test PRIVATE include)
  target_link_libraries(${PROJECT_NAME}_feedback_validation_test
    humanoid_transport::humanoid_transport)

  ament_add_gtest(${PROJECT_NAME}_tuple_ownership_test test/tuple_ownership_test.cpp)
  target_include_directories(${PROJECT_NAME}_tuple_ownership_test PRIVATE include)
  target_link_libraries(${PROJECT_NAME}_tuple_ownership_test
    humanoid_transport::humanoid_transport)

  # These read the generated artifacts in place, so an emitter change that this package does not
  # accept fails here rather than at launch.
  set(GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/unitree_g1/generated")
  set(LEG_GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../model/robots/leg_prototype/generated")

  ament_add_gtest(${PROJECT_NAME}_safety_manifest_parser_test
    test/safety_manifest_parser_test.cpp)
  target_link_libraries(${PROJECT_NAME}_safety_manifest_parser_test ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_safety_manifest_parser_test PRIVATE
    HUMANOID_TEST_SAFETY_MANIFEST_PATH="${GENERATED_DIR}/safety_manifest.yaml")

  ament_add_gtest(${PROJECT_NAME}_hardware_contract_test test/hardware_contract_test.cpp)
  target_link_libraries(${PROJECT_NAME}_hardware_contract_test ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_hardware_contract_test PRIVATE
    HUMANOID_TEST_ROBOT_URDF_PATH="${GENERATED_DIR}/robot.urdf"
    HUMANOID_TEST_LEG_URDF_PATH="${LEG_GENERATED_DIR}/robot.urdf")

  # The leg prototype is the robot with a linkage: its generated description, manifest and MJCF.
  ament_add_gtest(${PROJECT_NAME}_four_bar_linkage_test test/four_bar_linkage_test.cpp)
  target_link_libraries(${PROJECT_NAME}_four_bar_linkage_test ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_four_bar_linkage_test PRIVATE
    HUMANOID_TEST_LEG_URDF_PATH="${LEG_GENERATED_DIR}/robot.urdf")

  ament_add_gtest(${PROJECT_NAME}_drive_layout_test test/drive_layout_test.cpp)
  target_link_libraries(${PROJECT_NAME}_drive_layout_test ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_drive_layout_test PRIVATE
    HUMANOID_TEST_ROBOT_URDF_PATH="${GENERATED_DIR}/robot.urdf"
    HUMANOID_TEST_LEG_URDF_PATH="${LEG_GENERATED_DIR}/robot.urdf"
    HUMANOID_TEST_LEG_SAFETY_MANIFEST_PATH="${LEG_GENERATED_DIR}/safety_manifest.yaml")

  # The component's linkage against the plant SIL simulates. MuJoCo as humanoid_transport_mujoco
  # finds it: its CMake config, else the library and header.
  find_package(mujoco QUIET)
  if(NOT mujoco_FOUND)
    find_library(MUJOCO_LIBRARY NAMES mujoco REQUIRED)
    find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h REQUIRED)
    add_library(mujoco::mujoco UNKNOWN IMPORTED)
    set_target_properties(mujoco::mujoco PROPERTIES
      IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
  endif()
  ament_add_gtest(${PROJECT_NAME}_four_bar_mujoco_test test/four_bar_mujoco_test.cpp)
  target_link_libraries(${PROJECT_NAME}_four_bar_mujoco_test ${PROJECT_NAME} mujoco::mujoco)
  target_compile_definitions(${PROJECT_NAME}_four_bar_mujoco_test PRIVATE
    HUMANOID_TEST_LEG_URDF_PATH="${LEG_GENERATED_DIR}/robot.urdf"
    HUMANOID_TEST_LEG_MJCF_PATH="${LEG_GENERATED_DIR}/scene.xml")

  # ADR-007: these run against a fault-injection ActuatorTransport, never against a substituted
  # SystemInterface. mock_components::GenericSystem is prohibited in any environment that produces
  # release evidence, because adopting it deletes this class from the tested configuration.
  # ament_add_gmock(test_tuple_ownership test/test_tuple_ownership.cpp)
  # target_link_libraries(test_tuple_ownership ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
