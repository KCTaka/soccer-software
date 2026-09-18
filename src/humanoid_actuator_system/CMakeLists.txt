cmake_minimum_required(VERSION 3.22)
project(humanoid_actuator_system VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror)
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
  src/humanoid_actuator_system.cpp
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
install(FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../model/generated/safety_manifest.yaml"
    DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gmock REQUIRED)

  # ADR-007: these run against a fault-injection ActuatorTransport, never against a substituted
  # SystemInterface. mock_components::GenericSystem is prohibited in any environment that produces
  # release evidence, because adopting it deletes this class from the tested configuration.
  # ament_add_gmock(test_tuple_ownership test/test_tuple_ownership.cpp)
  # target_link_libraries(test_tuple_ownership ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
