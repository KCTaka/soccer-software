cmake_minimum_required(VERSION 3.22)
project(humanoid_transport_stm32 VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wsuggest-override -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(humanoid_transport REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/joint_policy.cpp
  src/joint_wiring.cpp
  src/master_link.cpp
  src/parameter_node.cpp
  src/serial_port.cpp
  src/stm32_serial_transport.cpp
  src/wire_protocol.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  humanoid_transport::humanoid_transport
  pluginlib::pluginlib
  rclcpp::rclcpp)

# Export the plugin for the humanoid_transport base class package.
pluginlib_export_plugin_description_file(humanoid_transport stm32_transport_plugin.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_wire_protocol_test test/wire_protocol_test.cpp)
  target_link_libraries(${PROJECT_NAME}_wire_protocol_test ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_wire_protocol_test PRIVATE test)

  ament_add_gtest(${PROJECT_NAME}_joint_wiring_test test/joint_wiring_test.cpp)
  target_link_libraries(${PROJECT_NAME}_joint_wiring_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_joint_policy_test test/joint_policy_test.cpp)
  target_link_libraries(${PROJECT_NAME}_joint_policy_test ${PROJECT_NAME})

  # The fake master is a pseudo-terminal stand-in for the STM32; see test/fake_master.hpp for what
  # it does and does not prove.
  add_library(${PROJECT_NAME}_fake_master STATIC test/fake_master.cpp)
  target_include_directories(${PROJECT_NAME}_fake_master PUBLIC test)
  target_link_libraries(${PROJECT_NAME}_fake_master PUBLIC ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_master_link_test test/master_link_test.cpp)
  target_link_libraries(${PROJECT_NAME}_master_link_test ${PROJECT_NAME}_fake_master)

  ament_add_gtest(${PROJECT_NAME}_transport_test test/stm32_transport_test.cpp)
  target_link_libraries(${PROJECT_NAME}_transport_test ${PROJECT_NAME}_fake_master)

  # Custom main: configure() declares ROS parameters on the transport's own node, which needs
  # rclcpp::init(), and the overrides are passed as --ros-args like ros2_control_node passes them.
  ament_add_gtest(${PROJECT_NAME}_parameters_test
    test/stm32_parameters_test.cpp
    test/stm32_parameters_test_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(${PROJECT_NAME}_parameters_test
    ${PROJECT_NAME}_fake_master rclcpp::rclcpp)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(humanoid_transport pluginlib rclcpp)
ament_package()
