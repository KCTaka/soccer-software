cmake_minimum_required(VERSION 3.22)
project(humanoid_mit_controller VERSION 0.1.0 LANGUAGES CXX)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  humanoid_transport
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
)

find_package(ament_cmake REQUIRED)
find_package(humanoid_cmake REQUIRED)
humanoid_set_compiler_options()
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/mit_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME} PUBLIC
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  humanoid_transport::humanoid_transport
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  sensor_msgs::sensor_msgs)

pluginlib_export_plugin_description_file(
  controller_interface humanoid_mit_controller_plugin.xml)

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
  find_package(humanoid_test_support REQUIRED)
  ament_add_gmock(${PROJECT_NAME}_gains_test test/gains_test.cpp)
  target_link_libraries(${PROJECT_NAME}_gains_test
    ${PROJECT_NAME} humanoid_test_support::humanoid_test_support)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
