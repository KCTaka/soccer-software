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
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(humanoid_transport pluginlib rclcpp)
ament_package()
