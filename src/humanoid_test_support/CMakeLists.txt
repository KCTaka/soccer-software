cmake_minimum_required(VERSION 3.22)
project(humanoid_test_support VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(humanoid_cmake REQUIRED)
humanoid_set_compiler_options()
find_package(humanoid_transport REQUIRED)

# Header-only, and it links neither gtest nor gmock: each test builds those from gtest_vendor
# through ament_add_gtest/ament_add_gmock, and these headers compile against the one it built.
add_library(${PROJECT_NAME} INTERFACE)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_20)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} INTERFACE humanoid_transport::humanoid_transport)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The extras define humanoid_test_robots_dir(); a consumer gets it from find_package.
  include(cmake/humanoid_test_support-extras.cmake)

  ament_add_gmock(${PROJECT_NAME}_test test/test_support_test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  humanoid_test_robots_dir(${PROJECT_NAME}_test)
endif()

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(humanoid_transport)
ament_package(CONFIG_EXTRAS cmake/humanoid_test_support-extras.cmake)
