cmake_minimum_required(VERSION 3.22)
project(humanoid_transport_mujoco VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(humanoid_transport REQUIRED)
find_package(pluginlib REQUIRED)

# Support both mujoco_vendor (CONFIG) and system-wide MuJoCo installation
find_package(mujoco CONFIG QUIET)
if(mujoco_FOUND)
  set(MUJOCO_LIBS mujoco::mujoco)
else()
  find_library(MUJOCO_LIBS mujoco REQUIRED)
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h REQUIRED)
endif()

add_library(${PROJECT_NAME} SHARED
  src/mujoco_actuator_transport.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  $<$<NOT:$<BOOL:${mujoco_FOUND}>>:${MUJOCO_INCLUDE_DIR}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  humanoid_transport::humanoid_transport
  pluginlib::pluginlib
  ${MUJOCO_LIBS}
)

pluginlib_export_plugin_description_file(humanoid_transport mujoco_transport_plugin.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(test_conformance test/conformance_test.cpp)
  target_link_libraries(test_conformance ${PROJECT_NAME})
  
  # Set the environment variable for the test to find the generated MJCF model
  set(TEST_MJCF_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../model/generated/robot.mjcf")
  get_filename_component(MUJOCO_ROOT_DIR "${MUJOCO_INCLUDE_DIR}" DIRECTORY)
  set_tests_properties(test_conformance PROPERTIES
    ENVIRONMENT "HUMANOID_MJCF_PATH=${TEST_MJCF_PATH};LD_LIBRARY_PATH=${MUJOCO_ROOT_DIR}/lib:$ENV{LD_LIBRARY_PATH}")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(humanoid_transport pluginlib)
ament_package()