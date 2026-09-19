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

# MuJoCo: try CMake config first (MuJoCo 3.x ships one), fall back to find_library.
find_package(mujoco QUIET)
if(NOT mujoco_FOUND)
  find_library(MUJOCO_LIBRARY NAMES mujoco REQUIRED)
  find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h REQUIRED)
  if(NOT MUJOCO_LIBRARY OR NOT MUJOCO_INCLUDE_DIR)
  message(FATAL_ERROR
      "MuJoCo not found. Install to /opt/mujoco-3.4.0 and run: "
      "echo '/opt/mujoco-3.4.0/lib' > /etc/ld.so.conf.d/mujoco.conf && ldconfig")
  endif()
  add_library(mujoco::mujoco UNKNOWN IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}")
endif()

add_library(${PROJECT_NAME} SHARED
  src/mujoco_actuator_transport.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${PROJECT_NAME} PUBLIC
  humanoid_transport::humanoid_transport
  pluginlib::pluginlib
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
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(${PROJECT_NAME}_conformance_test test/conformance_test.cpp)
  target_link_libraries(${PROJECT_NAME}_conformance_test ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(humanoid_transport pluginlib)
ament_package()