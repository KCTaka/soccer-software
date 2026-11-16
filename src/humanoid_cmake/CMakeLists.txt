cmake_minimum_required(VERSION 3.22)
project(humanoid_cmake VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

# find_package(humanoid_cmake) includes the extras, which load these from the same directory.
install(
  FILES cmake/humanoid_set_compiler_options.cmake cmake/Findmujoco.cmake
  DESTINATION share/${PROJECT_NAME}/cmake)

ament_package(CONFIG_EXTRAS cmake/humanoid_cmake-extras.cmake)
