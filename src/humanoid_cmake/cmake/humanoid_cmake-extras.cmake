# Run by find_package(humanoid_cmake), in the scope of the package that calls it.

include("${humanoid_cmake_DIR}/humanoid_set_compiler_options.cmake")

# Makes find_package(mujoco) use the Findmujoco.cmake installed beside this file.
list(APPEND CMAKE_MODULE_PATH "${humanoid_cmake_DIR}")
list(REMOVE_DUPLICATES CMAKE_MODULE_PATH)
