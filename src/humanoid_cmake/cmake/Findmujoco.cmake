# Findmujoco.cmake: the MuJoCo C library, as the imported target mujoco::mujoco.
#
# MuJoCo's release archive ships no CMake package config, so this finds the header and the library
# directly. Put the install on CMAKE_PREFIX_PATH, as the dev container does with /opt/mujoco, and
# put its lib/ on the dynamic loader's path for run time.

find_path(mujoco_INCLUDE_DIR mujoco/mujoco.h)
find_library(mujoco_LIBRARY NAMES mujoco)
mark_as_advanced(mujoco_INCLUDE_DIR mujoco_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(mujoco
  REQUIRED_VARS mujoco_LIBRARY mujoco_INCLUDE_DIR
  REASON_FAILURE_MESSAGE
  "Install MuJoCo 3.4.0 to /opt/mujoco-3.4.0, add it to CMAKE_PREFIX_PATH, and run: echo '/opt/mujoco-3.4.0/lib' > /etc/ld.so.conf.d/mujoco.conf && ldconfig")

if(mujoco_FOUND AND NOT TARGET mujoco::mujoco)
  add_library(mujoco::mujoco UNKNOWN IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${mujoco_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${mujoco_INCLUDE_DIR}")
endif()
