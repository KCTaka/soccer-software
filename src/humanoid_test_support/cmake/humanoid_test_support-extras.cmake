# Run by find_package(humanoid_test_support), in the scope of the package that calls it.

# humanoid_test_robots_dir(<target>)
#
# Hands <target> the directory humanoid_description installs the generated robots in, as the
# compile definition HUMANOID_TEST_ROBOTS_DIR that humanoid_test_support/generated_robots.hpp
# reads. The path is fixed at compile time, so a test never reads the environment and never skips
# (docs/standards/CPP_GUIDELINES.md §4 and §8).
function(humanoid_test_robots_dir target)
  find_package(humanoid_description REQUIRED)
  # humanoid_description_DIR is <prefix>/share/humanoid_description/cmake.
  cmake_path(SET robots_dir NORMALIZE "${humanoid_description_DIR}/../robots")
  target_compile_definitions(${target} PRIVATE HUMANOID_TEST_ROBOTS_DIR="${robots_dir}")
endfunction()

# humanoid_generated_robots(<var> [WITH <file>])
#
# Sets <var> to the names of the robots humanoid_description installs, sorted, keeping only those
# with <file> under their directory when WITH is given (WITH mjcf/scene.xml: the robots SIL can
# simulate). For a test that must be built once per robot, such as one that hands the robot to
# rclcpp::init(), which a process can do only once at affordable cost; a test that can iterate
# the robots at run time uses generated_robots() instead. The list is re-read on every build, so
# a newly generated robot is picked up without a manual reconfigure. No robot is a configure
# error, never an empty, silently passing set of tests.
function(humanoid_generated_robots var)
  cmake_parse_arguments(arg "" "WITH" "" ${ARGN})
  find_package(humanoid_description REQUIRED)
  cmake_path(SET robots_dir NORMALIZE "${humanoid_description_DIR}/../robots")
  file(GLOB entries LIST_DIRECTORIES true CONFIGURE_DEPENDS "${robots_dir}/*")
  set(robots)
  foreach(entry IN LISTS entries)
    if(IS_DIRECTORY "${entry}" AND (NOT arg_WITH OR EXISTS "${entry}/${arg_WITH}"))
      cmake_path(GET entry FILENAME robot)
      list(APPEND robots "${robot}")
    endif()
  endforeach()
  if(NOT robots)
    message(FATAL_ERROR "no generated robot under ${robots_dir} has '${arg_WITH}'")
  endif()
  list(SORT robots)
  set(${var} "${robots}" PARENT_SCOPE)
endfunction()
