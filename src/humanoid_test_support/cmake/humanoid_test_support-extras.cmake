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
