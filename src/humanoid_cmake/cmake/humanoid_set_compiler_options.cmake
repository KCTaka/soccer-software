# humanoid_set_compiler_options()
#
# The workspace's C++ policy, defined once: C++20 without compiler extensions, and every warning
# an error, including -Wsuggest-override (docs/standards/CPP_GUIDELINES.md §1). Call it right after
# project(). It is a macro, not a function, so the settings land in the calling package's directory
# scope and reach every target the package defines, its tests included. A CMAKE_CXX_STANDARD given
# on the command line still wins.
macro(humanoid_set_compiler_options)
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
  endif()
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)

  if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
    add_compile_options(
      -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wsuggest-override -Werror)
  endif()
endmacro()
