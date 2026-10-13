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
  src/joint_policy.cpp
  src/joint_wiring.cpp
  src/link_probe.cpp
  src/master_link.cpp
  src/parameter_node.cpp
  src/serial_port.cpp
  src/stm32_serial_transport.cpp
  src/wire_protocol.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
# generated/ holds the wire code vendored from a pinned soccer-firmware tag
# (generated/vendor.sh + FIRMWARE_PIN). wire_protocol.hpp includes it as
# "robot_wire_gen.hpp"; it is installed flat into the package include root so the
# same spelling resolves in the install tree.
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/generated>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC
  humanoid_transport::humanoid_transport
  pluginlib::pluginlib
  rclcpp::rclcpp)

# Export the plugin for the humanoid_transport base class package.
pluginlib_export_plugin_description_file(humanoid_transport stm32_transport_plugin.xml)

# Read-only first-contact check for real hardware: ros2 run humanoid_transport_stm32 stm32_link_probe
add_executable(stm32_link_probe src/link_probe_main.cpp)
target_link_libraries(stm32_link_probe ${PROJECT_NAME})
install(TARGETS stm32_link_probe RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
# The vendored generated header ships flat in the include root (matching the
# "robot_wire_gen.hpp" include spelling); protocol.yaml ships as runtime data for
# the hardware interface's self-describing log.
install(FILES generated/robot_wire_gen.hpp DESTINATION include/${PROJECT_NAME})
install(FILES generated/protocol.yaml DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # generated/ is vendored verbatim from a pinned soccer-firmware tag (firmware
  # house style, style-checked there) — exclude it from our linters.
  set(AMENT_LINT_AUTO_FILE_EXCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/generated/robot_wire_gen.hpp")
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_wire_protocol_test test/wire_protocol_test.cpp)
  target_link_libraries(${PROJECT_NAME}_wire_protocol_test ${PROJECT_NAME})
  target_include_directories(${PROJECT_NAME}_wire_protocol_test PRIVATE test)

  # The vendored wire code must match the pinned firmware release: FIRMWARE_PIN's
  # SCHEMA_HASH drives the test, so re-vendoring without bumping the pin (or the
  # reverse) fails the build. The golden fixtures prove header<->bytes agreement.
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/generated/FIRMWARE_PIN" _firmware_pin)
  string(REGEX MATCH "SCHEMA_HASH=(0x[0-9A-Fa-f]+)" _ "${_firmware_pin}")
  set(_pinned_hash "${CMAKE_MATCH_1}")
  if(NOT _pinned_hash)
    message(FATAL_ERROR "generated/FIRMWARE_PIN has no SCHEMA_HASH=0x... line")
  endif()
  ament_add_gtest(${PROJECT_NAME}_wire_pin_test test/wire_pin_test.cpp)
  target_link_libraries(${PROJECT_NAME}_wire_pin_test ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_wire_pin_test PRIVATE
    "WIRE_PINNED_SCHEMA_HASH=${_pinned_hash}"
    "WIRE_FIXTURE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/generated/fixtures\"")

  ament_add_gtest(${PROJECT_NAME}_joint_wiring_test test/joint_wiring_test.cpp)
  target_link_libraries(${PROJECT_NAME}_joint_wiring_test ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_joint_policy_test test/joint_policy_test.cpp)
  target_link_libraries(${PROJECT_NAME}_joint_policy_test ${PROJECT_NAME})

  # The fake master is a pseudo-terminal stand-in for the STM32; see test/fake_master.hpp for what
  # it does and does not prove.
  add_library(${PROJECT_NAME}_fake_master STATIC test/fake_master.cpp)
  target_include_directories(${PROJECT_NAME}_fake_master PUBLIC test)
  target_link_libraries(${PROJECT_NAME}_fake_master PUBLIC ${PROJECT_NAME})

  ament_add_gtest(${PROJECT_NAME}_master_link_test test/master_link_test.cpp)
  target_link_libraries(${PROJECT_NAME}_master_link_test ${PROJECT_NAME}_fake_master)

  ament_add_gtest(${PROJECT_NAME}_link_probe_test test/link_probe_test.cpp)
  target_link_libraries(${PROJECT_NAME}_link_probe_test ${PROJECT_NAME}_fake_master)

  ament_add_gtest(${PROJECT_NAME}_transport_test test/stm32_transport_test.cpp)
  target_link_libraries(${PROJECT_NAME}_transport_test ${PROJECT_NAME}_fake_master)

  # Custom main: configure() declares ROS parameters on the transport's own node, which needs
  # rclcpp::init(), and the overrides are passed as --ros-args like ros2_control_node passes them.
  ament_add_gtest(${PROJECT_NAME}_parameters_test
    test/stm32_parameters_test.cpp
    test/stm32_parameters_test_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(${PROJECT_NAME}_parameters_test
    ${PROJECT_NAME}_fake_master rclcpp::rclcpp)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(humanoid_transport pluginlib rclcpp)
ament_package()
