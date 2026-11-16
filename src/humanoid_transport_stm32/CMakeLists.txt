cmake_minimum_required(VERSION 3.22)
project(humanoid_transport_stm32 VERSION 0.1.0 LANGUAGES CXX)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  humanoid_transport
  pluginlib
  rclcpp
)

find_package(ament_cmake REQUIRED)
find_package(humanoid_cmake REQUIRED)
humanoid_set_compiler_options()
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

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
  find_package(ament_cmake_gmock REQUIRED)
  find_package(humanoid_test_support REQUIRED)
  # generated/ is vendored verbatim from a pinned soccer-firmware tag (firmware
  # house style, style-checked there) — exclude it from our linters: the C++ wire
  # header and the firmware_sm/ C headers the fake master's shim compiles against.
  file(GLOB _vendored_sources
    "${CMAKE_CURRENT_SOURCE_DIR}/generated/robot_wire_gen.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/generated/firmware_sm/*.h")
  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${_vendored_sources})
  ament_lint_auto_find_test_dependencies()

  # The host logic alone: the wire codec, the joint wiring and the per-phase request policy.
  foreach(name IN ITEMS wire_protocol joint_wiring joint_policy)
    ament_add_gmock(${PROJECT_NAME}_${name}_test test/${name}_test.cpp)
    target_link_libraries(${PROJECT_NAME}_${name}_test
      ${PROJECT_NAME} humanoid_test_support::humanoid_test_support)
  endforeach()

  # The generated Stm32SerialTransport joints map of every robot that has one, against the robot's
  # generated safety manifest (humanoid_test_robots_dir: a compile definition, never the
  # environment).
  find_package(yaml-cpp REQUIRED)
  ament_add_gmock(${PROJECT_NAME}_generated_wiring_test test/generated_wiring_test.cpp)
  target_link_libraries(${PROJECT_NAME}_generated_wiring_test
    ${PROJECT_NAME} humanoid_test_support::humanoid_test_support yaml-cpp)
  humanoid_test_robots_dir(${PROJECT_NAME}_generated_wiring_test)

  # The vendored wire code must match the pinned firmware release: FIRMWARE_PIN's
  # SCHEMA_HASH drives the test, so re-vendoring without bumping the pin (or the
  # reverse) fails the build. The golden fixtures prove header<->bytes agreement.
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/generated/FIRMWARE_PIN" _firmware_pin)
  string(REGEX MATCH "SCHEMA_HASH=(0x[0-9A-Fa-f]+)" _ "${_firmware_pin}")
  set(_pinned_hash "${CMAKE_MATCH_1}")
  if(NOT _pinned_hash)
    message(FATAL_ERROR "generated/FIRMWARE_PIN has no SCHEMA_HASH=0x... line")
  endif()
  ament_add_gmock(${PROJECT_NAME}_wire_pin_test test/wire_pin_test.cpp)
  target_link_libraries(${PROJECT_NAME}_wire_pin_test ${PROJECT_NAME})
  target_compile_definitions(${PROJECT_NAME}_wire_pin_test PRIVATE
    "WIRE_PINNED_SCHEMA_HASH=${_pinned_hash}"
    "WIRE_FIXTURE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/generated/fixtures\"")

  # The fake master is a pseudo-terminal stand-in for the STM32; see test/fake_master.hpp for what
  # it does and does not prove.
  # fake_master runs the firmware's real decision logic: fw_drive_sm.cpp is the isolated TU that
  # includes the vendored firmware drive_sm.h (generated/firmware_sm, PRIVATE so protocol.h's
  # macros never reach a TU that also pulls the C++ wire wrapper).
  add_library(${PROJECT_NAME}_fake_master STATIC test/fake_master.cpp test/fw_drive_sm.cpp)
  target_include_directories(${PROJECT_NAME}_fake_master PUBLIC test)
  target_include_directories(${PROJECT_NAME}_fake_master PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/generated/firmware_sm")
  target_link_libraries(${PROJECT_NAME}_fake_master PUBLIC
    ${PROJECT_NAME} humanoid_test_support::humanoid_test_support)
  # fw_drive_sm.cpp is the one TU that compiles the vendored firmware C headers (drive_sm.h,
  # motor_config.h). Those are firmware house style -- motor_config.h uses C99 designated array
  # initializers, which C++ allows only as an extension -- so this file alone drops -Wpedantic.
  set_source_files_properties(test/fw_drive_sm.cpp PROPERTIES COMPILE_OPTIONS "-Wno-pedantic")

  # Against the fake master:
  #   fw_boundary  the firmware drive-SM enum values (through fw_drive_sm) equal wire::'s
  #   fake_master  the master logic it emulates by hand: the version-gate enable refusal and the
  #                robot_state reporting priority
  #   master_link, link_probe, transport  the host side over the pseudo-terminal
  #   conformance  the ADR-001 suite every ActuatorTransport passes (humanoid_test_support)
  foreach(name IN ITEMS fw_boundary fake_master master_link link_probe transport conformance)
    ament_add_gmock(${PROJECT_NAME}_${name}_test test/${name}_test.cpp)
    target_link_libraries(${PROJECT_NAME}_${name}_test ${PROJECT_NAME}_fake_master)
  endforeach()

  # Custom main: configure() declares ROS parameters on the transport's own node, which needs
  # rclcpp::init(), and the overrides are passed as --ros-args like ros2_control_node passes them.
  ament_add_gmock(${PROJECT_NAME}_parameters_test
    test/parameters_test.cpp
    test/parameters_test_main.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(${PROJECT_NAME}_parameters_test
    ${PROJECT_NAME}_fake_master rclcpp::rclcpp)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
