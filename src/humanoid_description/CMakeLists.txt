cmake_minimum_required(VERSION 3.22)
project(humanoid_description VERSION 0.1.0 LANGUAGES NONE)

find_package(ament_cmake REQUIRED)

# robots/<robot>/ is wholly generated (python3 -m model.generators.cli generate --robot <robot>)
# and installs as it is: the URDF names its meshes as package://humanoid_description/robots/...,
# and the MJCF finds them through a meshdir relative to its own directory.
install(DIRECTORY robots DESTINATION share/${PROJECT_NAME})

ament_package()
