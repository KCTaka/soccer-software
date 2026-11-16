# Run by find_package(humanoid_transport_mujoco), before its exported targets are imported. The
# library links mujoco::mujoco, an imported target MuJoCo's install does not provide, so a
# consumer must define it the way this package's own build did.
find_package(humanoid_cmake REQUIRED)
find_package(mujoco REQUIRED)
