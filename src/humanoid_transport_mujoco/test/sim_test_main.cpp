// Copyright 2026 UTRA-RoboSoccer
// The main of the conformance and simulator test binary: the transport with its default
// parameters, for this binary's robot.

#include "sim_fixture.hpp"

int main(int argc, char ** argv)
{
  return humanoid::transport_mujoco::testing::run_tests(argc, argv, {});
}
