// Copyright 2026 UTRA-RoboSoccer
// The ADR-001 transport conformance suite against MujocoActuatorTransport, over every joint this
// binary's robot's generated MJCF actuates; CMake builds the binary once per robot SIL can
// simulate. A simulator transport is not exempt from the suite (ADR-007).

#include <memory>

#include "humanoid_test_support/transport_conformance.hpp"
#include "sim_fixture.hpp"

namespace
{

using humanoid::test_support::ConformanceHarness;
using humanoid::test_support::TransportConformance;
using humanoid::test_support::TransportName;
using humanoid::test_support::TransportUnderTest;
using humanoid::transport_mujoco::testing::SimulatedRobot;
using humanoid::transport_mujoco::testing::this_robot;

class SimulatorHarness : public ConformanceHarness
{
public:
  humanoid::transport::ActuatorTransport & transport() override {return sim_.transport;}

  bool configure() override {return sim_.transport.configure(sim_.manifest, sim_.safety);}

  const humanoid::transport::JointManifest & joint_manifest() const override
  {
    return sim_.manifest;
  }

private:
  SimulatedRobot sim_;
};

std::unique_ptr<ConformanceHarness> make_harness() {return std::make_unique<SimulatorHarness>();}

INSTANTIATE_TEST_SUITE_P(
  Mujoco, TransportConformance,
  ::testing::Values(TransportUnderTest{this_robot().name, make_harness}), TransportName{});

}  // namespace
