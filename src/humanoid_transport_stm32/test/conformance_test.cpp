// Copyright 2026 UTRA-RoboSoccer
// The ADR-001 transport conformance suite against Stm32SerialTransport, behind a fake master on a
// pseudo-terminal running the three-joint rig. Its mirrored, offset joint puts every sign and
// offset the wire mapping applies under the suite's unit and sign checks.

#include <memory>

#include "fake_master.hpp"
#include "fake_rig.hpp"
#include "humanoid_test_support/transport_conformance.hpp"
#include "humanoid_transport_stm32/stm32_serial_transport.hpp"

namespace
{

using humanoid::test_support::ConformanceHarness;
using humanoid::test_support::TransportConformance;
using humanoid::test_support::TransportName;
using humanoid::test_support::TransportUnderTest;
namespace rig = humanoid::transport_stm32::testing;

class FakeMasterHarness : public ConformanceHarness
{
public:
  FakeMasterHarness()
  : fake_(rig::rig_master()), params_(rig::rig_parameters(fake_.device()))
  {
    fake_.start();
  }

  humanoid::transport::ActuatorTransport & transport() override {return transport_;}

  bool configure() override {return transport_.configure_with(params_, manifest_, safety_);}

  const humanoid::transport::JointManifest & joint_manifest() const override {return manifest_;}

private:
  // Declared first, destroyed last: the transport releases the motors through it on destruction.
  rig::FakeMaster fake_;
  humanoid::transport_stm32::Stm32SerialTransport transport_;
  humanoid::transport_stm32::Stm32SerialTransport::Parameters params_;
  humanoid::transport::JointManifest manifest_ = rig::rig_manifest();
  humanoid::transport::SafetyManifest safety_ = rig::rig_safety();
};

std::unique_ptr<ConformanceHarness> make_harness() {return std::make_unique<FakeMasterHarness>();}

INSTANTIATE_TEST_SUITE_P(
  Stm32, TransportConformance,
  ::testing::Values(TransportUnderTest{"FakeMaster", make_harness}), TransportName{});

}  // namespace
