// Copyright 2026 UTRA-RoboSoccer
// The generated robots' descriptions as this component loads them, and every linkage they hold,
// for the tests that check them once per robot or once per linkage rather than naming one.
#ifndef GENERATED_DESCRIPTION_HPP_
#define GENERATED_DESCRIPTION_HPP_

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "humanoid_actuator_system/drive_layout.hpp"
#include "humanoid_actuator_system/four_bar_linkage.hpp"
#include "humanoid_test_support/generated_robots.hpp"

namespace humanoid::actuator_system::testing
{

/// The single <ros2_control> block of the URDF at `path`, as ros2_control_node parses it, or
/// nothing if the file does not hold exactly one.
inline std::optional<hardware_interface::HardwareInfo> parse_description(
  const std::filesystem::path & path)
{
  std::ifstream file(path);
  std::stringstream urdf;
  urdf << file.rdbuf();
  const auto components = hardware_interface::parse_control_resources_from_urdf(urdf.str());
  if (components.size() != 1U) {
    return std::nullopt;
  }
  return components.front();
}

/// A <transmission>'s four-bar parameters, read here independently of drive_layout().
inline FourBarParameters four_bar_parameters(const hardware_interface::TransmissionInfo & t)
{
  const auto real = [&t](const char * name) {return std::stod(t.parameters.at(name));};
  const auto sign = [&t](const char * name) {return std::stoi(t.parameters.at(name));};
  FourBarParameters p;
  p.ground_length_m = real("ground_length_m");
  p.input_crank_length_m = real("input_crank_length_m");
  p.coupler_length_m = real("coupler_length_m");
  p.output_crank_length_m = real("output_crank_length_m");
  p.input_zero_angle_rad = real("input_zero_angle_rad");
  p.output_zero_angle_rad = real("output_zero_angle_rad");
  p.coupler_zero_angle_rad = real("coupler_zero_angle_rad");
  p.input_axis_sign = sign("input_axis_sign");
  p.output_axis_sign = sign("output_axis_sign");
  p.coupler_axis_sign = sign("coupler_axis_sign");
  return p;
}

/// One linkage of one generated robot, named by its <transmission>.
struct GeneratedLinkage
{
  test_support::GeneratedRobot robot;
  std::string name;
};

inline std::ostream & operator<<(std::ostream & out, const GeneratedLinkage & linkage)
{
  return out << linkage.robot.name << "/" << linkage.name;
}

/// gtest's instance name: robot and linkage.
struct LinkageName
{
  template<typename ParamInfo>
  std::string operator()(const ParamInfo & info) const
  {
    return info.param.robot.name + "_" + info.param.name;
  }
};

/// Every linkage of the generated robots `keep` accepts, such as those with an MJCF.
template<typename Predicate>
std::vector<GeneratedLinkage> generated_linkages_where(Predicate keep)
{
  std::vector<GeneratedLinkage> linkages;
  for (const auto & robot : test_support::generated_robots_where(keep)) {
    if (const auto info = parse_description(robot.urdf())) {
      for (const auto & transmission : info->transmissions) {
        linkages.push_back({robot, transmission.name});
      }
    }
  }
  return linkages;
}

/// Every linkage of every generated robot.
inline std::vector<GeneratedLinkage> generated_linkages()
{
  return generated_linkages_where([](const test_support::GeneratedRobot &) {return true;});
}

/// A linkage as the component builds it: its transmission, its output joint's limits, and the
/// FourBarLinkage drive_layout() made.
struct LoadedLinkage
{
  hardware_interface::HardwareInfo info;
  DriveLayout layout;
  hardware_interface::TransmissionInfo transmission;
  /// Index into layout.command_joints and layout.linkages of the linkage's output.
  std::size_t output;
  FourBarParameters parameters;
  double lower;
  double upper;

  [[nodiscard]] const FourBarLinkage & linkage() const {return layout.linkages[output]->linkage;}
  [[nodiscard]] const std::string & output_joint() const {return layout.command_joints[output];}
  [[nodiscard]] const std::string & input_joint() const
  {
    return layout.passive_joints[layout.linkages[output]->input_state];
  }
  [[nodiscard]] const std::string & coupler_joint() const
  {
    return layout.passive_joints[layout.linkages[output]->coupler_state];
  }
};

/// `linkage` loaded from its robot's description, or why it could not be.
inline std::variant<LoadedLinkage, std::string> load(const GeneratedLinkage & linkage)
{
  auto info = parse_description(linkage.robot.urdf());
  if (!info) {
    return "no single <ros2_control> block in " + linkage.robot.urdf().string();
  }
  auto layout = drive_layout(*info);
  if (const auto * error = std::get_if<std::string>(&layout)) {
    return *error;
  }
  auto & drives = std::get<DriveLayout>(layout);
  for (const auto & transmission : info->transmissions) {
    if (transmission.name != linkage.name) {
      continue;
    }
    const auto & output = transmission.joints.front().name;
    for (std::size_t i = 0; i < drives.command_joints.size(); ++i) {
      if (drives.command_joints[i] == output && drives.linkages[i]) {
        const auto & limits = info->limits.at(output);
        return LoadedLinkage{
          *info, drives, transmission, i, four_bar_parameters(transmission), limits.min_position,
          limits.max_position};
      }
    }
  }
  return "no linkage '" + linkage.name + "' drives a command joint";
}

/// `count` evenly spaced samples of [lower, upper], both ends included.
inline std::vector<double> samples(double lower, double upper, int count = 81)
{
  std::vector<double> out;
  for (int k = 0; k < count; ++k) {
    out.push_back(lower + (upper - lower) * k / (count - 1));
  }
  return out;
}

}  // namespace humanoid::actuator_system::testing

#endif  // GENERATED_DESCRIPTION_HPP_
