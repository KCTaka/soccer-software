// Copyright 2026 UTRA-RoboSoccer

#include "humanoid_actuator_system/drive_layout.hpp"

#include <format>

#include <algorithm>
#include <array>
#include <charconv>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <variant>
#include <vector>

namespace humanoid::actuator_system
{

namespace
{
constexpr std::string_view kInputRole = "input";
constexpr std::string_view kOutputRole = "output";
constexpr std::string_view kCouplerRole = "coupler";

// Where each of kFourBarParams goes.
constexpr std::array<std::pair<std::string_view, double FourBarParameters::*>, 7> kRealParams{{
  {"ground_length_m", &FourBarParameters::ground_length_m},
  {"input_crank_length_m", &FourBarParameters::input_crank_length_m},
  {"coupler_length_m", &FourBarParameters::coupler_length_m},
  {"output_crank_length_m", &FourBarParameters::output_crank_length_m},
  {"input_zero_angle_rad", &FourBarParameters::input_zero_angle_rad},
  {"output_zero_angle_rad", &FourBarParameters::output_zero_angle_rad},
  {"coupler_zero_angle_rad", &FourBarParameters::coupler_zero_angle_rad},
}};
constexpr std::array<std::pair<std::string_view, int FourBarParameters::*>, 3> kSignParams{{
  {"input_axis_sign", &FourBarParameters::input_axis_sign},
  {"output_axis_sign", &FourBarParameters::output_axis_sign},
  {"coupler_axis_sign", &FourBarParameters::coupler_axis_sign},
}};
static_assert(kRealParams.size() + kSignParams.size() == kFourBarParams.size());

std::optional<std::size_t> index_of(const std::vector<std::string> & names, std::string_view name);
std::variant<FourBarParameters, std::string> four_bar_parameters(
  const hardware_interface::TransmissionInfo & transmission);
template<typename T>
std::variant<T, std::string> parse_number(
  const hardware_interface::TransmissionInfo & transmission, std::string_view name);
}  // namespace

std::variant<DriveLayout, std::string> drive_layout(const hardware_interface::HardwareInfo & info)
{
  DriveLayout layout;
  for (const auto & joint : info.joints) {
    (joint.command_interfaces.empty() ? layout.passive_joints : layout.command_joints).push_back(
      joint.name);
  }
  layout.transport_joints = layout.command_joints;
  layout.linkages.assign(layout.command_joints.size(), std::nullopt);
  std::vector<int> uses(layout.passive_joints.size(), 0);

  for (const auto & transmission : info.transmissions) {
    const auto fail = [&transmission](std::string_view reason) {
        return std::format("transmission '{}': {}", transmission.name, reason);
      };
    if (transmission.type != kFourBarPlugin) {
      return fail(std::format(
          "plugin '{}' is not supported; the only transmission is '{}'", transmission.type,
          kFourBarPlugin));
    }
    if (transmission.actuators.size() != 1U || transmission.actuators.front().role != kInputRole) {
      return fail("needs exactly one <actuator>, with role 'input'");
    }
    const auto joint_with_role = [&transmission](std::string_view role) -> const std::string * {
        const auto matches = [role](const auto & joint) {return joint.role == role;};
        if (std::ranges::count_if(transmission.joints, matches) != 1) {
          return nullptr;
        }
        return &std::ranges::find_if(transmission.joints, matches)->name;
      };
    const auto * output = joint_with_role(kOutputRole);
    const auto * coupler = joint_with_role(kCouplerRole);
    if (transmission.joints.size() != 2U || !output || !coupler) {
      return fail("needs exactly two <joint>s, one with role 'output' and one with role 'coupler'");
    }
    const auto & input = transmission.actuators.front().name;

    const auto output_index = index_of(layout.command_joints, *output);
    const auto input_index = index_of(layout.passive_joints, input);
    const auto coupler_index = index_of(layout.passive_joints, *coupler);
    if (!output_index) {
      return fail(std::format("output '{}' is not a joint with command interfaces", *output));
    }
    if (!input_index || !coupler_index) {
      return fail(std::format(
          "input '{}' and coupler '{}' must both be state-only joints", input, *coupler));
    }
    if (layout.linkages[*output_index]) {
      return fail(std::format("output '{}' is already driven by another transmission", *output));
    }

    auto parameters = four_bar_parameters(transmission);
    if (const auto * error = std::get_if<std::string>(&parameters)) {
      return fail(*error);
    }
    const auto limits = info.limits.find(*output);
    if (limits == info.limits.end() || !limits->second.has_position_limits) {
      return fail(std::format("output '{}' has no position limits in the description", *output));
    }
    auto linkage = FourBarLinkage::create(
      std::get<FourBarParameters>(parameters), limits->second.min_position,
      limits->second.max_position);
    if (const auto * error = std::get_if<std::string>(&linkage)) {
      return fail(*error);
    }

    layout.linkages[*output_index] =
      LinkageDrive{std::get<FourBarLinkage>(linkage), *input_index, *coupler_index};
    layout.transport_joints[*output_index] = input;
    ++uses[*input_index];
    ++uses[*coupler_index];
  }

  for (std::size_t i = 0U; i < layout.passive_joints.size(); ++i) {
    if (uses[i] != 1) {
      return std::format(
        "state-only joint '{}' belongs to {} transmissions; it must belong to exactly one",
        layout.passive_joints[i], uses[i]);
    }
  }
  return layout;
}

namespace
{

std::optional<std::size_t> index_of(const std::vector<std::string> & names, std::string_view name)
{
  const auto it = std::ranges::find(names, name);
  if (it == names.end()) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(it - names.begin());
}

/// The four-bar's parameters from the transmission's <param>s: every one of kFourBarParams, and no
/// other, each a number that parses in full.
std::variant<FourBarParameters, std::string> four_bar_parameters(
  const hardware_interface::TransmissionInfo & transmission)
{
  for (const auto & [name, _] : transmission.parameters) {
    if (std::ranges::find(kFourBarParams, name) == kFourBarParams.end()) {
      return std::format("unknown <param> '{}'", name);
    }
  }
  FourBarParameters p;
  for (const auto & [name, member] : kRealParams) {
    auto value = parse_number<double>(transmission, name);
    if (const auto * error = std::get_if<std::string>(&value)) {
      return *error;
    }
    p.*member = std::get<double>(value);
  }
  for (const auto & [name, member] : kSignParams) {
    auto value = parse_number<int>(transmission, name);
    if (const auto * error = std::get_if<std::string>(&value)) {
      return *error;
    }
    p.*member = std::get<int>(value);
  }
  return p;
}

/// <param> `name` of `transmission` as a T, or why it is not one.
template<typename T>
std::variant<T, std::string> parse_number(
  const hardware_interface::TransmissionInfo & transmission, std::string_view name)
{
  const auto it = transmission.parameters.find(std::string{name});
  if (it == transmission.parameters.end()) {
    return std::format("required <param> '{}' is not set", name);
  }
  const auto & text = it->second;
  T value{};
  const auto [end, error] = std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc{} || end != text.data() + text.size()) {
    return std::format("<param> '{}' is '{}', not a number", name, text);
  }
  return value;
}

}  // namespace

}  // namespace humanoid::actuator_system
