// Copyright 2026 UTRA-RoboSoccer
// Every robot model/generators has generated, as humanoid_description installs it. A test of a
// generated file runs once per robot here instead of naming one, so a robot added to
// model/robots.yaml is checked without editing a test:
//
//   class Description : public ::testing::TestWithParam<GeneratedRobot> {};
//   TEST_P(Description, ...) {... GetParam().urdf() ...}
//   INSTANTIATE_TEST_SUITE_P(
//     EveryRobot, Description, ::testing::ValuesIn(generated_robots()), RobotName{});
//
// The robots directory reaches the test as the compile definition HUMANOID_TEST_ROBOTS_DIR, set by
// humanoid_test_robots_dir(<target>) from this package's CMake: never through the environment. If
// no robot is installed the suite has no instance, which gtest reports as a failure, not a skip.
#ifndef HUMANOID_TEST_SUPPORT__GENERATED_ROBOTS_HPP_
#define HUMANOID_TEST_SUPPORT__GENERATED_ROBOTS_HPP_

#ifndef HUMANOID_TEST_ROBOTS_DIR
#error "HUMANOID_TEST_ROBOTS_DIR is not defined: call humanoid_test_robots_dir(<target>) in CMake"
#endif

#include <algorithm>
#include <filesystem>
#include <optional>
#include <ostream>
#include <string>
#include <system_error>
#include <vector>

namespace humanoid::test_support
{

/// One generated robot: its name in model/robots.yaml and its installed files.
struct GeneratedRobot
{
  std::string name;
  std::filesystem::path directory;

  /// Every robot has a description and a safety manifest.
  [[nodiscard]] std::filesystem::path urdf() const {return directory / "urdf" / "robot.urdf";}
  [[nodiscard]] std::filesystem::path safety_manifest() const
  {
    return directory / "config" / "safety_manifest.yaml";
  }

  /// The simulated world, for a robot generated with an MJCF.
  [[nodiscard]] std::optional<std::filesystem::path> mjcf_scene() const
  {
    return existing(directory / "mjcf" / "scene.xml");
  }

  /// The Stm32SerialTransport joints map, for a robot driven through the STM32 master.
  [[nodiscard]] std::optional<std::filesystem::path> stm32_wiring() const
  {
    return existing(directory / "config" / "stm32_wiring.yaml");
  }

private:
  static std::optional<std::filesystem::path> existing(const std::filesystem::path & path)
  {
    return std::filesystem::exists(path) ? std::optional{path} : std::nullopt;
  }
};

/// gtest prints a parameter with this, so a failure names the robot.
inline std::ostream & operator<<(std::ostream & out, const GeneratedRobot & robot)
{
  return out << robot.name;
}

/// Every generated robot, in name order.
inline std::vector<GeneratedRobot> generated_robots()
{
  std::vector<GeneratedRobot> robots;
  std::error_code error;
  for (const auto & entry :
    std::filesystem::directory_iterator(HUMANOID_TEST_ROBOTS_DIR, error))
  {
    if (entry.is_directory()) {
      robots.push_back({entry.path().filename().string(), entry.path()});
    }
  }
  std::ranges::sort(robots, {}, &GeneratedRobot::name);
  return robots;
}

/// The generated robots `keep` accepts, such as those with an MJCF.
template<typename Predicate>
std::vector<GeneratedRobot> generated_robots_where(Predicate keep)
{
  auto robots = generated_robots();
  std::erase_if(robots, [&keep](const GeneratedRobot & robot) {return !keep(robot);});
  return robots;
}

/// INSTANTIATE_TEST_SUITE_P's name generator: each instance is named after its robot.
struct RobotName
{
  template<typename ParamInfo>
  std::string operator()(const ParamInfo & info) const {return info.param.name;}
};

}  // namespace humanoid::test_support

#endif  // HUMANOID_TEST_SUPPORT__GENERATED_ROBOTS_HPP_
