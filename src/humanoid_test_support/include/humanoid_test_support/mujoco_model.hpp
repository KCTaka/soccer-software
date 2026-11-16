// Copyright 2026 UTRA-RoboSoccer
// Owning MuJoCo models and data in a test, and loading a model from a file or from MJCF text. The
// test target links mujoco::mujoco (humanoid_cmake's Findmujoco.cmake).
#ifndef HUMANOID_TEST_SUPPORT__MUJOCO_MODEL_HPP_
#define HUMANOID_TEST_SUPPORT__MUJOCO_MODEL_HPP_

#include <mujoco/mujoco.h>
#include <gtest/gtest.h>

#include <array>
#include <filesystem>
#include <memory>
#include <string_view>

namespace humanoid::test_support
{

struct ModelDeleter
{
  void operator()(mjModel * model) const noexcept {mj_deleteModel(model);}
};
struct DataDeleter
{
  void operator()(mjData * data) const noexcept {mj_deleteData(data);}
};
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

/// The model in the MJCF file at `path`. If it does not load, a failed test and nullptr.
inline ModelPtr load_model(const std::filesystem::path & path)
{
  std::array<char, 1024> error{};
  ModelPtr model(mj_loadXML(path.c_str(), nullptr, error.data(), static_cast<int>(error.size())));
  EXPECT_NE(model, nullptr) << path << ": " << error.data();
  return model;
}

/// The model the MJCF text `xml` describes, loaded through MuJoCo's virtual file system so that a
/// test can cover geometry the robot models lack. If it does not load, a failed test and nullptr.
inline ModelPtr load_model_xml(std::string_view xml)
{
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  mj_addBufferVFS(&vfs, "model.xml", xml.data(), static_cast<int>(xml.size()));
  std::array<char, 1024> error{};
  ModelPtr model(mj_loadXML("model.xml", &vfs, error.data(), static_cast<int>(error.size())));
  mj_deleteVFS(&vfs);
  EXPECT_NE(model, nullptr) << error.data();
  return model;
}

inline DataPtr make_data(const mjModel & model) {return DataPtr(mj_makeData(&model));}

}  // namespace humanoid::test_support

#endif  // HUMANOID_TEST_SUPPORT__MUJOCO_MODEL_HPP_
