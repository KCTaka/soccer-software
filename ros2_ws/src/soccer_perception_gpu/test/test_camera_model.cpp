// Regression guard for the projection defect described in
// docs/architecture/perception_gpu_migration.md §2: the Python fieldline_node
// hardcoded a 640x480 pinhole model and applied it to 1280x720 images.
#include <gtest/gtest.h>

#include <cmath>

#include <sensor_msgs/msg/camera_info.hpp>

#include "soccer_perception_gpu/camera_model.hpp"

using soccer_perception_gpu::CameraModel;

namespace
{

// Exactly what the ZED Mini publishes on /robot_1/camera/camera_info.
sensor_msgs::msg::CameraInfo zed_mini_hd720()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 1280;
  info.height = 720;
  info.k = {732.9060668945312, 0.0, 623.8761596679688,
    0.0, 732.9060668945312, 354.6865539550781,
    0.0, 0.0, 1.0};
  return info;
}

}  // namespace

TEST(CameraModel, RefusesToProjectBeforeCameraInfo)
{
  CameraModel cam;
  EXPECT_FALSE(cam.valid());
  EXPECT_FALSE(cam.project_flat_ground(640, 700).has_value());
}

TEST(CameraModel, TakesIntrinsicsFromCameraInfo)
{
  CameraModel cam;
  cam.update(zed_mini_hd720());
  ASSERT_TRUE(cam.valid());
  EXPECT_EQ(cam.width(), 1280);
  EXPECT_EQ(cam.height(), 720);
  EXPECT_NEAR(cam.fx(), 732.906, 1e-3);
  EXPECT_NEAR(cam.cx(), 623.876, 1e-3);
  EXPECT_NEAR(cam.cy(), 354.687, 1e-3);
}

TEST(CameraModel, IgnoresDegenerateCameraInfo)
{
  CameraModel cam;
  sensor_msgs::msg::CameraInfo bad;  // all zeros, as published before calibration
  cam.update(bad);
  EXPECT_FALSE(cam.valid());
}

TEST(CameraModel, RejectsPixelsAboveTheHorizon)
{
  CameraModel cam;
  cam.update(zed_mini_hd720());
  cam.set_extrinsics(0.30, 0.35);
  // v=500 sits above the true horizon for this tilt. The old hardcoded model
  // mapped it to a confident 3.27 m ground point, feeding the MCL a fiction.
  EXPECT_FALSE(cam.project_flat_ground(640, 500).has_value());
  EXPECT_FALSE(cam.project_flat_ground(640, 0).has_value());
}

TEST(CameraModel, ProjectsBelowHorizonPixelsForward)
{
  CameraModel cam;
  cam.update(zed_mini_hd720());
  cam.set_extrinsics(0.30, 0.35);

  const auto p = cam.project_flat_ground(640, 700);
  ASSERT_TRUE(p.has_value());
  EXPECT_GT((*p)[0], 0.0);              // in front of the robot
  EXPECT_NEAR((*p)[1], 0.0, 0.20);      // near the optical axis -> near y=0
  EXPECT_NEAR((*p)[0], 3.31, 0.05);     // value the corrected model produces
}

TEST(CameraModel, NearerPixelsAreLowerInTheImage)
{
  CameraModel cam;
  cam.update(zed_mini_hd720());
  cam.set_extrinsics(0.30, 0.35);

  const auto near_p = cam.project_flat_ground(640, 719);
  const auto far_p = cam.project_flat_ground(640, 650);
  ASSERT_TRUE(near_p.has_value());
  ASSERT_TRUE(far_p.has_value());
  EXPECT_LT((*near_p)[0], (*far_p)[0]);
}

TEST(CameraModel, LeftOfCentreProjectsToPositiveY)
{
  CameraModel cam;
  cam.update(zed_mini_hd720());
  cam.set_extrinsics(0.30, 0.35);

  const auto left = cam.project_flat_ground(200, 700);
  const auto right = cam.project_flat_ground(1100, 700);
  ASSERT_TRUE(left.has_value());
  ASSERT_TRUE(right.has_value());
  EXPECT_GT((*left)[1], 0.0);   // base frame is x forward, y left
  EXPECT_LT((*right)[1], 0.0);
}

// The defect itself, stated as a test: had the model been built from the old
// hardcoded numbers, the same pixel would land metres away from the truth.
TEST(CameraModel, HardcodedVgaIntrinsicsWouldMisprojectByMetres)
{
  CameraModel correct;
  correct.update(zed_mini_hd720());
  correct.set_extrinsics(0.30, 0.35);

  sensor_msgs::msg::CameraInfo vga;
  vga.width = 640;
  vga.height = 480;
  vga.k = {550.0, 0.0, 320.0, 0.0, 550.0, 240.0, 0.0, 0.0, 1.0};
  CameraModel wrong;
  wrong.update(vga);
  wrong.set_extrinsics(0.30, 0.35);

  const auto a = correct.project_flat_ground(640, 700);
  const auto b = wrong.project_flat_ground(640, 700);
  ASSERT_TRUE(a.has_value());
  ASSERT_TRUE(b.has_value());
  EXPECT_GT(std::hypot((*a)[0] - (*b)[0], (*a)[1] - (*b)[1]), 2.0);
}
