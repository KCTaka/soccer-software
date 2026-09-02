// Tests for the pixel -> ground-point pipeline in the field-line node.
//
// The robot is on a desk in a room with no field lines, so none of this can be
// checked against a real pitch. Every assertion here is therefore geometric:
// either a property that must hold for any correct projection, or a regression
// guard for a defect that was actually present in the shipped code.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <vector>

#include <sensor_msgs/msg/camera_info.hpp>

#include "soccer_perception_gpu/ground_points.hpp"

using soccer_perception_gpu::CameraModel;
using soccer_perception_gpu::GroundPoint;
using soccer_perception_gpu::project_to_ground;
using soccer_perception_gpu::stride_for;
using soccer_perception_gpu::thin_to_budget;

namespace
{

// Exactly what the ZED Mini publishes on /robot_1/camera/camera_info.
CameraModel zed_mini()
{
  sensor_msgs::msg::CameraInfo info;
  info.width = 1280;
  info.height = 720;
  info.k = {732.9060668945312, 0.0, 623.8761596679688,
    0.0, 732.9060668945312, 354.6865539550781,
    0.0, 0.0, 1.0};
  CameraModel cam;
  cam.update(info);
  cam.set_extrinsics(0.30, 0.35);
  return cam;
}

std::vector<GroundPoint> thinned(std::vector<GroundPoint> pts, int budget)
{
  thin_to_budget(pts, budget);
  return pts;
}

}  // namespace

// ── Range gating ──

TEST(GroundPoints, GatesOnRadialDistanceNotForwardDistance)
{
  // The defect: the node gated on x alone. A ray leaving the optical axis
  // sideways just below the horizon lands with a small x and an enormous y,
  // which sails through a forward-only gate while being nowhere near the robot.
  const CameraModel cam = zed_mini();

  // Sweep the bottom-left corner region for a pixel whose forward distance is
  // inside 4 m but whose radial distance is not.
  bool found = false;
  for (double u = 0.0; u < 1280.0 && !found; u += 1.0) {
    for (double v = 360.0; v < 720.0 && !found; v += 1.0) {
      const auto p = cam.project_flat_ground(u, v);
      if (!p) {continue;}
      const double x = (*p)[0], y = (*p)[1];
      if (x < 4.0 && std::hypot(x, y) > 4.0) {
        found = true;
        // The old forward-only gate would have kept this point.
        EXPECT_LT(x, 4.0);
        // The radial gate rejects it.
        const auto kept = project_to_ground(
          cam, {{static_cast<float>(u), static_cast<float>(v)}}, 4.0);
        EXPECT_TRUE(kept.empty())
          << "pixel (" << u << ", " << v << ") -> x=" << x << " y=" << y
          << " range=" << std::hypot(x, y);
      }
    }
  }
  ASSERT_TRUE(found) << "expected the wide FOV to produce a point that is near "
                        "in x but far in range";
}

TEST(GroundPoints, KeepsPointsInsideTheRange)
{
  const CameraModel cam = zed_mini();
  // Straight down the optical axis, well below the horizon.
  const auto pts = project_to_ground(cam, {{623.9f, 700.0f}}, 4.0);
  ASSERT_EQ(pts.size(), 1u);
  EXPECT_GT(pts[0].x, 0.0);
  EXPECT_LE(pts[0].range, 4.0);
  EXPECT_DOUBLE_EQ(pts[0].range, std::hypot(pts[0].x, pts[0].y));
}

TEST(GroundPoints, DropsPixelsAboveTheHorizon)
{
  const CameraModel cam = zed_mini();
  // Top of the image: with a 0.35 rad down-tilt this is above the horizon.
  const auto pts = project_to_ground(cam, {{623.9f, 0.0f}}, 100.0);
  EXPECT_TRUE(pts.empty());
}

TEST(GroundPoints, GrazingTheHorizonProducesNoUsablePoint)
{
  // Rays that are almost horizontal give a huge but finite range. Nothing
  // should survive a sane gate, and nothing should be infinite or NaN.
  const CameraModel cam = zed_mini();
  std::vector<std::array<float, 2>> row;
  for (float v = 300.0f; v < 480.0f; v += 0.25f) {
    row.push_back({623.9f, v});
  }
  const auto pts = project_to_ground(cam, row, 4.0);
  for (const auto & p : pts) {
    EXPECT_TRUE(std::isfinite(p.x));
    EXPECT_TRUE(std::isfinite(p.y));
    EXPECT_LE(p.range, 4.0);
    EXPECT_GT(p.x, 0.0);
  }
}

TEST(GroundPoints, RefusesToProjectWithoutIntrinsics)
{
  CameraModel cam;                       // no CameraInfo yet
  cam.set_extrinsics(0.30, 0.35);
  EXPECT_TRUE(project_to_ground(cam, {{600.0f, 700.0f}}, 4.0).empty());
}

TEST(GroundPoints, RejectsANonPositiveRange)
{
  const CameraModel cam = zed_mini();
  EXPECT_TRUE(project_to_ground(cam, {{623.9f, 700.0f}}, 0.0).empty());
  EXPECT_TRUE(project_to_ground(cam, {{623.9f, 700.0f}}, -1.0).empty());
}

TEST(GroundPoints, HandlesAnEmptyMask)
{
  // A frame with no grass, which is exactly what the robot sees indoors.
  EXPECT_TRUE(project_to_ground(zed_mini(), {}, 4.0).empty());
}

// ── Budgeting ──

TEST(GroundPoints, StrideNeverCollapsesToZero)
{
  EXPECT_EQ(stride_for(0, 200), 1u);
  EXPECT_EQ(stride_for(5, 200), 1u);
  EXPECT_EQ(stride_for(200, 200), 1u);
  EXPECT_EQ(stride_for(400, 200), 2u);
  EXPECT_EQ(stride_for(1000, 0), 1000u);   // a zero budget must not divide by zero
  EXPECT_EQ(stride_for(1000, -7), 1000u);
}

TEST(GroundPoints, ThinningRespectsTheBudget)
{
  std::vector<GroundPoint> pts;
  for (int i = 0; i < 1000; ++i) {
    pts.push_back(GroundPoint{1.0, 0.0, 1.0});
  }
  EXPECT_LE(thinned(pts, 200).size(), 200u);
  EXPECT_LE(thinned(pts, 1).size(), 1u);
  EXPECT_EQ(thinned(pts, 5000).size(), 1000u);
}

TEST(GroundPoints, ThinningSpreadsAcrossTheWholeSet)
{
  // Not just the first N: the near and far ends of the set must both survive.
  std::vector<GroundPoint> pts;
  for (int i = 0; i < 1000; ++i) {
    const double x = 0.5 + 0.003 * i;
    pts.push_back(GroundPoint{x, 0.0, x});
  }
  const auto kept = thinned(pts, 50);
  ASSERT_FALSE(kept.empty());
  EXPECT_NEAR(kept.front().x, 0.5, 1e-9);
  EXPECT_GT(kept.back().x, 3.0);
}

TEST(GroundPoints, FilteringHappensBeforeThinning)
{
  // The defect: the node computed its stride over raw pixels and only then
  // discarded the ones above the horizon, so a frame that is mostly sky
  // delivered a fraction of the points it was asked for.
  //
  // The gate here is deliberately wide so the horizon is the only thing
  // rejecting anything; this test is about ordering, not about range.
  const CameraModel cam = zed_mini();
  std::vector<std::array<float, 2>> pixels;
  for (int i = 0; i < 900; ++i) {          // above the horizon, all rejected
    pixels.push_back({623.9f, static_cast<float>(i % 200)});
  }
  for (int i = 0; i < 100; ++i) {          // usable ground pixels
    pixels.push_back({623.9f, 640.0f + static_cast<float>(i % 60)});
  }

  auto pts = project_to_ground(cam, pixels, 1000.0);
  ASSERT_EQ(pts.size(), 100u) << "every below-horizon pixel should survive a wide gate";
  thin_to_budget(pts, 50);
  EXPECT_EQ(pts.size(), 50u) << "the budget should be spent on surviving points";

  // Thinning first would have kept every 20th of 1000 pixels = 50 candidates,
  // of which only the handful that are below the horizon survive.
  std::size_t naive = 0;
  for (std::size_t i = 0; i < pixels.size(); i += stride_for(pixels.size(), 50)) {
    if (cam.project_flat_ground(pixels[i][0], pixels[i][1])) {++naive;}
  }
  EXPECT_LT(naive, 50u) << "the old order under-delivers against the budget";
}

TEST(GroundPoints, TheFourMetreGateIsWhatTheNoiseModelAsksFor)
{
  // soccer_localization/sensor_model.py puts ground projection error at 0.51 m
  // at 4 m and 1.12 m at 6 m for this mount. Widening the gate to 6 m admits
  // points the particle filter should not be weighting. This pins the fact that
  // the two extra metres are not free.
  const CameraModel cam = zed_mini();
  std::vector<std::array<float, 2>> column;
  for (float v = 380.0f; v < 720.0f; v += 0.5f) {
    column.push_back({623.9f, v});
  }
  const auto at_four = project_to_ground(cam, column, 4.0);
  const auto at_six = project_to_ground(cam, column, 6.0);
  ASSERT_FALSE(at_four.empty());
  EXPECT_GT(at_six.size(), at_four.size());
  for (const auto & p : at_four) {
    EXPECT_LE(p.range, 4.0);
  }
}
