// Turning segmented pixels into trustworthy ground points.
//
// Split out of fieldline_component.cpp so it can be unit-tested without a ROS
// node or a camera. The robot is currently on a desk in a room with no field
// lines, so this geometry cannot be validated visually; it is validated here
// instead.
//
// Three things happen, in this order, and the order matters:
//
//   1. Project every candidate pixel onto the ground plane. Rays at or above
//      the horizon are dropped.
//   2. Drop points beyond `max_range_m` by *radial* distance. Gating on the
//      forward coordinate alone is not enough: a ray that leaves the optical
//      axis sideways just below the horizon lands at, say, x = 0.4 m and
//      y = 60 m, which passes any forward-only gate while being nowhere near
//      the robot.
//   3. Only then thin the survivors to the point budget. Thinning first —
//      which is what this code used to do — means the budget is spent on
//      pixels that are about to be thrown away, so a frame where most of the
//      image is above the horizon delivers a small fraction of the points it
//      was asked for.
//
// The range limit is not a free parameter. Ground projection error grows as
// r^2/h (see soccer_localization/sensor_model.py): at 0.30 m mount height it is
// 0.07 m at 1 m, 0.51 m at 4 m and 1.12 m at 6 m. Points past ~4 m cost the
// particle filter more than they pay for.
#ifndef SOCCER_PERCEPTION_GPU__GROUND_POINTS_HPP_
#define SOCCER_PERCEPTION_GPU__GROUND_POINTS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "soccer_perception_gpu/camera_model.hpp"

namespace soccer_perception_gpu
{

struct GroundPoint
{
  double x;      ///< forward, metres
  double y;      ///< left, metres
  double range;  ///< radial distance from the camera mast, metres
};

/// Stride that thins `count` items down to at most `budget`, never zero.
inline std::size_t stride_for(std::size_t count, int budget)
{
  const std::size_t b = static_cast<std::size_t>(std::max(1, budget));
  return count <= b ? 1u : (count + b - 1u) / b;
}

/// Project pixels onto the ground and keep only what is inside `max_range_m`.
inline std::vector<GroundPoint> project_to_ground(
  const CameraModel & camera,
  const std::vector<std::array<float, 2>> & pixels,
  double max_range_m)
{
  std::vector<GroundPoint> out;
  if (!camera.valid() || !(max_range_m > 0.0)) {return out;}
  out.reserve(pixels.size());

  for (const auto & px : pixels) {
    const auto p = camera.project_flat_ground(px[0], px[1]);
    if (!p) {continue;}                                   // at or above the horizon
    const double x = (*p)[0], y = (*p)[1];
    if (!std::isfinite(x) || !std::isfinite(y)) {continue;}
    // A ray grazing the horizon yields an enormous but finite range; the radial
    // gate is what actually rejects it.
    const double range = std::hypot(x, y);
    if (range > max_range_m) {continue;}
    // Behind the camera cannot happen for a forward-facing mount, but a badly
    // configured tilt would make it happen silently.
    if (x <= 0.0) {continue;}
    out.push_back(GroundPoint{x, y, range});
  }
  return out;
}

/// Evenly thin an already-filtered set down to `max_points`, in place.
inline void thin_to_budget(std::vector<GroundPoint> & points, int max_points)
{
  const std::size_t step = stride_for(points.size(), max_points);
  if (step == 1u) {return;}
  std::size_t kept = 0;
  for (std::size_t i = 0; i < points.size(); i += step) {
    points[kept++] = points[i];
  }
  points.resize(kept);
}

}  // namespace soccer_perception_gpu

#endif  // SOCCER_PERCEPTION_GPU__GROUND_POINTS_HPP_
