// Pinhole model + flat-ground projection.
//
// The intrinsics come from `sensor_msgs/CameraInfo` at run time. They are NOT
// hardcoded: the Python fieldline_node hardcoded a 640x480 model
// (fx=550, cx=320, cy=240) while the ZED publishes 1280x720 with fx=732.9,
// cx=623.9, cy=354.7. Every projected point was wrong by 2.5-8.5 m and pixels
// above the true horizon were being turned into fake ground points.
// See docs/architecture/perception_gpu_migration.md §2.
#ifndef SOCCER_PERCEPTION_GPU__CAMERA_MODEL_HPP_
#define SOCCER_PERCEPTION_GPU__CAMERA_MODEL_HPP_

#include <array>
#include <cmath>
#include <optional>

#include <sensor_msgs/msg/camera_info.hpp>

namespace soccer_perception_gpu
{

class CameraModel
{
public:
  /// Intrinsics are unset until the first CameraInfo arrives. Nodes must not
  /// project anything while this returns false.
  bool valid() const {return valid_;}

  void update(const sensor_msgs::msg::CameraInfo & info)
  {
    // k = [fx 0 cx; 0 fy cy; 0 0 1], row-major.
    if (info.k[0] <= 0.0 || info.k[4] <= 0.0 || info.width == 0 || info.height == 0) {
      return;
    }
    fx_ = info.k[0];
    fy_ = info.k[4];
    cx_ = info.k[2];
    cy_ = info.k[5];
    width_ = static_cast<int>(info.width);
    height_ = static_cast<int>(info.height);
    valid_ = true;
  }

  /// Mounting geometry, from the URDF / calibration rather than the camera.
  void set_extrinsics(double mount_height_m, double tilt_rad)
  {
    mount_height_ = mount_height_m;
    tilt_ = tilt_rad;
  }

  int width() const {return width_;}
  int height() const {return height_;}
  double fx() const {return fx_;}
  double fy() const {return fy_;}
  double cx() const {return cx_;}
  double cy() const {return cy_;}

  /// Intersect the ray through pixel (u, v) with the ground plane.
  /// Returns {x forward, y left} in a base-aligned frame, or nullopt when the
  /// ray runs at or above the horizon.
  std::optional<std::array<double, 2>> project_flat_ground(double u, double v) const
  {
    if (!valid_) {return std::nullopt;}

    const double dx = (u - cx_) / fx_;
    const double dy = (v - cy_) / fy_;
    const double n = std::sqrt(dx * dx + dy * dy + 1.0);
    const double rx = dx / n, ry = dy / n, rz = 1.0 / n;

    const double ct = std::cos(tilt_), st = std::sin(tilt_);
    const double fwd_x = rz * ct + ry * st;
    const double fwd_y = -rx;
    const double fwd_z = -ry * ct + rz * st;

    if (fwd_z >= -1e-6) {return std::nullopt;}  // at or above the horizon

    const double t = mount_height_ / -fwd_z;
    return std::array<double, 2>{fwd_x * t, fwd_y * t};
  }

  /// Back-project with a measured stereo depth (metres), in the optical frame.
  std::array<double, 3> project_with_depth(double u, double v, double depth_m) const
  {
    return {(u - cx_) / fx_ * depth_m, (v - cy_) / fy_ * depth_m, depth_m};
  }

private:
  bool valid_ = false;
  double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
  int width_ = 0, height_ = 0;
  double mount_height_ = 0.30;
  double tilt_ = 0.35;
};

}  // namespace soccer_perception_gpu

#endif  // SOCCER_PERCEPTION_GPU__CAMERA_MODEL_HPP_
