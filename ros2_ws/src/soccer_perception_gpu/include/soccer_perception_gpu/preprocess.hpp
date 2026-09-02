// Image -> NCHW float tensor, and the inverse mapping for detections.
//
// Measured on this Orin (1280x720 -> 384x384, single thread): 3.5 ms, about
// 6.4% of one core at 18 Hz. That is small enough that a CUDA kernel is not
// worth the build complexity yet; see the upgrade path in
// docs/architecture/perception_gpu_migration.md §7.
#ifndef SOCCER_PERCEPTION_GPU__PREPROCESS_HPP_
#define SOCCER_PERCEPTION_GPU__PREPROCESS_HPP_

#include <algorithm>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace soccer_perception_gpu
{

/// Maps letterboxed network coordinates back onto the source image.
struct Letterbox
{
  double scale = 1.0;   // source pixels -> network pixels
  int pad_x = 0;        // left padding, in network pixels
  int pad_y = 0;        // top padding, in network pixels
  int src_w = 0;
  int src_h = 0;

  /// Network-space pixel -> source-image pixel.
  void to_source(double nx, double ny, double & sx, double & sy) const
  {
    sx = std::clamp((nx - pad_x) / scale, 0.0, static_cast<double>(src_w - 1));
    sy = std::clamp((ny - pad_y) / scale, 0.0, static_cast<double>(src_h - 1));
  }
};

/// Resize `src` into a `size` x `size` letterbox and write NCHW RGB float
/// values scaled to [0, 1] into `out`. `out` is resized to 3*size*size.
inline Letterbox letterbox_nchw(
  const cv::Mat & src, int size, std::vector<float> & out,
  cv::Mat & scratch_resized, cv::Mat & scratch_canvas)
{
  Letterbox lb;
  lb.src_w = src.cols;
  lb.src_h = src.rows;
  lb.scale = std::min(
    static_cast<double>(size) / src.cols, static_cast<double>(size) / src.rows);

  const int rw = std::max(1, static_cast<int>(std::round(src.cols * lb.scale)));
  const int rh = std::max(1, static_cast<int>(std::round(src.rows * lb.scale)));
  lb.pad_x = (size - rw) / 2;
  lb.pad_y = (size - rh) / 2;

  cv::resize(src, scratch_resized, cv::Size(rw, rh), 0, 0, cv::INTER_LINEAR);

  if (scratch_canvas.empty() || scratch_canvas.rows != size ||
    scratch_canvas.cols != size || scratch_canvas.type() != scratch_resized.type())
  {
    scratch_canvas.create(size, size, scratch_resized.type());
  }
  scratch_canvas.setTo(cv::Scalar::all(114));  // standard letterbox grey
  scratch_resized.copyTo(scratch_canvas(cv::Rect(lb.pad_x, lb.pad_y, rw, rh)));

  out.resize(static_cast<size_t>(3) * size * size);
  const int plane = size * size;
  // scratch_canvas is BGR8; the network wants RGB, so channel 2 goes first.
  for (int y = 0; y < size; ++y) {
    const uint8_t * row = scratch_canvas.ptr<uint8_t>(y);
    for (int x = 0; x < size; ++x) {
      const int idx = y * size + x;
      out[0 * plane + idx] = row[3 * x + 2] * (1.0f / 255.0f);
      out[1 * plane + idx] = row[3 * x + 1] * (1.0f / 255.0f);
      out[2 * plane + idx] = row[3 * x + 0] * (1.0f / 255.0f);
    }
  }
  return lb;
}

}  // namespace soccer_perception_gpu

#endif  // SOCCER_PERCEPTION_GPU__PREPROCESS_HPP_
