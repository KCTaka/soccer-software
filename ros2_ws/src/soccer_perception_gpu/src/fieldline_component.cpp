// Field-line extraction, as a composable node.
//
// Two backends behind one contract:
//   * TensorRT segmentation net when `engine_path` is set (the production target)
//   * classical HSV at reduced resolution otherwise, so the stack still
//     localizes before a segmentation net has been trained
//
// Both paths project through intrinsics taken from `camera_info`. The Python
// node this replaces hardcoded a 640x480 model against 1280x720 images, which
// put every line point 2.5-8.5 m from its true position and invented ground
// points from above-horizon pixels.
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <soccer_msgs/msg/field_feature.hpp>
#include <soccer_msgs/msg/field_feature_array.hpp>

#include "soccer_perception_gpu/camera_model.hpp"
#include "soccer_perception_gpu/ground_points.hpp"
#include "soccer_perception_gpu/preprocess.hpp"
#include "soccer_perception_gpu/trt_engine.hpp"

namespace soccer_perception_gpu
{

class FieldlineComponent : public rclcpp::Node
{
public:
  explicit FieldlineComponent(const rclcpp::NodeOptions & options)
  : Node("fieldline_node", options)
  {
    engine_path_ = declare_parameter<std::string>("engine_path", "");
    input_name_ = declare_parameter<std::string>("input_tensor", "input");
    mask_name_ = declare_parameter<std::string>("mask_tensor", "mask");
    line_class_ = declare_parameter<int>("line_class", 1);
    mask_threshold_ = declare_parameter<double>("mask_threshold", 0.5);
    max_points_ = declare_parameter<int>("max_points", 200);
    // 4 m, not 6: ground projection error grows as r^2/h, reaching 0.51 m at 4 m
    // and 1.12 m at 6 m for this 0.30 m mount. See ground_points.hpp and
    // docs/architecture/localization_tuning.md §3.
    max_range_m_ = declare_parameter<double>("max_range_m", 4.0);
    confidence_ = declare_parameter<double>("confidence", 0.8);
    // Hard ceiling on how many mask pixels are projected in one frame. A white
    // wall or an overexposed frame can light up a six-figure pixel count; this
    // bounds the per-frame work regardless of what the segmentation returns.
    max_mask_pixels_ = declare_parameter<int>("max_mask_pixels", 20000);
    // Classical fallback runs at 1/scale resolution: measured 29.0% of a core at
    // full 1280x720, about 9.3% at 640x360, for the same line geometry.
    fallback_downscale_ = declare_parameter<int>("fallback_downscale", 2);

    const auto image_topic = declare_parameter<std::string>("image_topic", "camera/image_raw");
    const auto info_topic = declare_parameter<std::string>("camera_info_topic", "camera/camera_info");
    camera_.set_extrinsics(
      declare_parameter<double>("mount_height_m", 0.30),
      declare_parameter<double>("tilt_rad", 0.35));

    pub_ = create_publisher<soccer_msgs::msg::FieldFeatureArray>("field_features", 10);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        const bool was_valid = camera_.valid();
        camera_.update(*msg);
        if (!was_valid && camera_.valid()) {
          RCLCPP_INFO(
            get_logger(), "intrinsics locked: %dx%d fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
            camera_.width(), camera_.height(), camera_.fx(), camera_.fy(),
            camera_.cx(), camera_.cy());
        }
      });
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(msg);});

    if (!engine_path_.empty()) {
      try {
        load_engine();
      } catch (const std::exception & e) {
        // Never leave a half-configured engine behind: a zero mask dimension
        // would divide by zero on the first frame, and a wrong input shape would
        // silently produce garbage. Falling back is the honest failure.
        RCLCPP_ERROR(
          get_logger(), "segmentation engine unusable (%s) - using HSV fallback", e.what());
        engine_.reset();
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "no engine_path set - using the classical HSV fallback at 1/%d resolution",
        fallback_downscale_);
    }
  }

private:
  /// Load the engine and validate its shapes. Throws if anything is unusable,
  /// which drops the node onto the classical fallback rather than into a
  /// division by zero on the first frame.
  void load_engine()
  {
    auto engine = std::make_unique<TrtEngine>(engine_path_);

    const auto & in = engine->tensor(input_name_);
    if (in.shape.size() != 4) {
      throw std::runtime_error("input '" + input_name_ + "' is not NCHW");
    }
    const int ih = static_cast<int>(in.shape[2]), iw = static_cast<int>(in.shape[3]);
    if (ih <= 0 || iw <= 0) {
      throw std::runtime_error("input has a dynamic or zero spatial dimension");
    }
    if (ih != iw) {
      // letterbox_nchw() squares the image; a non-square net would be fed a
      // distorted crop and every projected point would be wrong.
      throw std::runtime_error("input is not square; letterboxing assumes it is");
    }

    const auto & m = engine->tensor(mask_name_);
    if (m.shape.size() < 3) {
      throw std::runtime_error("mask '" + mask_name_ + "' has too few dimensions");
    }
    const int ch = m.shape.size() == 4 ? static_cast<int>(m.shape[1]) : 1;
    const int mh = static_cast<int>(m.shape[m.shape.size() - 2]);
    const int mw = static_cast<int>(m.shape[m.shape.size() - 1]);
    if (ch <= 0 || mh <= 0 || mw <= 0) {
      throw std::runtime_error("mask has a dynamic or zero dimension");
    }
    if (ch > 1 && (line_class_ < 0 || line_class_ >= ch)) {
      throw std::runtime_error(
              "line_class " + std::to_string(line_class_) + " is outside the mask's " +
              std::to_string(ch) + " channels");
    }

    engine_ = std::move(engine);
    input_size_ = ih;
    mask_channels_ = ch;
    mask_h_ = mh;
    mask_w_ = mw;
    RCLCPP_INFO(
      get_logger(), "segmentation ready: %dx%d input -> %dx%dx%d mask, line class %d",
      input_size_, input_size_, mask_channels_, mask_h_, mask_w_, line_class_);
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    soccer_msgs::msg::FieldFeatureArray out;
    out.header.stamp = msg->header.stamp;
    out.header.frame_id = "base_link";

    // Publishing an empty array is honest; publishing points projected through
    // guessed intrinsics is what caused the original defect.
    if (!camera_.valid()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "no camera_info yet - not projecting anything");
      pub_->publish(out);
      return;
    }

    try {
      const cv_bridge::CvImageConstPtr cv = cv_bridge::toCvShare(msg, "bgr8");
      if (cv->image.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "empty image on %s", sub_->get_topic_name());
        pub_->publish(out);
        return;
      }
      pixels_.clear();
      if (engine_) {
        segment_trt(cv->image, pixels_);
      } else {
        segment_hsv(cv->image, pixels_);
      }
      project(pixels_, out);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "field-line extraction failed: %s", e.what());
    }
    pub_->publish(out);
  }

  void segment_trt(const cv::Mat & bgr, std::vector<std::array<float, 2>> & pixels)
  {
    const Letterbox lb = letterbox_nchw(bgr, input_size_, input_buf_, scratch_a_, scratch_b_);
    engine_->set_input(input_name_, input_buf_.data(), input_buf_.size());
    engine_->infer();
    const std::vector<float> & mask = engine_->output(mask_name_);

    const size_t plane = static_cast<size_t>(mask_h_) * mask_w_;
    // A mismatch here means the engine returned a different shape than it
    // advertised. Reading past the end would be a silent memory error.
    if (mask.size() < plane * static_cast<size_t>(mask_channels_)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "mask tensor is %zu floats, expected at least %zu - dropping frame",
        mask.size(), plane * static_cast<size_t>(mask_channels_));
      return;
    }

    const double mx = static_cast<double>(input_size_) / mask_w_;
    const double my = static_cast<double>(input_size_) / mask_h_;

    for (int y = 0; y < mask_h_; ++y) {
      for (int x = 0; x < mask_w_; ++x) {
        const size_t idx = static_cast<size_t>(y) * mask_w_ + x;
        bool is_line;
        if (mask_channels_ <= 1) {
          is_line = 1.0f / (1.0f + std::exp(-mask[idx])) >= mask_threshold_;
        } else {
          int best = 0;
          float best_v = mask[idx];
          for (int c = 1; c < mask_channels_; ++c) {
            const float v = mask[c * plane + idx];
            if (v > best_v) {best_v = v; best = c;}
          }
          is_line = best == line_class_;
        }
        if (!is_line) {continue;}
        double sx, sy;
        lb.to_source((x + 0.5) * mx, (y + 0.5) * my, sx, sy);
        pixels.push_back({static_cast<float>(sx), static_cast<float>(sy)});
      }
    }
  }

  void segment_hsv(const cv::Mat & bgr, std::vector<std::array<float, 2>> & pixels)
  {
    const int s = std::max(1, fallback_downscale_);
    // A downscale larger than the image would produce a zero-sized Mat.
    if (s > 1 && bgr.cols / s >= 8 && bgr.rows / s >= 8) {
      cv::resize(bgr, small_, cv::Size(bgr.cols / s, bgr.rows / s), 0, 0, cv::INTER_AREA);
    } else {
      small_ = bgr;
    }
    cv::cvtColor(small_, hsv_, cv::COLOR_BGR2HSV);
    cv::inRange(hsv_, cv::Scalar(35, 40, 40), cv::Scalar(85, 255, 255), grass_);
    cv::inRange(hsv_, cv::Scalar(0, 0, 180), cv::Scalar(180, 60, 255), white_);
    // Kernel scales with the image so the fallback behaves the same at any scale.
    const int k = std::max(3, (25 / s) | 1);
    cv::dilate(grass_, field_, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k)));
    cv::bitwise_and(white_, field_, lines_);

    cv::findNonZero(lines_, nonzero_);
    const size_t found = nonzero_.total();
    if (found == 0) {return;}
    // Bound the work before touching it. An overexposed frame or a white wall
    // inside a green-dominated scene can light up most of the image.
    const size_t harvest = stride_for(found, max_mask_pixels_);
    if (harvest > 1) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "%zu candidate line pixels exceeds max_mask_pixels=%d; taking every %zu",
        found, max_mask_pixels_, harvest);
    }
    pixels.reserve(found / harvest + 1);
    for (size_t i = 0; i < found; i += harvest) {
      const cv::Point p = nonzero_.at<cv::Point>(static_cast<int>(i));
      pixels.push_back(
        {static_cast<float>((p.x + 0.5) * s), static_cast<float>((p.y + 0.5) * s)});
    }
  }

  void project(
    const std::vector<std::array<float, 2>> & pixels,
    soccer_msgs::msg::FieldFeatureArray & out) const
  {
    // Filter first, thin second. Thinning first spends the budget on points
    // that are about to be rejected for being above the horizon or out of range.
    auto points = project_to_ground(camera_, pixels, max_range_m_);
    thin_to_budget(points, max_points_);

    out.features.reserve(points.size());
    for (const auto & p : points) {
      soccer_msgs::msg::FieldFeature f;
      f.type = soccer_msgs::msg::FieldFeature::TYPE_LINE_POINT;
      f.position.x = p.x;
      f.position.y = p.y;
      f.position.z = 0.0;
      // A flat per-point confidence. It is not a function of range, and the MCL
      // ignores it: the particle filter derives its own range-dependent sigma
      // from the same geometry (soccer_localization/sensor_model.py).
      f.confidence = confidence_;
      out.features.push_back(f);
    }
  }

  std::string engine_path_, input_name_, mask_name_;
  int line_class_ = 1, max_points_ = 200, fallback_downscale_ = 2;
  int max_mask_pixels_ = 20000;
  int input_size_ = 0, mask_channels_ = 1, mask_h_ = 0, mask_w_ = 0;
  double mask_threshold_ = 0.5, max_range_m_ = 4.0, confidence_ = 0.8;

  CameraModel camera_;
  std::unique_ptr<TrtEngine> engine_;
  std::vector<float> input_buf_;
  std::vector<std::array<float, 2>> pixels_;
  cv::Mat scratch_a_, scratch_b_, small_, hsv_, grass_, white_, field_, lines_, nonzero_;

  rclcpp::Publisher<soccer_msgs::msg::FieldFeatureArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
};

}  // namespace soccer_perception_gpu

RCLCPP_COMPONENTS_REGISTER_NODE(soccer_perception_gpu::FieldlineComponent)
