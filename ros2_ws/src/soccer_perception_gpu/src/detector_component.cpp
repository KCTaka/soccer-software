// RF-DETR object detection on TensorRT, as a composable node.
//
// Loaded into the ZED camera container so `image_raw` arrives as an
// intra-process shared_ptr instead of a 3.69 MB DDS message. Replaces the
// Python HSV detector_node (44.4% of a core measured).
//
// Engine contract, from the benchmarked export:
//   input  "input"  1x3xSxS  fp32, RGB, [0,1], letterboxed
//   output "dets"   1xNx4    fp32, cxcywh normalised to [0,1]
//   output "labels" 1xNxC    fp32, per-class logits (sigmoid, not softmax)
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <soccer_msgs/msg/bounding_box.hpp>
#include <soccer_msgs/msg/bounding_boxes.hpp>

#include "soccer_perception_gpu/preprocess.hpp"
#include "soccer_perception_gpu/trt_engine.hpp"

namespace soccer_perception_gpu
{

class DetectorComponent : public rclcpp::Node
{
public:
  explicit DetectorComponent(const rclcpp::NodeOptions & options)
  : Node("detector_node", options)
  {
    engine_path_ = declare_parameter<std::string>("engine_path", "");
    input_name_ = declare_parameter<std::string>("input_tensor", "input");
    boxes_name_ = declare_parameter<std::string>("boxes_tensor", "dets");
    labels_name_ = declare_parameter<std::string>("labels_tensor", "labels");
    score_threshold_ = declare_parameter<double>("score_threshold", 0.45);
    max_detections_ = declare_parameter<int>("max_detections", 20);
    const auto image_topic = declare_parameter<std::string>("image_topic", "camera/image_raw");

    // COCO ids that the stock RF-DETR export emits, mapped onto our classes.
    // Retrain on soccer data to get a real "goalpost"; see the doc, §8.
    const auto ids = declare_parameter<std::vector<int64_t>>(
      "class_ids", std::vector<int64_t>{37, 1});
    const auto names = declare_parameter<std::vector<std::string>>(
      "class_names", std::vector<std::string>{"ball", "robot"});
    if (ids.size() != names.size()) {
      RCLCPP_ERROR(get_logger(), "class_ids and class_names differ in length; detector disabled");
    } else {
      for (size_t i = 0; i < ids.size(); ++i) {
        class_map_[static_cast<int>(ids[i])] = names[i];
      }
    }

    pub_ = create_publisher<soccer_msgs::msg::BoundingBoxes>("detections", 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {on_image(msg);});

    if (engine_path_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "no engine_path set - detector_node will publish EMPTY detections. "
        "Point engine_path at an RF-DETR .engine built by the trtexec that "
        "matches TensorRT %d.", TrtEngine::runtime_version());
      return;
    }
    try {
      engine_ = std::make_unique<TrtEngine>(engine_path_);
      const auto & in = engine_->tensor(input_name_);
      if (in.shape.size() != 4 || in.shape[1] != 3 || in.shape[2] != in.shape[3]) {
        throw std::runtime_error("expected a square 1x3xSxS input tensor");
      }
      input_size_ = static_cast<int>(in.shape[2]);
      const auto & boxes = engine_->tensor(boxes_name_);
      const auto & labels = engine_->tensor(labels_name_);
      num_queries_ = static_cast<int>(boxes.shape[1]);
      num_classes_ = static_cast<int>(labels.shape[2]);
      RCLCPP_INFO(
        get_logger(),
        "RF-DETR ready: %dx%d input, %d queries, %d classes (TensorRT %d, headers %d)",
        input_size_, input_size_, num_queries_, num_classes_,
        TrtEngine::runtime_version(), TrtEngine::compiled_version());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "failed to load engine: %s", e.what());
      engine_.reset();
    }
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    soccer_msgs::msg::BoundingBoxes out;
    out.header = msg->header;

    if (!engine_) {
      // Keep publishing so downstream liveness checks still see the topic, but
      // never let "no engine" masquerade as "no objects in view".
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "detector_node has NO ENGINE - publishing empty detections. Nothing will "
        "ever be seen until engine_path points at a loadable .engine.");
      pub_->publish(out);
      return;
    }

    try {
      // toCvShare avoids a copy when the encoding already matches; the ZED
      // publishes bgra8 so this converts once into our own scratch buffer.
      const cv_bridge::CvImageConstPtr cv = cv_bridge::toCvShare(msg, "bgr8");
      const Letterbox lb =
        letterbox_nchw(cv->image, input_size_, input_buf_, scratch_a_, scratch_b_);

      engine_->set_input(input_name_, input_buf_.data(), input_buf_.size());
      engine_->infer();
      decode(engine_->output(boxes_name_), engine_->output(labels_name_), lb, out);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "inference failed: %s", e.what());
    }
    pub_->publish(out);
  }

  void decode(
    const std::vector<float> & boxes, const std::vector<float> & logits,
    const Letterbox & lb, soccer_msgs::msg::BoundingBoxes & out) const
  {
    struct Hit
    {
      float score;
      int query;
      int cls;
    };
    std::vector<Hit> hits;

    for (int q = 0; q < num_queries_; ++q) {
      for (const auto & [cls, name] : class_map_) {
        if (cls < 0 || cls >= num_classes_) {continue;}
        // RF-DETR is sigmoid-scored per class, not softmax over classes.
        const float logit = logits[static_cast<size_t>(q) * num_classes_ + cls];
        const float score = 1.0f / (1.0f + std::exp(-logit));
        if (score >= score_threshold_) {hits.push_back({score, q, cls});}
      }
    }
    std::sort(
      hits.begin(), hits.end(), [](const Hit & a, const Hit & b) {return a.score > b.score;});
    if (static_cast<int>(hits.size()) > max_detections_) {
      hits.resize(static_cast<size_t>(max_detections_));
    }

    for (const auto & h : hits) {
      const float * b = &boxes[static_cast<size_t>(h.query) * 4];
      // cxcywh normalised to the letterboxed canvas -> corners in canvas pixels.
      const double cx = b[0] * input_size_, cy = b[1] * input_size_;
      const double w = b[2] * input_size_, hgt = b[3] * input_size_;

      double x0, y0, x1, y1;
      lb.to_source(cx - w / 2.0, cy - hgt / 2.0, x0, y0);
      lb.to_source(cx + w / 2.0, cy + hgt / 2.0, x1, y1);

      soccer_msgs::msg::BoundingBox box;
      box.class_id = class_map_.at(h.cls);
      box.probability = h.score;
      box.xmin = static_cast<int64_t>(std::llround(x0));
      box.ymin = static_cast<int64_t>(std::llround(y0));
      box.xmax = static_cast<int64_t>(std::llround(x1));
      box.ymax = static_cast<int64_t>(std::llround(y1));
      // Ground-contact point: bottom-centre, what projection_node back-projects.
      box.xbase = static_cast<int64_t>(std::llround((x0 + x1) / 2.0));
      box.ybase = static_cast<int64_t>(std::llround(y1));
      box.id = -1;
      out.bounding_boxes.push_back(box);
    }
  }

  std::string engine_path_, input_name_, boxes_name_, labels_name_;
  double score_threshold_ = 0.45;
  int max_detections_ = 20;
  int input_size_ = 0, num_queries_ = 0, num_classes_ = 0;
  std::unordered_map<int, std::string> class_map_;

  std::unique_ptr<TrtEngine> engine_;
  std::vector<float> input_buf_;
  cv::Mat scratch_a_, scratch_b_;

  rclcpp::Publisher<soccer_msgs::msg::BoundingBoxes>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace soccer_perception_gpu

RCLCPP_COMPONENTS_REGISTER_NODE(soccer_perception_gpu::DetectorComponent)
