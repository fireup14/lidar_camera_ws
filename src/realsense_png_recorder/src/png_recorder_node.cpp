#include <algorithm>
#include <charconv>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace fs = std::filesystem;

class PngRecorderNode : public rclcpp::Node {
public:
  PngRecorderNode() : Node("png_recorder_node") {
    input_topic_ = declare_parameter<std::string>("input_topic", "/camera/camera/color/image_raw");
    output_dir_ = declare_parameter<std::string>(
      "output_dir",
      "docs/realsense_png");
    ensureOutputDirectory();

    subscription_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      // Match the RealSense publisher used by this workspace.  The default
      // SensorDataQoS is best-effort, which is normally compatible with a
      // reliable publisher but can fail to receive frames with some DDS
      // network configurations.
      rclcpp::QoS(1).reliable(),
      std::bind(&PngRecorderNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "PNG recorder started. topic=%s, output_dir=%s. It will save one PNG and exit.",
      input_topic_.c_str(), output_dir_.c_str());
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (image_saved_) {
      return;
    }

    ++frame_counter_;

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      // RealSense publishes color frames as rgb8, whereas cv::imwrite
      // interprets a three-channel image as BGR. Convert explicitly so the
      // saved PNG preserves the original colours.
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Failed to convert image message: %s", ex.what());
      return;
    }

    ensureOutputDirectory();
    const std::string file_name = buildFileName();
    if (file_name.empty()) {
      RCLCPP_ERROR(get_logger(), "Could not determine the next PNG file name.");
      return;
    }

    const fs::path output_path = fs::path(output_dir_) / file_name;

    if (cv::imwrite(output_path.string(), cv_ptr->image)) {
      image_saved_ = true;
      RCLCPP_INFO(
        get_logger(), "Saved image #%llu to %s",
        static_cast<unsigned long long>(frame_counter_), output_path.c_str());
      rclcpp::shutdown();
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to save PNG file: %s", output_path.c_str());
    }
  }

  void ensureOutputDirectory() {
    std::error_code ec;
    if (!fs::exists(output_dir_) && !fs::create_directories(output_dir_, ec)) {
      RCLCPP_ERROR(
        get_logger(), "Failed to create output directory %s: %s",
        output_dir_.c_str(), ec.message().c_str());
    }
  }

  std::string buildFileName() const {
    std::uint64_t largest_index = 0;
    std::error_code ec;

    fs::directory_iterator iterator(output_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(
        get_logger(), "Failed to inspect output directory %s: %s",
        output_dir_.c_str(), ec.message().c_str());
      return "";
    }

    for (const auto & entry : iterator) {

      if (!entry.is_regular_file() || entry.path().extension() != ".png") {
        continue;
      }

      const std::string stem = entry.path().stem().string();
      if (stem.empty() || stem.find_first_not_of("0123456789") != std::string::npos) {
        continue;
      }

      std::uint64_t index = 0;
      const auto result = std::from_chars(stem.data(), stem.data() + stem.size(), index);
      if (result.ec == std::errc{} && result.ptr == stem.data() + stem.size()) {
        largest_index = std::max(largest_index, index);
      }
    }

    if (largest_index == std::numeric_limits<std::uint64_t>::max()) {
      RCLCPP_ERROR(get_logger(), "PNG file index has reached its maximum value.");
      return "";
    }

    std::ostringstream oss;
    oss << std::setw(5) << std::setfill('0') << (largest_index + 1) << ".png";
    return oss.str();
  }

  std::string input_topic_;
  std::string output_dir_;
  std::uint64_t frame_counter_{0};
  bool image_saved_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PngRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
