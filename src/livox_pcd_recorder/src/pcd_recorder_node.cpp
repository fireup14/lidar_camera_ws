#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace fs = std::filesystem;

class PcdRecorderNode : public rclcpp::Node {
public:
  PcdRecorderNode()
  : Node("pcd_recorder_node"),
    cloud_buffer_(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>()) {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    message_type_ = declare_parameter<std::string>("message_type", "pointcloud2");
    output_dir_ = declare_parameter<std::string>(
      "output_dir",
      "/home/fire/Desktop/Lidar_Camera_Calibrator/lidar_camera_ws/docs/livox_pcd");
    save_duration_sec_ = declare_parameter<double>("save_duration_sec", 5.0);
    save_binary_ = declare_parameter<bool>("save_binary", true);
    save_on_shutdown_ = declare_parameter<bool>("save_on_shutdown", true);
    save_once_ = declare_parameter<bool>("save_once", false);

    if (save_duration_sec_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "save_duration_sec <= 0, reset to 5.0");
      save_duration_sec_ = 5.0;
    }

    ensureOutputDirectory();

    if (message_type_ == "custom") {
      custom_subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PcdRecorderNode::customPointCloudCallback, this, std::placeholders::_1));
    } else {
      message_type_ = "pointcloud2";
      pointcloud2_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PcdRecorderNode::pointCloudCallback, this, std::placeholders::_1));
    }

    flush_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(save_duration_sec_)),
      std::bind(&PcdRecorderNode::flushBufferedCloud, this));

    RCLCPP_INFO(
      get_logger(),
      "PCD recorder started. topic=%s, type=%s, duration=%.2f s, output_dir=%s",
      input_topic_.c_str(), message_type_.c_str(), save_duration_sec_, output_dir_.c_str());
  }

  ~PcdRecorderNode() override {
    if (save_on_shutdown_ && !save_once_completed_) {
      flushBufferedCloud();
    }
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZI> incoming_cloud;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      const bool has_intensity = hasField(*msg, "intensity");
      incoming_cloud.reserve(static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height));

      if (has_intensity) {
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
        for (; iter_x != iter_x.end();
             ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
          if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
            continue;
          }

          pcl::PointXYZI point;
          point.x = *iter_x;
          point.y = *iter_y;
          point.z = *iter_z;
          point.intensity = *iter_intensity;
          incoming_cloud.push_back(point);
        }
      } else {
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
          if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
            continue;
          }

          pcl::PointXYZI point;
          point.x = *iter_x;
          point.y = *iter_y;
          point.z = *iter_z;
          point.intensity = 0.0f;
          incoming_cloud.push_back(point);
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Failed to parse PointCloud2 message: %s", ex.what());
      return;
    }

    if (incoming_cloud.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    *cloud_buffer_ += incoming_cloud;
    cloud_buffer_->width = static_cast<std::uint32_t>(cloud_buffer_->size());
    cloud_buffer_->height = 1;
    cloud_buffer_->is_dense = false;
  }

  void customPointCloudCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
    if (msg->points.empty()) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> incoming_cloud;
    incoming_cloud.reserve(msg->points.size());

    for (const auto & src_point : msg->points) {
      if (!std::isfinite(src_point.x) || !std::isfinite(src_point.y) || !std::isfinite(src_point.z)) {
        continue;
      }

      pcl::PointXYZI point;
      point.x = src_point.x;
      point.y = src_point.y;
      point.z = src_point.z;
      point.intensity = static_cast<float>(src_point.reflectivity);
      incoming_cloud.push_back(point);
    }

    if (incoming_cloud.empty()) {
      return;
    }

    std::lock_guard<std::mutex> lock(buffer_mutex_);
    *cloud_buffer_ += incoming_cloud;
    cloud_buffer_->width = static_cast<std::uint32_t>(cloud_buffer_->size());
    cloud_buffer_->height = 1;
    cloud_buffer_->is_dense = false;
  }

  bool hasField(const sensor_msgs::msg::PointCloud2 & msg, const std::string & field_name) const {
    for (const auto & field : msg.fields) {
      if (field.name == field_name) {
        return true;
      }
    }
    return false;
  }

  void flushBufferedCloud() {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_to_save(new pcl::PointCloud<pcl::PointXYZI>());

    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      if (cloud_buffer_->empty()) {
        return;
      }
      cloud_to_save.swap(cloud_buffer_);
      cloud_buffer_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }

    cloud_to_save->width = static_cast<std::uint32_t>(cloud_to_save->size());
    cloud_to_save->height = 1;
    cloud_to_save->is_dense = false;

    ensureOutputDirectory();
    const std::string file_name = buildFileName();
    if (file_name.empty()) {
      RCLCPP_ERROR(get_logger(), "Could not determine the next PCD file name.");
      return;
    }

    const fs::path output_path = fs::path(output_dir_) / file_name;

    const int result = save_binary_
      ? pcl::io::savePCDFileBinary(output_path.string(), *cloud_to_save)
      : pcl::io::savePCDFileASCII(output_path.string(), *cloud_to_save);

    if (result == 0) {
      RCLCPP_INFO(
        get_logger(), "Saved %zu points to %s",
        cloud_to_save->size(), output_path.c_str());
      if (save_once_) {
        save_once_completed_ = true;
        rclcpp::shutdown();
      }
    } else {
      RCLCPP_ERROR(
        get_logger(), "Failed to save PCD file: %s",
        output_path.c_str());
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
      if (!entry.is_regular_file() || entry.path().extension() != ".pcd") {
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
      RCLCPP_ERROR(get_logger(), "PCD file index has reached its maximum value.");
      return "";
    }

    std::ostringstream oss;
    oss << std::setw(5) << std::setfill('0') << (largest_index + 1) << ".pcd";
    return oss.str();
  }

  std::string input_topic_;
  std::string message_type_;
  std::string output_dir_;
  double save_duration_sec_{5.0};
  bool save_binary_{true};
  bool save_on_shutdown_{true};
  bool save_once_{false};
  bool save_once_completed_{false};

  std::mutex buffer_mutex_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_subscription_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_subscription_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdRecorderNode>());
  rclcpp::shutdown();
  return 0;
}
