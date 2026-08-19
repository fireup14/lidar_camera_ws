#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/imgcodecs.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace fs = std::filesystem;

class SyncCaptureNode : public rclcpp::Node {
public:
  using ImageMsg = sensor_msgs::msg::Image;
  using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
  using CustomMsg = livox_ros_driver2::msg::CustomMsg;
  using PointCloud2SyncPolicy =
    message_filters::sync_policies::ApproximateTime<PointCloud2Msg, ImageMsg>;
  using CustomSyncPolicy =
    message_filters::sync_policies::ApproximateTime<CustomMsg, ImageMsg>;

  SyncCaptureNode()
  : Node("sync_capture_node") {
    lidar_input_topic_ = declare_parameter<std::string>("lidar_input_topic", "/livox/lidar");
    lidar_message_type_ = declare_parameter<std::string>("lidar_message_type", "pointcloud2");
    lidar_output_dir_ = declare_parameter<std::string>(
      "lidar_output_dir",
      "/home/fire/Desktop/Lidar_Camera_Calibrator/lidar_camera_ws/docs/livox_pcd");
    lidar_save_duration_sec_ = declare_parameter<double>("lidar_save_duration_sec", 0.5);
    lidar_save_binary_ = declare_parameter<bool>("lidar_save_binary", true);
    image_input_topic_ = declare_parameter<std::string>(
      "image_input_topic", "/camera/camera/color/image_raw");
    image_output_dir_ = declare_parameter<std::string>(
      "image_output_dir",
      "/home/fire/Desktop/Lidar_Camera_Calibrator/lidar_camera_ws/docs/realsense_png");
    max_point_distance_m_ = declare_parameter<double>("max_point_distance_m", 0.0);
    queue_size_ = declare_parameter<int>("queue_size", 10);
    sync_tolerance_sec_ = declare_parameter<double>("sync_tolerance_sec", 0.1);

    if (queue_size_ <= 0) {
      queue_size_ = 10;
    }
    if (sync_tolerance_sec_ <= 0.0) {
      sync_tolerance_sec_ = 0.1;
    }
    if (lidar_save_duration_sec_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "lidar_save_duration_sec <= 0, reset to 0.5");
      lidar_save_duration_sec_ = 0.5;
    }
    if (max_point_distance_m_ < 0.0) {
      RCLCPP_WARN(get_logger(), "max_point_distance_m < 0, disabling distance filtering");
      max_point_distance_m_ = 0.0;
    }
    max_point_distance_squared_ = max_point_distance_m_ * max_point_distance_m_;

    ensureOutputDirectory(lidar_output_dir_);
    ensureOutputDirectory(image_output_dir_);

    image_sub_.subscribe(this, image_input_topic_, rmw_qos_profile_sensor_data);
    if (lidar_message_type_ == "custom") {
      custom_lidar_sub_.subscribe(this, lidar_input_topic_, rmw_qos_profile_sensor_data);
      custom_lidar_capture_sub_ = create_subscription<CustomMsg>(
        lidar_input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SyncCaptureNode::customLidarCaptureCallback, this, std::placeholders::_1));
      custom_sync_ = std::make_shared<message_filters::Synchronizer<CustomSyncPolicy>>(
        CustomSyncPolicy(queue_size_), custom_lidar_sub_, image_sub_);
      custom_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_sec_));
      custom_sync_->registerCallback(
        std::bind(&SyncCaptureNode::customSyncCallback, this, std::placeholders::_1, std::placeholders::_2));
    } else {
      if (lidar_message_type_ != "pointcloud2") {
        RCLCPP_WARN(get_logger(), "Unknown lidar_message_type '%s'; using pointcloud2.",
          lidar_message_type_.c_str());
      }
      lidar_message_type_ = "pointcloud2";
      lidar_sub_.subscribe(this, lidar_input_topic_, rmw_qos_profile_sensor_data);
      lidar_capture_sub_ = create_subscription<PointCloud2Msg>(
        lidar_input_topic_, rclcpp::SensorDataQoS(),
        std::bind(&SyncCaptureNode::lidarCaptureCallback, this, std::placeholders::_1));
      pointcloud2_sync_ = std::make_shared<message_filters::Synchronizer<PointCloud2SyncPolicy>>(
        PointCloud2SyncPolicy(queue_size_), lidar_sub_, image_sub_);
      pointcloud2_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_sec_));
      pointcloud2_sync_->registerCallback(
        std::bind(&SyncCaptureNode::pointCloud2SyncCallback, this, std::placeholders::_1, std::placeholders::_2));
    }

    RCLCPP_INFO(
      get_logger(),
      "Sync capture node started. lidar_topic=%s, type=%s, image_topic=%s, tolerance=%.3f s, "
      "pcd_duration=%.3f s, max_point_distance=%.3f m",
      lidar_input_topic_.c_str(), lidar_message_type_.c_str(), image_input_topic_.c_str(),
      sync_tolerance_sec_, lidar_save_duration_sec_, max_point_distance_m_);
  }

private:
  void pointCloud2SyncCallback(
    const PointCloud2Msg::ConstSharedPtr & cloud_msg,
    const ImageMsg::ConstSharedPtr & image_msg) {
    if (!beginCapture(*image_msg, rclcpp::Time(cloud_msg->header.stamp).nanoseconds())) {
      return;
    }
    appendPointCloud(*cloud_msg);
  }

  void customSyncCallback(
    const CustomMsg::ConstSharedPtr & cloud_msg,
    const ImageMsg::ConstSharedPtr & image_msg) {
    if (!beginCapture(*image_msg, rclcpp::Time(cloud_msg->header.stamp).nanoseconds())) {
      return;
    }
    appendPointCloud(*cloud_msg);
  }

  bool beginCapture(const ImageMsg & image_msg, std::int64_t cloud_stamp_ns) {
    if (capture_started_) {
      return false;
    }

    capture_started_ = true;
    const std::string file_stem = buildNextFileStem();
    if (file_stem.empty()) {
      RCLCPP_ERROR(get_logger(), "Failed to determine the next synchronized capture file name.");
      rclcpp::shutdown();
      return false;
    }

    if (!saveImage(image_msg, file_stem)) {
      RCLCPP_ERROR(get_logger(), "Failed to save synchronized PNG.");
      rclcpp::shutdown();
      return false;
    }

    capture_file_stem_ = file_stem;
    first_cloud_stamp_ns_ = cloud_stamp_ns;
    capture_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(lidar_save_duration_sec_)),
      std::bind(&SyncCaptureNode::saveAccumulatedPointCloud, this));

    RCLCPP_INFO(
      get_logger(), "Synchronized PNG saved; accumulating LiDAR data for %.3f s.",
      lidar_save_duration_sec_);
    return true;
  }

  void lidarCaptureCallback(const PointCloud2Msg::SharedPtr cloud_msg) {
    if (!capture_started_ || !capture_timer_) {
      return;
    }

    if (rclcpp::Time(cloud_msg->header.stamp).nanoseconds() == first_cloud_stamp_ns_) {
      return;
    }

    appendPointCloud(*cloud_msg);
  }

  void customLidarCaptureCallback(const CustomMsg::SharedPtr cloud_msg) {
    if (!capture_started_ || !capture_timer_) {
      return;
    }

    if (rclcpp::Time(cloud_msg->header.stamp).nanoseconds() == first_cloud_stamp_ns_) {
      return;
    }

    appendPointCloud(*cloud_msg);
  }

  void appendPointCloud(const PointCloud2Msg & cloud_msg) {
    pcl::PointCloud<pcl::PointXYZI> incoming_cloud;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_msg, "z");
      const bool has_intensity = hasField(cloud_msg, "intensity");
      incoming_cloud.reserve(
        static_cast<std::size_t>(cloud_msg.width) * static_cast<std::size_t>(cloud_msg.height));

      if (has_intensity) {
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(cloud_msg, "intensity");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
          if (!isPointAccepted(*iter_x, *iter_y, *iter_z)) {
            continue;
          }
          incoming_cloud.push_back(pcl::PointXYZI{*iter_x, *iter_y, *iter_z, *iter_intensity});
        }
      } else {
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
          if (!isPointAccepted(*iter_x, *iter_y, *iter_z)) {
            continue;
          }
          incoming_cloud.push_back(pcl::PointXYZI{*iter_x, *iter_y, *iter_z, 0.0F});
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

    cloud_buffer_ += incoming_cloud;
    cloud_buffer_.width = static_cast<std::uint32_t>(cloud_buffer_.size());
    cloud_buffer_.height = 1;
    cloud_buffer_.is_dense = false;
  }

  void appendPointCloud(const CustomMsg & cloud_msg) {
    pcl::PointCloud<pcl::PointXYZI> incoming_cloud;
    incoming_cloud.reserve(cloud_msg.points.size());

    for (const auto & src_point : cloud_msg.points) {
      if (!isPointAccepted(src_point.x, src_point.y, src_point.z)) {
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

    cloud_buffer_ += incoming_cloud;
    cloud_buffer_.width = static_cast<std::uint32_t>(cloud_buffer_.size());
    cloud_buffer_.height = 1;
    cloud_buffer_.is_dense = false;
  }

  void saveAccumulatedPointCloud() {
    capture_timer_->cancel();
    if (cloud_buffer_.empty()) {
      RCLCPP_ERROR(get_logger(), "No LiDAR points received during the synchronized capture window.");
      rclcpp::shutdown();
      return;
    }

    const fs::path output_path = fs::path(lidar_output_dir_) / (capture_file_stem_ + ".pcd");
    const int result = lidar_save_binary_
      ? pcl::io::savePCDFileBinary(output_path.string(), cloud_buffer_)
      : pcl::io::savePCDFileASCII(output_path.string(), cloud_buffer_);

    if (result != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to write PCD file: %s", output_path.c_str());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Synchronized PCD and PNG saved. PCD points=%zu. Exiting.", cloud_buffer_.size());
    rclcpp::shutdown();
  }

  bool saveImage(const ImageMsg & image_msg, const std::string & file_stem) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to convert image message: %s", ex.what());
      return false;
    }

    const fs::path output_path = fs::path(image_output_dir_) / (file_stem + ".png");
    if (!cv::imwrite(output_path.string(), cv_ptr->image)) {
      RCLCPP_ERROR(get_logger(), "Failed to write PNG file: %s", output_path.c_str());
      return false;
    }

    RCLCPP_INFO(get_logger(), "Saved synchronized PNG to %s", output_path.c_str());
    return true;
  }

  void ensureOutputDirectory(const std::string & path) {
    std::error_code ec;
    if (!fs::exists(path) && !fs::create_directories(path, ec)) {
      RCLCPP_ERROR(get_logger(), "Failed to create output directory %s: %s", path.c_str(), ec.message().c_str());
    }
  }

  bool hasField(const PointCloud2Msg & cloud_msg, const std::string & field_name) const {
    return std::any_of(
      cloud_msg.fields.begin(), cloud_msg.fields.end(),
      [&field_name](const auto & field) {return field.name == field_name;});
  }

  bool isPointAccepted(float x, float y, float z) const {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return false;
    }

    return max_point_distance_m_ <= 0.0 ||
           x * x + y * y + z * z <= max_point_distance_squared_;
  }

  std::string buildNextFileStem() const {
    const auto findLargestIndex = [this](const std::string & output_dir, const std::string & extension,
      std::uint64_t & largest_index) {
        std::error_code ec;
        fs::directory_iterator iterator(output_dir, ec);
        if (ec) {
          RCLCPP_ERROR(
            get_logger(), "Failed to inspect output directory %s: %s",
            output_dir.c_str(), ec.message().c_str());
          return false;
        }

        for (const auto & entry : iterator) {
          if (!entry.is_regular_file() || entry.path().extension() != extension) {
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
        return true;
      };

    std::uint64_t largest_index = 0;
    if (!findLargestIndex(lidar_output_dir_, ".pcd", largest_index) ||
      !findLargestIndex(image_output_dir_, ".png", largest_index)) {
      return "";
    }

    if (largest_index == std::numeric_limits<std::uint64_t>::max()) {
      RCLCPP_ERROR(get_logger(), "Synchronized capture file index has reached its maximum value.");
      return "";
    }

    std::ostringstream oss;
    oss << std::setw(5) << std::setfill('0') << (largest_index + 1);
    return oss.str();
  }

  std::string lidar_input_topic_;
  std::string lidar_message_type_;
  std::string lidar_output_dir_;
  double lidar_save_duration_sec_{0.5};
  bool lidar_save_binary_{true};
  std::string image_input_topic_;
  std::string image_output_dir_;
  double max_point_distance_m_{0.0};
  double max_point_distance_squared_{0.0};
  int queue_size_{10};
  double sync_tolerance_sec_{0.1};
  bool capture_started_{false};
  std::int64_t first_cloud_stamp_ns_{0};
  std::string capture_file_stem_;
  pcl::PointCloud<pcl::PointXYZI> cloud_buffer_;

  message_filters::Subscriber<PointCloud2Msg> lidar_sub_;
  message_filters::Subscriber<CustomMsg> custom_lidar_sub_;
  message_filters::Subscriber<ImageMsg> image_sub_;
  rclcpp::Subscription<PointCloud2Msg>::SharedPtr lidar_capture_sub_;
  rclcpp::Subscription<CustomMsg>::SharedPtr custom_lidar_capture_sub_;
  std::shared_ptr<message_filters::Synchronizer<PointCloud2SyncPolicy>> pointcloud2_sync_;
  std::shared_ptr<message_filters::Synchronizer<CustomSyncPolicy>> custom_sync_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncCaptureNode>());
  rclcpp::shutdown();
  return 0;
}
