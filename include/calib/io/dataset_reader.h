/*
 * OA-LICalib:
 * Observability-Aware Intrinsic and Extrinsic Calibration of LiDAR-IMU Systems
 *
 * Copyright (C) 2022 Jiajun Lv
 * Copyright (C) 2022 Xingxing Zuo
 * Copyright (C) 2022 Kewei Hu
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DATASET_READER_H
#define DATASET_READER_H

/// read rosbag2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_filter.hpp>
#include <rosbag2_storage/storage_options.hpp>

/// ros message
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <velodyne_msgs/msg/velodyne_scan.hpp>

#include <pcl_conversions/pcl_conversions.h> /// fromROSMsg toROSMsg

#include <sensor_data/cloud_type.h>
#include <sensor_data/imu_data.h>

using SO3d = Sophus::SO3<double>;

//#include <sensor_data/lidar_ouster.h>
//#include <sensor_data/lidar_rs_16.h>
#include <sensor_data/lidar_vlp_16.h>
#include <sensor_data/lidar_vlp_points.h>

#include <utils/eigen_utils.hpp>
#include <utils/math_utils.h>

#include <Eigen/Core>
#include <fstream>
#include <random>

namespace liso {

namespace IO {

class LioDataset {
public:
  LioDataset(LidarModelType lidar_model) : lidar_model_(lidar_model) {}

  void Init() {
    velodyne16_convert_ = nullptr;
    vlp_point_convert_ = nullptr;

    if (lidar_model_ == VLP_16_packet || lidar_model_ == VLP_16_SIMU) {
      velodyne16_convert_ = std::make_shared<Velodyne16>();
      std::cout << "LiDAR model set as VLP_16." << std::endl;
    } else if (lidar_model_ == VLP_16_points ||
               lidar_model_ == VLP_32E_points) {
      VelodyneType vlp_type = (lidar_model_ == VLP_16_points) ? VLP16 : VLP32E;
      vlp_point_convert_ = std::make_shared<VelodynePoints>(vlp_type);
      std::cout << "LiDAR model set as velodyne_points." << std::endl;
    } else if (lidar_model_ == Ouster_16_points ||
               lidar_model_ == Ouster_32_points ||
               lidar_model_ == Ouster_64_points ||
               lidar_model_ == Ouster_128_points) {
      throw std::runtime_error("LiDAR model not supported.");
    } else if (lidar_model_ == RS_16) {
      throw std::runtime_error("LiDAR model not supported.");
    } else {
      std::cout << "LiDAR model " << lidar_model_ << " not support yet."
                << std::endl;
    }
  }

  bool Read(const std::string path, const std::string imu_topic,
            const std::string lidar_topic, const double bag_start = 0.0,
            const double bag_durr = -1.0, const std::string vicon_topic = "") {

    Reset();
    Init();

    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"), "Reading rosbag2: %s",
                path.c_str());
    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"),
                "IMU topic: %s, LiDAR topic: %s", imu_topic.c_str(),
                lidar_topic.c_str());

    // Create storage options
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = path;
    storage_options.storage_id = "sqlite3";

    // Set up topic filter
    auto filter = rosbag2_storage::StorageFilter{};
    filter.topics.push_back(imu_topic);
    filter.topics.push_back(lidar_topic);
    if (!vicon_topic.empty()) {
      filter.topics.push_back(vicon_topic);
    }

    // Create reader
    rosbag2_cpp::Reader reader;
    try {
      reader.open(storage_options);
      reader.set_filter(filter);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("dataset_reader"),
                   "Failed to open rosbag: %s", e.what());
      return false;
    }

    // Check if bag has messages
    if (!reader.has_next()) {
      RCLCPP_ERROR(rclcpp::get_logger("dataset_reader"),
                   "No messages found in bag");
      return false;
    }

    // Get bag metadata for time calculation
    auto original_start =
        reader.get_metadata().starting_time.time_since_epoch();
    int start_msec = bag_start * 1000;
    auto offset_s = std::chrono::milliseconds(start_msec);
    auto start_ns = original_start + offset_s;

    int dur_msec = bag_durr * 1000;
    auto bag_durr_s = std::chrono::milliseconds(dur_msec);
    auto end_ns = (bag_durr < 0) ? start_ns + reader.get_metadata().duration
                                 : start_ns + bag_durr_s;

    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"), "bag start time: %ld ns",
                start_ns.count());
    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"), "bag end time: %ld ns",
                end_ns.count());

    // Serialization objects
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serialization;
    rclcpp::Serialization<velodyne_msgs::msg::VelodyneScan>
        velodyne_serialization;
    rclcpp::Serialization<geometry_msgs::msg::TransformStamped>
        vicon_serialization;

    // Message containers
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    auto pc_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    auto velodyne_msg = std::make_shared<velodyne_msgs::msg::VelodyneScan>();
    auto vicon_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();

    // Counters
    int imu_msgs = 0, lidar_msgs = 0, vicon_msgs = 0;

    // Read messages
    while (reader.has_next()) {
      auto bag_message = reader.read_next();
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

      const auto message_timestamp =
          std::chrono::nanoseconds(bag_message->time_stamp);

      // Skip messages outside time range
      if (message_timestamp < start_ns || message_timestamp > end_ns) {
        continue;
      }

      // Process IMU messages
      if (bag_message->topic_name == imu_topic) {
        try {
          imu_serialization.deserialize_message(&serialized_msg, imu_msg.get());

          IMUData imu_data;
          imu_data.timestamp = rclcpp::Time(imu_msg->header.stamp).seconds();
          imu_data.gyro = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                          imu_msg->angular_velocity.y,
                                          imu_msg->angular_velocity.z);
          imu_data.accel = Eigen::Vector3d(imu_msg->linear_acceleration.x,
                                           imu_msg->linear_acceleration.y,
                                           imu_msg->linear_acceleration.z);
          imu_data.orientation = Eigen::Quaterniond(
              imu_msg->orientation.w, imu_msg->orientation.x,
              imu_msg->orientation.y, imu_msg->orientation.z);
          imu_data_.push_back(imu_data);
          imu_msgs++;
        } catch (const std::exception &e) {
          RCLCPP_WARN(rclcpp::get_logger("dataset_reader"),
                      "Failed to deserialize IMU message: %s", e.what());
        }
      }
      // Process LiDAR messages
      else if (bag_message->topic_name == lidar_topic) {
        try {
          if (lidar_model_ == VLP_16_packet) {
            // Process Velodyne packet data
            velodyne_serialization.deserialize_message(&serialized_msg,
                                                       velodyne_msg.get());

            LiDARFeature lidar_feature;
            velodyne16_convert_->unpack_scan(velodyne_msg, lidar_feature);
            scan_data_.push_back(lidar_feature);
            scan_timestamps_.push_back(lidar_feature.timestamp);
            lidar_msgs++;
          } else if (lidar_model_ == VLP_16_points ||
                     lidar_model_ == VLP_32E_points) {
            // Process point cloud data
            pc_serialization.deserialize_message(&serialized_msg, pc_msg.get());

            LiDARFeature lidar_feature;
            vlp_point_convert_->get_organized_and_raw_cloud(pc_msg,
                                                            lidar_feature);
            scan_data_.push_back(lidar_feature);
            scan_timestamps_.push_back(lidar_feature.timestamp);
            lidar_msgs++;
          }
        } catch (const std::exception &e) {
          RCLCPP_WARN(rclcpp::get_logger("dataset_reader"),
                      "Failed to deserialize LiDAR message: %s", e.what());
        }
      }
      // Process Vicon messages
      else if (!vicon_topic.empty() && bag_message->topic_name == vicon_topic) {
        try {
          vicon_serialization.deserialize_message(&serialized_msg,
                                                  vicon_msg.get());

          PoseData pose_data;
          pose_data.timestamp = rclcpp::Time(vicon_msg->header.stamp).seconds();
          pose_data.position =
              Eigen::Vector3d(vicon_msg->transform.translation.x,
                              vicon_msg->transform.translation.y,
                              vicon_msg->transform.translation.z);
          Eigen::Quaterniond quat(
              vicon_msg->transform.rotation.w, vicon_msg->transform.rotation.x,
              vicon_msg->transform.rotation.y, vicon_msg->transform.rotation.z);
          pose_data.orientation = SO3d(quat);
          vicon_data_.push_back(pose_data);
          vicon_msgs++;
        } catch (const std::exception &e) {
          RCLCPP_WARN(rclcpp::get_logger("dataset_reader"),
                      "Failed to deserialize Vicon message: %s", e.what());
        }
      }
    }

    // Verify data
    if (imu_data_.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("dataset_reader"),
                   "No IMU data loaded from topic: %s", imu_topic.c_str());
      return false;
    }

    if (scan_data_.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger("dataset_reader"),
                   "No LiDAR data loaded from topic: %s", lidar_topic.c_str());
      return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"),
                "Successfully loaded: %d IMU, %d LiDAR messages between %.3f "
                "and %.3f seconds",
                imu_msgs, lidar_msgs, start_ns.count() / 1e9,
                end_ns.count() / 1e9);

    if (!vicon_topic.empty()) {
      RCLCPP_INFO(rclcpp::get_logger("dataset_reader"),
                  "Loaded %d Vicon messages", vicon_msgs);
    }

    return true;
  }

  void AdjustIMUViconData() {
    assert(imu_data_.size() > 0 && "No IMU data. Check your bag and imu topic");
    assert(vicon_data_.size() > 0 &&
           "No vicon data. Check your bag and vicon topic");

    start_time_ =
        std::min(imu_data_.front().timestamp, vicon_data_.front().timestamp);
    end_time_ =
        std::max(imu_data_.back().timestamp, vicon_data_.back().timestamp);

    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"),
                "Adjusting timestamps: start=%.6f, end=%.6f, duration=%.3f",
                start_time_, end_time_, end_time_ - start_time_);

    for (size_t i = 0; i < imu_data_.size(); i++) {
      imu_data_[i].timestamp -= start_time_;
    }

    for (size_t i = 0; i < vicon_data_.size(); i++) {
      vicon_data_[i].timestamp -= start_time_;
    }
  }

  void AdjustDatasetTime() {
    assert(imu_data_.size() > 0 && "No IMU data. Check your bag and imu topic");
    assert(scan_data_.size() > 0 &&
           "No scan data. Check your bag and lidar topic");

    assert(scan_timestamps_.front() < imu_data_.back().timestamp &&
           scan_timestamps_.back() > imu_data_.front().timestamp &&
           "Invalid dataset. Check your dataset timing..");

    start_time_ =
        std::min(scan_timestamps_.front(), imu_data_.front().timestamp);
    end_time_ = std::max(scan_timestamps_.back(), imu_data_.back().timestamp);

    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"), "start_time set as %.6f",
                start_time_);

    for (size_t i = 0; i < imu_data_.size(); i++) {
      imu_data_[i].timestamp -= start_time_;
    }

    for (size_t j = 0; j < scan_data_.size(); j++) {
      scan_data_[j].timestamp -= start_time_;
      scan_timestamps_.at(j) -= start_time_;
      for (size_t i = 0; i < scan_data_[j].full_features->size(); i++) {
        scan_data_[j].full_features->points[i].timestamp -= start_time_;
      }
    }

    if (!vicon_data_.empty()) {
      for (size_t i = 0; i < vicon_data_.size(); i++) {
        vicon_data_[i].timestamp -= start_time_;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"),
                "IMU timestamp from: %.6f to %.6f", imu_data_.front().timestamp,
                imu_data_.back().timestamp);
    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"),
                "scan timestamp from: %.6f to %.6f",
                scan_data_.front().timestamp,
                scan_data_.back().full_features->points.back().timestamp);
  }

  void AddSimulationTimeoffset(double added_timeoffset) {
    for (size_t j = 0; j < scan_data_.size(); j++) {
      scan_data_[j].timestamp += added_timeoffset;
      for (size_t i = 0; i < scan_data_[j].full_features->size(); i++) {
        scan_data_[j].full_features->points[i].timestamp += added_timeoffset;
      }
    }
    for (size_t i = 0; i < scan_timestamps_.size(); i++) {
      scan_timestamps_[i] += added_timeoffset;
    }
  }

  void Reset() {
    imu_data_.clear();
    scan_data_.clear();
    scan_timestamps_.clear();
    vicon_data_.clear();
    start_time_ = 0.0;
    end_time_ = 0.0;
  }

  // Getters
  double get_start_time() const { return start_time_; }
  double get_end_time() const { return end_time_; }
  const std::vector<double> &get_scan_timestamps() const {
    return scan_timestamps_;
  }
  const Eigen::aligned_vector<IMUData> &get_imu_data() const {
    return imu_data_;
  }
  const Eigen::aligned_vector<PoseData> &get_vicon_data() const {
    return vicon_data_;
  }
  const std::vector<LiDARFeature> &get_scan_data() const { return scan_data_; }

private:
  Eigen::aligned_vector<IMUData> imu_data_;
  std::vector<LiDARFeature> scan_data_;
  std::vector<double> scan_timestamps_;
  Eigen::aligned_vector<PoseData> vicon_data_;

  double start_time_ = 0.0;
  double end_time_ = 0.0;

  Velodyne16::Ptr velodyne16_convert_;
  VelodynePoints::Ptr vlp_point_convert_;

  LidarModelType lidar_model_;
};

} // namespace IO
} // namespace liso

#endif // DATASET_READER_H