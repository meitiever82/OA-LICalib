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
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/rclcpp.hpp>

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

template <typename MsgType>
inline bool loadmsg(const std::string bag_path, const std::string topic,
                    std::vector<std::shared_ptr<MsgType>> &msgs, 
                    const double bag_start = 0, const double bag_durr = -1) {
  // Create rosbag2 reader
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = "sqlite3";
  
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";
  
  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("dataset_reader"), 
                 "Failed to open rosbag: %s", e.what());
    return false;
  }
  
  // Get topic metadata
  auto topics_and_types = reader.get_all_topics_and_types();
  bool topic_found = false;
  for (const auto& topic_info : topics_and_types) {
    if (topic_info.name == topic) {
      topic_found = true;
      break;
    }
  }
  
  if (!topic_found) {
    RCLCPP_ERROR(rclcpp::get_logger("dataset_reader"),
                 "Topic %s not found in bag file", topic.c_str());
    return false;
  }
  
  // Set up topic filter
  rosbag2_storage::StorageFilter filter;
  filter.topics.push_back(topic);
  reader.set_filter(filter);
  
  // Time filtering variables
  rclcpp::Time start_time_ros(0, 0, RCL_ROS_TIME);
  rclcpp::Time end_time_ros(0, 0, RCL_ROS_TIME);
  bool first_msg = true;
  
  // Serialization setup
  rclcpp::Serialization<MsgType> serialization;
  
  // Read messages
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    
    if (bag_message->topic_name == topic) {
      rclcpp::Time msg_time(bag_message->time_stamp);
      
      // Handle time filtering
      if (first_msg) {
        start_time_ros = msg_time;
        if (bag_start > 0) {
          start_time_ros = rclcpp::Time(start_time_ros.nanoseconds() + 
                                       static_cast<int64_t>(bag_start * 1e9));
        }
        if (bag_durr > 0) {
          end_time_ros = rclcpp::Time(start_time_ros.nanoseconds() + 
                                     static_cast<int64_t>(bag_durr * 1e9));
        }
        first_msg = false;
      }
      
      // Apply time filtering
      if (bag_start > 0 && msg_time < start_time_ros) continue;
      if (bag_durr > 0 && msg_time > end_time_ros) break;
      
      // Deserialize message
      rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
      auto msg = std::make_shared<MsgType>();
      
      try {
        serialization.deserialize_message(&serialized_msg, msg.get());
        msgs.push_back(msg);
      } catch (const std::exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("dataset_reader"),
                    "Failed to deserialize message: %s", e.what());
        continue;
      }
    }
  }
  
  reader.close();
  
  if (msgs.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("dataset_reader"),
                 "No messages found for topic %s", topic.c_str());
    return false;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("dataset_reader"), "Loaded %zu messages from topic %s",
              msgs.size(), topic.c_str());
  RCLCPP_INFO(rclcpp::get_logger("dataset_reader"),
              "Time start | end | duration = %.6f | %.6f | %.3f",
              rclcpp::Time(msgs.front()->header.stamp).seconds(),
              rclcpp::Time(msgs.back()->header.stamp).seconds(),
              rclcpp::Time(msgs.back()->header.stamp).seconds() -
                  rclcpp::Time(msgs.front()->header.stamp).seconds());
  
  return true;
}

class LioDataset {
public:
  LioDataset(LidarModelType lidar_model) : lidar_model_(lidar_model) {}

  void Init() {
    velodyne16_convert_ = nullptr;
    vlp_point_convert_ = nullptr;
    // p_robosense_convert_ = nullptr;

    if (lidar_model_ == VLP_16_packet || lidar_model_ == VLP_16_SIMU) {
      velodyne16_convert_ = std::make_shared<Velodyne16>();
      std::cout << "LiDAR model set as VLP_16." << std::endl;
    } //
    else if (lidar_model_ == VLP_16_points || lidar_model_ == VLP_32E_points) {
      VelodyneType vlp_type = (lidar_model_ == VLP_16_points) ? VLP16 : VLP32E;
      vlp_point_convert_ = std::make_shared<VelodynePoints>(vlp_type);
      std::cout << "LiDAR model set as velodyne_points." << std::endl;
    } //
    /*else if (lidar_model_ == Ouster_16_points ||
             lidar_model_ == Ouster_32_points ||
             lidar_model_ == Ouster_64_points ||
             lidar_model_ == Ouster_128_points) {
      OusterRingNo ring_no = OusterRingNo::Ring128;
      if (lidar_model_ == Ouster_16_points)
        ring_no = Ring16;
      else if (lidar_model_ == Ouster_32_points)
        ring_no = Ring32;
      else if (lidar_model_ == Ouster_64_points)
        ring_no = Ring64;
      else if (lidar_model_ == Ouster_128_points)
        ring_no = Ring128;

      ouster_convert_ = std::make_shared<OusterLiDAR>(ring_no);
      lidar_model_ = LidarModelType::Ouster;
      std::cout << "LiDAR model set as Ouster " << int(ring_no) << " points.\n";
    } //
    else if (lidar_model_ == RS_16) {
      p_robosense_convert_ = std::make_shared<RobosenseCorrection>(
          RobosenseCorrection::ModelType::RS_16);
      std::cout << "LiDAR model set as RS_16." << std::endl;
    } //*/
    else {
      std::cout << "LiDAR model " << lidar_model_ << " not support yet."
                << std::endl;
    }
  }

  bool Read(const std::string path, const std::string imu_topic,
            const std::string lidar_topic, const double bag_start = -1.0,
            const double bag_durr = -1.0, const std::string vicon_topic = "") {
    
    Reset();
    Init();
    
    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"), 
                "Reading rosbag2: %s", path.c_str());
    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"), 
                "IMU topic: %s, LiDAR topic: %s", 
                imu_topic.c_str(), lidar_topic.c_str());
    
    // Read IMU data
    std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_msgs;
    if (!loadmsg<sensor_msgs::msg::Imu>(path, imu_topic, imu_msgs, bag_start, bag_durr)) {
      RCLCPP_ERROR(rclcpp::get_logger("dataset_reader"), 
                   "Failed to load IMU messages from topic: %s", imu_topic.c_str());
      return false;
    }
    
    // Convert IMU messages to internal format
    for (const auto& imu_msg : imu_msgs) {
      IMUData imu_data;
      imu_data.timestamp = rclcpp::Time(imu_msg->header.stamp).seconds();
      imu_data.gyro = Eigen::Vector3d(imu_msg->angular_velocity.x,
                                      imu_msg->angular_velocity.y,
                                      imu_msg->angular_velocity.z);
      imu_data.accel = Eigen::Vector3d(imu_msg->linear_acceleration.x,
                                       imu_msg->linear_acceleration.y,
                                       imu_msg->linear_acceleration.z);
      imu_data_.push_back(imu_data);
    }
    
    // Read LiDAR data based on model type
    bool lidar_success = false;
    
    if (lidar_model_ == VLP_16_packet) {
      // Read Velodyne packet data
      std::vector<velodyne_msgs::msg::VelodyneScan::SharedPtr> lidar_msgs;
      if (loadmsg<velodyne_msgs::msg::VelodyneScan>(path, lidar_topic, lidar_msgs, bag_start, bag_durr)) {
        for (const auto& scan_msg : lidar_msgs) {
          LiDARFeature lidar_feature;
          velodyne16_convert_->unpack_scan(scan_msg, lidar_feature);
          scan_data_.push_back(lidar_feature);
          scan_timestamps_.push_back(lidar_feature.timestamp);
        }
        lidar_success = true;
      }
    } 
    else if (lidar_model_ == VLP_16_points || lidar_model_ == VLP_32E_points) {
      // Read point cloud data
      std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> lidar_msgs;
      if (loadmsg<sensor_msgs::msg::PointCloud2>(path, lidar_topic, lidar_msgs, bag_start, bag_durr)) {
        if (lidar_model_ == VLP_16_points) {
          for (const auto& cloud_msg : lidar_msgs) {
            LiDARFeature lidar_feature;
            vlp_point_convert_->get_organized_and_raw_cloud(cloud_msg, lidar_feature);
            scan_data_.push_back(lidar_feature);
            scan_timestamps_.push_back(lidar_feature.timestamp);
          }
        } else {
          // VLP_32E_points - also use vlp_point_convert
          for (const auto& cloud_msg : lidar_msgs) {
            LiDARFeature lidar_feature;
            vlp_point_convert_->get_organized_and_raw_cloud(cloud_msg, lidar_feature);
            scan_data_.push_back(lidar_feature);
            scan_timestamps_.push_back(lidar_feature.timestamp);
          }
        }
        lidar_success = true;
      }
    }
    
    if (!lidar_success) {
      RCLCPP_ERROR(rclcpp::get_logger("dataset_reader"), 
                   "Failed to load LiDAR messages from topic: %s", lidar_topic.c_str());
      return false;
    }
    
    // Read Vicon data if topic is provided
    if (!vicon_topic.empty()) {
      std::vector<geometry_msgs::msg::TransformStamped::SharedPtr> vicon_msgs;
      if (loadmsg<geometry_msgs::msg::TransformStamped>(path, vicon_topic, vicon_msgs, bag_start, bag_durr)) {
        for (const auto& vicon_msg : vicon_msgs) {
          PoseData pose_data;
          pose_data.timestamp = rclcpp::Time(vicon_msg->header.stamp).seconds();
          pose_data.position = Eigen::Vector3d(vicon_msg->transform.translation.x,
                                               vicon_msg->transform.translation.y,
                                               vicon_msg->transform.translation.z);
          Eigen::Quaterniond quat(vicon_msg->transform.rotation.w,
                                   vicon_msg->transform.rotation.x,
                                   vicon_msg->transform.rotation.y,
                                   vicon_msg->transform.rotation.z);
          pose_data.orientation = SO3d(quat);
          vicon_data_.push_back(pose_data);
        }
        RCLCPP_INFO(rclcpp::get_logger("dataset_reader"), 
                    "Loaded %zu Vicon messages", vicon_data_.size());
      } else {
        RCLCPP_WARN(rclcpp::get_logger("dataset_reader"), 
                    "Failed to load Vicon messages from topic: %s", vicon_topic.c_str());
      }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("dataset_reader"), 
                "Successfully loaded dataset: %zu IMU, %zu LiDAR messages",
                imu_data_.size(), scan_data_.size());
    
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
           "Unvalid dataset. Check your dataset.. ");

    start_time_ =
        std::min(scan_timestamps_.front(), imu_data_.front().timestamp);
    end_time_ = std::max(scan_timestamps_.back(), imu_data_.back().timestamp);
    std::cout << "start_time set as " << start_time_ << std::endl;

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

    std::cout << "IMU timestamp from : " << imu_data_.front().timestamp
              << " to " << imu_data_.back().timestamp << std::endl;
    std::cout << "scan timestamp from : " << scan_data_.front().timestamp
              << " to "
              << scan_data_.back().full_features->points.back().timestamp
              << std::endl;
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
  }

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

public:
  // rosbag2 reader - no need to store as member variable

  Eigen::aligned_vector<IMUData> imu_data_;

  std::vector<LiDARFeature> scan_data_;

  std::vector<double> scan_timestamps_;

  Eigen::aligned_vector<PoseData> vicon_data_;

  double start_time_;
  double end_time_;

  Velodyne16::Ptr velodyne16_convert_;
  VelodynePoints::Ptr vlp_point_convert_;
  // OusterLiDAR::Ptr ouster_convert_;
  // RobosenseCorrection::Ptr p_robosense_convert_;

  LidarModelType lidar_model_;
};

} // namespace IO
} // namespace liso

#endif // DATASET_READER_H
