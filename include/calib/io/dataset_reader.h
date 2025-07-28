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

/// read rosbag - COMMENTED OUT FOR ROS2 MIGRATION
// TODO: Reimplement using rosbag2 API
/*#include <boost/foreach.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/reader_interfaces/base_reader_interface.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#define foreach BOOST_FOREACH*/

/// ros message
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <pcl_conversions/pcl_conversions.h> /// fromROSMsg toROSMsg

#include <sensor_data/cloud_type.h>
#include <sensor_data/imu_data.h>

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

// TODO: Reimplement loadmsg function for rosbag2
/*template <typename MsgType, typename MsgTypePtr>
inline bool loadmsg(const std::string bag_path, const std::string topic,
                    std::vector<MsgTypePtr> &msgs, const double bag_start = 0,
                    const double bag_durr = -1) {
  rosbag::Bag bag;
  bag.open(bag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(topic);

  rosbag::View view_full;
  rosbag::View view;

  // Start a few seconds in from the full view time
  // If we have a negative duration then use the full bag length
  view_full.addQuery(bag);
  ros::Time time_init = view_full.getBeginTime();
  time_init += ros::Duration(bag_start);
  ros::Time time_finish = (bag_durr < 0) ? view_full.getEndTime()
                                         : time_init + ros::Duration(bag_durr);
  view.addQuery(bag, rosbag::TopicQuery(topics), time_init, time_finish);

  // Check to make sure we have data to play
  if (view.size() == 0) {
    RCLCPP_ERROR(rclcpp::get_logger("dataset_reader"),
                 "No messages to play on specified topics.  Exiting.");
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  // Step through the rosbag
  for (const rosbag::MessageInstance &m : view) {
    // Handle IMU measurement
    MsgTypePtr msgPtr = m.instantiate<MsgType>();
    if (msgPtr != NULL) {
      msgs.push_back(msgPtr);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("dataset_reader"), "load topic %s",
              topic.c_str());
  RCLCPP_INFO(rclcpp::get_logger("dataset_reader"),
              "time start | end | duration= %.6f | %.6f | %.3f",
              rclcpp::Time(msgs.at(0)->header.stamp).seconds(),
              rclcpp::Time(msgs.back()->header.stamp).seconds(),
              rclcpp::Time(msgs.back()->header.stamp).seconds() -
                  rclcpp::Time(msgs.at(0)->header.stamp).seconds());

  return true;
}*/

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

  // TODO: Reimplement Read function for rosbag2
  // Temporarily disabled for ROS2 migration
  /*
  bool Read(const std::string path, const std::string imu_topic,
            const std::string lidar_topic, const double bag_start = -1.0,
            const double bag_durr = -1.0, const std::string vicon_topic = "") {
    // Implementation commented out for ROS2 migration
    // TODO: Reimplement using rosbag2 API
    return false;
  }
  */

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
  // std::shared_ptr<rosbag::Bag> bag_; // TODO: Replace with rosbag2 equivalent

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
