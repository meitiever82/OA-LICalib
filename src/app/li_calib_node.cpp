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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <calib/calib_manager.h>
#include <memory>
#include <pangolin/pangolin.h>
#include <sensor_data/imu_data.h>
#include <trajectory/se3_trajectory.h>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <oa_licalib/msg/imu_array.hpp>
#include <oa_licalib/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace liso;

class CalibUI : public LICalibrManager {
public:
  CalibUI(const YAML::Node &config_node, rclcpp::Node::SharedPtr &ros_node)
      : LICalibrManager(config_node, ros_node), iteration_num_(1),
        pan_opt_time_offset_("ui.opt_time_offset", false, false, true),
        pan_opt_lidar_intrinsic_("ui.opt_lidar_intrinsic", false, false, true),
        pan_opt_imu_intrinsic_("ui.opt_imu_intrinsic", false, false, true),
        pan_apply_lidar_intrinstic_("ui.apply_lidar_intrinstic", false, false,
                                    true) {
    if (config_node["iteration_num"])
      iteration_num_ = config_node["iteration_num"].as<int>();

    /// Vicon data
    pub_trajectory_raw_ =
        ros_node->create_publisher<oa_licalib::msg::PoseArray>("/path_raw", 10);
    pub_trajectory_est_ =
        ros_node->create_publisher<oa_licalib::msg::PoseArray>("/path_est", 10);
    /// IMU fitting results
    pub_imu_raw_array_ = ros_node->create_publisher<oa_licalib::msg::ImuArray>(
        "/imu_raw_array", 10);
    pub_imu_est_array_ = ros_node->create_publisher<oa_licalib::msg::ImuArray>(
        "/imu_est_array", 10);
    /// lidar matching results
    pub_target_cloud_ =
        ros_node->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/target_cloud", 10);
    pub_source_cloud_ =
        ros_node->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/source_cloud", 10);

    /// spline trajectory
    pub_spline_trajectory_ = ros_node->create_publisher<nav_msgs::msg::Path>(
        "/spline_trajectory", 10);

    /// lidar trajectory
    pub_lidar_trajectory_ = ros_node->create_publisher<nav_msgs::msg::Path>(
        "/lidar_trajectory", 10);

    // 设置ROS相关的回调
    SetVisualizationCallbacks(
        // LOAM correspondence publisher - 使用 this 指针调用自己的方法
        [this](const auto &traj, const auto &correspondences) {
          this->PublishLoamCorrespondence(traj, correspondences);
        },
        // IMU data publisher
        [this](const auto &traj, const auto &imu_data) {
          this->PublishIMUData(traj, imu_data);
        },
        // Spline trajectory publisher
        [this](const auto &traj, double start, double end, double dt) {
          this->PublishSplineTrajectory(traj, start, end, dt);
        });
  }

  void InitGui() {
    pangolin::CreateWindowAndBind("Main", UI_WIDTH, 485);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));

    pangolin::Var<std::function<void(void)>> initialization(
        "ui.Initialization", std::bind(&CalibUI::Initialization, this));

    pangolin::Var<std::function<void(void)>> data_association_in_odom(
        "ui.DataAssociationInOdom",
        std::bind(&CalibUI::DataAssociationInOdom, this));

    pangolin::Var<std::function<void(void)>> data_association_in_locator(
        "ui.DataAssociationInLocator",
        std::bind(&CalibUI::DataAssociationInLocator, this));

    pangolin::Var<std::function<void(void)>> batch_optimization(
        "ui.BatchOptimization", std::bind(&CalibUI::BatchOptimization, this));

    pangolin::Var<std::function<void(void)>> refinement(
        "ui.Refinement", std::bind(&CalibUI::Refinement, this));

    std::cout << "\nInitUI Done. \n";
  }

  void RenderingLoop() {
    while (!pangolin::ShouldQuit() && rclcpp::ok()) {
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      this->calib_param_manager_->calib_option.opt_time_offset =
          (pan_opt_time_offset_ == true) ? true : false;
      this->calib_param_manager_->calib_option.opt_lidar_intrinsic =
          (pan_opt_lidar_intrinsic_ == true) ? true : false;
      this->calib_param_manager_->calib_option.opt_IMU_intrinsic =
          (pan_opt_imu_intrinsic_ == true) ? true : false;

      this->calib_param_manager_->calib_option.apply_lidar_intrinstic_to_scan =
          (pan_apply_lidar_intrinstic_ == true) ? true : false;

      pangolin::FinishFrame();
      usleep(10);
    }
  }

  void RunSimulation() {
    TicToc timer;

    timer.tic();
    this->Initialization();
    std::cout << "[Paper] Initialization costs " << std::fixed
              << std::setprecision(2) << timer.toc() << " ms\n";

    // if has map, then use locator
    if (this->calib_param_manager_->locator_segment_param.empty()) {
      this->DataAssociationInOdom();
    } else {
      this->DataAssociationInLocator();
    }

    timer.tic();
    this->BatchOptimization();
    std::cout << "[Paper] First opt costs " << std::fixed
              << std::setprecision(2) << timer.toc() << " ms\n";

    SaveCalibResult(0);

    for (size_t iter = 1; iter < iteration_num_; iter++) {
      if (iter > 1) {
        this->calib_param_manager_->calib_option.opt_lidar_intrinsic = true;
        this->calib_param_manager_->calib_option.opt_IMU_intrinsic = true;
        this->calib_param_manager_->calib_option.opt_time_offset = true;
      }
      timer.tic();
      this->Refinement();
      std::cout << "[Paper] Refinement costs " << std::fixed
                << std::setprecision(2) << timer.toc() << " ms\n";

      if (iter > 10) {
        this->calib_param_manager_->calib_option
            .apply_lidar_intrinstic_to_scan = true;
      }
      SaveCalibResult(iter);
    }
  }

  void Run() {
    this->Initialization();

    // 没有地图就用里程计
    if (this->calib_param_manager_->locator_segment_param.empty()) {
      this->DataAssociationInOdom();
    } else {
      this->DataAssociationInLocator();
    }

    this->BatchOptimization();

    for (size_t iter = 0; iter < iteration_num_; iter++) {
      this->calib_param_manager_->calib_option.opt_time_offset = true;
      this->Refinement();
    }

    std::cout << "Calibration finished." << std::endl;
  }

  void SaveCalibResult(size_t iteration) {
    std::ofstream outfile;
    std::string calib_result_file =
        this->cache_path_parent_ + "/simu_calib_result.txt";
    outfile.open(calib_result_file, std::ios::app);

    Eigen::Vector3d p_LinI = this->calib_param_manager_->p_LinI;
    Eigen::Quaterniond q_LtoI = this->calib_param_manager_->q_LtoI;
    Eigen::Matrix<double, 6, 1> Mw =
        this->calib_param_manager_->imu_intrinsic.Mw_vec_;
    Eigen::Matrix<double, 6, 1> Ma =
        this->calib_param_manager_->imu_intrinsic.Ma_vec_;
    Eigen::Matrix<double, 9, 1> Aw =
        this->calib_param_manager_->imu_intrinsic.Aw_vec_;
    Eigen::Quaterniond q_WtoA =
        this->calib_param_manager_->imu_intrinsic.q_WtoA_;

    double time_offset =
        this->calib_param_manager_->segment_param[0].time_offset;
    outfile << this->bag_name_ << "," << iteration << "," << time_offset << ","
            << p_LinI(0) << "," << p_LinI(1) << "," << p_LinI(2) << ","
            << q_LtoI.x() << "," << q_LtoI.y() << "," << q_LtoI.z() << ","
            << q_LtoI.w();
    for (int i = 0; i < 6; i++) {
      outfile << "," << Mw(i);
    }
    for (int i = 0; i < 6; i++) {
      outfile << "," << Ma(i);
    }
    outfile << "," << q_WtoA.x() << "," << q_WtoA.y() << "," << q_WtoA.z()
            << "," << q_WtoA.w();

    //    for (int i = 0; i < 9; i++) {
    //      outfile << "," << Aw(i);
    //    }
    outfile << "\n";
    outfile.close();

    /// Save lidar intrinsic
    std::ofstream lidar_file;
    std::string lidar_file_name =
        this->cache_path_ + "/lidar-" + std::to_string(iteration) + ".txt";
    lidar_file.open(lidar_file_name);
    std::cout << "Save lidar intrinsic to " << lidar_file_name << std::endl;

    lidar_file << "dist_scale,dist_offset_mm,vert_offset_mm,horiz_offset_mm,"
               << "vert_degree,delta_horiz_degree\n";

    const auto laser_param_vec =
        this->calib_param_manager_->lidar_intrinsic.GetLaserParamVec();
    int order[16] = {15, 13, 11, 9, 7, 5, 3, 1, 14, 12, 10, 8, 6, 4, 2, 0};
    for (size_t dsr = 0; dsr < 16; dsr++) {
      std::vector<double> v = laser_param_vec.at(order[dsr]);
      lidar_file << v[0] << ","              //
                 << v[1] * 1000 << ","       //
                 << v[2] * 1000 << ","       //
                 << v[3] * 1000 << ","       //
                 << v[4] * 180 / M_PI << "," //
                 << v[5] * 180 / M_PI << std::endl;
    }
    lidar_file.close();
  }

private:
  inline void PublishIMUData(std::shared_ptr<Trajectory> trajectory,
                             const Eigen::aligned_vector<IMUData> &imu_data) {
    if (pub_imu_raw_array_->get_subscription_count() == 0 &&
        pub_imu_est_array_->get_subscription_count() == 0)
      return;

    oa_licalib::msg::ImuArray imu_array_raw;
    oa_licalib::msg::ImuArray imu_array_est;

    for (auto const &v : imu_data) {
      if (!trajectory->GetTrajQuality(v.timestamp)) {
        continue;
      }
      geometry_msgs::msg::Vector3 gyro, accel;
      gyro.x = v.gyro.x();
      gyro.y = v.gyro.y();
      gyro.z = v.gyro.z();
      accel.x = v.accel.x();
      accel.y = v.accel.y();
      accel.z = v.accel.z();
      imu_array_raw.timestamps.push_back(v.timestamp);
      imu_array_raw.angular_velocities.push_back(gyro);
      imu_array_raw.linear_accelerations.push_back(accel);

      auto const param = trajectory->GetTrajParam();

      Eigen::Vector3d w_b =
          trajectory->rotVelBody(v.timestamp) + param->gyro_bias;
      Eigen::Vector3d a_w = trajectory->transAccelWorld(v.timestamp);
      SE3d pose = trajectory->pose(v.timestamp);
      Eigen::Vector3d a_b =
          pose.so3().inverse() * (a_w + param->gravity) + param->acce_bias;

      geometry_msgs::msg::Vector3 gyro2, accel2;
      gyro2.x = w_b.x();
      gyro2.y = w_b.y();
      gyro2.z = w_b.z();
      accel2.x = a_b.x();
      accel2.y = a_b.y();
      accel2.z = a_b.z();
      imu_array_est.timestamps.push_back(v.timestamp);
      imu_array_est.angular_velocities.push_back(gyro2);
      imu_array_est.linear_accelerations.push_back(accel2);
    }
    auto now = rclcpp::Clock().now();
    imu_array_raw.header.stamp = now;
    imu_array_raw.header.frame_id = "/imu";

    imu_array_est.header = imu_array_raw.header;

    pub_imu_raw_array_->publish(imu_array_raw);
    pub_imu_est_array_->publish(imu_array_est);
  }

  inline void
  PublishViconData(std::shared_ptr<Trajectory> trajectory,
                   const Eigen::aligned_vector<PoseData> &vicon_data) {
    if (pub_trajectory_raw_->get_subscription_count() == 0 &&
        pub_trajectory_est_->get_subscription_count() == 0)
      return;

    oa_licalib::msg::PoseArray vicon_path_raw;
    oa_licalib::msg::PoseArray vicon_path_est;

    for (auto const &v : vicon_data) {
      geometry_msgs::msg::Vector3 position;
      geometry_msgs::msg::Quaternion orientation;

      // raw data
      position.x = v.position.x();
      position.y = v.position.y();
      position.z = v.position.z();
      auto quat = v.orientation.unit_quaternion();
      orientation.w = quat.w();
      orientation.x = quat.x();
      orientation.y = quat.y();
      orientation.z = quat.z();

      vicon_path_raw.timestamps.push_back(v.timestamp);
      vicon_path_raw.positions.push_back(position);
      vicon_path_raw.orientations.push_back(orientation);

      // estiamted pose
      SE3d pose;
      if (!trajectory->GetLidarPose(v.timestamp, pose))
        continue;
      auto trans = pose.translation();
      position.x = trans.x();
      position.y = trans.y();
      position.z = trans.z();
      auto quat2 = pose.unit_quaternion();
      orientation.w = quat2.w();
      orientation.x = quat2.x();
      orientation.y = quat2.y();
      orientation.z = quat2.z();
      vicon_path_est.timestamps.push_back(v.timestamp);
      vicon_path_est.positions.push_back(position);
      vicon_path_est.orientations.push_back(orientation);
    }

    auto now = rclcpp::Clock().now();
    vicon_path_raw.header.frame_id = "/map";
    vicon_path_raw.header.stamp = now;
    vicon_path_est.header = vicon_path_raw.header;

    pub_trajectory_raw_->publish(vicon_path_raw);
    pub_trajectory_est_->publish(vicon_path_est);
  }

  inline void
  PublishViconData(const Eigen::aligned_vector<PoseData> &vicon_est,
                   const Eigen::aligned_vector<PoseData> &vicon_data) {
    if (pub_trajectory_raw_->get_subscription_count() == 0 &&
        pub_trajectory_est_->get_subscription_count() == 0)
      return;

    oa_licalib::msg::PoseArray vicon_path_raw;
    oa_licalib::msg::PoseArray vicon_path_est;

    for (auto const &v : vicon_data) {
      geometry_msgs::msg::Vector3 position;
      geometry_msgs::msg::Quaternion orientation;

      // raw data
      position.x = v.position.x();
      position.y = v.position.y();
      position.z = v.position.z();
      auto quat = v.orientation.unit_quaternion();
      orientation.w = quat.w();
      orientation.x = quat.x();
      orientation.y = quat.y();
      orientation.z = quat.z();

      vicon_path_raw.timestamps.push_back(v.timestamp);
      vicon_path_raw.positions.push_back(position);
      vicon_path_raw.orientations.push_back(orientation);
    }

    for (auto const &v : vicon_est) {
      // estiamted pose
      geometry_msgs::msg::Vector3 position;
      geometry_msgs::msg::Quaternion orientation;

      position.x = v.position.x();
      position.y = v.position.y();
      position.z = v.position.z();
      auto quat = v.orientation.unit_quaternion();
      orientation.w = quat.w();
      orientation.x = quat.x();
      orientation.y = quat.y();
      orientation.z = quat.z();

      vicon_path_est.timestamps.push_back(v.timestamp);
      vicon_path_est.positions.push_back(position);
      vicon_path_est.orientations.push_back(orientation);
    }

    auto now = rclcpp::Clock().now();
    vicon_path_raw.header.frame_id = "/map";
    vicon_path_raw.header.stamp = now;
    vicon_path_est.header = vicon_path_raw.header;

    pub_trajectory_raw_->publish(vicon_path_raw);
    pub_trajectory_est_->publish(vicon_path_est);
  }

  inline void PublishIMUOrientationData(
      std::shared_ptr<Trajectory> trajectory,
      const Eigen::aligned_vector<PoseData> &orientation_data) {
    oa_licalib::msg::PoseArray imu_ori_path_raw;
    oa_licalib::msg::PoseArray imu_ori_path_est;

    for (auto const &v : orientation_data) {
      geometry_msgs::msg::Vector3 position;
      geometry_msgs::msg::Quaternion orientation;

      // raw data
      position.x = v.position.x();
      position.y = v.position.y();
      position.z = v.position.z();
      auto quat = v.orientation.unit_quaternion();
      orientation.w = quat.w();
      orientation.x = quat.x();
      orientation.y = quat.y();
      orientation.z = quat.z();

      imu_ori_path_raw.timestamps.push_back(v.timestamp);
      imu_ori_path_raw.positions.push_back(position);
      imu_ori_path_raw.orientations.push_back(orientation);

      // estiamted pose
      SE3d pose = trajectory->pose(v.timestamp);
      auto trans = pose.translation();
      position.x = trans.x();
      position.y = trans.y();
      position.z = trans.z();
      auto quat2 = pose.unit_quaternion();
      orientation.w = quat2.w();
      orientation.x = quat2.x();
      orientation.y = quat2.y();
      orientation.z = quat2.z();
      imu_ori_path_est.timestamps.push_back(v.timestamp);
      imu_ori_path_est.positions.push_back(position);
      imu_ori_path_est.orientations.push_back(orientation);
    }

    auto now = rclcpp::Clock().now();
    imu_ori_path_raw.header.frame_id = "/map";
    imu_ori_path_raw.header.stamp = now;
    imu_ori_path_est.header = imu_ori_path_raw.header;

    pub_trajectory_raw_->publish(imu_ori_path_raw);
    pub_trajectory_est_->publish(imu_ori_path_est);
  }

  inline void PublishLoamCorrespondence(
      std::shared_ptr<Trajectory> trajectory,
      const Eigen::aligned_vector<PointCorrespondence> &point_measurement) {
    if (pub_source_cloud_->get_subscription_count() == 0 &&
        pub_target_cloud_->get_subscription_count() == 0)
      return;

    VPointCloud target_cloud, source_cloud;
    for (const PointCorrespondence &cor : point_measurement) {
      if (cor.t_map < trajectory->minTime() ||
          cor.t_point < trajectory->minTime()) {
        std::cout << RED << "[PublishLoamCorrespondence] skip " << cor.t_map
                  << "; " << cor.t_point << RESET << std::endl;
        continue;
      }

      // SE3d T_MtoG = trajectory->GetLidarPose(cor.t_map);
      SE3d T_LktoG = trajectory->GetLidarPose(cor.t_point);

      // Eigen::Vector3d p_inM = T_MtoG.inverse() * T_LktoG * cor.point;
      Eigen::Vector3d p_inM = T_LktoG * cor.point;

      VPoint p_target, p_source;
      Eigen::Vector3d p_intersect;
      if (cor.geo_type == GeometryType::Plane) {
        double dist = p_inM.dot(cor.geo_plane.head(3)) + cor.geo_plane[3];
        p_intersect = p_inM + dist * cor.geo_plane.head(3);

        p_target.intensity = 100;
        p_source.intensity = 100;
      } else {
        double t = (p_inM - cor.geo_point).dot(cor.geo_normal);
        p_intersect = cor.geo_point - t * cor.geo_normal;

        p_target.intensity = 50;
        p_source.intensity = 50;
      }

      p_target.x = p_intersect[0];
      p_target.y = p_intersect[1];
      p_target.z = p_intersect[2];
      target_cloud.push_back(p_target);
      p_source.x = p_inM[0];
      p_source.y = p_inM[1];
      p_source.z = p_inM[2];
      source_cloud.push_back(p_source);
    }

    sensor_msgs::msg::PointCloud2 target_msg, source_msg;
    pcl::toROSMsg(target_cloud, target_msg);
    pcl::toROSMsg(source_cloud, source_msg);

    auto now = rclcpp::Clock().now();
    target_msg.header.stamp = now;
    target_msg.header.frame_id = "/map";
    source_msg.header = target_msg.header;

    pub_target_cloud_->publish(target_msg);
    pub_source_cloud_->publish(source_msg);
  }

  inline void PublishSplineTrajectory(std::shared_ptr<Trajectory> trajectory,
                                      double min_time, double max_time,
                                      double dt) {
    if (min_time < trajectory->minTime())
      min_time = trajectory->minTime();
    if (max_time > trajectory->maxTime())
      max_time = trajectory->maxTime();

    if (pub_spline_trajectory_->get_subscription_count() != 0) {
      rclcpp::Time t_temp(0);
      std::vector<geometry_msgs::msg::PoseStamped> poses_geo;
      for (double t = min_time; t < max_time; t += dt) {
        SE3d pose = trajectory->pose(t);
        geometry_msgs::msg::PoseStamped poseIinG;
        poseIinG.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
        poseIinG.header.frame_id = "/map";
        auto trans = pose.translation();
        poseIinG.pose.position.x = trans.x();
        poseIinG.pose.position.y = trans.y();
        poseIinG.pose.position.z = trans.z();
        auto quat2 = pose.unit_quaternion();
        poseIinG.pose.orientation.w = quat2.w();
        poseIinG.pose.orientation.x = quat2.x();
        poseIinG.pose.orientation.y = quat2.y();
        poseIinG.pose.orientation.z = quat2.z();
        poses_geo.push_back(poseIinG);
      }
      auto time_now = rclcpp::Clock().now();
      nav_msgs::msg::Path traj_path;
      traj_path.header.stamp = time_now;
      traj_path.header.frame_id = "/map";
      traj_path.poses = poses_geo;

      pub_spline_trajectory_->publish(traj_path);
    }

    if (pub_lidar_trajectory_->get_subscription_count() != 0) {
      rclcpp::Time t_temp(0);
      std::vector<geometry_msgs::msg::PoseStamped> poses_geo;
      for (double t = min_time; t < max_time; t += dt) {
        SE3d pose;
        if (trajectory->GetLidarPose(t, pose)) {
          geometry_msgs::msg::PoseStamped poseIinG;
          poseIinG.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
          poseIinG.header.frame_id = "/map";
          auto trans = pose.translation();
          poseIinG.pose.position.x = trans.x();
          poseIinG.pose.position.y = trans.y();
          poseIinG.pose.position.z = trans.z();
          auto quat2 = pose.unit_quaternion();
          poseIinG.pose.orientation.w = quat2.w();
          poseIinG.pose.orientation.x = quat2.x();
          poseIinG.pose.orientation.y = quat2.y();
          poseIinG.pose.orientation.z = quat2.z();
          poses_geo.push_back(poseIinG);
        }
      }
      auto time_now = rclcpp::Clock().now();
      nav_msgs::msg::Path traj_path;
      traj_path.header.stamp = time_now;
      traj_path.header.frame_id = "/map";
      traj_path.poses = poses_geo;

      pub_lidar_trajectory_->publish(traj_path);
    }
  }

private:
  int iteration_num_;

  rclcpp::Publisher<oa_licalib::msg::PoseArray>::SharedPtr pub_trajectory_raw_;
  rclcpp::Publisher<oa_licalib::msg::PoseArray>::SharedPtr pub_trajectory_est_;
  rclcpp::Publisher<oa_licalib::msg::ImuArray>::SharedPtr pub_imu_raw_array_;
  rclcpp::Publisher<oa_licalib::msg::ImuArray>::SharedPtr pub_imu_est_array_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_target_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_source_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_spline_trajectory_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_lidar_trajectory_;

  static constexpr int UI_WIDTH = 300;

  pangolin::Var<bool> pan_opt_time_offset_;
  pangolin::Var<bool> pan_opt_lidar_intrinsic_;
  pangolin::Var<bool> pan_opt_imu_intrinsic_;
  pangolin::Var<bool> pan_apply_lidar_intrinstic_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Create node with private namespace
  auto node = std::make_shared<rclcpp::Node>("li_calib_node");

  // Get config path parameter
  std::string config_path = node->declare_parameter<std::string>(
      "config_path", "/config/li-calib.yaml");
  if (!config_path.empty() && config_path.substr(0, 1) != "/") {
    config_path = "/" + config_path;
  }
  // Get package path using ament_index
  std::string package_name = "oa_licalib";
  std::string PACKAGE_PATH;
  try {
    PACKAGE_PATH = ament_index_cpp::get_package_share_directory(package_name);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to get package path for %s: %s",
                 package_name.c_str(), e.what());
    rclcpp::shutdown();
    return -1;
  }

  std::string config_file_path = PACKAGE_PATH + config_path;
  YAML::Node config_node;
  try {
    config_node = YAML::LoadFile(config_file_path);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load config file %s: %s",
                 config_file_path.c_str(), e.what());
    rclcpp::shutdown();
    return -1;
  }

  CalibUI calib_ui(config_node, node);

  bool use_gui = config_node["use_gui"].as<bool>();
  if (use_gui) {
    calib_ui.InitGui();
    calib_ui.RenderingLoop();
  } else {
    // calib_ui.Run();
    calib_ui.RunSimulation();
  }

  rclcpp::shutdown();
  return 0;
}
