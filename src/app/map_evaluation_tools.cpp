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

#include <rclcpp/rclcpp.hpp>
#include <utils/map_evaluation_utils.h>

using namespace std;

int main( int argc, char** argv ) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("map_evaluation_tools");
  
  std::string map_path;
  int step_size;
  double radius;
  bool punish_solitar_points;
  int min_neighbors;

  node->declare_parameter("map_path", std::string(" "));
  node->declare_parameter("step_size", 1);
  node->declare_parameter("radius", 0.3);
  node->declare_parameter("min_neighbors", 15);
  node->declare_parameter("punish_solitar_points", false);

  map_path = node->get_parameter("map_path").as_string();
  step_size = node->get_parameter("step_size").as_int();
  radius = node->get_parameter("radius").as_double();
  min_neighbors = node->get_parameter("min_neighbors").as_int();
  punish_solitar_points = node->get_parameter("punish_solitar_points").as_bool();

  MapEvaluationTool met(map_path, step_size, radius, min_neighbors, punish_solitar_points);
  met.Process();

  rclcpp::shutdown();
  return 0;
}
