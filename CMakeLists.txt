cmake_minimum_required(VERSION 3.8)
project(oa_licalib)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -msse4.2")

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(fmt REQUIRED)

# Find OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(angles REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
# find_package(ndt_omp REQUIRED)
# Manual ndt_omp setup since find_package has issues
set(ndt_omp_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/../ndt_omp/include")
set(ndt_omp_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/../../install/ndt_omp/lib/ndt_omp/libndt_omp.a")
find_package(rosbag2_cpp REQUIRED)
find_package(velodyne_msgs REQUIRED)

# Find external dependencies
find_package(Eigen3 REQUIRED)
find_package(PCL 1.12 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Set up Ceres (use system installation)
find_package(Ceres 2.1 REQUIRED)
message(STATUS "Found Ceres version: ${CERES_VERSION}")

# Set up Pangolin (use system installation)
find_package(Pangolin REQUIRED)

message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Found Ceres headers in: ${CERES_INCLUDE_DIRS}")

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FeatureCloud.msg"
  "msg/ImuArray.msg"
  "msg/PoseArray.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${ndt_omp_INCLUDE_DIRS}
  ${boost_INCLUDE_DIR}
)

include_directories(thirdparty/Sophus)

# Library dependencies
set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  angles
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
  ament_index_cpp
  rosbag2_cpp
  velodyne_msgs
)

# Set link libraries used by all binaries
set(thirdparty_libraries
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
  ${Pangolin_LIBRARIES}
  ceres
  ${ndt_omp_LIBRARIES}
  ${Boost_LIBRARIES}
  fmt::fmt
)

# Core calibration library
add_library(lib_calib SHARED
  src/trajectory/se3_trajectory.cpp
  src/trajectory/trajectory_estimator.cpp
  src/trajectory/trajectory_viewer.cpp
  src/calib/lidar_localization.cpp
  src/calib/lidar_ndt_odometry.cpp
  src/calib/surfel_association.cpp
  src/calib/inertial_initializer.cpp
  src/calib/scan_undistortion.cpp
)

ament_target_dependencies(lib_calib ${dependencies})
target_link_libraries(lib_calib ${thirdparty_libraries})

# Wait for message generation
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lib_calib "${cpp_typesupport_target}")

# Main calibration node
add_executable(li_calib_node src/app/li_calib_node.cpp src/calib/calib_helper.cpp)
ament_target_dependencies(li_calib_node ${dependencies})
target_link_libraries(li_calib_node lib_calib ${thirdparty_libraries})
target_link_libraries(li_calib_node "${cpp_typesupport_target}")

# Map evaluation tools
add_executable(map_evaluation_tools src/app/map_evaluation_tools.cpp)
ament_target_dependencies(map_evaluation_tools ${dependencies})
target_link_libraries(map_evaluation_tools ${thirdparty_libraries})

# Install targets
install(TARGETS
  lib_calib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  li_calib_node
  map_evaluation_tools
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  config
  data
  script
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  include/
  DESTINATION include/
)

# Install Python scripts
install(PROGRAMS
  script/PlotCollection.py
  script/liso_plots.py
  script/plot_lidar_intrinsic_data.py
  script/plot_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})
ament_export_include_directories(include)

install(EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()