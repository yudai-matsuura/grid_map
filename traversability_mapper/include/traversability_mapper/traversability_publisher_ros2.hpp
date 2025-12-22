// Copyright (c) 2025 Tohoku Univ. Space Robotics Lab.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TRAVERSABILITY_PUBLISHER_HPP
#define TRAVERSABILITY_PUBLISHER_HPP

#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// #include "traversability_msgs/msg/classified_region.hpp"
// #include "traversability_msgs/msg/grid_cell_data.hpp"
#include "lbr_terrain_analysis/msg/grid_cell_geometric_data.hpp"
#include "lbr_terrain_analysis/msg/grid_cell_array.hpp"


class TraversabilityPublisher : public rclcpp::Node
{
public:
  TraversabilityPublisher();

private:
  // Classify region
  // void classifiedRegionCallback(const traversability_msgs::msg::ClassifiedRegion::SharedPtr msg);

  void gridCellArrayCallback(const lbr_terrain_analysis::msg::GridCellArray::SharedPtr msg);
  // void gridCellArrayCallback(const traversability_msgs::msg::GridCellArray::SharedPtr msg);

  // make gradation color
  Eigen::Vector3f getGradationColor(float value);

  // project link to 2D map
  void projection_timer_callback();

  // Transform
  std::optional<geometry_msgs::msg::TransformStamped> lookupTransform(
    const std::string & target_frame,
    const std::string & source_frame);

  // Transform point
  std::optional<geometry_msgs::msg::PointStamped> transformPoint(
    const geometry_msgs::msg::PointStamped & point_in,
    const geometry_msgs::msg::TransformStamped & transform);

  // Publish robot path
  void updateAndPublishPath(geometry_msgs::msg::TransformStamped & t_2d);



  // Publisher
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Subscriber
  // rclcpp::Subscription<traversability_msgs::msg::ClassifiedRegion>::SharedPtr classified_region_sub_;
  rclcpp::Subscription<lbr_terrain_analysis::msg::GridCellArray>::SharedPtr grid_cell_array_sub_;
  // rclcpp::Subscription<traversability_msgs::msg::GridCellArray>::SharedPtr grid_cell_array_sub_;

  // Variables
  grid_map::GridMap map_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr projection_timer_;
  nav_msgs::msg::Path path_msg_;
};


#endif // TRAVERSABILITY_PUBLISHER_HPP