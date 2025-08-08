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

#include "traversability_mapper/traversability_publisher_ros2.hpp"

TraversabilityPublisher::TraversabilityPublisher() : Node("traversability_publisher_ros2")
{
  // Publisher
  grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/grid_map", 10);

  // Subscriber
  classified_region_sub_ = this->create_subscription<traversability_msgs::msg::ClassifiedRegion>(
      "/classified_region", 10, std::bind(&TraversabilityPublisher::classifiedRegionCallback, this, std::placeholders::_1));

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize grid map
  map_.setFrameId("odom");
  map_.setGeometry(grid_map::Length(5.0, 5.0), 0.05);
  map_.add("traversability", 0.0);
  map_.add("color", 0.0);
  map_.setBasicLayers({"traversability", "color"});

  RCLCPP_INFO(this->get_logger(), "Created map with size %f x %f m (%i x %i cells).",
          map_.getLength().x(), map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
}

void TraversabilityPublisher::classifiedRegionCallback(const traversability_msgs::msg::ClassifiedRegion::SharedPtr msg)
{
  // RCLCPP_INFO(this->get_logger(), "classifiedRegionCallback is called !");

  map_.setTimestamp(this->get_clock()->now().nanoseconds());

  float traversability_value = 0.0;
  float packed_color = 0.0;


  if (msg->classification == traversability_msgs::msg::ClassifiedRegion::CLASSIFICATION_WHEEL) {
      traversability_value = 1.0;
      Eigen::Vector3f rgb(0.0f, 0.0f, 1.0f);  // Blue (R, G, B)
      grid_map::colorVectorToValue(rgb, packed_color);
  RCLCPP_INFO(this->get_logger(), "classification WHEEL !");
  } else if (msg->classification == traversability_msgs::msg::ClassifiedRegion::CLASSIFICATION_GRIPPER) {
      traversability_value = 2.0;
      Eigen::Vector3f rgb(0.0f, 1.0f, 0.0f);  // Green (R, G, B)
      grid_map::colorVectorToValue(rgb, packed_color);
      RCLCPP_INFO(this->get_logger(), "classification GRIPPER !");
  } else {
      return;
  }

  const sensor_msgs::msg::PointCloud2& pointcloud = msg->region_pointcloud;
  try {
    // 1. ロボットの現在位置（odom座標系でのbase_linkの位置）を取得
    geometry_msgs::msg::TransformStamped robot_pose_transform;
    std::string robot_frame = "base_link"; // ロボットの基準フレーム
    robot_pose_transform = tf_buffer_->lookupTransform(map_.getFrameId(), robot_frame, tf2::TimePointZero);

    // 2. 地図の中心をロボットの現在位置に移動
    grid_map::Position robot_position(robot_pose_transform.transform.translation.x, robot_pose_transform.transform.translation.y);
    map_.move(robot_position);

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get robot pose to move map: %s", ex.what());
    // 地図を移動できなくても処理は続行する
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(map_.getFrameId(), pointcloud.header.frame_id, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                pointcloud.header.frame_id.c_str(), map_.getFrameId().c_str(), ex.what());
    return;
  }

for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x"), iter_y(pointcloud, "y"), iter_z(pointcloud, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
      // 修正点: base_link座標系の点を準備
      geometry_msgs::msg::PointStamped point_in_source_frame;
      point_in_source_frame.header.frame_id = pointcloud.header.frame_id;
      point_in_source_frame.header.stamp = pointcloud.header.stamp;
      point_in_source_frame.point.x = *iter_x;
      point_in_source_frame.point.y = *iter_y;
      point_in_source_frame.point.z = *iter_z;

      // 修正点: odom座標系に点を変換
      geometry_msgs::msg::PointStamped point_in_target_frame;
      tf2::doTransform(point_in_source_frame, point_in_target_frame, transform_stamped);

      // 修正点: 変換後の座標を使用
      grid_map::Position point_position(point_in_target_frame.point.x, point_in_target_frame.point.y);
      grid_map::Index index;
      if (map_.getIndex(point_position, index)) {
          map_.at("traversability", index) = traversability_value;
          map_.at("color", index) = packed_color;
      }
  }

  auto output_msg = grid_map::GridMapRosConverter::toMessage(map_);
  grid_map_pub_->publish(std::move(output_msg));
  // RCLCPP_INFO(this->get_logger(), "grid map published !");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversabilityPublisher>());
  rclcpp::shutdown();
  return 0;
}