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

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Initialize
  projection_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&TraversabilityPublisher::projection_timer_callback, this));

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

  float suitability = msg->wheel_suitability;
  float normalized_suitability = suitability / 100;
  normalized_suitability = std::max(0.0f, std::min(1.0f, normalized_suitability));

  traversability_value = normalized_suitability;

  // Clamp value to 25 ~ 75 for fuzzy
  float clamped_suitability = std::max(25.0f, std::min(suitability, 75.0f));
  float renormalized_for_color = (clamped_suitability - 25.0f) / 50.0f; // 50.0fは(75.0f-25.0f)

  // Map color
  Eigen::Vector3f rgb = getRainbowColor(1.0f - renormalized_for_color);
  grid_map::colorVectorToValue(rgb, packed_color);

  const sensor_msgs::msg::PointCloud2& pointcloud = msg->region_pointcloud;
  try {
    // Get the robot's current position
    geometry_msgs::msg::TransformStamped robot_pose_transform;
    std::string robot_frame = "base_link";
    robot_pose_transform = tf_buffer_->lookupTransform(map_.getFrameId(), robot_frame, tf2::TimePointZero);

    // 2. Move the center of the map to the robot's current position.
    grid_map::Position robot_position(robot_pose_transform.transform.translation.x, robot_pose_transform.transform.translation.y);
    map_.move(robot_position);

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get robot pose to move map: %s", ex.what());
    // Processing continues even if the map cannot be moved.
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
      // Prepare points in the base_link coordinate system
      geometry_msgs::msg::PointStamped point_in_source_frame;
      point_in_source_frame.header.frame_id = pointcloud.header.frame_id;
      point_in_source_frame.header.stamp = pointcloud.header.stamp;
      point_in_source_frame.point.x = *iter_x;
      point_in_source_frame.point.y = *iter_y;
      point_in_source_frame.point.z = *iter_z;

      // Transform points to the odom coordinate system
      geometry_msgs::msg::PointStamped point_in_target_frame;
      tf2::doTransform(point_in_source_frame, point_in_target_frame, transform_stamped);

      // Use the transformed coordinates
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

Eigen::Vector3f TraversabilityPublisher::getRainbowColor(float value) {
  Eigen::Vector3f rgb(0.0f, 0.0f, 0.0f);
  if (value < 0.5f) {
    float t = value / 0.5f;
    rgb.x() = t;
    rgb.y() = 1.0f;
    rgb.z() = 0.0f;
  } else {
    float t = (value - 0.5f) / 0.5f; 
    rgb.x() = 1.0f;
    rgb.y() = 1.0f - t;
    rgb.z() = 0.0f;
  }
  return rgb;
}

void TraversabilityPublisher::projection_timer_callback()
{
  geometry_msgs::msg::TransformStamped t;
  try {
      // Get base_link pose
      t = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get 'base_link' transform: %s", ex.what());
      return;
  }

  // Make new frame
  geometry_msgs::msg::TransformStamped t_2d;
  t_2d.header.stamp = this->get_clock()->now();
  t_2d.header.frame_id = "odom";
  t_2d.child_frame_id = "base_link_2d";

  // z = 0
  t_2d.transform.translation.x = t.transform.translation.x;
  t_2d.transform.translation.y = t.transform.translation.y;
  t_2d.transform.translation.z = 0.0;

  // Get RPY from quaternion
  tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Make new Quaternion by yaw only
  tf2::Quaternion q_2d;
  q_2d.setRPY(0, 0, yaw); // roll = 0 , pitch = 0

  t_2d.transform.rotation.x = q_2d.x();
  t_2d.transform.rotation.y = q_2d.y();
  t_2d.transform.rotation.z = q_2d.z();
  t_2d.transform.rotation.w = q_2d.w();

  tf_broadcaster_->sendTransform(t_2d);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversabilityPublisher>());
  rclcpp::shutdown();
  return 0;
}