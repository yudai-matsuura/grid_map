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
  // classified_region_sub_ = this->create_subscription<traversability_msgs::msg::ClassifiedRegion>(
  //   "/classified_region", 10, std::bind(&TraversabilityPublisher::classifiedRegionCallback, this, std::placeholders::_1));

  // grid_cell_array_sub_ = this->create_subscription<traversability_msgs::msg::GridCellArray>(
  //   "/grid_cells", 10, std::bind(&TraversabilityPublisher::gridCellArrayCallback, this, std::placeholders::_1));

  grid_cell_array_sub_ = this->create_subscription<lbr_msgs::msg::GridCellArray>(
    "/grid_cells", 10, std::bind(&TraversabilityPublisher::gridCellArrayCallback, this, std::placeholders::_1));

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
  map_.setGeometry(grid_map::Length(10.0, 10.0), 0.3, grid_map::Position(0.0, 0.0));
  map_.add("traversability", 0.0);
  map_.add("roughness", 0.0);
  map_.add("slope_angle", 0.0);
  map_.add("roughness_color", 0.0);
  map_.add("slope_color", 0.0);
  map_.add("traversability_color", 0.0);
  map_.setBasicLayers({"traversability", "traversability_color"});

  RCLCPP_INFO(this->get_logger(), "Created map with size %f x %f m (%i x %i cells).",
          map_.getLength().x(), map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
}

void TraversabilityPublisher::gridCellArrayCallback(
  const lbr_msgs::msg::GridCellArray::SharedPtr msg)
{
  map_.setTimestamp(this->get_clock()->now().nanoseconds());

  // move map by robot position
std::string source_frame = "base_link";
std::string target_frame = map_.getFrameId();  // 通常 "odom"
auto tf_opt = lookupTransform(target_frame, source_frame);
if (!tf_opt) {
  RCLCPP_WARN(this->get_logger(), "TF not available between %s and %s",
              source_frame.c_str(), target_frame.c_str());
  return;
}
auto transform_stamped = *tf_opt;

float w_r = 0.5;
float w_s = 0.5;

for (const auto &cell : msg->cells) {
  // base_link → odom
  geometry_msgs::msg::PointStamped point_in, point_out;
  point_in.header.frame_id = source_frame;
  point_in.point.x = cell.position.x;
  point_in.point.y = cell.position.y;
  point_in.point.z = cell.position.z;

  auto transformed = transformPoint(point_in, transform_stamped);
  if (!transformed) continue;

  grid_map::Position pos(transformed->point.x, transformed->point.y);
  grid_map::Index index;
  if (!map_.getIndex(pos, index)) continue;

  // Roughness
  float roughness = cell.roughness;
  map_.at("roughness", index) = roughness;
  Eigen::Vector3f rgb_r = getGradationColor(roughness);
  float packed_color_r;
  grid_map::colorVectorToValue(rgb_r, packed_color_r);
  map_.at("roughness_color", index) = packed_color_r;

  // Slope angle
  float slope_angle = cell.angle;
  map_.at("slope_angle", index) = slope_angle;
  Eigen::Vector3f rgb_s = getGradationColor(slope_angle);
  float packed_color_s;
  grid_map::colorVectorToValue(rgb_s, packed_color_s);
  map_.at("slope_color", index) = packed_color_s;

  // Geometric traversability
  float geometric_traversability = w_r * (1.0f - roughness) + w_s * (1.0f - slope_angle);
  geometric_traversability = std::clamp(geometric_traversability, 0.0f, 1.0f);
  map_.at("traversability", index) = geometric_traversability;
  float inverse_traversability = 1.0f - geometric_traversability;
  Eigen::Vector3f rgb_t = getGradationColor(inverse_traversability);
  float packed_color_t;
  grid_map::colorVectorToValue(rgb_t, packed_color_t);
  map_.at("traversability_color", index) = packed_color_t;
}

  // === Publish grid map ===
  auto output_msg = grid_map::GridMapRosConverter::toMessage(map_);
  grid_map_pub_->publish(std::move(output_msg));
}


// void TraversabilityPublisher::classifiedRegionCallback(const traversability_msgs::msg::ClassifiedRegion::SharedPtr msg)
// {
//   map_.setTimestamp(this->get_clock()->now().nanoseconds());
//   float traversability_value = 0.0;
//   float packed_color = 0.0;
//   float suitability = msg->wheel_suitability;
//   float normalized_suitability = suitability / 100;
//   normalized_suitability = std::max(0.0f, std::min(1.0f, normalized_suitability));
//   traversability_value = normalized_suitability;

//   // Clamp value to kSuitabilityMin ~ kSuitabilityMax for fuzzy
//   const float kSuitabilityMin = 25.0f;
//   const float kSuitabilityMax = 75.0f;
//   float clamped_suitability = std::max(kSuitabilityMin, std::min(suitability, kSuitabilityMax));
//   float renormalized_for_color = (clamped_suitability - kSuitabilityMin) / (kSuitabilityMax - kSuitabilityMin);

//   // Map color
//   Eigen::Vector3f rgb = getGradationColor(1.0f - renormalized_for_color);
//   grid_map::colorVectorToValue(rgb, packed_color);

//   // Get the robot's current position
//   std::string source_frame = "base_link";
//   std::string target_frame = map_.getFrameId(); // odom frame
//   auto robot_tf = lookupTransform(target_frame, source_frame);
//   // Move the center of the map to the robot's current position.
//   if (robot_tf) {
//     grid_map::Position robot_position(robot_tf->transform.translation.x, robot_tf->transform.translation.y);
//     map_.move(robot_position);
//   }

//   // Transform point cloud to odom frame
//   const sensor_msgs::msg::PointCloud2 & pointcloud = msg->region_pointcloud;
//   auto transform_stamped_opt = lookupTransform(map_.getFrameId(), pointcloud.header.frame_id);
//   if (!transform_stamped_opt) {
//     RCLCPP_WARN(this->get_logger(), "Could not get transform from %s to %s",
//                 pointcloud.header.frame_id.c_str(), map_.getFrameId().c_str());
//     return;
//   }
//   auto transform_stamped = *transform_stamped_opt;

//   // Process each point in the point cloud
// for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x"), iter_y(pointcloud, "y"), iter_z(pointcloud, "z");
//       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
//   {
//       // Prepare points in the source frame (from point cloud header)
//       geometry_msgs::msg::PointStamped point_in_source_frame;
//       point_in_source_frame.header.frame_id = pointcloud.header.frame_id;
//       point_in_source_frame.header.stamp = pointcloud.header.stamp;
//       point_in_source_frame.point.x = *iter_x;
//       point_in_source_frame.point.y = *iter_y;
//       point_in_source_frame.point.z = *iter_z;

//       // Transform points to the odom coordinate
//       auto transformed_point_opt = transformPoint(point_in_source_frame, transform_stamped);
//       if (!transformed_point_opt) {
//           RCLCPP_WARN(this->get_logger(), "Could not transform point");
//           continue;
//       }

//       grid_map::Position point_position(transformed_point_opt->point.x, transformed_point_opt->point.y);
//       grid_map::Index index;
//       if (map_.getIndex(point_position, index)) {
//           // Write score and color information to cells
//           map_.at("traversability", index) = traversability_value;
//           map_.at("traversability_color", index) = packed_color;
//       }
//   }
//   // Publish grid map
//   auto output_msg = grid_map::GridMapRosConverter::toMessage(map_);
//   grid_map_pub_->publish(std::move(output_msg));
// }

Eigen::Vector3f TraversabilityPublisher::getGradationColor(float value) {
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

std::optional<geometry_msgs::msg::TransformStamped> TraversabilityPublisher::lookupTransform(
  const std::string & target_frame,
  const std::string & source_frame)
{
  try {
    return tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s",
                source_frame.c_str(), target_frame.c_str());
    return std::nullopt;
  }
}

std::optional<geometry_msgs::msg::PointStamped> TraversabilityPublisher::transformPoint(
  const geometry_msgs::msg::PointStamped & point_in,
  const geometry_msgs::msg::TransformStamped & transform)
{
  try {
    geometry_msgs::msg::PointStamped point_out;
    tf2::doTransform(point_in, point_out, transform);
    return point_out;
  } catch (tf2::TransformException & ex){
    RCLCPP_WARN(this->get_logger(), "Could not transform point");
    return std::nullopt;
  }
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