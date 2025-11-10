#include "traversability_mapper/traversability_publisher_ros2.hpp"

TraversabilityPublisher::TraversabilityPublisher() : Node("traversability_publisher_ros2")
{
  // Publisher
  grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/grid_map", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/robot_path", 10);

  // Subscriber
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

  path_msg_.header.frame_id = "nav";

  // Initialize grid map
  map_.setFrameId("nav");
  map_.setGeometry(grid_map::Length(10.0, 10.0), 0.7, grid_map::Position(0.0, 0.0));
  map_.add("roughness", 0.0);
  map_.add("slope_angle", 0.0);
  map_.add("traversability", 0.0);
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
  std::string source_frame = "tcp_base";
  std::string target_frame = map_.getFrameId();  // nav
  auto tf_opt = lookupTransform(target_frame, source_frame);
  if (!tf_opt) {
    RCLCPP_WARN(this->get_logger(), "TF not available between %s and %s",
                source_frame.c_str(), target_frame.c_str());
    return;
  }
  auto transform_stamped = *tf_opt;
  // move map center to robot position
  grid_map::Position robot_pos(transform_stamped.transform.translation.x,
    transform_stamped.transform.translation.y);
  map_.move(robot_pos);
  // Wight
  float w_r = 0.5;
  float w_s = 0.5;

  for (const auto &cell : msg->cells) {
    // tcp_base → nav
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
    float geometric_traversability = 1.0f - (w_r * roughness + w_s * slope_angle);
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

Eigen::Vector3f TraversabilityPublisher::getGradationColor(float value)
{
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
      t = tf_buffer_->lookupTransform("nav", "tcp_base", tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get 'nav' transform: %s", ex.what());
      return;
  }
  // Convert to 2D and broadcast
  geometry_msgs::msg::TransformStamped t_2d;
  t_2d.header.stamp = this->get_clock()->now();
  t_2d.header.frame_id = "nav";
  t_2d.child_frame_id = "base_link_2d";
  t_2d.transform.translation.x = t.transform.translation.x;
  t_2d.transform.translation.y = t.transform.translation.y;
  t_2d.transform.translation.z = 0.0;
  // Get RPY from quaternion
  tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  tf2::Quaternion q_2d;
  q_2d.setRPY(0, 0, yaw); // yaw only
  t_2d.transform.rotation.x = q_2d.x();
  t_2d.transform.rotation.y = q_2d.y();
  t_2d.transform.rotation.z = q_2d.z();
  t_2d.transform.rotation.w = q_2d.w();

  tf_broadcaster_->sendTransform(t_2d);
  // Robot path
  updateAndPublishPath(t_2d);
}

void TraversabilityPublisher::updateAndPublishPath(geometry_msgs::msg::TransformStamped & t_2d)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = t_2d.header;
  pose.pose.position.x = t_2d.transform.translation.x;
  pose.pose.position.y = t_2d.transform.translation.y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = t_2d.transform.rotation;
  // Add when robot move certain distance
  const double min_distance = 0.1;
  if (!path_msg_.poses.empty()) {
    const auto & last_pose = path_msg_.poses.back();
    double dx = pose.pose.position.x - last_pose.pose.position.x;
    double dy = pose.pose.position.y - last_pose.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_distance) {
      return;
    }
  }
  // Update
  path_msg_.header.stamp = this->get_clock()->now();
  path_msg_.poses.push_back(pose);
  // Limit number of stored poses
  const size_t max_points = 1000;
  if (path_msg_.poses.size() > max_points) {
    path_msg_.poses.erase(path_msg_.poses.begin());
  }
  // Publish path
  path_pub_->publish(path_msg_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TraversabilityPublisher>());
  rclcpp::shutdown();
  return 0;
}