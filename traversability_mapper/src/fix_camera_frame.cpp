#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class FrameIdFixer : public rclcpp::Node
{
public:
  FrameIdFixer()
  : Node("fix_camera_frame")
  {
    // Subscriber
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/hcru2/pt_stereo_rect/left/image", 10,
      std::bind(&FrameIdFixer::imageCallback, this, std::placeholders::_1));

    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/hcru2/pt_stereo_rect/right/camera_info", 10,
      std::bind(&FrameIdFixer::infoCallback, this, std::placeholders::_1));

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/hcru2/pt_stereo_sgm/depth", 10,
      std::bind(&FrameIdFixer::depthCallback, this, std::placeholders::_1));

    // Publisher
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/fixed/left/image", 10);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/fixed/left/camera_info", 10);
    depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/fixed/left/depth", 10);

    RCLCPP_INFO(this->get_logger(), "FixCameraFrame node started.");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto fixed_msg = *msg;
    fixed_msg.header.frame_id = "camera_left";
    image_pub_->publish(fixed_msg);
  }

  void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    auto fixed_msg = *msg;
    fixed_msg.header.frame_id = "camera_left";
    info_pub_->publish(fixed_msg);
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    auto fixed_msg = *msg;
    fixed_msg.header.frame_id = "camera_left";
    depth_pub_->publish(fixed_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrameIdFixer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
