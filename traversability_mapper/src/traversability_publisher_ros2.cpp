#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <limits>

#include "traversability_msgs/msg/region_data.hpp"

class TraversabilityPublisher : public rclcpp::Node
{
public:
    TraversabilityPublisher() : Node("traversability_publisher_ros2")
    {
        // Publisher
        grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/grid_map", 10);

        // Subscriber
        region_data_sub_ = this->create_subscription<traversability_msgs::msg::RegionData>(
            "/region_data", 10, std::bind(&TraversabilityPublisher::regionCallback, this, std::placeholders::_1));

        // Initialize grid map
        map_.setFrameId("base_link");
        map_.setGeometry(grid_map::Length(5.0, 5.0), 0.05);
        map_.add("wheel_area", 0.0);
        map_.add("gripper_area", 0.0);
        // map_.add("traversability");
        RCLCPP_INFO(this->get_logger(), "Created map with size %f x %f m (%i x %i cells).",
                map_.getLength().x(), map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
    }

private:
    void regionCallback(const traversability_msgs::msg::RegionData::SharedPtr msg)
    {
        map_.setTimestamp(this->get_clock()->now().nanoseconds());
        map_.get("wheel_area").setConstant(std::numeric_limits<float>::quiet_NaN());
        map_.get("gripper_area").setConstant(std::numeric_limits<float>::quiet_NaN());

        // get roughness score and pointcloud from message
        double roughness = msg->roughness;
        const sensor_msgs::msg::PointCloud2 & pointcloud = msg->region_pointcloud;
        // judge gripper or wheel
        // int traversability_value;
        std::string target_layer = "";
        // std::string other_layer = "";
        if (roughness < 0.4) {
            target_layer = "wheel_area"; //Wheel
            // other_layer = "gripper_area";
        } else {
            target_layer = "gripper_area"; //Gripper
            // other_layer = "wheel_area";
        }
        // project pointcloud to gid map
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(pointcloud, "y");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
            grid_map::Position point_position(*iter_x, *iter_y);

            grid_map::Index index;
            if (map_.getIndex(point_position, index)) {
                map_.at(target_layer, index) = 1.0;
                // map_.at(other_layer, index) = 0.0;
            }
        }
        auto output_msg = grid_map::GridMapRosConverter::toMessage(map_);
        grid_map_pub_->publish(std::move(output_msg));
        RCLCPP_INFO_ONCE(this->get_logger(), "Grid map published.");
    }

    // member variables
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
    rclcpp::Subscription<traversability_msgs::msg::RegionData>::SharedPtr region_data_sub_;
    grid_map::GridMap map_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TraversabilityPublisher>());
    rclcpp::shutdown();
    return 0;
}