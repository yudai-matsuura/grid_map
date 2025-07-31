#include "rclcpp/rclcpp.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

class TraversabilityPublisher : public rclcpp::Node
{
public:
    TraversabilityPublisher() : Node("traversability_publisher_ros2")
    {
        publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>("/grid_map", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TraversabilityPublisher::onTimer, this)
        );

        // Initialize grid map
        map_.setFrameId("map");
        map_.setGeometry(grid_map::Length(2.0, 2.0), 0.05);
        map_.add("traversability");
        RCLCPP_INFO(this->get_logger(), "Created map with size %f x %f m (%i x %i cells).",
                map_.getLength().x(), map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
    }

private:
    void onTimer()
    {
        map_.setTimestamp(this->get_clock()->now().nanoseconds());
        map_.get("traversability").setConstant(0);
        grid_map::Matrix& layerData = map_["traversability"];
        double radius = 0.4;
        int cell_count = 0;
        for (grid_map::CircleIterator iterator(map_, grid_map::Position(0.0, 0.0), radius);
            !iterator.isPastEnd(); ++iterator) {
                const grid_map::Index index = *iterator;
                // map_.at("traversability", *iterator) = 1; // Area that can move with Wheel
                layerData(index(0), index(1)) = 1.0;
                cell_count++;
        }
        RCLCPP_INFO(this->get_logger(), "Set %d cells to 1", cell_count);
        auto message = grid_map::GridMapRosConverter::toMessage(map_);
        publisher_->publish(std::move(message));
        RCLCPP_INFO_ONCE(this->get_logger(), "Grid map published.");
    }

    // member variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr publisher_;
    grid_map::GridMap map_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TraversabilityPublisher>());
    rclcpp::shutdown();
    return 0;
}