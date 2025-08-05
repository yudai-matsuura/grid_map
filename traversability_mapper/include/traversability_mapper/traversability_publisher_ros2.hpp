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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "traversability_msgs/msg/classified_region.hpp"


class TraversabilityPublisher : public rclcpp::Node
{
public:
    TraversabilityPublisher();

private:
    void classifiedRegionCallback(const traversability_msgs::msg::ClassifiedRegion::SharedPtr msg);

    // Publisher
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
    // Subscriber
    rclcpp::Subscription<traversability_msgs::msg::ClassifiedRegion>::SharedPtr classified_region_sub_;
    // Variables
    grid_map::GridMap map_;
};


#endif // TRAVERSABILITY_PUBLISHER_HPP