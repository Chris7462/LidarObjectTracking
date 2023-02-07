#pragma once

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>


class BoxVisualizer: public rclcpp::Node
{
  public:
    BoxVisualizer();
    ~BoxVisualizer() = default;
  private:
    // subscriber
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_subscriber_;

    // publisher
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr box_publisher_;

    void box_vis_callback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr marker_msg);
};
