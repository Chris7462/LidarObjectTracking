#include "lidar_object_tracking/box_vis.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BoxVisualizer>());
  rclcpp::shutdown();

  return 0;
}
