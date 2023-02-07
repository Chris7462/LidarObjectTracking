// local header
#include "lidar_object_tracking/ground_removal.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundRemoval>());
  rclcpp::shutdown();

  return 0;
}
