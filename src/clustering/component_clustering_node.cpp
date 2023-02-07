// local header
#include "lidar_object_tracking/component_clustering.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComponentClustering>());
  rclcpp::shutdown();

  return 0;
}
