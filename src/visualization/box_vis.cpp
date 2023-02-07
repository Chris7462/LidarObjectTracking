// C++ header
#include <Eigen/StdVector>

// PCL header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

// local header
#include "lidar_object_tracking/box_vis.hpp"


BoxVisualizer::BoxVisualizer()
  : Node("box_vis")
{
  rclcpp::QoS qos(10);

  marker_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    "clustered_marker", qos, std::bind(&BoxVisualizer::box_vis_callback,
    this, std::placeholders::_1));

  box_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "object_box", qos);
}

void BoxVisualizer::box_vis_callback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr marker_msg)
{
  visualization_msgs::msg::MarkerArray clustered_clouds(*marker_msg);
  int count = 0;
  for (auto& marker: clustered_clouds.markers) {
    Eigen::Vector4f centroid;
    Eigen::Vector4f min;
    Eigen::Vector4f max;

    pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
    for (auto& pts: marker.points) {
      cloud_cluster.push_back(pcl::PointXYZ(pts.x, pts.y, pts.z));
    }

    pcl::compute3DCentroid(cloud_cluster, centroid);
    pcl::getMinMax3D(cloud_cluster, min, max);

    marker.ns = "cube";
    marker.id = count++;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = centroid[0];
    marker.pose.position.y = centroid[1];
    marker.pose.position.z = centroid[2];
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = (max[0]-min[0]);
    marker.scale.y = (max[1]-min[1]);
    marker.scale.z = (max[2]-min[2]);

    if (marker.scale.x < 0.1) {
      marker.scale.x = 0.1;
    }

    if (marker.scale.y < 0.1) {
      marker.scale.y = 0.1;
    }

    if (marker.scale.z < 0.1) {
      marker.scale.z = 0.1;
    }

    marker.color.g = 1.0F;
    marker.color.a = 1.0F;

    marker.lifetime = rclcpp::Duration(0, 5e8);
  }

  box_publisher_->publish(clustered_clouds);
}
