#pragma once

// C++ header
#include <vector>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// PCL header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local header
#include "lidar_object_tracking/cell.hpp"


class GroundRemoval: public rclcpp::Node
{
  public:
    GroundRemoval();
    ~GroundRemoval() = default;

  private:
    // subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr top_pc_subscriber_;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr elevated_cloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_cloud_publisher_;

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg);

    float min_dist_;      // rMin
    float max_dist_;      // rMax
    int num_bin_;         // numBin
    int num_channel_;     // numChannel
    float min_height_;    // tHmin
    float max_height_;    // tHmax
    float sensor_height_; // hSensor
    float hdiff_;         // tHDiff

    void remove_ground(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr elevated_cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud);

    void distance_filter(const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
      pcl::PointCloud<pcl::PointXYZ>& filtered_cloud);

    void mapping_polar_grid(const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
      std::vector<std::vector<Cell>>& polar_data);

    void get_cell_index_from_point(float x, float y, int& ch, int& bin);

    void compute_height_diff_adjacent_cell(std::vector<Cell>& channel_data);

    void apply_median_filter(std::vector<std::vector<Cell>>& polar_data);

    void outlier_filter(std::vector<std::vector<Cell>>& polar_data);
};
