#pragma once

// C++ header
#include <vector>

// OpenCV header
#include <opencv2/opencv.hpp>

// ROS header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// PCL header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class ComponentClustering: public rclcpp::Node
{
  public:
    ComponentClustering();
    ~ComponentClustering() = default;

  private:
    // subscriber
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr elevated_cloud_subscriber_;

    // publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_cloud_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    void elevated_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg);

    // clustering related constants
    int num_grid_;
    float roi_m_;
    int kernel_size_;

    // bounding box related constants
    float pic_scale_;
    float lslope_dist_;
    int lnum_points_;
    int rnd_points_;
    float sensor_height_;
    float thr_height_min_;
    float thr_height_max_;
    float thr_width_min_;
    float thr_width_max_;
    float thr_len_min_;
    float thr_len_max_;
    float thr_area_max_;
    float tpt_per_m3_;
    float min_len_ratio_;
    float thr_ratio_min_;
    float thr_ratio_max_;

    void component_clustering(const pcl::PointCloud<pcl::PointXYZ>& elevated_cloud,
      std::vector<std::vector<int>>& cartesian_data, int& num_cluster);

    void map_cartesian_grid(const pcl::PointCloud<pcl::PointXYZ>& elevated_cloud,
      std::vector<std::vector<int>>& cartesian_data);

    void find_component(std::vector<std::vector<int>>& cartesian_data, int& cluster_id);

    void search(std::vector<std::vector<int>>& cartesian_data, int cluster_id, int x, int y);

    void make_clustered_cloud(const pcl::PointCloud<pcl::PointXYZ>& elevated_cloud,
      std::vector<std::vector<int>>& cartesian_data,
      pcl::PointCloud<pcl::PointXYZRGB>& clustered_cloud);

    std::vector<pcl::PointCloud<pcl::PointXYZ>> box_fitting(
      const pcl::PointCloud<pcl::PointXYZ>& elevated_cloud,
      const std::vector<std::vector<int>>& cartesian_data, int num_cluster);

    void get_clustered_points(const pcl::PointCloud<pcl::PointXYZ>& elevated_cloud,
      const std::vector<std::vector<int>>& cartesian_data,
      std::vector<pcl::PointCloud<pcl::PointXYZ>>& clustered_points);

    void get_bounding_box(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clustered_points,
      std::vector<pcl::PointCloud<pcl::PointXYZ>>& bb_points);

    bool rule_based_filter(const std::vector<cv::Point2f>& pc_points,
      float max_z, int num_points);

    void get_points_in_pc_frame(const cv::Point2f rect_points[],
      std::vector<cv::Point2f>& pc_points, int offset_x, int offset_y);
};
