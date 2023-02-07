// c++ header
#include <cmath>
#include <algorithm>

// ROS header
#include <pcl_conversions/pcl_conversions.h>

// local header
#include "lidar_object_tracking/gauss_blur.hpp"
#include "lidar_object_tracking/ground_removal.hpp"


GroundRemoval::GroundRemoval()
  : Node("ground_removal_node")
{
  this->declare_parameter("min_dist", 3.0);
  this->declare_parameter("max_dist", 120.0);
  this->declare_parameter("num_bin", 120);
  this->declare_parameter("num_channel", 80);
  this->declare_parameter("min_height", -1.9);
  this->declare_parameter("max_height", -1.0);
  this->declare_parameter("sensor_height", 1.73);
  this->declare_parameter("hdiff", 0.4);

  // load from parameter if provided
  min_dist_ = static_cast<float>(this->get_parameter("min_dist").get_parameter_value().get<double>());
  max_dist_ = static_cast<float>(this->get_parameter("max_dist").get_parameter_value().get<double>());
  num_bin_ = this->get_parameter("num_bin").get_parameter_value().get<int>();
  num_channel_ = this->get_parameter("num_channel").get_parameter_value().get<int>();
  min_height_ = static_cast<float>(this->get_parameter("min_height").get_parameter_value().get<double>());
  max_height_ = static_cast<float>(this->get_parameter("max_height").get_parameter_value().get<double>());
  sensor_height_ = static_cast<float>(this->get_parameter("sensor_height").get_parameter_value().get<double>());
  hdiff_ = static_cast<float>(this->get_parameter("hdiff").get_parameter_value().get<double>());

  rclcpp::QoS qos(10);

  top_pc_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "kitti/velo/pointcloud", qos, std::bind(&GroundRemoval::point_cloud_callback,
    this, std::placeholders::_1));

  elevated_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "elevated_cloud", qos);

  ground_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "ground_cloud", qos);
}

void GroundRemoval::point_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr elevated_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());

  fromROSMsg(*pc_msg, *input_cloud);

  remove_ground(input_cloud, elevated_cloud, ground_cloud);

  rclcpp::Time current_time = rclcpp::Node::now();

  sensor_msgs::msg::PointCloud2 elevated_cloud_msg;
  pcl::toROSMsg(*elevated_cloud, elevated_cloud_msg);
  elevated_cloud_msg.header.frame_id = "lidar_link";  //pc_msg->header.frame_id;
  elevated_cloud_msg.header.stamp = current_time;
  elevated_cloud_publisher_->publish(elevated_cloud_msg);

  sensor_msgs::msg::PointCloud2 ground_cloud_msg;
  pcl::toROSMsg(*ground_cloud, ground_cloud_msg);
  ground_cloud_msg.header.frame_id = "lidar_link";  //pc_msg->header.frame_id;
  ground_cloud_msg.header.stamp = current_time;
  ground_cloud_publisher_->publish(ground_cloud_msg);
}

void GroundRemoval::remove_ground(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr input_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr elevated_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud)
{
  // Filter point cloud by the min and max distance threshold
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  distance_filter(*input_cloud, *filtered_cloud);

  // Convert the data to polar grid
  std::vector<std::vector<Cell>> polar_data(num_channel_, std::vector<Cell>(num_bin_));
  mapping_polar_grid(*filtered_cloud, polar_data);

  for (int channel = 0; channel < num_channel_; ++channel) {
    for (int bin = 0; bin < num_bin_; ++bin) {
      float z = polar_data[channel][bin].get_min_z();
      if (z > min_height_ && z < max_height_) {
        polar_data[channel][bin].update_height(z);
      } else if (z > max_height_) {
        polar_data[channel][bin].update_height(sensor_height_);
      } else {
        polar_data[channel][bin].update_height(min_height_);
      }
    }
    gauss_smoothing(polar_data[channel], 1, 3);

    compute_height_diff_adjacent_cell(polar_data[channel]);

    for (int bin = 0; bin < num_bin_; ++bin) {
      if (polar_data[channel][bin].get_smoothed() < max_height_ &&
          polar_data[channel][bin].get_hdiff() < hdiff_) {
        polar_data[channel][bin].update_ground();
      } else if (polar_data[channel][bin].get_height() < max_height_ &&
          polar_data[channel][bin].get_hdiff() < hdiff_) {
        polar_data[channel][bin].update_ground();
      }
    }
  }

  // implement median_filter
  apply_median_filter(polar_data);

  // smoothing spot with outlier?
  outlier_filter(polar_data);

  for (const auto& point: *filtered_cloud) {
    int ch;
    int bin;
    get_cell_index_from_point(point.x, point.y, ch, bin);

    if (ch < 0 || ch >= num_channel_ || bin < 0 || bin >= num_bin_) {
      continue;
    }

    if (polar_data[ch][bin].is_this_ground()) {
      float h_ground = polar_data[ch][bin].get_hground();
      if (point.z < (h_ground +0.25F)) {
        ground_cloud->push_back(point);
      } else {
        elevated_cloud->push_back(point);
      }
    } else {
      elevated_cloud->push_back(point);
    }
  }
}

void GroundRemoval::distance_filter(const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
  pcl::PointCloud<pcl::PointXYZ>& filtered_cloud)
{
  for (const auto& point: input_cloud) {
    float distance = std::sqrt(point.x * point.x + point.y * point.y);
    if (distance < min_dist_ || distance > max_dist_) {
      continue;
    } else {
      filtered_cloud.push_back(point);
    }
  }
}

void GroundRemoval::mapping_polar_grid(const pcl::PointCloud<pcl::PointXYZ>& input_cloud,
  std::vector<std::vector<Cell>>& polar_data)
{
  for (auto& point: input_cloud) {
    int ch, bin;
    get_cell_index_from_point(point.x, point.y, ch, bin);
    if (ch < 0 || ch >= num_channel_ || bin < 0 || bin >= num_bin_) {
      continue;
    } else {
      polar_data[ch][bin].update_min_z(point.z);
    }
  }
}

void GroundRemoval::get_cell_index_from_point(float x, float y, int& ch, int& bin)
{
  float distance = std::sqrt(x * x + y * y);

  // normalize
  float ch_p = (std::atan2(y, x) + M_PI) / (2 * M_PI);
  float bin_p = (distance - min_dist_) / (max_dist_ - min_dist_);

  // index
  ch = std::floor(ch_p * num_channel_);
  bin = std::floor(bin_p * num_bin_);
}

void GroundRemoval::compute_height_diff_adjacent_cell(std::vector<Cell>& channel_data)
{
  for (int i = 0; i < num_channel_; ++i) {
    float hdiff{};
    if (i == 0) {
      hdiff = channel_data[i].get_height() - channel_data[i+1].get_height();
    } else if (i == num_channel_ - 1) {
      hdiff = channel_data[i].get_height() - channel_data[i-1].get_height();
    } else {
      float pre_hdiff = channel_data[i].get_height() - channel_data[i-1].get_height();
      float post_hdiff = channel_data[i].get_height() - channel_data[i+1].get_height();
      hdiff = (pre_hdiff > post_hdiff ? pre_hdiff : post_hdiff);
    }
    channel_data[i].update_hdiff(hdiff);
  }
}

void GroundRemoval::apply_median_filter(std::vector<std::vector<Cell>>& polar_data)
{
  for (int ch = 1; ch < num_channel_-1; ++ch) {
    for (int bin = 1; bin < num_bin_-1; ++bin) {
      if (!polar_data[ch][bin].is_this_ground()) {
        // target cell is non-ground and surrounded by ground cells
        if (polar_data[ch][bin+1].is_this_ground() &&
            polar_data[ch][bin-1].is_this_ground() &&
            polar_data[ch+1][bin].is_this_ground() &&
            polar_data[ch-1][bin].is_this_ground()) {
          std::vector<float> sur {polar_data[ch][bin+1].get_height(),
                                  polar_data[ch][bin-1].get_height(),
                                  polar_data[ch+1][bin].get_height(),
                                  polar_data[ch-1][bin].get_height()};
          std::sort(sur.begin(), sur.end());
          polar_data[ch][bin].update_height((sur[1]+sur[2]) * 0.5F);
          polar_data[ch][bin].update_ground();
        }
      }
    }
  }
}

void GroundRemoval::outlier_filter(std::vector<std::vector<Cell>>& polar_data)
{
  for (int ch = 1; ch < num_channel_-1; ++ch) {
    for (int bin = 1; bin < num_bin_-2; ++bin) {
      if (polar_data[ch][bin].is_this_ground() &&
          polar_data[ch][bin+1].is_this_ground() &&
          polar_data[ch][bin-1].is_this_ground() &&
          polar_data[ch][bin+2].is_this_ground()) {
        float h1 = polar_data[ch][bin-1].get_height();
        float h2 = polar_data[ch][bin].get_height();
        float h3 = polar_data[ch][bin+1].get_height();
        float h4 = polar_data[ch][bin+2].get_height();
        if (h1 != min_height_ && h2 == min_height_ && h3 != min_height_) {
          polar_data[ch][bin].update_height((h1+h3) / 2.0F);
          polar_data[ch][bin].update_ground();
        } else if (h1 != min_height_ && h2 == min_height_ && h3 == min_height_ && h4 != min_height_) {
          polar_data[ch][bin].update_height((h1+h4) / 2.0F);
          polar_data[ch][bin].update_ground();
        }
      }
    }
  }
}
