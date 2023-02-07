// C++ header
#include <cmath>
#include <random>
//#include <cassert>

// ROS header
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>

// local header
#include "lidar_object_tracking/component_clustering.hpp"


ComponentClustering::ComponentClustering()
  : Node("component_clustering_node")
{
  this->declare_parameter("num_grid", 200);
  this->declare_parameter("roi_m", 30.0);
  this->declare_parameter("kernel_size", 3);
  this->declare_parameter("pic_scale", 30.0); // 900 / roi_m
  this->declare_parameter("lslope_dist", 3.0);
  this->declare_parameter("lnum_points", 300);
  this->declare_parameter("rnd_points", 30);
  this->declare_parameter("sensor_height", 1.73);
  this->declare_parameter("thr_height_min", 1.0);
  this->declare_parameter("thr_height_max", 2.6);
  this->declare_parameter("thr_width_min", 0.25);
  this->declare_parameter("thr_width_max", 3.5);
  this->declare_parameter("thr_len_min", 0.5);
  this->declare_parameter("thr_len_max", 14.0);
  this->declare_parameter("thr_area_max", 20.0);
  this->declare_parameter("tpt_per_m3", 8.0);
  this->declare_parameter("min_len_ratio", 3.0);
  this->declare_parameter("thr_ratio_min", 1.3);
  this->declare_parameter("thr_ratio_max", 5.0);

  // load from parameter if provided
  num_grid_ = this->get_parameter("num_grid").get_parameter_value().get<int>();
  roi_m_ = static_cast<float>(this->get_parameter("roi_m").get_parameter_value().get<double>());
  kernel_size_ = this->get_parameter("kernel_size").get_parameter_value().get<int>();
  pic_scale_ = static_cast<float>(this->get_parameter("pic_scale").get_parameter_value().get<double>());
  lslope_dist_ = static_cast<float>(this->get_parameter("lslope_dist").get_parameter_value().get<double>());
  lnum_points_ = this->get_parameter("lnum_points").get_parameter_value().get<int>();
  rnd_points_ = this->get_parameter("rnd_points").get_parameter_value().get<int>();
  sensor_height_ = static_cast<float>(this->get_parameter("sensor_height").get_parameter_value().get<double>());
  thr_height_min_ = static_cast<float>(this->get_parameter("thr_height_min").get_parameter_value().get<double>());
  thr_height_max_ = static_cast<float>(this->get_parameter("thr_height_max").get_parameter_value().get<double>());
  thr_width_min_ = static_cast<float>(this->get_parameter("thr_width_min").get_parameter_value().get<double>());
  thr_width_max_ = static_cast<float>(this->get_parameter("thr_width_max").get_parameter_value().get<double>());
  thr_len_min_ = static_cast<float>(this->get_parameter("thr_len_min").get_parameter_value().get<double>());
  thr_len_max_ = static_cast<float>(this->get_parameter("thr_len_max").get_parameter_value().get<double>());
  thr_area_max_ = static_cast<float>(this->get_parameter("thr_area_max").get_parameter_value().get<double>());
  tpt_per_m3_ = static_cast<float>(this->get_parameter("tpt_per_m3").get_parameter_value().get<double>());
  min_len_ratio_ = static_cast<float>(this->get_parameter("min_len_ratio").get_parameter_value().get<double>());
  thr_ratio_min_ = static_cast<float>(this->get_parameter("thr_ratio_min").get_parameter_value().get<double>());
  thr_ratio_max_ = static_cast<float>(this->get_parameter("thr_ratio_max").get_parameter_value().get<double>());

  rclcpp::QoS qos(10);

  elevated_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "elevated_cloud", qos, std::bind(&ComponentClustering::elevated_cloud_callback,
    this, std::placeholders::_1));

  clustered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "clustered_cloud", qos);

  marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "clustered_marker", qos);
}

void ComponentClustering::elevated_cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pc_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr elevated_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

  fromROSMsg(*pc_msg, *elevated_cloud);

  int num_cluster = 0;
  std::vector<std::vector<int>> cartesian_data(num_grid_, std::vector<int>(num_grid_));

  component_clustering(*elevated_cloud, cartesian_data, num_cluster);

  // for visualization
  make_clustered_cloud(*elevated_cloud, cartesian_data, *clustered_cloud);

  sensor_msgs::msg::PointCloud2 clustered_cloud_msg;
  pcl::toROSMsg(*clustered_cloud, clustered_cloud_msg);
  clustered_cloud_msg.header.frame_id = "lidar_link";
  clustered_cloud_msg.header.stamp = rclcpp::Node::now();
  clustered_cloud_publisher_->publish(clustered_cloud_msg);

  // Box fitting
  std::vector<pcl::PointCloud<pcl::PointXYZ>> bounding_boxs {
    box_fitting(*elevated_cloud, cartesian_data, num_cluster)};

  visualization_msgs::msg::MarkerArray marker_array;
  int count = 0;
  for (auto& bbox: bounding_boxs) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "lidar_link";
    marker.header.stamp = this->get_clock()->now();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.id = count++;
    for (auto& pts: bbox) {
      geometry_msgs::msg::Point point;
      point.x = pts.x;
      point.y = pts.y;
      point.z = pts.z;
      marker.points.push_back(point);
    }
    marker_array.markers.push_back(marker);
  }
  // publish the box
  marker_publisher_->publish(marker_array);
}

void ComponentClustering::component_clustering(const pcl::PointCloud<pcl::PointXYZ>& elevated_cloud,
  std::vector<std::vector<int>>& cartesian_data, int& num_cluster)
{
    // map 120m radius data(polar grid data) into 100x100 cartesian grid,
    // parameter might need to be modified
    // in this case 30mx30m with 100x100x grid
    map_cartesian_grid(elevated_cloud, cartesian_data);
    find_component(cartesian_data, num_cluster);
}

void ComponentClustering::map_cartesian_grid(const pcl::PointCloud<pcl::PointXYZ>& elevated_cloud,
  std::vector<std::vector<int>>& cartesian_data)
{
  for (const auto& point: elevated_cloud) {
    float xc = point.x + roi_m_ / 2.0F;
    float yc = point.y + roi_m_ / 2.0F;

    // exclude outside roi points
    if (xc < 0.0F || xc >= roi_m_ || yc < 0.0F || yc >= roi_m_) {
      continue;
    } else {
      int xi = std::floor(num_grid_ * xc / roi_m_);
      int yi = std::floor(num_grid_ * yc / roi_m_);
      cartesian_data[xi][yi] = -1;
    }
  }
}

void ComponentClustering::find_component(std::vector<std::vector<int>>& cartesian_data, int& cluster_id)
{
  for (int x = 0; x < num_grid_; ++x) {
    for (int y = 0; y < num_grid_; ++y) {
      if (cartesian_data[x][y] == -1) {
        ++cluster_id;
        search(cartesian_data, cluster_id, x, y);
      }
    }
  }
}


void ComponentClustering::search(std::vector<std::vector<int>>& cartesian_data, int cluster_id, int x, int y)
{
  cartesian_data[x][y] = cluster_id;
  int mean = kernel_size_ / 2;
  for (int kx = 0; kx < kernel_size_; ++kx) {
    int kxi = kx-mean;
    if ((x + kxi) < 0 || (x + kxi) >= num_grid_) {
      continue;
    }
    for (int ky = 0; ky < kernel_size_; ++ky) {
      int kyi = ky-mean;
      if ((y + kyi) < 0 || (y + kyi) >= num_grid_) {
        continue;
      }
      if (cartesian_data[x + kxi][y + kyi] == -1) {
        search(cartesian_data, cluster_id, x+kxi, y+kyi);
      }
    }
  }
}

void ComponentClustering::make_clustered_cloud(const pcl::PointCloud<pcl::PointXYZ>& elevated_cloud,
  std::vector<std::vector<int>>& cartesian_data,
  pcl::PointCloud<pcl::PointXYZRGB>& clustered_cloud)
{
  for (const auto& point: elevated_cloud) {
    float xc = point.x + roi_m_ / 2.0F;
    float yc = point.y + roi_m_ / 2.0F;

    // exclude outside roi points
    if (xc < 0.0F || xc >= roi_m_ || yc < 0.0F || yc >= roi_m_) {
      continue;
    }

    int xi = static_cast<int>(std::floor(num_grid_ * xc / roi_m_));
    int yi = static_cast<int>(std::floor(num_grid_ * yc / roi_m_));

    int cluster_num = cartesian_data[xi][yi];
    if (cluster_num != 0) {
      pcl::PointXYZRGB out_pt;
      pcl::copyPoint(point, out_pt);
      out_pt.r = (500 * cluster_num) % 255;
      out_pt.g = (100 * cluster_num) % 255;
      out_pt.b = (150 * cluster_num) % 255;
      clustered_cloud.push_back(out_pt);
    }
  }
}

std::vector<pcl::PointCloud<pcl::PointXYZ>> ComponentClustering::box_fitting(
  const pcl::PointCloud<pcl::PointXYZ>& elevated_cloud,
  const std::vector<std::vector<int>>& cartesian_data, int num_cluster)
{
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clustered_points(num_cluster);
  get_clustered_points(elevated_cloud, cartesian_data, clustered_points);
  std::vector<pcl::PointCloud<pcl::PointXYZ>> bb_points;
  get_bounding_box(clustered_points, bb_points);

  return bb_points;
}

void ComponentClustering::get_clustered_points(const pcl::PointCloud<pcl::PointXYZ>& elevated_cloud,
  const std::vector<std::vector<int>>& cartesian_data,
  std::vector<pcl::PointCloud<pcl::PointXYZ>>& clustered_points)
{
  for (auto& point: elevated_cloud) {
    float xc = point.x + roi_m_ / 2.0F;
    float yc = point.y + roi_m_ / 2.0F;
    if (xc < 0.0F || xc >= roi_m_ || yc < 0.0F || yc >= roi_m_) {
      continue;
    }

    int xi = static_cast<int>(std::floor(num_grid_ * xc / roi_m_));
    int yi = static_cast<int>(std::floor(num_grid_ * yc / roi_m_));

    int cluster_num = cartesian_data[xi][yi];
    int vid = cluster_num - 1;
    if (cluster_num != 0) {
      clustered_points[vid].push_back(point);
    }
  }
}

void ComponentClustering::get_bounding_box(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& clustered_points,
  std::vector<pcl::PointCloud<pcl::PointXYZ>>& bb_points)
{
  for (auto& cluster: clustered_points) {
    float init_px = cluster[0].x + roi_m_ / 2.0F;
    float init_py = cluster[0].x + roi_m_ / 2.0F;

    int init_x = static_cast<int>(std::floor(init_px * pic_scale_));
    int init_y = static_cast<int>(std::floor(init_py * pic_scale_));

    int init_pic_x = init_x;
    int init_pic_y = static_cast<int>(pic_scale_ * roi_m_ - init_y);

    int offset_init_x = static_cast<int>(roi_m_ * pic_scale_ / 2.0F - init_pic_x);
    int offset_init_y = static_cast<int>(roi_m_ * pic_scale_ / 2.0F - init_pic_y);

    int num_points = cluster.size();
    std::vector<cv::Point> point_vec(num_points);
    std::vector<cv::Point2f> pc_points(4);

    float min_mx;
    float max_mx;
    float min_my;
    float max_my;

    float min_m = 999.0F;
    float max_m = -999.0F;
    float max_z = -99.0F;

    // for center of gravity
    float sum_x = 0.0F;
    float sum_y = 0.0F;

    for (int ipt = 0; ipt < num_points; ++ipt) {
      float px = cluster[ipt].x;
      float py = cluster[ipt].y;
      float pz = cluster[ipt].z;

      // cast (-15 < x,y < 15) into (0 < x,y < 30)
      float roi_x = px + roi_m_ / 2.0F;
      float roi_y = py + roi_m_ / 2.0F;

      // cast 30m x 30m into 900 x 900 scale
      int x = static_cast<int>(std::floor(roi_x * pic_scale_));
      int y = static_cast<int>(std::floor(roi_y * pic_scale_));

      // cast into image coordinate
      int pic_x = x;
      int pic_y = static_cast<int>(pic_scale_ * roi_m_ - y);

      // offset so that the object would be locate at the center
      int offset_x = pic_x + offset_init_x;
      int offset_y = pic_y + offset_init_y;

      point_vec[ipt] = cv::Point(offset_x, offset_y);

      float m = py/px;
      if (m < min_m) {
        min_m = m;
        min_mx = px;
        min_my = py;
      }
      if (m > max_m) {
        max_m = m;
        max_mx = px;
        max_my = py;
      }

      // get max_z
      if (pz > max_z) {
        max_z = pz;
      }

      sum_x += offset_x;
      sum_y += offset_y;
    }

    // L shape fitting parameters
    float x_dist = max_mx - min_mx;
    float y_dist = max_my - min_my;
    float slope_dist = std::sqrt(x_dist*x_dist + y_dist*y_dist);
    float slope = (max_my - min_my) / (max_mx - min_mx);

    // random variable
    std::mt19937_64 mt(0);
    std::uniform_int_distribution<> rand_points(0, num_points-1);

    // start L-shape fitting for car like object
    // lSlopeDist = 30, lnumPoints = 300
    if (slope_dist > lslope_dist_ && num_points > lnum_points_) {
      float max_dist = 0.0F;
      float max_dx;
      float max_dy;

      // 80 random points, get max distance
      for (int i = 0; i < rnd_points_; ++i) {
        int pind = rand_points(mt);
        //assert(pind >= 0 && pind < cluster.size());
        float xi = cluster[pind].x;
        float yi = cluster[pind].y;

        // from equation of distance between line and point
        float dist = std::abs(slope * xi - yi + max_my - slope * max_mx)/ sqrt(slope * slope +1);
        if (dist > max_dist) {
          max_dist = dist;
          max_dx = xi;
          max_dy = yi;
        }
      }

      // for center of gravity
      //max_dx = sum_x / cluster.size();
      //max_dy = sum_y / cluster.size();

      // vector adding
      float max_mvecx = max_mx - max_dx;
      float max_mvecy = max_my - max_dy;
      float min_mvecx = min_mx - max_dx;
      float min_mvecy = min_my - max_dy;
      float last_x = max_dx + max_mvecx + min_mvecx;
      float last_y = max_dy + max_mvecy + min_mvecy;

      pc_points[0] = cv::Point2f(min_mx, min_my);
      pc_points[1] = cv::Point2f(max_dx, max_dy);
      pc_points[2] = cv::Point2f(max_mx, max_my);
      pc_points[3] = cv::Point2f(last_x, last_y);

      bool is_promising = rule_based_filter(pc_points, max_z, num_points);

      if (!is_promising) {
        continue;
      }
    } else {  // MAR fitting
      cv::RotatedRect rect_info = cv::minAreaRect(point_vec);
      cv::Point2f rect_points[4];
      rect_info.points(rect_points);

      // convert points back to lidar coordinate
      get_points_in_pc_frame(rect_points, pc_points, offset_init_x, offset_init_y);

      // rule based filter
      bool is_promising = rule_based_filter(pc_points, max_z, num_points);
      if (!is_promising) {
        continue;
      }
    }

    // make pcl cloud for 3D bounding box
    pcl::PointCloud<pcl::PointXYZ> one_bbox;
    for (int pcl_h = 0; pcl_h < 2; ++pcl_h) {
      for (int pcl_p = 0; pcl_p < 4; ++pcl_p) {
        pcl::PointXYZ o;
        o.x = pc_points[pcl_p].x;
        o.y = pc_points[pcl_p].y;
        o.z = (pcl_h == 0 ? -sensor_height_ : max_z);
        one_bbox.push_back(o);
      }
    }
    bb_points.push_back(one_bbox);
  }
}

bool ComponentClustering::rule_based_filter(const std::vector<cv::Point2f>& pc_points,
  float max_z, int num_points)
{
  bool is_promising = false;

  // minimum points thresh
  if (num_points < 100) {
    return is_promising;
  }

  // length is longest side of the rectangle while width is the shorter side
  float width, length, height, area, mass; //ratio, mass;

  float x1 = pc_points[0].x;
  float y1 = pc_points[0].y;
  float x2 = pc_points[1].x;
  float y2 = pc_points[1].y;
  float x3 = pc_points[2].x;
  float y3 = pc_points[2].y;

  float dist1 = std::sqrt((x1-x2) * (x1-x2) + (y1-y2) * (y1-y2));
  float dist2 = std::sqrt((x3-x2) * (x3-x2) + (y3-y2) * (y3-y2));

  if (dist1 > dist2) {
    length = dist1;
    width = dist2;
  } else {
    length = dist2;
    width = dist1;
  }

  // assuming ground = sensor height
  height = max_z + sensor_height_;

  // assuming right angle
  area = dist1 * dist2;
  mass = area * height;
  //ratio = length / width;

  // start rule based filtering
  if (height > thr_height_min_ && height < thr_height_max_ &&
      width > thr_width_min_ && width < thr_width_max_ &&
      length > thr_len_min_ && length < thr_len_max_ &&
      area < thr_area_max_ && num_points > mass * tpt_per_m3_) {
    is_promising = true;
    return is_promising;
  //if (length > min_len_ratio_) {
  //  if (ratio > thr_ratio_min_ && ratio < thr_ratio_max_) {
  //    is_promising = true;
  //    return is_promising;
  //  }
  //} else {
  //  is_promising = true;
  //  return is_promising;
  //}
  } else {
    return is_promising;
  }
}


void ComponentClustering::get_points_in_pc_frame(const cv::Point2f rect_points[],
  std::vector<cv::Point2f>& pc_points, int offset_x, int offset_y)
{
  // loop 4 rect points
  for (int pti = 0; pti < 4; ++pti) {
    float picx = rect_points[pti].x;
    float picy = rect_points[pti].y;

    // reverse offset
    float roffset_x = picx - offset_x;
    float roffset_y = picy - offset_y;

    // reverse from image coordinate to euclidan coordinate
    float r_x = roffset_x;
    float r_y = pic_scale_ * roi_m_ - roffset_y;

    // reverse to 30m x 30m scale
    float rm_x = r_x / pic_scale_;
    float rm_y = r_y / pic_scale_;

    // reverse from (0 < x, y < 30) to (-15 < x, y < 15)
    float pc_x = rm_x - roi_m_ / 2.0F;
    float pc_y = rm_y - roi_m_ / 2.0F;
    cv::Point2f point(pc_x, pc_y);

    pc_points[pti] = point;
  }
}
