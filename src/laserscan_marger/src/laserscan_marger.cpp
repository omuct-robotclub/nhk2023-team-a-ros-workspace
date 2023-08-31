#include "laserscan_marger/laserscan_marger.hpp"
#include <functional>
#include <algorithm>
#include <limits>
#include <chrono>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/logging.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_ros/transforms.hpp>
#include "rclcpp_components/register_node_macro.hpp"


using namespace std::chrono_literals;
using namespace std;


namespace laserscan_marger
{
LaserScanMarger::LaserScanMarger(const rclcpp::NodeOptions& options) : LaserScanMarger("laserscan_marger_node", options)
{}

LaserScanMarger::LaserScanMarger(const std::string name,
                                 const rclcpp::NodeOptions& options) : Node(name, options), scan_timeout_{0, 0}
{
  declare_parameter("laser_frame", "base_link");
  declare_parameter("publish_frequency", 100.0);
  declare_parameter("out_points", 300);
  declare_parameter("out_angle_min", 0.0);
  declare_parameter("out_angle_max", 2 * M_PI);
  declare_parameter("out_range_min", 0.2);
  declare_parameter("out_range_max", 10.0);
  declare_parameter("scan_input_topics", std::vector<std::string>());
  declare_parameter("scan_timeout", 0.2);
  
  double freq = get_parameter("publish_frequency").as_double();
  n_out_points_ = get_parameter("out_points").as_int();
  laser_frame_ = get_parameter("laser_frame").as_string();
  out_angle_min_ = get_parameter("out_angle_min").as_double();
  out_angle_max_ = get_parameter("out_angle_max").as_double();
  out_range_min_ = get_parameter("out_range_min").as_double();
  out_range_max_ = get_parameter("out_range_max").as_double();
  auto scan_input_topics = get_parameter("scan_input_topics").as_string_array();
  scan_timeout_ = rclcpp::Duration::from_seconds(get_parameter("scan_timeout").as_double());

  tf_buf_.reset(new tf2_ros::Buffer(get_clock()));
  tf_listener_.reset(new tf2_ros::TransformListener(*tf_buf_));

  // debug_pcl_pub_ = create_publisher<PointCloud2>("debug_pcl", rclcpp::SensorDataQoS());

  scan_pub_ = create_publisher<LaserScan>("scan_out", rclcpp::SensorDataQoS());
  for (const auto& topic : scan_input_topics){
    scan_subs_.push_back(
      create_subscription<LaserScan>(
        topic,
        rclcpp::SensorDataQoS(),
        std::bind(&LaserScanMarger::scan_sub_callback, this, std::placeholders::_1)));
  }

  timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(1000 / freq)), std::bind(&LaserScanMarger::timer_callback, this));
}

void LaserScanMarger::scan_sub_callback(LaserScan::ConstSharedPtr msg)
{
  auto frame_id = msg->header.frame_id;
  scan_buffer_[frame_id] = msg;
  // RCLCPP_INFO_STREAM(get_logger(), "Received: " << frame_id);
}

void LaserScanMarger::timer_callback()
{
  LaserScan out;
  out.angle_min = out_angle_min_;
  out.angle_max = out_angle_max_;
  out.range_min = out_range_min_;
  out.range_max = out_range_max_;
  out.angle_increment = (out_angle_max_ - out_angle_min_) / n_out_points_;
  out.header.frame_id = laser_frame_;

  builtin_interfaces::msg::Time latest;
  vector<string> outdated_scans;
  rclcpp::Time now = get_clock()->now();
  rclcpp::Time oldest = now;
  for (const auto& scan : scan_buffer_){
    // auto stamp = scan.second->header.stamp;
    rclcpp::Time stamp(scan.second->header.stamp);
    if (now - stamp > scan_timeout_) {
      outdated_scans.push_back(scan.first);
    } else if (stamp < oldest) {
      oldest = stamp;
    }
    // if (stamp.sec > latest.sec || (stamp.sec == latest.sec && stamp.nanosec > latest.nanosec)){
    //   latest = stamp;
    // }
  }
  for (const auto& scan_name : outdated_scans) {
    scan_buffer_.erase(scan_name);
  }

  out.header.stamp = oldest;

  out.ranges.resize(n_out_points_);
  out.intensities.resize(n_out_points_);
  std::fill(out.ranges.begin(), out.ranges.end(), std::numeric_limits<double>::quiet_NaN());
  std::fill(out.intensities.begin(), out.intensities.end(), 0.0);

  std::vector<Vec3> points;
  laser_geometry::LaserProjection proj;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  for (const auto& scan : scan_buffer_){
    auto key = scan.first;
    auto val = scan.second;
    try {
      const auto transform_msg = tf_buf_->lookupTransform(laser_frame_, val->header.frame_id, oldest/*val->header.stamp*/);
      tf2::Stamped<tf2::Transform> transform;
      tf2::fromMsg(transform_msg, transform);

      sensor_msgs::msg::PointCloud2 pc2_msg;
      sensor_msgs::msg::PointCloud2 pc2_msg_transformed;
      proj.projectLaser(*val, pc2_msg);
      pcl::PointCloud<pcl::PointXYZ> sub_cloud;
      pcl_ros::transformPointCloud(laser_frame_, transform_msg, pc2_msg, pc2_msg_transformed);
      pcl::fromROSMsg(pc2_msg_transformed, sub_cloud);
      pcl::concatenate(cloud, sub_cloud, cloud);
    } catch (std::exception& e) {
      // RCLCPP_WARN(get_logger(), "Error while processing scan what(): %s", e.what());
    }
  }

  // sensor_msgs::msg::PointCloud2 pc2_msg;
  // pcl::toROSMsg(cloud, pc2_msg);
  // pc2_msg.header.frame_id = laser_frame_;
  // pc2_msg.header.stamp = out.header.stamp;

  // debug_pcl_pub_->publish(pc2_msg);

  for (const auto& pt : cloud.points){
    double angle = atan2(pt.y, pt.x);
    if (angle < 0) {
      angle = 2 * M_PI + angle;
    }

    if (angle < out.angle_min || out.angle_max < angle){
      continue;
    }

    double range = hypot(pt.x, pt.y);
    size_t idx = (angle - out.angle_min) / out.angle_increment;

    if (idx < out.ranges.size()) {
      if (std::isnan(out.ranges[idx]) || (out.ranges[idx] < range && out.range_min <= range && range <= out.range_max)){
        out.ranges[idx] = range;
      }
    }
  }

  scan_pub_->publish(out);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(laserscan_marger::LaserScanMarger)
