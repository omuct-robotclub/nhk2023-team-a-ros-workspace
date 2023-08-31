#ifndef LASERSCAN_MARGER_LASERSCAN_MARGER_HPP
#define LASERSCAN_MARGER_LASERSCAN_MARGER_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <memory>
#include <unordered_map>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>


namespace laserscan_marger
{

class LaserScanMarger : public rclcpp::Node
{
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Vec3 = tf2::Vector3;
  using Quat = tf2::Quaternion;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;

public:
  LaserScanMarger(const std::string name, const rclcpp::NodeOptions& options=rclcpp::NodeOptions());
  LaserScanMarger(const rclcpp::NodeOptions& options=rclcpp::NodeOptions());

protected:
  void scan_sub_callback(LaserScan::ConstSharedPtr msg);
  // void marge_scans(LaserScan& out);
  void timer_callback();

  int64_t n_out_points_;
  double out_angle_min_, out_angle_max_, out_range_min_, out_range_max_;

  std::string laser_frame_;

  std::unordered_map<std::string, LaserScan::ConstSharedPtr> scan_buffer_;
  rclcpp::Publisher<LaserScan>::SharedPtr scan_pub_;
  std::vector<rclcpp::Subscription<LaserScan>::SharedPtr> scan_subs_;

  rclcpp::Publisher<PointCloud2>::SharedPtr debug_pcl_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Duration scan_timeout_;

  std::shared_ptr<tf2_ros::Buffer> tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}

#endif