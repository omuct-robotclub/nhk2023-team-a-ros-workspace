#ifndef LD06_H
#define LD06_H

#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tofbf.h"
#include <string>

using namespace std::chrono_literals;

class LD06 : public rclcpp::Node
{
  public:
    LD06();

  private:
    CmdInterfaceLinux cmd_port_;
    rclcpp::TimerBase::SharedPtr loop_timer_;
    std::string port_name_;
    std::string lidar_frame_;
    double range_threshold_;
    double timestamp_offset_;
    bool warned_;
    std::vector<std::string> serial_port_candidates_;
    LiPkg * lidar_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub_;
    void publishLoop();
};

#endif