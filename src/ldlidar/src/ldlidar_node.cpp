#include <ldlidar_node.h>

using namespace std::chrono_literals;

LD06::LD06() : Node("ld06_node") {
  std::string topic_name = this->declare_parameter("topic_name", "scan");
  lidar_frame_ = this->declare_parameter("lidar_frame", "laser");
  range_threshold_ = this->declare_parameter("range_threshold", 0.005);
  timestamp_offset_ = this->declare_parameter("timestamp_offset", 0.0);
  serial_port_candidates_ = this->declare_parameter("serial_port_candidates", std::vector<std::string>());
  warned_ = false;

  lidar_ = new LiPkg;
  lidar_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);

  lidar_->SetLidarFrame(lidar_frame_);
  lidar_->SetRangeThreshold(range_threshold_);
  lidar_->SetTimestampOffset(timestamp_offset_);

  cmd_port_.SetReadCallback([this](const char* byte, size_t len) {
    if (lidar_->Parse((uint8_t*)byte, len)) {
      lidar_->AssemblePacket();
    }
  });

  for (const auto& port_name : serial_port_candidates_) {
    RCLCPP_INFO(get_logger(), "Trying port %s", port_name.c_str());
    port_name_ = port_name;
    if (cmd_port_.Open(port_name_)) {
      RCLCPP_INFO(get_logger(), "LiDAR_LD06 started successfully");
      break;
    } else {
      RCLCPP_ERROR(get_logger(), "Can't open %s", port_name.c_str());
    }
  }

  loop_timer_ = this->create_wall_timer(100ms, std::bind(&LD06::publishLoop, this));
}

void LD06::publishLoop() {
  if (!cmd_port_.IsOpened()) {
    for (auto&& port_name : serial_port_candidates_) {
      if (cmd_port_.Open(port_name)) {
        RCLCPP_INFO(get_logger(), "LiDAR_LD06 started successfully");
        warned_ = false;
        cmd_port_.SetReadCallback([](const char*, size_t){});
        delete lidar_;
        lidar_ = new LiPkg();
        lidar_->SetLidarFrame(lidar_frame_);
        lidar_->SetRangeThreshold(range_threshold_);
        lidar_->SetTimestampOffset(timestamp_offset_);
        cmd_port_.SetReadCallback([this](const char* byte, size_t len) {
          if (lidar_->Parse((uint8_t*)byte, len)) {
            lidar_->AssemblePacket();
          }
        });
        break;
      } else {
        if (!warned_) {
          RCLCPP_ERROR(get_logger(), "Can't open %s", port_name.c_str());
          warned_ = true;
        }
      }
    }
  }

  if (!cmd_port_.IsOpened()) {
    return;
  }

  if (cmd_port_.IsDisconnected()) {
    RCLCPP_ERROR(get_logger(), "Disconnected. reconnecting to '%s'", port_name_.c_str());
    cmd_port_.Close();
  }

  if (lidar_->IsFrameReady()) {
    lidar_pub_->publish(lidar_->GetLaserScan());
    lidar_->ResetFrameReady();
  }
}