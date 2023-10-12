//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef INTERFACE_EMCL2_H__
#define INTERFACE_EMCL2_H__

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include "emcl/ExpResetMcl2.h"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2/LinearMath/Transform.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_msgs/msg/float32.hpp"

namespace emcl2 {

class EMcl2Node : public rclcpp::Node
{
public:
	EMcl2Node();
	~EMcl2Node();

	void loop(void);
	int getOdomFreq(void);
private:
	std::shared_ptr<ExpResetMcl2> pf_;
	// ros::NodeHandle nh_;
	// ros::NodeHandle private_nh_;

	rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particlecloud_pub_;
	rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr alpha_pub_;
	std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> laser_scan_subs_;
	rclcpp::SubscriptionBase::SharedPtr initial_pose_sub_;
	rclcpp::SubscriptionBase::SharedPtr map_sub_;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr global_loc_srv_;

	rclcpp::TimerBase::SharedPtr wall_timer_;

	std::string footprint_frame_id_;
	std::string global_frame_id_;
	std::string odom_frame_id_;
	// std::string scan_frame_id_;
	std::string base_frame_id_;

	// std_msgs::msg::Header scan_header_;
	
	std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
	std::shared_ptr<tf2_ros::TransformListener> tfl_;
	std::shared_ptr<tf2_ros::Buffer> tf_;

	nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
	tf2::Transform latest_tf_;

	int odom_freq_;
	bool init_request_;
	bool simple_reset_request_;
	double init_x_, init_y_, init_t_;

	struct OdomPose {
		rclcpp::Time stamp;
		double x, y, t;
	};

	std::optional<ExpResetMcl2> last_mcl_;
	std::vector<OdomPose> odom_history_;

	void publishPose(double x, double y, double t,
			double x_dev, double y_dev, double t_dev,
			double xy_cov, double yt_cov, double tx_cov);
	void publishOdomFrame(double x, double y, double t);
	void publishParticles(void);
	void sendTf(void);
	bool getOdomPose(double& x, double& y, double& yaw);
	bool getLidarPose(const std_msgs::msg::Header& header, double& x, double& y, double& yaw, bool& inv);

	void initCommunication(void);
	void initPF(void);
	std::shared_ptr<LikelihoodFieldMap> initMap(void);
	std::shared_ptr<OdomModel> initOdometry(void);

	void cbMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
	void cbScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
	bool cbSimpleReset(std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res);
	void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
};

}

#endif
