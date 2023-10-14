//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/emcl2_node.h"
#include "emcl/Mcl.h"
#include "emcl/Pose.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/float32.hpp"
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>

namespace emcl2 {

EMcl2Node::EMcl2Node()
		: Node("emcl2"),
			init_request_{false},
			simple_reset_request_{false}
{
	using namespace std::chrono_literals;
	using namespace std::placeholders;

	tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
	tf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

	std::vector<std::string> scan_topics = declare_parameter("scan_topics", std::vector<std::string>({"scan"}));
	for (const auto& topic : scan_topics) {
		laser_scan_subs_[topic] = create_subscription<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS(), std::bind(&EMcl2Node::cbScan, this, _1));
	}

	particlecloud_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", rclcpp::SensorDataQoS());
	pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mcl_pose", rclcpp::SystemDefaultsQoS());
	alpha_pub_ = create_publisher<std_msgs::msg::Float32>("alpha", rclcpp::SystemDefaultsQoS());
	initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::SystemDefaultsQoS(), std::bind(&EMcl2Node::initialPoseReceived, this, _1));
	map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("map", rclcpp::SystemDefaultsQoS().reliable().transient_local(), std::bind(&EMcl2Node::cbMap, this, _1));
	odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(), std::bind(&EMcl2Node::cbOdom, this, _1));

	global_loc_srv_ = create_service<std_srvs::srv::Empty>("global_localization", std::bind(&EMcl2Node::cbSimpleReset, this, _1, _2));

	global_frame_id_ = declare_parameter("global_frame_id", std::string("map"));
	footprint_frame_id_ = declare_parameter("footprint_frame_id", std::string("base_footprint"));
	odom_frame_id_ = declare_parameter("odom_frame_id", std::string("odom"));
	base_frame_id_ = declare_parameter("base_frame_id", std::string("base_link"));

	declare_parameter("laser_likelihood_max_dist", 0.2);
	declare_parameter("num_particles", 100);

	declare_parameter("odom_fw_dev_per_fw", 0.19);
	declare_parameter("odom_fw_dev_per_rot", 0.0001);
	declare_parameter("odom_rot_dev_per_fw", 0.13);
	declare_parameter("odom_rot_dev_per_rot", 0.2);

	declare_parameter("initial_pose_x", 0.0);
	declare_parameter("initial_pose_y", 0.0);
	declare_parameter("initial_pose_a", 0.0);

	declare_parameter("alpha_threshold", 0.5);
	declare_parameter("expansion_radius_position", 0.1);
	declare_parameter("expansion_radius_orientation", 0.2);
	declare_parameter("expansion_max_linear_velocity", 0.1);
	declare_parameter("expansion_max_angular_velocity", 0.1);

	declare_parameter("extraction_rate", 0.1);
	declare_parameter("range_threshold", 0.1);
	declare_parameter("sensor_reset", true);
}


void EMcl2Node::initPF(void)
{
	std::shared_ptr<LikelihoodFieldMap> map = initMap();

	Pose init_pose;
	init_pose.x_ = get_parameter("initial_pose_x").as_double();
	init_pose.y_ = get_parameter("initial_pose_y").as_double();
	init_pose.t_ = get_parameter("initial_pose_a").as_double();

	pf_ = std::make_shared<Mcl>(
		init_pose, get_parameter("num_particles").as_int(), getOdomModel(), map);

	pf_->alpha_threshold = get_parameter("alpha_threshold").as_double();
	pf_->expansion_radius_position = get_parameter("expansion_radius_position").as_double();
	pf_->expansion_radius_orientation = get_parameter("expansion_radius_orientation").as_double();
	pf_->extraction_rate = get_parameter("extraction_rate").as_double();
	pf_->range_threshold = get_parameter("range_threshold").as_double();
	pf_->sensor_reset = get_parameter("sensor_reset").as_bool();
}

OdomModel EMcl2Node::getOdomModel(void)
{
	double ff, fr, rf, rr;
	ff = get_parameter("odom_fw_dev_per_fw").as_double();
	fr = get_parameter("odom_fw_dev_per_rot").as_double();
	rf = get_parameter("odom_rot_dev_per_fw").as_double();
	rr = get_parameter("odom_rot_dev_per_rot").as_double();
	return OdomModel(ff, fr, rf, rr);
}

std::shared_ptr<LikelihoodFieldMap> EMcl2Node::initMap(void)
{
	double likelihood_range = get_parameter("laser_likelihood_max_dist").as_double();

	int num = get_parameter("num_particles").as_int();

	return std::shared_ptr<LikelihoodFieldMap>(new LikelihoodFieldMap(*map_, likelihood_range));
}

void EMcl2Node::cbScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
	if (pf_ == nullptr) return;

	if (last_mcl_.has_value()) {
		*pf_ = *last_mcl_;
	}
	
	rclcpp::Time stamp = msg->header.stamp;
	bool is_moving = false;
	while (!odom_history_.empty()) {
		const auto& odom = odom_history_.front();
		is_moving = odom.is_moving;
		if (stamp < odom.stamp) break;
		pf_->motionUpdate(odom.pose);
		odom_history_.pop_front();
	}

	double lx, ly, lt;
	bool inv;
	if(not getLidarPose(msg->header, lx, ly, lt, inv)){
		RCLCPP_INFO(get_logger(), "can't get lidar pose info");
		return;
	}

	pf_->sensorUpdate(convert_scan(*msg), lx, ly, lt, inv, !is_moving);

	last_mcl_ = *pf_;

	for (const auto& odom : odom_history_) {
		pf_->motionUpdate(odom.pose);
	}
}

void EMcl2Node::initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
	init_request_ = true;
	init_x_ = msg->pose.pose.position.x;
	init_y_ = msg->pose.pose.position.y;
	init_t_ = tf2::getYaw(msg->pose.pose.orientation);
}

void EMcl2Node::cbOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
	if (pf_ == nullptr) return;

	if(init_request_){
		last_mcl_.reset();
		pf_->initialize(init_x_, init_y_, init_t_);
		init_request_ = false;
	}
	else if(simple_reset_request_){
		last_mcl_.reset();
		pf_->simpleReset();
		simple_reset_request_ = false;
	}

	if (prev_odom_msg_ == nullptr) {
		prev_odom_msg_ = msg;
		return;
	}
	
	double dt = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(prev_odom_msg_->header.stamp)).seconds();

	prev_odom_msg_ = msg;
	
	Pose odom{msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation)};

	bool is_moving = false;
	if (prev_odom_.has_value()) {
		auto linear_max = get_parameter("expansion_max_linear_velocity").as_double();
		auto angular_max = get_parameter("expansion_max_angular_velocity").as_double();
		auto diff = odom - *prev_odom_;
		auto linear_speed = std::hypot(diff.x_, diff.y_) / dt;
		auto angular_speed = std::abs(diff.t_) / dt;
		// RCLCPP_INFO(get_logger(), "linear: %7.3lf angular: %7.3lf", linear_speed, angular_speed);
		is_moving = linear_speed > linear_max || angular_speed > angular_max;
	}
	prev_odom_ = odom;
	odom_history_.push_back({get_clock()->now(), odom, is_moving});

	pf_->motionUpdate(odom);

	Pose mean;
	double x_var, y_var, t_var, xy_cov, yt_cov, tx_cov;
	pf_->meanPose(mean, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov);

	publishOdomFrame(mean);
	publishPose(mean, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov);
	publishParticles();

	std_msgs::msg::Float32 alpha_msg;
	alpha_msg.data = static_cast<float>(pf_->alpha_);
	alpha_pub_->publish(alpha_msg);
}

void EMcl2Node::publishPose(Pose pose,
			double x_dev, double y_dev, double t_dev,
			double xy_cov, double yt_cov, double tx_cov)
{
	geometry_msgs::msg::PoseWithCovarianceStamped p;
	p.header.frame_id = global_frame_id_;
	p.header.stamp = get_clock()->now();
	p.pose.pose.position.x = pose.x_;
	p.pose.pose.position.y = pose.y_;

	p.pose.covariance[6*0 + 0] = x_dev;
	p.pose.covariance[6*1 + 1] = y_dev;
	p.pose.covariance[6*2 + 2] = t_dev;

	p.pose.covariance[6*0 + 1] = xy_cov;
	p.pose.covariance[6*1 + 0] = xy_cov;
	p.pose.covariance[6*0 + 2] = tx_cov;
	p.pose.covariance[6*2 + 0] = tx_cov;
	p.pose.covariance[6*1 + 2] = yt_cov;
	p.pose.covariance[6*2 + 1] = yt_cov;
	
	tf2::Quaternion q;
	q.setRPY(0, 0, pose.t_);
	tf2::convert(q, p.pose.pose.orientation);

	pose_pub_->publish(p);
}

void EMcl2Node::publishOdomFrame(Pose pose)
{
	geometry_msgs::msg::PoseStamped odom_to_map;
	try{
		tf2::Quaternion q;
		q.setRPY(0, 0, pose.t_);
		tf2::Transform tmp_tf(q, tf2::Vector3(pose.x_, pose.y_, 0.0));

		geometry_msgs::msg::PoseStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = footprint_frame_id_;
		tmp_tf_stamped.header.stamp = rclcpp::Time(0);
		tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);
		
		tf_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);

	}catch(tf2::TransformException){
		RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform");
		return;
	}
	tf2::convert(odom_to_map.pose, latest_tf_);
	
	rclcpp::Time transform_expiration = get_clock()->now() + rclcpp::Duration::from_seconds(0.2);
	geometry_msgs::msg::TransformStamped tmp_tf_stamped;
	tmp_tf_stamped.header.frame_id = global_frame_id_;
	tmp_tf_stamped.header.stamp = transform_expiration;
	tmp_tf_stamped.child_frame_id = odom_frame_id_;
	tf2::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
	
	tfb_->sendTransform(tmp_tf_stamped);
}

void EMcl2Node::publishParticles(void)
{
	geometry_msgs::msg::PoseArray cloud_msg;
	cloud_msg.header.stamp = get_clock()->now();
	cloud_msg.header.frame_id = global_frame_id_;
	cloud_msg.poses.resize(pf_->particles_.size());

	for(int i=0;i<pf_->particles_.size();i++){		
		cloud_msg.poses[i].position.x = pf_->particles_[i].p_.x_;
		cloud_msg.poses[i].position.y = pf_->particles_[i].p_.y_;
		cloud_msg.poses[i].position.z = 0; 

		tf2::Quaternion q;
		q.setRPY(0, 0, pf_->particles_[i].p_.t_);
		tf2::convert(q, cloud_msg.poses[i].orientation);
	}		
	particlecloud_pub_->publish(cloud_msg);
}

bool EMcl2Node::getOdomPose(double& x, double& y, double& yaw)
{
	geometry_msgs::msg::PoseStamped ident;
	ident.header.frame_id = footprint_frame_id_;
	ident.header.stamp = rclcpp::Time(0);
	tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
	
	geometry_msgs::msg::PoseStamped odom_pose;
	try{
		this->tf_->transform(ident, odom_pose, odom_frame_id_);
	}catch(tf2::TransformException e){
    		RCLCPP_WARN(get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}
	x = odom_pose.pose.position.x;
	y = odom_pose.pose.position.y;
	yaw = tf2::getYaw(odom_pose.pose.orientation);

	return true;
}

bool EMcl2Node::getLidarPose(const std_msgs::msg::Header& header, double& x, double& y, double& yaw, bool& inv)
{
	geometry_msgs::msg::PoseStamped ident;
	ident.header = header;
	tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
	
	geometry_msgs::msg::PoseStamped lidar_pose;
	try{
		this->tf_->transform(ident, lidar_pose, base_frame_id_);
	}catch(tf2::TransformException e){
    RCLCPP_WARN(get_logger(), "Failed to compute lidar pose, skipping scan (%s)", e.what());
		return false;
	}
	x = lidar_pose.pose.position.x;
	y = lidar_pose.pose.position.y;

	double roll, pitch;
	tf2::getEulerYPR(lidar_pose.pose.orientation, yaw, pitch, roll);
	inv = (fabs(pitch) > M_PI/2 || fabs(roll) > M_PI/2) ? true : false;

	return true;
}

Scan EMcl2Node::convert_scan(const sensor_msgs::msg::LaserScan& msg) {
	Scan scan;
	scan.ranges_.resize(msg.ranges.size());

	for(int i=0; i<msg.ranges.size(); i++)
		scan.ranges_[i] = msg.ranges[i];

	scan.angle_min_ = msg.angle_min;
	scan.angle_max_ = msg.angle_max;
	scan.angle_increment_ = msg.angle_increment;
	scan.range_min_= msg.range_min;
	scan.range_max_= msg.range_max;
	scan.scan_increment_ = 1;
	return scan;
}

bool EMcl2Node::cbSimpleReset(std_srvs::srv::Empty::Request::SharedPtr req, std_srvs::srv::Empty::Response::SharedPtr res)
{
	return simple_reset_request_ = true;
}

void EMcl2Node::cbMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
	map_ = msg;
	initPF();
}

}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<emcl2::EMcl2Node>();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}

