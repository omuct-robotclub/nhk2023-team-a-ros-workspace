//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/emcl2_node.h"
#include "emcl/Pose.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/float32.hpp"

namespace emcl2 {

EMcl2Node::EMcl2Node() : Node("emcl2")
{
	using namespace std::chrono_literals;
	initCommunication();

	odom_freq_ = declare_parameter("odom_freq", 20);

	init_request_ = false;
	simple_reset_request_ = false;

	wall_timer_ = create_wall_timer(1s / odom_freq_, std::bind(&EMcl2Node::loop, this));
}

EMcl2Node::~EMcl2Node()
{
}

void EMcl2Node::initCommunication(void)
{
	using namespace std::placeholders;
	particlecloud_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", rclcpp::SensorDataQoS());
	pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mcl_pose", rclcpp::SystemDefaultsQoS());
	alpha_pub_ = create_publisher<std_msgs::msg::Float32>("alpha", rclcpp::SystemDefaultsQoS());
	laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&EMcl2Node::cbScan, this, _1));
	initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", rclcpp::SystemDefaultsQoS(), std::bind(&EMcl2Node::initialPoseReceived, this, _1));
	map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("map", rclcpp::SystemDefaultsQoS().reliable().transient_local(), std::bind(&EMcl2Node::cbMap, this, _1));

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

	tfb_.reset(new tf2_ros::TransformBroadcaster(*this));
	tf_.reset(new tf2_ros::Buffer(this->get_clock()));
	tfl_.reset(new tf2_ros::TransformListener(*tf_));
}

void EMcl2Node::initPF(void)
{
	std::shared_ptr<LikelihoodFieldMap> map = std::move(initMap());
	std::shared_ptr<OdomModel> om = std::move(initOdometry());

	Scan scan;
	scan.range_min_ = declare_parameter("laser_min_range", 0.0);
	scan.range_max_ = declare_parameter("laser_max_range", 100000000.0);
	scan.scan_increment_ = declare_parameter("scan_increment", 1);

	Pose init_pose;
	init_pose.x_ = declare_parameter("initial_pose_x", 0.0);
	init_pose.y_ = declare_parameter("initial_pose_y", 0.0);
	init_pose.t_ = declare_parameter("initial_pose_a", 0.0);

	int num_particles;
	double alpha_th;
	double ex_rad_pos, ex_rad_ori;
	num_particles = get_parameter("num_particles").as_int();
	alpha_th = declare_parameter("alpha_threshold", 0.5);
	ex_rad_pos = declare_parameter("expansion_radius_position", 0.1);
	ex_rad_ori = declare_parameter("expansion_radius_orientation", 0.2);

	double extraction_rate, range_threshold;
	bool sensor_reset;
	extraction_rate = declare_parameter("extraction_rate", 0.1);
	range_threshold = declare_parameter("range_threshold", 0.1);
	sensor_reset = declare_parameter("sensor_reset", true);

	pf_.reset(new ExpResetMcl2(init_pose, num_particles, scan, om, map,
				alpha_th, ex_rad_pos, ex_rad_ori,
				extraction_rate, range_threshold, sensor_reset));
}

std::shared_ptr<OdomModel> EMcl2Node::initOdometry(void)
{
	double ff, fr, rf, rr;
	ff = get_parameter("odom_fw_dev_per_fw").as_double();
	fr = get_parameter("odom_fw_dev_per_rot").as_double();
	rf = get_parameter("odom_rot_dev_per_fw").as_double();
	rr = get_parameter("odom_rot_dev_per_rot").as_double();
	return std::shared_ptr<OdomModel>(new OdomModel(ff, fr, rf, rr));
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
	scan_header_ = msg->header;
	// scan_frame_id_ = msg->header.frame_id;
	pf_->setScan(msg);
}

void EMcl2Node::initialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
{
	init_request_ = true;
	init_x_ = msg->pose.pose.position.x;
	init_y_ = msg->pose.pose.position.y;
	init_t_ = tf2::getYaw(msg->pose.pose.orientation);
}

void EMcl2Node::loop(void)
{
	if (pf_ == nullptr) return;

	if(init_request_){
		pf_->initialize(init_x_, init_y_, init_t_);
		init_request_ = false;
	}
	else if(simple_reset_request_){
		pf_->simpleReset();
		simple_reset_request_ = false;
	}

	double x, y, t;
	if(not getOdomPose(x, y, t)){
		RCLCPP_INFO(get_logger(), "can't get odometry info");
		return;
	}
	pf_->motionUpdate(x, y, t);

	double lx, ly, lt;
	bool inv;
	if(not getLidarPose(lx, ly, lt, inv)){
		RCLCPP_INFO(get_logger(), "can't get lidar pose info");
		return;
	}

	/*
	struct timespec ts_start, ts_end;
	clock_gettime(CLOCK_REALTIME, &ts_start);
	*/
	pf_->sensorUpdate(lx, ly, lt, inv);
	/*
	clock_gettime(CLOCK_REALTIME, &ts_end);
	struct tm tm;
	localtime_r( &ts_start.tv_sec, &tm);
	printf("START: %02d.%09ld\n", tm.tm_sec, ts_start.tv_nsec);
	localtime_r( &ts_end.tv_sec, &tm);
	printf("END: %02d.%09ld\n", tm.tm_sec, ts_end.tv_nsec);
	*/

	double x_var, y_var, t_var, xy_cov, yt_cov, tx_cov;
	pf_->meanPose(x, y, t, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov);

	publishOdomFrame(x, y, t);
	publishPose(x, y, t, x_var, y_var, t_var, xy_cov, yt_cov, tx_cov);
	publishParticles();

	std_msgs::msg::Float32 alpha_msg;
	alpha_msg.data = static_cast<float>(pf_->alpha_);
	alpha_pub_->publish(alpha_msg);
}

void EMcl2Node::publishPose(double x, double y, double t,
			double x_dev, double y_dev, double t_dev,
			double xy_cov, double yt_cov, double tx_cov)
{
	geometry_msgs::msg::PoseWithCovarianceStamped p;
	p.header.frame_id = global_frame_id_;
	p.header.stamp = get_clock()->now();
	p.pose.pose.position.x = x;
	p.pose.pose.position.y = y;

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
	q.setRPY(0, 0, t);
	tf2::convert(q, p.pose.pose.orientation);

	pose_pub_->publish(p);
}

void EMcl2Node::publishOdomFrame(double x, double y, double t)
{
	geometry_msgs::msg::PoseStamped odom_to_map;
	try{
		tf2::Quaternion q;
		q.setRPY(0, 0, t);
		tf2::Transform tmp_tf(q, tf2::Vector3(x, y, 0.0));
				
		geometry_msgs::msg::PoseStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = footprint_frame_id_;
		tmp_tf_stamped.header.stamp = scan_header_.stamp;
		// tmp_tf_stamped.header.stamp = rclcpp::Time(0);
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
	// ident.header.stamp = rclcpp::Time(0);
	ident.header.stamp = scan_header_.stamp;
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

bool EMcl2Node::getLidarPose(double& x, double& y, double& yaw, bool& inv)
{
	geometry_msgs::msg::PoseStamped ident;
	// ident.header.frame_id = scan_frame_id_;
	// ident.header.stamp = rclcpp::Time(0);
	ident.header = scan_header_;
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

int EMcl2Node::getOdomFreq(void){
	return odom_freq_;
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

