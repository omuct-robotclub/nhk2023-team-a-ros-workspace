//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/ExpResetMcl2.h"
#include <rclcpp/logging.hpp>
#include <iostream>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <stdlib.h>
#include <cmath>

namespace emcl2 {

ExpResetMcl2::ExpResetMcl2(const Pose &p, int num,
				const OdomModel& odom_model,
				const std::shared_ptr<const LikelihoodFieldMap> map,
				double alpha_th, 
				double expansion_radius_position, double expansion_radius_orientation,
				double extraction_rate, double range_threshold, bool sensor_reset)
	: alpha_threshold_(alpha_th), 
	  expansion_radius_position_(expansion_radius_position),
	  expansion_radius_orientation_(expansion_radius_orientation),
	  extraction_rate_(extraction_rate),
	  range_threshold_(range_threshold),
	  sensor_reset_(sensor_reset),
	  Mcl::Mcl(p, num, odom_model, map)
{
}

void ExpResetMcl2::sensorUpdate(const sensor_msgs::msg::LaserScan& msg, double lidar_x, double lidar_y, double lidar_t, bool inv)
{
	const auto logger = rclcpp::get_logger("exp_reset_mcl2");
	Scan scan = scan_from_msg(msg);

	scan.lidar_pose_x_ = lidar_x;
	scan.lidar_pose_y_ = lidar_y;
	scan.lidar_pose_yaw_ = lidar_t;

	int i = 0;
	if(!inv){
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_min_ + (i++)*scan.angle_increment_)
			);
	}else{
		for(auto e : scan.ranges_)
			scan.directions_16bit_.push_back(
				Pose::get16bitRepresentation(scan.angle_max_ - (i++)*scan.angle_increment_)
			);
	}

	double valid_pct = 0.0;
	int valid_beams = scan.countValidBeams(&valid_pct);
	if(valid_beams == 0)
		return;

	for(auto &p : particles_)
		p.w_ *= p.likelihood(*map_, scan);

	RCLCPP_INFO(logger, "non penetration rate begin");
	if (alpha_threshold_ > 0.0) {
		// alpha_ = nonPenetrationRate( (int)(particles_.size()*extraction_rate_), *map_, scan);
		alpha_ = normalizeBelief()/valid_beams;
		RCLCPP_INFO(logger, "ALPHA: %f / %f", alpha_, alpha_threshold_);
		if(alpha_ < alpha_threshold_){
			RCLCPP_INFO(logger, "RESET");
			expansionReset();
			for(auto &p : particles_)
				p.w_ *= p.likelihood(*map_, scan);
		}
	}

	if(normalizeBelief() > 0.000001) {
		RCLCPP_INFO(logger, "resampling begin");
		resampling();
		RCLCPP_INFO(logger, "resampling end");
	}
	else {
		RCLCPP_INFO(logger, "reset weight begin");
		resetWeight();
		RCLCPP_INFO(logger, "reset weight end");
	}
}

double ExpResetMcl2::nonPenetrationRate(int skip, const LikelihoodFieldMap& map, Scan &scan)
{
	static uint16_t shift = 0;
	int counter = 0;
	int penetrating = 0;
	for(int i=shift%skip; i<particles_.size(); i+=skip){
		counter++;
		if(particles_[i].wallConflict(map, scan, range_threshold_, sensor_reset_))
			penetrating++;
	}
	shift++;

	std::cout << penetrating << " " << counter << std::endl;
	return (double)(counter - penetrating) / counter;
}

void ExpResetMcl2::expansionReset(void)
{
	for(auto &p : particles_){
		double length = 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_position_;
		double direction = 2*((double)rand()/RAND_MAX - 0.5)*M_PI;

		p.p_.x_ += length*cos(direction);
		p.p_.y_ += length*sin(direction);
		p.p_.t_ += 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_orientation_;
		p.w_ = 1.0/particles_.size();
	}
}

}
