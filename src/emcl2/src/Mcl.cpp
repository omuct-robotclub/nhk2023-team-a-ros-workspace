//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/Mcl.h"
#include "emcl/lookup_tables.h"
#include <rclcpp/logging.hpp>
#include <iostream>
#include <sensor_msgs/msg/detail/laser_scan__struct.hpp>
#include <stdlib.h>
#include <cmath>

namespace emcl2 {

Mcl::Mcl(const Pose &p, int num,
				const OdomModel& odom_model,
				const std::shared_ptr<const LikelihoodFieldMap> map)
	: alpha_threshold(0.5), 
	  expansion_radius_position(0.1),
	  expansion_radius_orientation(0.1),
	  extraction_rate(0.1),
	  range_threshold(0.1),
	  sensor_reset(true),
	  odom_model_{odom_model}
{
	odom_model_ = odom_model;
	map_ = map;

	if(num <= 0)
		RCLCPP_ERROR(rclcpp::get_logger("mcl"), "NO PARTICLE");

	Particle particle(p.x_, p.y_, p.t_, 1.0/num);
	for(int i=0; i<num; i++)
		particles_.push_back(particle);

	alpha_ = 1.0;
}

void Mcl::sensorUpdate(Scan scan, double lidar_x, double lidar_y, double lidar_t, bool inv)
{
	const auto logger = rclcpp::get_logger("exp_reset_mcl2");

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
	if (alpha_threshold > 0.0) {
		// alpha_ = nonPenetrationRate( (int)(particles_.size()*extraction_rate_), *map_, scan);
		alpha_ = normalizeBelief()/valid_beams;
		RCLCPP_INFO(logger, "ALPHA: %f / %f", alpha_, alpha_threshold);
		if(alpha_ < alpha_threshold){
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

double Mcl::nonPenetrationRate(int skip, const LikelihoodFieldMap& map, Scan &scan)
{
	static uint16_t shift = 0;
	int counter = 0;
	int penetrating = 0;
	for(int i=shift%skip; i<particles_.size(); i+=skip){
		counter++;
		if(particles_[i].wallConflict(map, scan, range_threshold, sensor_reset))
			penetrating++;
	}
	shift++;

	std::cout << penetrating << " " << counter << std::endl;
	return (double)(counter - penetrating) / counter;
}

void Mcl::expansionReset(void)
{
	for(auto &p : particles_){
		double length = 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_position;
		double direction = 2*((double)rand()/RAND_MAX - 0.5)*M_PI;

		p.p_.x_ += length*cos(direction);
		p.p_.y_ += length*sin(direction);
		p.p_.t_ += 2*((double)rand()/RAND_MAX - 0.5)*expansion_radius_orientation;
		p.w_ = 1.0/particles_.size();
	}
}

void Mcl::resampling(void)
{
	std::vector<double> accum;
	accum.push_back(particles_[0].w_);
	for(int i=1;i<particles_.size();i++){
		accum.push_back(accum.back() + particles_[i].w_);
	}

	std::vector<Particle> old(particles_);

	double start = (double)rand()/(RAND_MAX * particles_.size());
	double step = 1.0/particles_.size();

	std::vector<int> chosen;

	int tick = 0;
	for(int i=0; i<particles_.size(); i++){
		while(accum[tick] <= start + i*step){
			tick++;
			if(tick == particles_.size()){
				RCLCPP_ERROR(rclcpp::get_logger("mcl"), "RESAMPLING FAILED");
				exit(1);
			}	
		}	
		chosen.push_back(tick);
	}

	for(int i=0; i<particles_.size(); i++)
		particles_[i] = old[chosen[i]];
}

void Mcl::motionUpdate(double x, double y, double t)
{
	if(!last_odom_.has_value()){
		last_odom_ = Pose(x, y, t);
		prev_odom_ = Pose(x, y, t);
		return;
	}else
		last_odom_->set(x, y, t);

	Pose d = *last_odom_ - *prev_odom_;
	if(d.nearlyZero())
		return;

	double fw_length = sqrt(d.x_*d.x_ + d.y_*d.y_);
	double fw_direction = atan2(d.y_, d.x_) - prev_odom_->t_;

	odom_model_.setDev(fw_length, d.t_);

	for(auto &p : particles_)
		p.p_.move(fw_length, fw_direction, d.t_,
			odom_model_.drawFwNoise(), odom_model_.drawRotNoise());

	prev_odom_->set(*last_odom_);
}

void Mcl::meanPose(double &x_mean, double &y_mean, double &t_mean,
				double &x_dev, double &y_dev, double &t_dev,
				double &xy_cov, double &yt_cov, double &tx_cov)
{
	double x, y, t, t2;
	x = y = t = t2 = 0.0;
	for(const auto &p : particles_){
		x += p.p_.x_;
		y += p.p_.y_;
		t += p.p_.t_;
		t2 += normalizeAngle(p.p_.t_ + M_PI);
	}

	x_mean = x / particles_.size();
	y_mean = y / particles_.size();
	t_mean = t / particles_.size();
	double t2_mean = t2 / particles_.size();

	double xx, yy, tt, tt2;
	xx = yy = tt = tt2 = 0.0;
	for(const auto &p : particles_){
		xx += pow(p.p_.x_ - x_mean, 2);
		yy += pow(p.p_.y_ - y_mean, 2);
		tt += pow(p.p_.t_ - t_mean, 2);
		tt2 += pow(normalizeAngle(p.p_.t_ + M_PI) - t2_mean, 2);
	}

	if(tt > tt2){
		tt = tt2;
		t_mean = normalizeAngle(t2_mean - M_PI);
	}

	x_dev = xx/(particles_.size() - 1);
	y_dev = yy/(particles_.size() - 1);
	t_dev = tt/(particles_.size() - 1);

	double xy, yt, tx;
	xy = yt = tx = 0.0;
	for(const auto &p : particles_){
		xy += (p.p_.x_ - x_mean)*(p.p_.y_ - y_mean);
		yt += (p.p_.y_ - y_mean)*(normalizeAngle(p.p_.t_ - t_mean));
		tx += (p.p_.x_ - x_mean)*(normalizeAngle(p.p_.t_ - t_mean));
	}

	xy_cov = xy/(particles_.size() - 1);
	yt_cov = yt/(particles_.size() - 1);
	tx_cov = tx/(particles_.size() - 1);
}

double Mcl::normalizeAngle(double t)
{
	while(t > M_PI)
		t -= 2*M_PI;
	while(t < -M_PI)
		t += 2*M_PI;

	return t;
}

double Mcl::normalizeBelief(void)
{
	double sum = 0.0;
	for(const auto &p : particles_)
		sum += p.w_;

	if(sum < 0.000000000001)
		return sum;

	for(auto &p : particles_)
		p.w_ /= sum;

	return sum;
}

void Mcl::resetWeight(void)
{
	for(auto &p : particles_)
		p.w_ = 1.0/particles_.size();
}

void Mcl::initialize(double x, double y, double t)
{
	Pose new_pose(x, y, t);
	for(auto &p : particles_)
		p.p_ = new_pose;

	resetWeight();
}

void Mcl::simpleReset(void)
{
	std::vector<Pose> poses;
	map_->drawFreePoses(particles_.size(), poses);

	for(int i=0; i<poses.size(); i++){
		particles_[i].p_ = poses[i];
		particles_[i].w_ = 1.0/particles_.size();
	}
}

}
