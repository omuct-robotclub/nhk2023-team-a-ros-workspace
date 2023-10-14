//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/Particle.h"
#include "emcl/lookup_tables.h"
#include <cmath>

namespace emcl2 {
using lut::sin_lut;
using lut::cos_lut;

Particle::Particle(double x, double y, double t, double w) : p_(x, y, t)
{
	w_ = w;
}

double Particle::likelihood(const LikelihoodFieldMap& map, const Scan &scan)
{
	uint16_t t = p_.get16bitRepresentation();
	double lidar_x = p_.x_ + scan.lidar_pose_x_*cos_lut[t] 
				- scan.lidar_pose_y_*sin_lut[t];
	double lidar_y = p_.y_ + scan.lidar_pose_x_*sin_lut[t] 
				+ scan.lidar_pose_y_*cos_lut[t];
	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

	double ans = 0.0;
	for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_){
		if(not scan.valid(scan.ranges_[i]))
			continue;
		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
		double lx = lidar_x + scan.ranges_[i] * cos_lut[a];
		double ly = lidar_y + scan.ranges_[i] * sin_lut[a];

		ans += map.likelihood(lx, ly);
	}
	return ans;
}

// bool Particle::wallConflict(const LikelihoodFieldMap& map, const Scan &scan, double threshold, bool replace)
// {
// 	uint16_t t = p_.get16bitRepresentation();
// 	double lidar_x = p_.x_ + scan.lidar_pose_x_*cos_lut[t]
// 				- scan.lidar_pose_y_*sin_lut[t];
// 	double lidar_y = p_.y_ + scan.lidar_pose_x_*sin_lut[t]
// 				+ scan.lidar_pose_y_*cos_lut[t];
// 	uint16_t lidar_yaw = Pose::get16bitRepresentation(scan.lidar_pose_yaw_);

// 	const bool reverse = rand()%2;
// 	std::vector<int> order;
// 	if(reverse){
// 		for(int i=0;i<scan.ranges_.size();i+=scan.scan_increment_)
// 			order.push_back(i);
// 	}else{
// 		for(int i=scan.ranges_.size()-1;i>=0;i-=scan.scan_increment_)
// 			order.push_back(i);
// 	}

// 	int hit_counter = 0;
// 	for(int i : order){
// 		if(not scan.valid(scan.ranges_[i]))
// 			continue;
	
// 		double range = scan.ranges_[i];
// 		uint16_t a = scan.directions_16bit_[i] + t + lidar_yaw;
	
// 		double hit_lx, hit_ly;
// 		double hit_lx1, hit_ly1, r1;
// 		uint16_t a1;
// 		if(isPenetrating(lidar_x, lidar_y, range, a, map, hit_lx, hit_ly)){
// 			if(hit_counter == 0){
// 				hit_lx1 = hit_lx;
// 				hit_ly1 = hit_ly;
// 				r1 = range;
// 				a1 = a;
// 			}

// 			hit_counter++;
// 		}else
// 			hit_counter = 0;

// 		if(hit_counter*scan.angle_increment_ >= threshold){
// 			if(replace)
// 				sensorReset(lidar_x, lidar_y,
// 						r1, a1, hit_lx1, hit_ly1,
// 						range, a, hit_lx, hit_ly);
// 			return true;
// 		}
// 	}
// 	return false;
// }

// bool Particle::isPenetrating(double ox, double oy, double range, uint16_t direction,
// 		const LikelihoodFieldMap& map, double &hit_lx, double &hit_ly)
// {
// 	// bool hit = false;
// 	// for(double d=map.resolution_;d<range;d+=map.resolution_){
// 	// 	double lx = ox + d * cos_lut[direction];
// 	// 	double ly = oy + d * sin_lut[direction];

// 	// 	if((not hit) and map.likelihood(lx, ly) > 0.99){
// 	// 		hit = true;
// 	// 		hit_lx = lx;
// 	// 		hit_ly = ly;
// 	// 	}
// 	// 	else if(hit and map.likelihood(lx, ly) == 0.0){ // openspace after hit
// 	// 		return true; // penetration
// 	// 	}
// 	// }
// 	// return false;
// 	bool hit = false;
// 	double d = map.safe_distance(ox, oy);
// 	while (true) {
// 		double lx = ox + d * cos_lut[direction];
// 		double ly = oy + d * sin_lut[direction];

// 		if((not hit) and map.likelihood(lx, ly) > 0.99){
// 			hit = true;
// 			hit_lx = lx;
// 			hit_ly = ly;
// 		}
// 		else if(hit and map.likelihood(lx, ly) == 0.0){ // openspace after hit
// 			return true; // penetration
// 		}

// 		d += map.safe_distance(lx, ly);
// 		if (!map.contains(lx, ly) || d > range) break;
// 	}
// 	return false;
// }

void Particle::sensorReset(double ox, double oy,
		double range1, uint16_t direction1, double hit_lx1, double hit_ly1,
		double range2, uint16_t direction2, double hit_lx2, double hit_ly2)
{
	double p1_x = ox + range1 * cos_lut[direction1];
	double p1_y = oy + range1 * sin_lut[direction1];
	double p2_x = ox + range2 * cos_lut[direction2];
	double p2_y = oy + range2 * sin_lut[direction2];

	double cx = (hit_lx1 + hit_lx2)/2;
	double cy = (hit_ly1 + hit_ly2)/2;

	p_.x_ -= (p1_x + p2_x)/2 - cx;
	p_.y_ -= (p1_y + p2_y)/2 - cy;

	double theta_delta = atan2(p2_y - p1_y, p2_x - p1_x) - atan2(hit_ly2 - hit_ly1, hit_lx2 - hit_lx1);
	/*
	double d = std::sqrt((p_.x_ - cx)*(p_.x_ - cx) + (p_.y_ - cy)*(p_.y_ - cy));

	double theta = atan2(p_.y_ - cy, p_.x_ - cx) - theta_delta;
	p_.x_ = cx + d * std::cos(theta);
	p_.y_ = cy + d * std::cos(theta);
*/
	p_.t_ -= theta_delta;
}

Particle Particle::operator =(const Particle &p)
{
	p_ = p.p_;
	w_ = p.w_;
	return *this;
}

}
