//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later
//Some lines are derived from https://github.com/ros-planning/navigation/tree/noetic-devel/amcl. 

#include "emcl/LikelihoodFieldMap.h"
#include "emcl/Pose.h"
#include <limits>
#include <random>
#include <algorithm>

namespace emcl2 {

LikelihoodFieldMap::LikelihoodFieldMap(const nav_msgs::msg::OccupancyGrid &map, double likelihood_range)
{
	width_ = map.info.width;
	height_ = map.info.height;

	origin_x_ = map.info.origin.position.x;
	origin_y_ = map.info.origin.position.y;

	resolution_ = map.info.resolution;

	likelihoods_.resize(width_ * height_, 0.0);

	for(int y=0; y<height_; y++) {
		for(int x=0; x<width_; x++){
			int v = map.data[x + y*width_];
			if(v > 50)
				setLikelihood(x, y, likelihood_range);
			else if(0 <= v and v <= 50)
				free_cells_.push_back(std::pair<int, int>(x,y));
		}
	}

	normalize();
	updateDistanceField();
}

void LikelihoodFieldMap::setLikelihood(int x, int y, double range)
{
	int cell_num = (int)ceil(range/resolution_);
	std::vector<double> weights;
	for(int i=0;i<=cell_num;i++)
		weights.push_back(1.0 - (double)i/cell_num);

	for(int i=-cell_num; i<=cell_num; i++)
		for(int j=-cell_num; j<=cell_num; j++)
			if(i+x >= 0 and j+y >= 0 and i+x < width_ and j+y < height_)
				likelihoods_[getIndex(i+x, j+y)] = std::max(likelihoods_[getIndex(i+x, j+y)], 
			                         std::min(weights[abs(i)], weights[abs(j)]));
}

void LikelihoodFieldMap::normalize(void)
{
	double maximum = 0.0;
	for(int x=0; x<width_; x++)
		for(int y=0; y<height_; y++)
			maximum = std::max(likelihoods_[getIndex(x, y)], maximum);

	for(int x=0; x<width_; x++)
		for(int y=0; y<height_; y++)
			likelihoods_[getIndex(x, y)] /= maximum;
}

void LikelihoodFieldMap::updateDistanceField(void) {
	int r = 50;
	distance_field_.resize(width_ * height_, r * resolution_);
	
	std::vector<double> hypot_lut;
	hypot_lut.resize(width_ * height_);
	for(int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			hypot_lut[getIndex(x, y)] = std::max(std::hypot(x * resolution_, y * resolution_), resolution_);
		}
	}

	for (int y = 0; y < height_; y++) {
		for (int x = 0; x < width_; x++) {
			if (likelihoods_[getIndex(x, y)] > 0.99) {
				const int yy_s = std::max(0, y - r);
				const int yy_e = std::min(height_, y + r);
				for (int yy = yy_s; yy < yy_e; yy++) {
					const int xx_s = std::max(0, x - r);
					const int xx_e = std::min(width_, x + r);
					for (int xx = xx_s; xx < xx_e; xx++) {
						const int idx = getIndex(xx, yy);
						distance_field_[idx] = std::min(hypot_lut[getIndex(std::abs(xx - x), std::abs(yy - y))], distance_field_[idx]);
					}
				}
			}
		}
	}
}

void LikelihoodFieldMap::drawFreePoses(int num, std::vector<Pose> &result) const
{
	std::random_device seed_gen;
	std::mt19937 engine{seed_gen()};
	std::vector<std::pair<int, int> > chosen_cells;
	
	sample(free_cells_.begin(), free_cells_.end(), back_inserter(chosen_cells), num, engine);

	for(auto &c : chosen_cells){
		Pose p;
		p.x_ = c.first*resolution_ + resolution_*rand()/RAND_MAX + origin_x_;
		p.y_ = c.second*resolution_ + resolution_*rand()/RAND_MAX + origin_y_;
		p.t_ = 2*M_PI*rand()/RAND_MAX - M_PI;
		result.push_back(p);
	}
}

}
