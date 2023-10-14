//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef OCC_GRID_MAP_H__
#define OCC_GRID_MAP_H__

#include <vector>
#include <utility>
#include <cmath>
#include "emcl/Scan.h"
#include "emcl/Pose.h"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace emcl2 {

class LikelihoodFieldMap
{
public: 
	LikelihoodFieldMap(const nav_msgs::msg::OccupancyGrid &map, double likelihood_range);

	double likelihood(double x, double y) const {
		int ix = (int)std::floor((x - origin_x_)/resolution_);
		int iy = (int)std::floor((y - origin_y_)/resolution_);

		if(ix < 0 or iy < 0 or ix >= width_ or iy >= height_)
			return 0.0;

		return likelihoods_[getIndex(ix, iy)];
	}

	// double safe_distance(double x, double y) const {
	// 	int ix = (int)std::floor((x - origin_x_)/resolution_);
	// 	int iy = (int)std::floor((y - origin_y_)/resolution_);

	// 	if(ix < 0 or iy < 0 or ix >= width_ or iy >= height_)
	// 		return resolution_;

	// 	return distance_field_[getIndex(ix, iy)];
	// }

	bool contains(double x, double y) const {
		int ix = (int)std::floor((x - origin_x_)/resolution_);
		int iy = (int)std::floor((y - origin_y_)/resolution_);

		return !(ix < 0 or iy < 0 or ix >= width_ or iy >= height_);
	}

	size_t getIndex(int x, int y) const { return x + y * width_; }

	std::vector<double> likelihoods_;
	std::vector<double> distance_field_;
	int width_;
	int height_;

	double resolution_;
	double origin_x_;
	double origin_y_;

	void drawFreePoses(int num, std::vector<Pose> &result) const;
private:
	std::vector<std::pair<int, int> > free_cells_;

	void setLikelihood(int x, int y, double range);
	void normalize(void);
	// void updateDistanceField(void);
};

}

#endif

