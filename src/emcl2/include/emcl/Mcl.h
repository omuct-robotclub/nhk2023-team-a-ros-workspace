//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef PF_H__
#define PF_H__

#include <vector>
#include <sstream>
#include <random>
#include <optional>

#include "emcl/Particle.h"
#include "emcl/OdomModel.h"
#include "emcl/LikelihoodFieldMap.h"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace emcl2 {

class Mcl
{
public: 
	Mcl(const Pose &p, int num,
			const OdomModel &odom_model,
			const std::shared_ptr<const LikelihoodFieldMap> map);

	std::vector<Particle> particles_;
	double alpha_;

	void sensorUpdate(const sensor_msgs::msg::LaserScan& msg, double lidar_x, double lidar_y, double lidar_t, bool inv);
	void motionUpdate(double x, double y, double t);

	void initialize(double x, double y, double t);

	void meanPose(double &x_mean, double &y_mean, double &t_mean,
			double &x_var, double &y_var, double &t_var,
			double &xy_cov, double &yt_cov, double &tx_cov);

	void simpleReset(void);

	static double cos_[(1<<16)];
	static double sin_[(1<<16)];

	Scan scan_from_msg(const sensor_msgs::msg::LaserScan& msg);

protected:
	std::optional<Pose> last_odom_;
	std::optional<Pose> prev_odom_;

	double normalizeAngle(double t);
	void resampling(void);
	double normalizeBelief(void);
	void resetWeight(void);

	OdomModel odom_model_;
	std::shared_ptr<const LikelihoodFieldMap> map_;
};

}

#endif
