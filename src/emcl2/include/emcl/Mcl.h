//SPDX-FileCopyrightText: 2022 Ryuichi Ueda ryuichiueda@gmail.com
//SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef EXP_PF2_H__
#define EXP_PF2_H__

#include <vector>
#include <sstream>
#include <random>
#include <optional>

#include "emcl/Particle.h"
#include "emcl/OdomModel.h"
#include "emcl/LikelihoodFieldMap.h"

namespace emcl2 {

class Mcl
{
public: 
	Mcl(const Pose &p, int num,
			const OdomModel& odom_model,
			const std::shared_ptr<const LikelihoodFieldMap> map);

	void sensorUpdate(Scan scan, double lidar_x, double lidar_y, double lidar_t, bool inv, bool do_expansion_reset);

	void motionUpdate(Pose odom);

	void initialize(double x, double y, double t);

	void meanPose(Pose& mean,
			double &x_var, double &y_var, double &t_var,
			double &xy_cov, double &yt_cov, double &tx_cov);

	void simpleReset(void);

	std::optional<Pose> get_prev_odom() { return prev_odom_; }

	double alpha_threshold;
	double expansion_radius_position;
	double expansion_radius_orientation;

	double extraction_rate;
	double range_threshold;
	bool sensor_reset;

	std::vector<Particle> particles_;
	double alpha_;

private:
	std::optional<Pose> prev_odom_;
	std::shared_ptr<const LikelihoodFieldMap> map_;
	OdomModel odom_model_;

	double normalizeAngle(double t);
	void resampling(void);
	double normalizeBelief(void);
	void resetWeight(void);
	void expansionReset(void);

	// double nonPenetrationRate(int skip, const LikelihoodFieldMap& map, Scan &scan);
};

}

#endif
