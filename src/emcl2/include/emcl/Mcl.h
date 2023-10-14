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

	void sensorUpdate(Scan scan, double lidar_x, double lidar_y, double lidar_t, bool inv);

	void motionUpdate(double x, double y, double t);

	void initialize(double x, double y, double t);

	void meanPose(double &x_mean, double &y_mean, double &t_mean,
			double &x_var, double &y_var, double &t_var,
			double &xy_cov, double &yt_cov, double &tx_cov);

	void simpleReset(void);

	double alpha_threshold;
	double expansion_radius_position;
	double expansion_radius_orientation;

	double extraction_rate;
	double range_threshold;
	bool sensor_reset;

	std::vector<Particle> particles_;
	double alpha_;

private:
	std::optional<Pose> last_odom_, prev_odom_;
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
