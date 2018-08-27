#pragma once

#define _USE_MATH_DEFINES

#include "waypoint.h"
#include "math.h"
#include "path_follower/base_follower.h"

namespace motion {

	class Orbit : public BaseFollower {

	public:
		Orbit() = default;

		Orbit(float k_orbit) : k_orbit_(k_orbit) {}

		// Orbit path follower according to "Small Unmaned Aircraft" pg 184 by R. Beard and T.McClain"
		// c : center of orbit
		// row: radius of orbit
		// lamda: direction of orbit (clockwise or counter clockwise)
		// p : Position of agent
		// chi : Current heading of agent
		// chi_c : command heading (input/output)
		// h_c : command altitude (input/output)
		FollowerCommands orbit_follower(const coord_t& c, const coord_t& p, const double chi) override;

		void set_parameters(const FollowerParams& params) override;

	private:

		// Gains
		float k_orbit_ = 2;
		float rho_ = 1;
		float lambda_ = 1;

	};



}