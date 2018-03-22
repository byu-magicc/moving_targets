#pragma once

#define _USE_MATH_DEFINES

#include "waypoint.h"
#include "math.h"
#include "path_follower/base_follower.h"



namespace motion {

	class StraightLine : public BaseFollower {

	public:

		StraightLine() = default;

		StraightLine(float k_path, float chi_infinity) : k_path_(k_path), chi_infinity_(chi_infinity) {}
		
		// Straight line path follower according to "Small Unmaned Aircraft" pg 184 by R. Beard and T.McClain"
		// r : path definition or previous waypoint
		// q : Desired travel directory or previous waypoint - desired waypoint
		// p : Position of agent
		// chi : Current heading of agent
		// chi_c : command heading (input/output)
		// h_c : command altitude (input/output)
		FollowerCommands line_follower(const coord_t& r, const coord_t& q, const coord_t& p, const double& chi) override;

		void set_parameters(const FollowerParams& params) override;

	private:

		// gains
		float k_path_;
		float chi_infinity_;
		

	};



}