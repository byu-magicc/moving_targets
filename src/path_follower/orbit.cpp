#include "path_follower/orbit.h"

namespace motion {


FollowerCommands Orbit::orbit_follower(const coord_t& c, const coord_t& p, const double chi) {

	FollowerCommands commands;

	// altitude command
	commands.h_c = std::get<2>(c);

	// XY plane distance between agent and center or orbit
	double d = sqrt( pow(std::get<0>(p) - std::get<0>(c),2) + pow(std::get<1>(p) - std::get<1>(c),2) );

	// command course angle
	double chi_q = atan2(std::get<1>(p) - std::get<1>(c), std::get<0>(p) - std::get<0>(c));

	while (chi_q - chi < -M_PI) {
		chi_q+= 2*M_PI;
	}

	while (chi_q - chi > M_PI) {
		chi_q-= 2*M_PI;
	}

	commands.chi_c = chi_q + lambda_*(M_PI/2 + atan(k_orbit_*(d-rho_)/rho_));

	return commands;

}

//----------------------------------------------------------------------

void Orbit::set_parameters(const FollowerParams& params) {

	rho_ = params.rho;
	lambda_ = params.lambda;

}

}