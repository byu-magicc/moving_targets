#include "path_follower/straight_line.h"

namespace motion {


//----------------------------------------------------------------------

FollowerCommands StraightLine::line_follower(const coord_t& r, const coord_t& q, const coord_t& p, const double& chi) {

	FollowerCommands commands;


	// command altitude
	commands.h_c = std::get<2>(r) + std::get<2>(q);

	// desired heading
	double chi_q = atan2(std::get<1>(q), std::get<0>(q));

	while (chi_q - chi < -M_PI) {
		chi_q+= 2*M_PI;
	}

	while (chi_q - chi > M_PI) {
		chi_q-= 2*M_PI;
	}

	// Error
	double epy = -sin(chi_q)*(std::get<0>(p) - std::get<0>(r)) + cos(chi_q)*(std::get<1>(p)-std::get<1>(r));

	// Compute command course angle
	// atan saturates between -pi/2 and pi/2
	// Thus    -1 < 2/M_PI*atan(k_path_*epy) < 1
	commands.chi_c = chi_q - chi_infinity_*2.0/M_PI*atan(k_path_*epy);
	return commands;


}

//----------------------------------------------------------------------


void StraightLine::set_parameters(const FollowerParams& params) {

	k_path_ = params.k_path;
	chi_infinity_ = params.chi_infinity;
	
}




}