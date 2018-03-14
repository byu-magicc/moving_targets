#include "path_follower/linear_path.h"

namespace motion {

void LinearPath::followStrightLine(coord_t waypoint, coord_t pos, double heading) {

	// desired trajectory
	coord_t traj_d = waypoint - pos;

	// desired heading
	double heading_d = atan2(std::get<1>(traj_d), std::get<2>(traj_d));

	while (heading_d - heading < -M_PI) {
		heading_d+= 2*M_PI;
	}

	while (heading_d - heading > M_PI) {
		heading_d+= M_PI;
	}



}




}