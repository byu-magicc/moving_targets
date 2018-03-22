#pragma once

#include "waypoint.h"

namespace motion {

	class RadiusManager {

	public:

		RadiusManager() = default;

		RadiusManager(double radius) : radius_(radius) {}

		float manage_waypoints(const coord_t& pos, waypoints_t& waypoints, int traj);

	private:

		// If the agent is within this distance to the waypoint
		// Then the agent will start moving to the next waypoint
		float radius_ = 1;



	};


}