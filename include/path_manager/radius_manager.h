#pragma once

#include "waypoint.h"

namespace motion {

	class RadiusManager {

	public:

		RadiusManager() = default;

		coord_t manage_waypoints(const coord_t& pos, waypoints_t& waypoints, double radius);



	};


}