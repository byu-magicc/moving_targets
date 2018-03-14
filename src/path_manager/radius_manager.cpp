#include "path_manager/radius_manager.h"


namespace motion {

	
	motion::coord_t RadiusManager::manage_waypoints(const coord_t& pos, waypoints_t& waypoints, double radius) {

	    // Distance between current position and desired waypoint
		coord_t vect = waypoints[0]-pos;
	    float distance = get_magnitude(vect);

	    // switch to next waypoint. 
	    if (distance < radius) {

	        waypoints.push_back(waypoints[0]);
	        waypoints.erase(waypoints.begin());
	    }

	return waypoints[0];

	}

}