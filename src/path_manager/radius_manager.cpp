#include "path_manager/radius_manager.h"


namespace motion {

	
	float RadiusManager::manage_waypoints(const coord_t& pos, waypoints_t& waypoints, int traj) {

	    // Distance between current position and desired waypoint
		coord_t vect = waypoints[1]-pos;
	    float distance = get_magnitude(vect);

	    // switch to next waypoint. 
	    if (traj != 0 && distance < radius_) {

	        waypoints.push_back(waypoints[0]);
	        waypoints.erase(waypoints.begin());
	    }

	    return distance;

	}

}